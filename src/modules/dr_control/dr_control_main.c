/**
 * @file:     dr_control_main.c
 * @function: Dualrotor controller.
 * @author:   Haocheng Wen
 * @date:     30 Aug, 2014
 * @modify:   Name	Date
  
 ***********************************/

/* System include */
#include <nuttx/config.h>
#include <nuttx/i2c.h>
#include <nuttx/mtd.h>
#include <nuttx/fs/nxffs.h>
#include <nuttx/fs/ioctl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <stdbool.h>
#include <fcntl.h>
#include <sys/mount.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>

#include "systemlib/systemlib.h"
#include "systemlib/err.h"
#include "drivers/drv_pwm_output.h"

/* DR_control include */
#include "dr_control_main.h"
#include "params.h"
#include <systemlib/pid/pid.h>
#include <systemlib/lqr/lqr.h>

/*******************************************
		Global variables
*******************************************/
static bool thread_should_exit = false;	//daemon exit flag 
static bool thread_running = false;	//daemon status flag 
static int  control_task;		//Handle of daemon task / thread
static struct params p;			//parameter struct
static struct param_handles ph; 	//parameter handle
int path; 				//the path to pwm_output
int motor_down[2]={1000,1000};		//minimal value for motor
int servo_neutral[2]={1485,1525};	//neutral value for servo
float servo_k[2]={-535.1f,-505.9f};	//coefficient for servo: Hz/rad
int motor[2]={1000,1000};		//motor output
int servo[2]={1500,1500};		//servo output
float pitch_pid_val=0.0f;
float roll_pid_val =0.0f;
float yaw_pid_val  =0.0f;
float throttle_pid_val=0.0f;


/*******************************************
		Heads of function
*******************************************/
__EXPORT int dr_control_main(int argc, char *argv[]);

int dr_control_thread_main(int argc, char *argv[]);

static void usage(const char *reason);

void motor_output(void);


/*******************************************
		PWM output
*******************************************/
void motor_output(void)
{
	/*
	motor[0]->J2_8
	motor[1]->J2_7
	servo[0]->J2_6
	servo[1]->J2_4 !!!
	*/
	if(motor[0]<1050) motor[0]=1000;
	if(motor[0]>1800) motor[0]=1800;
	if(motor[1]<1050) motor[1]=1000;
	if(motor[1]>1800) motor[1]=1800;
	if(servo[0]<1300) servo[0]=1300;
	if(servo[0]>1650) servo[0]=1650;
	if(servo[1]<1300) servo[1]=1300;
	if(servo[1]>1700) servo[1]=1700;

	ioctl(path, PWM_SERVO_SET(0), motor[0]);
	ioctl(path, PWM_SERVO_SET(1), motor[1]);
	ioctl(path, PWM_SERVO_SET(2), servo[0]);
	ioctl(path, PWM_SERVO_SET(3), servo[1]);
}


/*******************************************
		Main function
*******************************************/
int dr_control_thread_main(int argc, char *argv[])
{
	
	/* Welcome */
	warnx("[dr_control] starting\n");
	thread_running = true;
	
	/* Open the pwm_output */
	path = open(PWM_OUTPUT_DEVICE_PATH, 0);
	// warnx(PWM_OUTPUT_DEVICE_PATH); // outputs /dev/pwm_output

	/* Initialize parameters, first the handles, then the values */
	parameters_init(&ph);
	parameters_update(&ph, &p);
	p.takeoff = 0;

	/* Initialize yaw */
	bool yaw_init = true;
	float yaw_init_val=0.0f;

#ifdef PID	
	/* Initialize PID param */
	pid_mode_t mode;

	PID_t pid_throttle;
	PID_t pid_roll;
	PID_t pid_pitch;
	PID_t pid_yaw;
	
	pid_init(&pid_throttle,  PID_MODE, 0.002f);
	pid_init(&pid_roll,  PID_MODE, 0.002f);
	pid_init(&pid_pitch, PID_MODE, 0.002f);
	pid_init(&pid_yaw,   PID_MODE, 0.002f);

	pid_set_parameters(&pid_throttle,  p.throttle_p,  p.throttle_i,  p.throttle_d,  0, 0);
	pid_set_parameters(&pid_roll,  p.roll_p,  p.roll_i,  p.roll_d,  0, 0);
	pid_set_parameters(&pid_pitch, p.pitch_p, p.pitch_i, p.pitch_d, 0, 0);
	pid_set_parameters(&pid_yaw,   p.yaw_p,   p.yaw_i,   p.yaw_d,   0, 0);
#endif

#ifdef LQR
#endif
	
	/* Declare and safely initialize all structs to zero */
	//	Status input	 
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));	
	struct vehicle_status_s vstatus;
	memset(&vstatus, 0, sizeof(vstatus));
	//	Control input
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct vehicle_rates_setpoint_s rates_sp;
	memset(&rates_sp, 0, sizeof(rates_sp));
	struct manual_control_setpoint_s manual_sp;
	memset(&manual_sp, 0, sizeof(manual_sp));


	/* Subscribe to topics. */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	int manual_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int vstatus_sub = orb_subscribe(ORB_ID(vehicle_status));
	int param_sub = orb_subscribe(ORB_ID(parameter_update));
	

	struct pollfd fds[2] = 
	{
		{ .fd = param_sub, .events = POLLIN },
		{ .fd = att_sub,   .events = POLLIN }
	};

	/**********************************************
			Main Loop
	**********************************************/
	while (!thread_should_exit) 
	{
		/* Wait for a sensor or param update */
		int ret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);
		if (ret < 0) 
		{
			warnx("poll error");
			continue;
		} else if (ret == 0) 
		{
			continue;
		} else 
		{
			/*********************************************** 
				Update parameters if they changed 
			***********************************************/
			if (fds[0].revents & POLLIN) 
			{
				/* Read from param to clear updated flag (uORB API requirement) */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), param_sub, &update);

				/* Update */
				parameters_update(&ph, &p);
				pid_set_parameters(&pid_throttle,  p.throttle_p,  p.throttle_i,  p.throttle_d,  0, 0);
				pid_set_parameters(&pid_roll,  p.roll_p,  p.roll_i,  p.roll_d,  0, 0);
				pid_set_parameters(&pid_pitch, p.pitch_p, p.pitch_i, p.pitch_d, 0, 0);
				pid_set_parameters(&pid_yaw,   p.yaw_p,   p.yaw_i,   p.yaw_d,   0, 0);
			}
			
			/***********************************************
					  Controller				
			    (Run the controller if attitude changed) 
			***********************************************/
			if (fds[1].revents & POLLIN) 
			{
				/* Get the time distance */
				static uint64_t last_run = 0;
				float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
				last_run = hrt_absolute_time();

				// Guard against too small (< 2ms) and too large (> 20ms) dt's
				if (dt < 0.002f) {
					dt = 0.002f;

				} else if (dt > 0.02f) {
					dt = 0.02f;
				}

				// Update the attitude
				orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
				// Initial yaw
				if(yaw_init==true){
					yaw_init_val = att.yaw;
					yaw_init=false;
				}
				

				// Update manual input if it changed
				bool manual_sp_updated;
				orb_check(manual_sp_sub, &manual_sp_updated);
				if (manual_sp_updated)
				{
					orb_copy(ORB_ID(manual_control_setpoint), manual_sp_sub, &manual_sp);
				}

				/* Controller implementation */
			#ifdef PID
				pitch_pid_val = pid_calculate(&pid_pitch, 0.0f, att.pitch, 0.0f, dt);
				roll_pid_val  = pid_calculate(&pid_roll,  0.0f, att.roll,  0.0f, dt);
				yaw_pid_val   = pid_calculate(&pid_yaw,   yaw_init_val, att.yaw, 0.0f, dt);
				//pitch_pid_val = pid_calculate(&pid_pitch, manual_sp.x, att.pitch, 0.0f, dt);
				//roll_pid_val  = pid_calculate(&pid_roll,  manual_sp.y, att.roll,  0.0f, dt);
				//yaw_pid_val   = pid_calculate(&pid_yaw,   manual_sp.r, att.yaw-yaw_init_val,   0.0f, dt);
				//throttle_pid_val = pid_calculate(&pid_m ,motor_control_sp, last_throttle,0.0f, dt);
				//throttle_pid_val = manual_sp.z;
			
				servo[0] = servo_neutral[0]+servo_k[0]*pitch_pid_val;
				servo[1] = servo_neutral[1]-servo_k[1]*pitch_pid_val;
				if(yaw_pid_val<0){
					servo[0]+=yaw_pid_val*servo_k[0];
				}else{
					servo[1]+=yaw_pid_val*servo_k[1];
				}

				motor[0] = motor_down[0]+roll_pid_val;
				motor[1] = motor_down[1]-roll_pid_val;
			#endif

			#ifdef LQR
				
			#endif

				/* PWM output */
				if(p.takeoff==0){		//Wait
					motor[0]=motor_down[0];
					motor[1]=motor_down[1];
				}else if(p.takeoff==1){		//Take off
					motor[0]+=100;
					motor[1]+=100;
				}else if(p.takeoff==-1){	//Land
					motor[0]-=100;
					motor[1]-=100;
				}else{				//Control input
					motor[0]+=p.takeoff;
					motor[1]+=p.takeoff;
				}
				motor_output();
			}
		}
	}

	warnx("[dr_control] exiting.\n");

	thread_running = false;
	
	return 0;
}


/*******************************************
		Usage of command
*******************************************/
static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: dr_control {start|stop|status}\n\n");
	exit(1);
}


/*******************************************
		Command function
*******************************************/
int dr_control_main(int argc, char *argv[])
{
	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("dr_control already running\n");
			exit(0);
		}

		/* Create task and enable the loop */
		thread_should_exit = false;
		control_task = task_spawn_cmd("dr_control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 5,
					 2048,
					 dr_control_thread_main,
					 (argv) ? (const char **)&argv[2] : (const char **)NULL);
		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");
		} else {
			warnx("\tnot started\n");
		}
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}







