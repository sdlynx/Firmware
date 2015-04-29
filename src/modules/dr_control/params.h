
/*
 * @file params.h
 * 
 * Definition of parameters
 */
#ifndef PARAMS_H_
#define PARAMS_H_

#include <systemlib/param/param.h>
#include "dr_control_main.h"

struct params {
	int takeoff;
#ifdef PID
	float throttle_p;
	float throttle_i;
	float throttle_d;
	float yaw_p;
	float yaw_i;
	float yaw_d;
	float roll_p;
	float roll_i;
	float roll_d;
	float pitch_p;
	float pitch_i;
	float pitch_d;
#endif

#ifdef LQR
#endif
};

struct param_handles {
	int takeoff;
#ifdef PID
	param_t throttle_p;
	param_t throttle_i;
	param_t throttle_d;
	param_t yaw_p;
	param_t yaw_i;
	param_t yaw_d;
	param_t roll_p;
	param_t roll_i;
	param_t roll_d;
	param_t pitch_p;
	param_t pitch_i;
	param_t pitch_d;
#endif

#ifdef LQR
#endif

};

/**
 * Initialize all parameter handles and values
 *
 */
int parameters_init(struct param_handles *h);

/**
 * Update all parameters
 *
 */
int parameters_update(const struct param_handles *h, struct params *p);

#endif /* PARAMS_H_ */
