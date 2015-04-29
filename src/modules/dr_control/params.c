
#include "params.h"

/* Command parameters */
PARAM_DEFINE_INT32(DR_takeoff, 0);



/* PID parameters */
//throttle
PARAM_DEFINE_FLOAT(DR_throttle_p, 0.0f);
PARAM_DEFINE_FLOAT(DR_throttle_i, 0.0f);
PARAM_DEFINE_FLOAT(DR_throttle_d, 0.0f);
// yaw
PARAM_DEFINE_FLOAT(DR_yaw_p, 0.0f);
PARAM_DEFINE_FLOAT(DR_yaw_i, 0.0f);
PARAM_DEFINE_FLOAT(DR_yaw_d, 0.0f);
//roll
PARAM_DEFINE_FLOAT(DR_roll_p, 0.0f);
PARAM_DEFINE_FLOAT(DR_roll_i, 0.0f);
PARAM_DEFINE_FLOAT(DR_roll_d, 0.0f);
//pitch
PARAM_DEFINE_FLOAT(DR_pitch_p, 0.0f);
PARAM_DEFINE_FLOAT(DR_pitch_i, 0.0f);
PARAM_DEFINE_FLOAT(DR_pitch_d, 0.0f);

/* LQR parameters */




int parameters_init(struct param_handles *h)
{
	h->takeoff =  param_find("DR_takeoff");

#ifdef PID
	/* PID parameters */
	h->throttle_p=param_find("DR_throttle_p");
	h->throttle_i=param_find("DR_throttle_i");
	h->throttle_d=param_find("DR_throttle_d");
	h->yaw_p   =  param_find("DR_yaw_p");
	h->yaw_i   =  param_find("DR_yaw_i");
	h->yaw_d   =  param_find("DR_yaw_d");
	h->roll_p  =  param_find("DR_roll_p");
	h->roll_i  =  param_find("DR_roll_i");
	h->roll_d  =  param_find("DR_roll_d");
	h->pitch_p =  param_find("DR_pitch_p");
	h->pitch_i =  param_find("DR_pitch_i");
	h->pitch_d =  param_find("DR_pitch_d");
#endif

#ifdef LQR
#endif

	return OK;
}

int parameters_update(const struct param_handles *h, struct params *p)
{
	param_get(h->takeoff, &(p->takeoff));

#ifdef PID
	param_get(h->throttle_p, &(p->throttle_p));
	param_get(h->throttle_i, &(p->throttle_i));
	param_get(h->throttle_d, &(p->throttle_d));
	param_get(h->yaw_p, &(p->yaw_p));
	param_get(h->yaw_i, &(p->yaw_i));
	param_get(h->yaw_d, &(p->yaw_d));
	param_get(h->roll_p, &(p->roll_p));
	param_get(h->roll_i, &(p->roll_i));
	param_get(h->roll_d, &(p->roll_d));
	param_get(h->pitch_p, &(p->pitch_p));
	param_get(h->pitch_i, &(p->pitch_i));
	param_get(h->pitch_d, &(p->pitch_d));
#endif

#ifdef LQR
#endif

	return OK;
}







