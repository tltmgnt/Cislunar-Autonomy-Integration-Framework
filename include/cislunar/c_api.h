#ifndef CISLUNAR_C_API_H
#define CISLUNAR_C_API_H
#ifdef __cplusplus
extern "C" {
#endif
typedef struct ca_controller ca_controller;
/* State order x,y,z,vx,vy,vz; normalized rotating-frame units. */
typedef struct { double state[6]; double sensor_age,battery,temperature; int sensor_valid,ground_link; } ca_observation;
typedef struct { double acceleration[3]; int mode; double used_delta_v; } ca_decision;
/* mode: 0 nominal, 1 disconnected, 2 safe. NULL on allocation failure. */
ca_controller* ca_create(void);
void ca_destroy(ca_controller*);
/* 0 success (including safe mode), -1 invalid pointer, -2 internal failure. */
int ca_step(ca_controller*,const ca_observation*,const double target[6],double dt,ca_decision*);
#ifdef __cplusplus
}
#endif
#endif
