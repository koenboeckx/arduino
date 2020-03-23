#ifndef CONTROLLERS_h
#define CONTROLLERS_h
#include <stdint.h>
float PI_controller(float u,float K,float Tz,float Tsample,float *integral_ptr,float *last_u_ptr);
float LL_controller(float u,float K,float Tz,float Tp,float Tsample,float *integral_ptr,float *last_u_ptr,float *last_y_ptr);
void LL_controller_disc(float K,float Zz,float Zp,float *u_ptr,float *last_u_ptr,float *y_ptr,float *last_y_ptr,uint8_t step,uint8_t do_conditioning);
#endif
