#include <controllers.h>
float PI_controller(float u,float K,float Tz,float Tsample,float *integral_ptr,float *last_u_ptr)
//discrete approximation of continuous PI Y=R.U with R=K(1+sTz)/s
//integral stores the state of the controller
//last_u is updated by function 
{
  float y;
  *integral_ptr=*integral_ptr+K*Tsample*(*last_u_ptr);//(u+last_u)/2 more difficult to condition
  y=K*u*Tz+(*integral_ptr);
  *last_u_ptr=u;
  return(y);
}
float LL_controller(float u,float K,float Tz,float Tp,float Tsample,float *integral_ptr,float *last_u_ptr,float *last_y_ptr)
{
//discrete approximation of continuous lead-lag Y=R.U with R=K(1+sTz)/(1+sTp)
//integral stores the state of the controller
//last_u is updated by function 
  float y;
  *integral_ptr=*integral_ptr+Tsample/Tp*(K*(*last_u_ptr)-(*last_y_ptr));
  y=K*u*Tz/Tp+(*integral_ptr);
  *last_u_ptr=u;
  *last_y_ptr=y;
  return(y);
} 
void LL_controller_disc(float K,float Zz,float Zp,float *u_ptr,float *last_u_ptr,float *y_ptr,float *last_y_ptr,uint8_t step,uint8_t do_conditioning)
{
//discrete lead-lag controller with zero in Zz and pole in Zp Y=R.U with R=K(z-Zz)/(1-Zp)
//step=1: compute output y
//step=2: 
//      if do_condtioning=1, recompute u corresponding to the true (saturated) y
//      prepare for next sampling period : update  last_u=u and last_y=y 
  float y;
if (step==1) //compute y
  {
  *y_ptr=K*((*u_ptr)-Zz*(*last_u_ptr))+Zp*(*last_y_ptr);
  }
 if (step==2)  
  {
    if (do_conditioning) //conditionning. compute u corresponding to y
       *u_ptr=((*y_ptr)-Zp*(*last_y_ptr))/K+Zz*(*last_u_ptr);
    //prepare for next sampling period 
    *last_u_ptr=*u_ptr;
    *last_y_ptr=*y_ptr;  
  }

  
} 


