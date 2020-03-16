#include "lib_el009.h"
#include "TimerOne.h"
#include <Wire.h>

#define I2CSLAVE 9 // slave address of the arduino simulator
#define T_SAMPLE 20E-3

/* communication modes */
#define COMMAND       1    /* set the controllable input of the system */
#define DISTURBANCE   2    /* apply disturbance to system */
#define STATE         3    /* set the initial value of the system */

//controller modes (requested by matlab and passed to my_callback) 
#define  OPEN_LOOP    0
#define  CLASSICAL    1//classical (PID family) current controller
#define  STATE_SPACE  2

float theta; // measurement coming from simulator

float my_param;//set by set_mode_param. Can be used in controller if needed.

void get_measurement(float *value)
{
  Wire.requestFrom(I2CSLAVE, 4);
  byte *bb = (byte *)value;
  int i = 0;
  while(Wire.available()){    // slave may send less than requested
   bb[i] = Wire.read(); // receive a byte as character  
   i++;
  }
}

void set_phi(float phi)
{
  Wire.beginTransmission(I2CSLAVE);
  Wire.write(COMMAND);
  write_i2c(phi);
  Wire.endTransmission();
}

void write_i2c(float val)
{
  byte* b = (byte *)&val;
  Wire.write(b,4);
}

int set_mode_param(byte mode,int n_param,float *buf)
//This function is called when matlab calls set_mode_param
//Set parameters sent by matlab if needed. Else just return 0.
//return 0 if no error
{ 
switch (mode) {
    case OPEN_LOOP:
       if (n_param==1)
       {
         my_param=buf[0];//if more parameters are sent, they are available in buf[1], ...
       }
       else 
          return(3);//error: bad n_param 
        break;
    default : 
      return(2);//mode not defined  
  }
  return(0);
}

void setup()
{
  el009_setup(T_SAMPLE);
  Wire.begin(); // join i2c bus as master
  Serial.println("..... ok .....");
}


void my_callback(float w,byte write_serial,byte mode)
{
  float   sin_phi = 0.0;
  float   phi     = 0.0;
  float   K = 0.;
  //----------read measurements----------
  get_measurement(&theta);
  //----------compute command-----------------
  mode=CLASSICAL;
   switch (mode) {
    case OPEN_LOOP:
      sin_phi = w;
      break;
    case CLASSICAL:
      // Implement classical current controller here
      sin_phi = K*(w-theta);
      break;
    case STATE_SPACE:
      // Implement classical state-space controller here
      break;
    default : 
      sin_phi=0;  
  }
  phi = asin(sin_phi);
  phi = 0.0;
  
  //------------issue command----------------
   set_phi(phi);
   //----------write measurements on the serial port----------
   //measurements can be read in matlab using get_response.m
   //you can chose the data to send (Max 3 values at 0.5kHz)
   if (write_serial)
   //write measurements you need in matlab.
   {
     float_write_serial(w);
     float_write_serial(phi);
     float_write_serial(theta);
   }
}


void loop()
{
 el009_loop();
}
