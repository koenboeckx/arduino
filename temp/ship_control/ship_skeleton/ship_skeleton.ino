#include "lib_el009.h"
#include "TimerOne.h"
#include <Wire.h>
#include <math.h>

#define I2CSLAVE 9 // slave address of the arduino simulator
#define T_SAMPLE 20E-3

/* communication modes */
#define COMMAND       1    /* set the controllable input of the system */
#define DISTURBANCE   2    /* apply disturbance to system */
#define STATE         3    /* set the initial value of the system */

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//controller modes (requested by matlab and passed to my_callback) 
#define OPEN_LOOP             0
#define CLASSICAL             1
#define STATE_SPACE           2
#define PROPORTIONAL          3
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

float theta = 0.0;  // measurement coming from simulator
float   phi = 0.0;  // command to issue to system

float my_param;//set by set_mode_param. Can be used in controller if needed.

// helper functions
float read_i2c()
{
  float val = 0.0;
  char *bb=(char *)&val;

  int i = 0;
  
  while (Wire.available()) { // slave may send less than requested
    bb[i] = Wire.read(); // receive a byte as character
    i++;
  }
  return val;
}


void write_i2c(float phi)
{
  byte* b_phi = (byte *)&phi;

  Wire.beginTransmission(I2CSLAVE);
  Wire.write(COMMAND);
  Wire.write(b_phi,4);
  Wire.endTransmission();
}

void getMeasurement(float *theta) {
  Wire.requestFrom(I2CSLAVE, 4);    // request 4 bytes from slave device
  *theta = read_i2c();

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void setup()
{
  // Initialize the 'Wire' class for the I2C-bus.
  Wire.begin();
  
  // Call the general initialization routine
  el009_setup(T_SAMPLE); // only serial & timer callback, no I2C
  
  //el009_setRxBaudRate(115200);
  el009_setRxBaudRate(9600);
}


void my_callback(float w, byte write_serial, byte mode)
{
  float K       = 0.;
  
  //----------read measurements----------
  //
  //----------compute command-----------------
   switch (mode) {
    case OPEN_LOOP:
      phi = w;
      break;
    case CLASSICAL:
      // Implement classical current controller here
      phi = K*(w-theta);
      break;
    case STATE_SPACE:
      // Implement classical state-space controller here
      break;
  }
  
   //----------write measurements on the serial port----------
   //measurements can be read in matlab using get_response.m
   //you can chose the data to send (Max 3 values at 0.5kHz)
   if (write_serial) //write measurements you need in matlab.
   {
     float_write_serial(w);
     float_write_serial(phi);
     float_write_serial(theta);
   }
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
int set_mode_param(byte mode, int n_param, float *buf)
// This function is called when matlab calls set_mode_param
// Set parameters sent by matlab if needed. Else just return 0.
//
// Modify this function as needed to communicate parameters from matlab to arduino
//
// Return 0 if no error
//
{
  switch (mode) {
    case OPEN_LOOP:
       if (n_param==1)
       {
         my_param = buf[0]; // If more parameters are sent, they are available in buf[1] and so on, ...
       }
       else
         return(3);// Error: bad n_param
       break;
    default :
      return(2);// Mode not defined
  }
  return(0);
}

void loop()
{
  el009_loop();

  // get meausrement and set command -> not possible in timer interrupt because through I2C
  getMeasurement(&theta);
  write_i2c(phi);
}
