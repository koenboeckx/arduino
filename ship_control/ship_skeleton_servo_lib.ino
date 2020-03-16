#include"ServoTimer2.h"  // include the Servo library for motor control
#include <Wire.h>    // incluse I2C for communication with MPU6050

#include "lib_el009.h"
#include "TimerOne.h"

//controller modes (requested by matlab and passed to my_callback) 
#define  OPEN_LOOP    0
#define  CLASSICAL    1  //classical controller
#define  STATE_SPACE  2

#define T_SAMPLE 20E-3
#define MPU_ADDR 0x68 // address of MPU6050
#define POTPIN   1
#define MOTORPIN 6
    
#define USE_FILTER 1 /* 0: not active; 1: active */
#define ALPHA 0.1

ServoTimer2 Motor;
float AcX,AcY,AcZ;
float theta;    // in radians
float phi = 90; // in degrees
float my_param, k_cl;//set by set_mode_param. Can be used in controller if needed.
float phi_meas;

int set_mode_param(byte mode,int n_param,float *buf)
/* This function is called when matlab calls set_mode_param
 * Set parameters sent by matlab if needed. Else just return 0.
 * returns 0 if no error
 */
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
    case CLASSICAL:
      if (n_param == 1)
      {
        k_cl = buf[0];
      }
      else k_cl = -1;
      break;
    default : 
      return(2);//mode not defined  
  }
  return(0);
}

void setup()
#
{
  // configure motor
  Motor.attach(MOTORPIN);  // attaches the servo on pin MOTORPIN to a servo object
  
  // activate MPU6050 communication
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  el009_setup(T_SAMPLE);

  el009_setTxBaudRate(9600);
  el009_setRxBaudRate(9600);
}


void my_callback(float w,byte write_serial,byte mode)
{
  float e;
  //----------read measurements----------
  // measurements are performed in loop()
  // measure_acc(&AcX, &AcY, &AcZ); // measure acceleration along three axes
  // theta = measure_roll();
  phi_meas = measure_arm();
 
//----------compute command-----------------
   switch (mode) {
    case OPEN_LOOP:
      phi = asinf(w)*180/PI;
      break;
    case CLASSICAL:
      // Implement classical current controller here
      e = w - theta;
      phi = k_cl * e;
      break;
    case STATE_SPACE:
      // Implement classical state-space controller here
      break;
    default : 
      phi = 180;  
  }

 
  //------------issue command----------------
   //Motor.write(constrain(phi, 0, 180));
   
   //----------write measurements on the serial port----------
   //measurements can be read in matlab using get_response.m
   //you can chose the data to send (Max 3 values at 0.5kHz)
   if (write_serial)
   //write measurements you need in matlab.
   {
    float_write_serial(w);
    float_write_serial(phi);
    float_write_serial(theta);
    float_write_serial(phi_meas);
   }
}


void loop()
{
 el009_loop();

 //----------read measurements----------
  measure_acc(&AcX, &AcY, &AcZ); // measure acceleration along three axes
  theta = measure_roll();

  //------------issue command----------------
  Motor.write(compute_pwm(phi+90));
}

void measure_acc(float* AcX, float* AcY, float* AcZ)
{
  int16_t AcX_, AcY_, AcZ_;
  int16_t Tmp,GyX,GyY,GyZ; // meausred but not used
  // -- read MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);  // request a total of 14 registers
  AcX_=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) + 0x3C (ACCEL_XOUT_L)    
  AcY_=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) + 0x3E (ACCEL_YOUT_L)
  AcZ_=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) + 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H)   + 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H)  + 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H)  + 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H)  + 0x48 (GYRO_ZOUT_L)

  if (USE_FILTER == 1){
    *AcX = (1-ALPHA) * *AcX + (float)AcX_;
    *AcY = (1-ALPHA) * *AcY + (float)AcY_;
    *AcZ = (1-ALPHA) * *AcZ + (float)AcZ_;
  } else {
    *AcX = (float)AcX_;
    *AcY = (float)AcY_;
    *AcZ = (float)AcZ_;
  }
}
float measure_arm()
{
  float val = analogRead(POTPIN);
  return (val - 189)/435*180;
}
float compute_pwm(float theta)
{
  /* transform input angle (in degrees) to PWM length */
  theta = constrain(theta, 20, 160);
  return map(theta, 0, 180, 750, 2350);
}

// do not include in skeleton for students:
float measure_roll()
{
  float angle, num;
  float AcX, AcY, AcZ;
  measure_acc(&AcX, &AcY, &AcZ);
  angle = (float)atan(AcX/AcZ);
  return angle;
}
