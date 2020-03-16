#include "lib_el009.h"
#include "TimerOne.h"
#include <Wire.h>
#define T_SAMPLE 20E-3
#define V_DC_BUS 1 //has no meaning here. Used so that V is duty_cycle

//controller modes (requested by matlab and passed to my_callback) 
#define  OPEN_LOOP 0
#define  CLASSICAL_CONTROLLER 1//classical (PID family) controller
#define  STATE_SPACE 2

float AcX=0,AcY=0,AcZ=0,theta;
#define MPU_ADDR 0x68 // address of MPU6050
#define USE_FILTER 1
#define ALPHA 0.01//0.1

float my_param;//set by set_mode_param. Can be used in controller if needed.
float KP = 0.0; // set by set_mode_param. gain of proportional controller
float K1 = 0.0, K2 = 0.0; // elements of the ss gain matrix K (single input)
float OA11, OA12, OA21, OA22, OB11, OB12, OB21, OB22; // elements of the observer matrices A & B
float x1, x2; // proxies for state variables

void measure_acc()
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

  if (USE_FILTER){
    AcX = (1.0-ALPHA) * AcX + ALPHA*(float)AcX_;
    AcY = (1.0-ALPHA) * AcY + ALPHA*(float)AcY_;
    AcZ = (1.0-ALPHA) * AcZ + ALPHA*(float)AcZ_;
  } else {
    AcX = (float)AcX_;
    AcY = (float)AcY_;
    AcZ = (float)AcZ_;
  }
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* transform input angle (in degrees) to PWM length */
float compute_pwm(float sin_phi)
{
  float phi;
  sin_phi = constrain(sin_phi, -1, 1);
  phi=asin(sin_phi);
  return mapf(phi, -PI/2, PI/2, 800E-6, 2300E-6);//puse width in s
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
    case CLASSICAL_CONTROLLER:
      if (n_param==1)
      {
        KP = buf[0];
      }
      else 
          return(3);//error: bad n_param 
       break;
    case STATE_SPACE:
      if (n_param==10) {
        OA11 = buf[0];
        OA12 = buf[1];
        OA21 = buf[2];
        OA22 = buf[3];
        OB11 = buf[4];
        OB12 = buf[5];
        OB21 = buf[6];
        OB22 = buf[7];
        K1   = buf[8];
        K2   = buf[9];}
        break;     

    default : 
      return(2);//mode not defined  
  }
  return(0);
}

void setup()
#
{
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  el009_setup(T_SAMPLE);
}


void my_callback(float w,byte write_serial,byte mode)
{
  float   sin_phi=0;
  float   duty_cycle;
  float y,u;

//----------compute command-----------------
   switch (mode) {
    case OPEN_LOOP:
      sin_phi = w;
      break;
      
    case CLASSICAL_CONTROLLER:
      // Implement classical current controller here
      sin_phi = KP*(w-theta);
      break;
    case STATE_SPACE:
      // Observer
      y = theta;
      u = sin_phi;
      x1 = OA11 * x1 + OA12 * x2 + OB11 * u + OB12 * y;
      x2 = OA12 * x1 + OA22 * x2 + OB21 * u + OB22 * y;

      // Controller
      sin_phi = w - K1 * x1 - K2 * x2;
      
      break;
    default : 
      sin_phi=0;  
  }

  float T_pulse=compute_pwm(sin_phi);
  duty_cycle=T_pulse/T_SAMPLE;

  //------------issue command----------------
   PWMAnalogWrite(duty_cycle,V_DC_BUS);
   //----------write measurements on the serial port----------
   //measurements can be read in matlab using get_response.m
   //you can chose the data to send (Max 3 values at 0.5kHz)
   if (write_serial)
   //write measurements you need in matlab.
   {
     float_write_serial(w);
     float_write_serial(theta);
     float_write_serial(sin_phi);
   }
}


float measure_roll()
{
  float angle;
  measure_acc();
  angle = (float)atan(AcX/AcZ);
  return angle;
}

void loop()
{
 theta = measure_roll();
 el009_loop();
}
