#include <Wire.h>//for I2C com
#include <TimerOne.h>
#include "utilities.h"

#define PWM_PIN 9 //9 or 10 for timer1

unsigned long TX_BAUD_RATE = 115200; 
unsigned long RX_BAUD_RATE = 9600;

float w       = 0;          // reference value
float next_w  = 0;
float Tsample = 0;

int cur_meas  = 0;
int n_meas_total = 2;
int n_meas_before= 1; //to avoid strange behavior before first get_response
int meas_just_finished = 0; 
int downsample = 1, cur_downsample = 0;

byte mode = 0;          //controller mode (sent to my_callback)
#define BUF_SIZE 10
float buf[BUF_SIZE];    // used to store mode parameters (passed to set_mode_params)

void common_setup(float Tsample) {
    Serial.begin(RX_BAUD_RATE);
    Serial.setTimeout(-1);

    Timer1.initialize(Tsample/1E-6);            // initialize timer1,
    Timer1.pwm(PWM_PIN,0);                    // setup pwm on pin PWM_PIN, 0% duty cycle
    //Timer1.attachInterrupt(common_callback);    // attaches common_callback() as a timer overflow
    
    Wire.begin(); // join i2c bus as master
}

void common_callback()
{
  if(cur_meas==n_meas_total-n_meas_before) w=next_w;
  if(cur_meas>0){
    cur_downsample--;
    if (cur_downsample==0) {
        cur_meas--;
        my_callback(w, 1, mode);    //write serial data
        if(cur_meas==0)  meas_just_finished=1; 
        cur_downsample=downsample;
    }
    
  } else {
   my_callback(w,0,mode);           //don't write serial data
   if  (meas_just_finished) {
        Serial.begin(RX_BAUD_RATE);
        meas_just_finished=0;
    }
  }
}

unsigned long read_serial_long()
{ 
   unsigned long val=0;
   char *bb=(char *)&val;
   Serial.readBytes(bb,4);
   return(val);
}

float read_serial_float()
{ 
   float val=0;
   char *bb=(char *)&val;
   Serial.readBytes(bb,4); 
   return(val);
}

void write_serial_long(unsigned long val)
{
   byte* b = (byte *) &val;
   Serial.write(b,4);
}

void write_serial_float(float val)
{
   byte* b = (byte *) &val;
   Serial.write(b,4);
}

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

void write_i2c(float val)
{
  byte* b = (byte *)&val;
  Wire.write(b,4);
}

void communication_loop() {
   float w0;
   unsigned long n_params;
   int err;
   
   if (Serial.available() > 0) {
      byte new_mode=(byte)read_serial_long();
      write_serial_long(new_mode);

      if (new_mode == 255) { // triggered by MATLAB 'get_response'
        n_meas_total = read_serial_long();
        write_serial_long(n_meas_total);

        n_meas_before = read_serial_long();
        write_serial_long(n_meas_before);

        downsample = read_serial_long();
        write_serial_long(downsample);

        next_w = read_serial_float();
        write_serial_float(next_w);

        delay(400);//to be sure echo has been sent before changing baud rate
        // Serial.begin(TX_BAUD_RATE);
        cur_downsample = downsample; // TODO: remove downsampling
        cur_meas = n_meas_total;    //triggers a measure sequence

      } else {  // triggered by MATLAB 'set_mode_params': read w0 and params and call set_mode_params
      
        w0=read_serial_float();
        write_serial_float(w0);
  
        n_params=read_serial_long();
        write_serial_long(n_params);
        // read in and echo back all params
        for (int k=0; k<n_params; k++){
            if (k < BUF_SIZE) {
                buf[k] = read_serial_float();
                write_serial_float(buf[k]);
            }
        }
        if (n_params > BUF_SIZE){
          write_serial_long(1); // returns an error on serial bus
        } else {
          if (n_params) { // only if params were passed
            err = set_mode_params(new_mode, n_params, buf);
          } else {        // no parameters. Only set mode and w0, keeping the existing parameters
            err=0;
          }
          write_serial_long(err);
          mode = new_mode;
          w = w0; // define initial set point
        }
      }
   }
}

