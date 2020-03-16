#ifndef LIB_EL009_h
#define LIB_EL009_h
#include<Arduino.h>//needed to use byte
// This function is called when matlab calls set_mode_param
// Set parameters sent by matlab if needed. Else just return 0
//   @param w The set point
//   @param write_serial If measurements should be written to serial port, write_serial=1, 0 otherwise
//   @param mode The mode of the controller (OPEN_LOOP, ...) to be sent by matlab using get_response
//   @return 0 if no error
void my_callback(float w,byte write_serial,byte mode);
int set_mode_param(byte mode,int n_param,float *buf);//should be written in .ino  
void el009_setup(float Tsample);
void el009_setup_no_timer();
void el009_setTxBaudRate(unsigned long i_TxBaudRate);
void el009_setRxBaudRate(unsigned long i_RxBaudRate);
void el009_callback();
void float_write_serial(float val);
void el009_loop();
void set_mode(byte m);
void set_w0(float w0);
float readAnalogMeasure(int analogPin,float K,int n_sample_mean);
void PWMAnalogWrite(float V,float V_DC_BUS);
void ladderAnalogWrite(float val);
#endif
