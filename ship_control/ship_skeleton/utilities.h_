#ifndef LIB_EL009_h
#define LIB_EL009_h
#include<Arduino.h>     //needed to use byte
void my_callback(float w, byte write_serial, byte mode);    //should be written in .ino
int set_mode_params(byte mode,int n_param,float *buf);       //should be written in .ino

void communication_loop();
void common_setup(float Tsample);
void common_callback();

void write_serial_float(float val);
void write_i2c(float val);
float read_i2c();

void empty_function();

#endif
