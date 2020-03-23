#ifndef LIB_BALLONBEAM_H
#define LIB_BALLONBEAM_H

/*
 */ 
 
#include <Arduino.h>
#include <Wire.h>
#include <Servo.h> 
#include <math.h> 
#define SLAVE_ADDRESS 0x04

void bobSetup();
void bobSetupEl009();
void bobReceiveData(int byteCount);
void bobCallBack(int byteCount);
void el009_callback();
int bobDegresToCommand(double AngleDegres);
double bobBeamToMotor( double AngleBeam);

void bobSetServoPosition( float servoPosition );
void bobSendData();

#endif 
