/*
 * @file feedback
 * @brief Move the ball to the order value
 *  Receive the ball position from the Raspberry
 *  Thanks to the calculations, change the value of the motor position to move the ball
 * Used with the Raspberry programm : MoveBall.exe
 * @date august 2014
 * @version 1
 */ 
 
#include <Wire.h>
#include <Servo.h> 
#include <math.h> 
#include <lib_ballonbeam.h> 
#include <lib_el009.h>

#define SLAVE_ADDRESS 0x04


Servo myservo;  // create servo object to control a servo   
int PosMot; // variable to store the servo position
double e_n;
double alpha_n;
double b = 0;
double e_n1 = 0;
double alpha_n1 =0;
int PosBallOrder = 180;
unsigned long time;

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Obsolete
void bobSetup() {
 //pinMode(13, OUTPUT);
 myservo.attach(9);          // attaches the servo on pin 9 to the servo object 
 Wire.begin(SLAVE_ADDRESS);  // initialize i2c as slave
 //Wire.onReceive(bobReceiveData);// define callbacks for i2c communication
 Wire.onReceive(bobCallBack);   // define callbacks for i2c communication
 Wire.onRequest(bobSendData);   // define callbacks for i2c communication
 myservo.write(100);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void bobEl009Callback(int length) {
  el009_callback();
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void bobSetupEl009() {
 //pinMode(13, OUTPUT);
 //digitalWrite(13, LOW);

 myservo.attach(9);          // attaches the servo on pin 9 to the servo object 
 Wire.begin(SLAVE_ADDRESS);  // initialize i2c as slave
 Wire.onReceive(bobEl009Callback);// define callbacks for i2c communication
 Wire.onRequest(bobSendData);   // define callbacks for i2c communication
 myservo.write(0);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This function is obsolete.
void bobReceiveData(int byteCount){// callback for received data
  int PosBall = Wire.read(); //read the ball position
  e_n = (double)(PosBall - PosBallOrder);

  double u = 0;
  //double u =  controller would come here;

  PosMot = bobDegresToCommand(bobBeamToMotor(u));
  myservo.write(PosMot);
  alpha_n1 = alpha_n;
  e_n1 = e_n;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// This routine contains the calibration of the servo in angle
int bobDegresToCommand(double AngleDegres){
	int command;
	command = (int) ((AngleDegres - 5.9044)/1.1654);
	return command;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Contains the mechanical setup between the angle of the beam
// and the angle of the servo motor
double bobBeamToMotor( double AngleBeam){
	double AngleMotor;
	double D = 0.031;
	double L = 0.127;
	double H1 = 0.014;
	double H2 = 0.005;
	AngleBeam = AngleBeam *3.14159265/180.0;
	double cosa = L/D * AngleBeam+ (H1-H2)/D;
	if (cosa >  1) cosa = 1;
	if (cosa < -1) cosa = -1;
	AngleMotor = acos( cosa )* 180.0 / 3.14159265;
	return AngleMotor;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void bobSendData(){// callback for sending data
 int PosMotor;
 PosMotor = myservo.read();
 Wire.write(PosMotor);
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void bobSetServoPosition( float servoPosition ){
  myservo.write( servoPosition );
}
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
