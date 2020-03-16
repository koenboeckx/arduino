//#include <Servo.h>  // include the Servo library
#include <Wire.h>
#include <TimerOne.h> // test compatibility with motor and mpu
#include"ServoTimer2.h"

#define T_SAMPLE 10000
#define MPU_ADDR 0x68 // address of MPU6050
#define POTPIN    1
#define MOTORPIN  6
#define PWM_PIN   10

#define USE_FILTER 1
#define ALPHA 0.9

ServoTimer2 Motor;
const int MPU_addr=0x68;  // I2C address of the MPU-6050
float ix = 0;
float AcX,AcY,AcZ;
float roll_angle, arm_angle, phi;
float u=75.0;


float w = 0.;
float k = 2.;

void setup() {
  Motor.attach(MOTORPIN);  // attaches the servo on pin 9 to a servo object

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  Serial.begin(115200);

  Timer1.initialize(T_SAMPLE);
  Timer1.pwm(PWM_PIN,0);                // setup pwm on pin PWM_PIN, 0% duty cycle
  Timer1.attachInterrupt(callback);
}

void callback()
{
  u = k*(w - roll_angle);
  phi = asin(u)/PI*180;
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

float compute_pwm(float theta)
{
  /* transform input angle (in degrees) to PWM length */
  theta = constrain(theta, 0, 180);
  return map(theta, 0, 180, 800, 2300);
}

// not include in skeleton for students:
float measure_roll()
{
  float angle;
  float AcX, AcY, AcZ;
  measure_acc(&AcX, &AcY, &AcZ);
  angle = (float)atan(AcX/AcZ);
  return angle;
}

float measure_arm()
{
  float val = analogRead(POTPIN);
  return (val - 100)/435*180;
}

void loop() {
  Motor.write(compute_pwm(phi + 90));
  roll_angle = measure_roll();
  measure_acc(&AcX, &AcY, &AcZ);
  arm_angle = measure_arm();
  
  Serial.print(roll_angle); Serial.print(" ");
  Serial.print(u); Serial.print(" ");
  Serial.print(phi);
  Serial.println();
  


  //delay(T_SAMPLE);
}
