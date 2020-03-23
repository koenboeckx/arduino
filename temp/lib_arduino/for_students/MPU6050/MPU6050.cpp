// MPU-6050 Accelerometer + Gyro
// -----------------------------
//
// By arduino.cc user "Krodal".
//
// June 2012
//      first version
// July 2013 
//      The 'int' in the union for the x,y,z
//      changed into int16_t to be compatible
//      with Arduino Due.
//
// Open Source / Public Domain
//
// Using Arduino 1.0.1
// It will not work with an older version, 
// since Wire.endTransmission() uses a parameter 
// to hold or release the I2C bus.
//
// Documentation:
// - The InvenSense documents:
//   - "MPU-6000 and MPU-6050 Product Specification",
//     PS-MPU-6000A.pdf
//   - "MPU-6000 and MPU-6050 Register Map and Descriptions",
//     RM-MPU-6000A.pdf or RS-MPU-6000A.pdf
//   - "MPU-6000/MPU-6050 9-Axis Evaluation Board User Guide"
//     AN-MPU-6000EVB.pdf
// 
// The accuracy is 16-bits.
//
// Temperature sensor from -40 to +85 degrees Celsius
//   340 per degrees, -512 at 35 degrees.
//
// At power-up, all registers are zero, except these two:
//      Register 0x6B (PWR_MGMT_2) = 0x40  (I read zero).
//      Register 0x75 (WHO_AM_I)   = 0x68.
// 

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050.h"

// --------------------------------------------------------
// MPU6050_init
//
// This initializes the MPU6050 chip 
//

int MPU6050_read_accel(accel_t_gyro_union * p_accel_t_gyro )
{
  int error;
  
  // Read the raw values.
  // Read 14 bytes at once, 
  // containing acceleration, temperature and gyro.
  // With the default settings of the MPU-6050,
  // there is no filter enabled, and the values
  // are not very stable.
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) p_accel_t_gyro, 14);
  //Serial.print(F("Read accel, temp and gyro, error = "));
  //Serial.println(error,DEC);
  
    // Swap all high and low bytes.
  // After this, the registers values are swapped, 
  // so the structure name like x_accel_l does no 
  // longer contain the lower byte.
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap

  SWAP ((p_accel_t_gyro->reg).x_accel_h, (p_accel_t_gyro->reg).x_accel_l);
  SWAP (p_accel_t_gyro->reg.y_accel_h, (p_accel_t_gyro->reg).y_accel_l);
  SWAP (p_accel_t_gyro->reg.z_accel_h, (p_accel_t_gyro->reg).z_accel_l);
  SWAP (p_accel_t_gyro->reg.t_h, p_accel_t_gyro->reg.t_l);
  SWAP (p_accel_t_gyro->reg.x_gyro_h, p_accel_t_gyro->reg.x_gyro_l);
  SWAP (p_accel_t_gyro->reg.y_gyro_h, p_accel_t_gyro->reg.y_gyro_l);
  SWAP (p_accel_t_gyro->reg.z_gyro_h, p_accel_t_gyro->reg.z_gyro_l);

  return(error);
}

// --------------------------------------------------------
// MPU6050_convert_T
//
// This initializes the MPU6050 chip 
double MPU6050_convert_T( accel_t_gyro_union * p_accel_t_gyro )
{
   return ( (double) p_accel_t_gyro->value.temperature + 12412.0) / 340.0;
}

// --------------------------------------------------------
// MPU6050_init
//
// This initializes the MPU6050 chip 
//

int MPU6050_init()
{
   int error;
   uint8_t c;
   
  // default at power-up:
  //    Gyro at 250 degrees second
  //    Acceleration at 2g
  //    Clock source at internal 8MHz
  //    The device is in sleep mode.
  //
 
  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
  //Serial.print(F("WHO_AM_I : "));
  //Serial.print(c,HEX);
  //Serial.print(F(", error = "));
  //Serial.println(error,DEC);

  // According to the datasheet, the 'sleep' bit
  // should read a '1'.
  // That bit has to be cleared, since the sensor
  // is in sleep mode at power-up. 
  error = MPU6050_read (MPU6050_PWR_MGMT_1, &c, 1);
  //Serial.print(F("PWR_MGMT_1 : "));
  //Serial.print(c,HEX);
  //Serial.print(F(", error = "));
  //Serial.println(error,DEC);


  // Clear the 'sleep' bit to start the sensor.
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);

  return (0);  // return : no error
}

// --------------------------------------------------------
// MPU6050_read
//
// This is a common function to read multiple bytes 
// from an I2C device.
//
// It uses the boolean parameter for Wire.endTransMission()
// to be able to hold or release the I2C-bus. 
// This is implemented in Arduino 1.0.1.
//
// Only this function is used to read. 
// There is no function for a single byte.
//
int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);
  if (n != 1)
    return (-10);

  n = Wire.endTransmission(false);    // hold the I2C-bus
  if (n != 0)
    return (n);

  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);

  return (0);  // return : no error
}


// --------------------------------------------------------
// MPU6050_write
//
// This is a common function to write multiple bytes to an I2C device.
//
// If only a single register is written,
// use the function MPU_6050_write_reg().
//
// Parameters:
//   start : Start address, use a define for the register
//   pData : A pointer to the data to write.
//   size  : The number of bytes to write.
//
// If only a single register is written, a pointer
// to the data has to be used, and the size is
// a single byte:
//   int data = 0;        // the data to write
//   MPU6050_write (MPU6050_PWR_MGMT_1, &c, 1);
//
int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;

  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  n = Wire.write(start);        // write the start address
  if (n != 1)
    return (-20);

  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
    return (-21);

  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);

  return (0);         // return : no error
}

// --------------------------------------------------------
// MPU6050_write_reg
//
// An extra function to write a single register.
// It is just a wrapper around the MPU_6050_write()
// function, and it is only a convenient function
// to make it easier to write a single register.
//
int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;

  error = MPU6050_write(reg, &data, 1);

  return (error);
}
