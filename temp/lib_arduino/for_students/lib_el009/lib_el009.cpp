#include <Wire.h>//for I2C com
#include <TimerOne.h>
#include <lib_el009.h> //must also be declared in ino  
//--------communication protocol supporting functions--------
unsigned long TX_BAUD_RATE=1000000;
unsigned long RX_BAUD_RATE=9600;
int SlaveDeviceID = 1;// ID for I2C communication
//#define RX_BAUD_RATE 9600
#define PWM_PIN 9 //9 or 10 for timer1
float w=0,next_w=0,Tsample=0;
int cur_meas=0,n_meas_total=2,n_meas_before=1;//to avoid strange behavior before first get_response
int meas_just_finished=0; 
int downsample=1,cur_downsample=0;
byte mode=0;//controller mode (sent to my_callback)
#define BUF_SIZE 10
float buf[BUF_SIZE];//to store mode parameters
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void el009_setTxBaudRate(unsigned long i_TxBaudRate)
{
  TX_BAUD_RATE = i_TxBaudRate;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void el009_setRxBaudRate(unsigned long i_RxBaudRate)
{
  RX_BAUD_RATE = i_RxBaudRate;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void float_write_serial(float val)
{
   byte* b = (byte *) &val;
    Serial.write(b,4);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
float float_read_serial()
{ 
   float val=0;
   char *bb=(char *)&val;
   /*byte *bb=(byte *)&val;
   for (int i=0;i<4;i++) 
     {
       bb[i]=Serial.read();
     }
*/
   Serial.readBytes(bb,4); 
   return(val);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ulong_write_serial(unsigned long val)
{
   byte* b = (byte *) &val;
    Serial.write(b,4);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
unsigned long ulong_read_serial()
{ 
   unsigned long val=0;
   char *bb=(char *)&val;
   /*byte *bb=(byte *)&val;
   for (int i=0;i<4;i++) 
     {
       bb[i]=Serial.read();
     }
*/
   Serial.readBytes(bb,4);
   return(val);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void el009_callback()
{
   if(cur_meas==n_meas_total-n_meas_before) w=next_w;
  if(cur_meas>0)
  {
    cur_downsample--;
    if (cur_downsample==0)
    {
    cur_meas--;
    my_callback(w,1,mode);//write serial data
     if(cur_meas==0)  meas_just_finished=1; 
     cur_downsample=downsample;
    }
    
  }
  else
 {
   my_callback(w,0,mode);//don't write serial data
    if  (meas_just_finished) 
        {
          Serial.begin(RX_BAUD_RATE);
          meas_just_finished=0;
        }
  }
}

void receiveEvent(int howMany)
{//called when set point sent via I2C
 float w0=0;
 byte* couplebyte_ptr = (byte*) &w0  ;
if(howMany<4)
   return; 
  for (int i=0; i<4; i++)  // convert bytes to float
    {
      byte x;
      x=Wire.read();
      couplebyte_ptr[i] =x;
    }
    w=w0;
   // Serial.println(couple);
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void el009_setup(float Tsample)
{
  Serial.begin(RX_BAUD_RATE);
  Serial.setTimeout(-1);
  Timer1.initialize(Tsample/1E-6);         // initialize timer1,
  Timer1.pwm(PWM_PIN,0);                // setup pwm on pin PWM_PIN, 0% duty cycle
  Timer1.attachInterrupt(el009_callback);  // attaches el009_callback() as a timer overflow interrupt 
  //to allow set point command via I2C 
  Wire.begin(SlaveDeviceID);     // join i2c bus with address #1
  Wire.onReceive(receiveEvent); // register event
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Used by the ball-on-beam device
void el009_setup_no_timer()
{
  Serial.begin(RX_BAUD_RATE);
  Serial.setTimeout(-1);
  TX_BAUD_RATE=115200;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void el009_loop()
//to be called in loop to communicate with matlab get_response.m
{
  unsigned long n_param;
  float w0;
  int err;
 /*if response request on serial port:
        -read parameters:  n_meas_total (ulong32),n_meas_before (ulong32),downsample (ulong32) and next_w (float)
        -echo next_w (float)
  */
  
  /*read on serial port: mode paramaeters*/
  /*   mode=255 is reserved and triggers a response. Parameters are: n_meas_total (ulong32),n_meas_before (ulong32),downsample (ulong32) and next_w (float)*/ 
  /*   use unsigned long because unsigned int is coded on 2 bytes on uno and on 4 bytes on Due*/
  
 if (Serial.available() > 0) {
     byte new_mode=(byte)ulong_read_serial();
     ulong_write_serial(new_mode);
     if (new_mode==255)
       //read parameters and trigger a response 
       {
         n_meas_total=ulong_read_serial();
         ulong_write_serial(n_meas_total);
         n_meas_before=ulong_read_serial();
         ulong_write_serial(n_meas_before);
         downsample=ulong_read_serial();
         ulong_write_serial(downsample);
         next_w=float_read_serial();
         float_write_serial(next_w);
         delay(400);//to be sure echo has been sent before changing baud rate
         Serial.begin(TX_BAUD_RATE);
         cur_downsample=downsample;
         cur_meas=n_meas_total;//trigger a measure sequence
	}
	else //read w0 and parameters and call set_mode_param (to be written in .ino)
	{
	     w0=float_read_serial();
             float_write_serial(w0);
	     n_param=ulong_read_serial(); 
	     ulong_write_serial(n_param);
	     for (int k=0;k<n_param;k++)
	      {
               if (k<BUF_SIZE)
                {
	           buf[k]=float_read_serial();
                   float_write_serial(buf[k]);
                }
	      }  
             if (n_param>BUF_SIZE)
                ulong_write_serial(1);//return an error via serial com 
             else
             {
               if (n_param) //parameters were passed
	          err=set_mode_param(new_mode,n_param,buf);
               else //no paramters. Only set the mode and w0, keeping the existing parameters
                  err=0;
               ulong_write_serial(err);
               mode=new_mode;
	       w=w0;//define initial set point
             }   
	}
  }
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void set_mode(byte m)
//can be used in setup to set de default mode
{
   mode=m;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void set_w0(float w0)
//can be used in setup to initial value of w
{
   w=w0;
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//--------input output supporting functions--------------
float readAnalogMeasure(int analogPin,float K,int n_sample_mean)
//V is measured on analogPin
//n_sample_mean are measured and averaged
//returned measure=K*V 
{
  int V_Int;
  float  V,meas;
  const int n_sample=1;
  V=0;
  for (int i=0;i<n_sample;i++)
  {
    V_Int=analogRead(analogPin);
    V+=(float)V_Int*5/1023;
  }
  V=V/n_sample;
  meas=V*K;
  return(meas);
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void PWMAnalogWrite(float V,float V_DC_BUS)
{
   float duty_cycle = V/V_DC_BUS;
   if(V<0) duty_cycle = 0;
   if(V>V_DC_BUS) duty_cycle = 1;
   //analogWrite(PWM_PIN, duty_cycle*255);//this is not synchronised 
                              // with timer1 -> use setPwmDuty instead
   Timer1.setPwmDuty(PWM_PIN, duty_cycle*1024);//100%=1024 -- this is 
                           // scaled using pwmPeriod=
                           //counter counts up to pwmPeriod and down to zeros
                           //for 1kHz, pwmPeriod=16Mhz/2/1kHz=8000 
                           // (2:up and down, no clock prescale because 
                           //8000< RESOLUTION=65536 (16 bits counter)

}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void ladderAnalogWrite(float val)
//DAC using R-2R ladder module (used in magnetic levitation) 
{
  int pins[]={
    13,12,11,10,9,8,7,6  };//LSB first
  boolean b=0;
  int val_int=round(val/5*254);//1 byte, max 5V
  if (val_int>254) val_int=254;
  if (val_int<0) val_int=0;
  byte inB=(byte)val_int;
  for (int i=0;i<8;i++)
  {
    b=bitRead(inB,i);
    digitalWrite(pins[i], b);
  }
}
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
