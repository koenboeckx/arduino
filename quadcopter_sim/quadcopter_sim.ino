/* koen - 28/03/19
 *  Simulates the behavior of a quadcopter (bicoptre). Predicts this behavior based on the 
 *  Runge-Kutta 4 prediction of the next value of the state variables theta, omega
 *  
 *  Resets the values every 20 seconds
 
 */

#include <TimerOne.h> // calculation of new values for state vars is triggered by timing event 
#include <math.h>
#include <Wire.h>     // communication via I2C protocol

#define BAUDRATE 115200
#define SAMPLING_PERIOD 10000 // 10000 us = 100 Hz
#define I2CADDR 1

// System Parameters
#define M_BEAM 10.0          // mass of beam
#define L_BEAM 1.2          // length of beam
#define KL     0.017          // coeff of motor left
#define KR     0.017          // coeff of motor right

#define DAMPING_COEFF 0.2  // incorporates energy loss after bounce (both cart and pendulum)
#define NOISE_VAR 0.002      // intensity of noise added to measurement

#define V_MIN  0.0
#define V_MAX 20.0

// State (and other) Variables
const float h = SAMPLING_PERIOD/1e6;    // time step
float theta, omega;

float ul = 10.0; // is set with function set_u()
float ur = 10.0;
float Troll = 0.0; // disturbance torque

int counter = 0; // counts the number of loops and resets state vars of 100 loops

void setup() {
  Timer1.initialize(SAMPLING_PERIOD);
  Timer1.attachInterrupt(one_step);
  Serial.begin(BAUDRATE);
  
  Wire.begin(I2CADDR);            // join i2c bus with address I2CADDR
  Wire.onRequest(send_measurement); // register event triggered when request is received
  Wire.onReceive(set_u);            // register event triggered when data is received
  

  randomSeed(analogRead(0)); // sets seed for RNG from noise on analog input
  
  reset();
}

void reset() { // resets state variables back to their original position
  theta = 0;
  omega = 0;//.1*random(-10, 10);
}

void loop() {

  Serial.print(theta); Serial.print(" ");
  Serial.print(omega); Serial.print(" ");
  Serial.print(ul);    Serial.print(" ");
  Serial.print(ur);    Serial.print(" ");
  Serial.println(Troll);   
  delay(50);

  counter++;

  if (counter == 1000) {
    counter = 0;
    //reset();
  }
}


//---------------------------- Communication function -----------------------------------------------------------------------------

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void send_measurement() {
  write_i2c( theta + NOISE_VAR*random(-1, 1));
}

void set_u(int n_bytes){
  read_i2c(12, &ul, &ur, &Troll);
  ul = constrain(ul, V_MIN, V_MAX);
  ur = constrain(ur, V_MIN, V_MAX);
}

float read_i2c(int n_bytes, float* ul, float* ur, float* T)
{ // reads in control "voltages" ul (left) and ur (right) and disturbance torque T
  char *bbl=(char *)ul; // byte array that shares mem location with float
  char *bbr=(char *)ur;
  char *bbT=(char *)T;

  int i = 0;
  
  for(int i=0; i<4; i++) {
    bbl[i] = Wire.read(); // receive a byte as character
  }
  

  for(int i=0; i<4; i++) {
    bbr[i] = Wire.read(); // receive a byte as character
  }
 

  for(int i=0; i<4; i++) {
    bbT[i] = Wire.read(); // receive a byte as character
  }
  
}

void write_i2c(float val)
{
  byte* b = (byte *)&val;
  Wire.write(b,4);
}

//---------------------------- Mathematical functions ------------------------------------------------------------------------------

float deriv_theta(float theta, float omega, float ul, float ur) { // compute first derivative of theta = omega
  return omega;
}

float deriv_omega(float theta, float omega, float ul, float ur) { // compute first derivative of omega
  float omega_dot = -6. / (M_BEAM*L_BEAM) * ((KL * sq(ul)) - (KR * sq(ur))) + Troll;
  return omega_dot;
}

void one_step() { // uses RK4 to compute new values of the state variables
  float theta_k1, omega_k1;
  float theta_k2, omega_k2;
  float theta_k3, omega_k3;
  float theta_k4, omega_k4;

  // compute k1
  theta_k1 = h * deriv_theta(theta, omega, ul, ur);
  omega_k1 = h * deriv_omega(theta, omega, ul, ur);

  // compute k2
  theta_k2 = h * deriv_theta(theta+theta_k1/2, omega+omega_k1/2, ul, ur);
  omega_k2 = h * deriv_omega(theta+theta_k1/2, omega+omega_k1/2, ul, ur);
  
  // compute k3
  theta_k3 = h * deriv_theta(theta+theta_k2/2, omega+omega_k2/2, ul, ur);
  omega_k3 = h * deriv_omega(theta+theta_k2/2, omega+omega_k2/2, ul, ur);

  // compute k4
  theta_k4 = h * deriv_theta(theta+theta_k3, omega+omega_k3, ul, ur);
  omega_k4 = h * deriv_omega(theta+theta_k3, omega+omega_k3, ul, ur);

  // update state variables
  theta = theta + 1.0/6 * (theta_k1 + 2*theta_k2 + 2*theta_k3 + theta_k4);
  omega = omega + 1.0/6 * (omega_k1 + 2*omega_k2 + 2*omega_k3 + omega_k4);
  
  // impose physical constraints
  // 1. if theta < -pi/6 or > pi/6 => the beam bounces back
  if (theta < -PI/6) {
     theta =  -PI/6;
     omega =  -omega * DAMPING_COEFF;
  }
  if (theta > PI/6){
     theta =  PI/6;
     omega = -omega * DAMPING_COEFF;
  }
  
}
