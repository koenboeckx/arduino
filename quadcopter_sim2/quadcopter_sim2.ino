/* koen - 06/04/19
 *  Generalization of the original quadcopter simulator
 */

#include <stdio.h>
#include <TimerOne.h> // calculation of new values for state vars is triggered by timing event 
#include <math.h>
#include <Wire.h>     // communication via I2C protocol

#define BAUDRATE 115200
#define SAMPLING_PERIOD 10000 // 5000 us = 200 Hz
#define I2CADDR 1

#define N_STATEVARS 2    /* number of state variables */
#define N_INPUTS    3    /* number of input variables */

/* System specific parameters */
#define M_BEAM 15.0             /* mass of beam         */
#define L_BEAM 1.2              /* length of beam       */
#define KL     0.017            /* coeff of motor left  */
#define KR     0.017            /* coeff of motor right */
#define V_MIN  0.0              /* minimum voltage that can be applied */
#define V_MAX  20.0             /* maximum voltage that can be applied */

#define NOISE_VAR 0.01

/* communication modes */
#define COMMAND       1    /* set the controllable input of the system */
#define DISTURBANCE   2    /* apply disturbance to system */
#define STATE         3    /* set the initial value of the system */

float state_vars[N_STATEVARS];       /* theta, omega */  
float state_vars_dot[N_STATEVARS];
float inputs[N_INPUTS];

const float h = SAMPLING_PERIOD/1e6;  /* time step for runge-kutta */

/*define variables used for representation, not computation */
float theta, omega;

float ul      = 0.0; // is set with set_command() 
float ur      = 0.0;
float Troll   = 0.0; // disturbance torque

void setup() {
  Timer1.initialize(SAMPLING_PERIOD);
  Timer1.attachInterrupt(next_step);
  
  Serial.begin(BAUDRATE);

  Wire.begin(I2CADDR);              // join i2c bus with address I2CADDR
  Wire.onRequest(send_measurement); // register event triggered when request is received
  Wire.onReceive(read_i2c);            // register event triggered when data is received

  randomSeed(analogRead(0)); // sets seed for RNG from noise on analog input
  
  init(state_vars);
}

void loop() {

  //Serial.print(counter); Serial.print(" ");
  Serial.print(theta); Serial.print(" ");
  Serial.print(omega); Serial.print(" ");
  Serial.print(ul);    Serial.print(" ");
  Serial.print(ur);    Serial.print(" ");
  Serial.println(Troll);   
  delay(50);
}

//---------------------------- Communication functions---------------------------------------------------------------------------

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void send_measurement() {
  write_i2c( theta + NOISE_VAR*random(-1, 1));
}



void read_i2c(int n_bytes)
{
  char first_byte = Wire.read(); // read first byte on Wire
  switch (first_byte) {
    case COMMAND:
      set_command(&inputs[0], &inputs[1]);
      break;
   case DISTURBANCE:
      set_disturbance(&inputs[2]);
      break;
   case STATE:
      set_state(state_vars);
      break;
  }
}

void set_state(float *state_vars)
{
  float temp;
  for (int f=0; f<N_STATEVARS; f++){
    byte *bb = (byte *)&temp;
    for(int i=0; i<4; i++) {
      bb[i] = Wire.read(); // receive a byte as character
    }
    state_vars[f] = temp;
  }
}

void set_disturbance(float *T)
{
  char *bb_T = (char *)T;
  // read in T
  for(int i=0; i<4; i++) {
    bb_T[i] = Wire.read(); // receive a byte as character
  }
}

void set_command(float *ul, float *ur)
{
  char *bb_ul = (char *)ul;
  char *bb_ur = (char *)ur;
  // read in ul
  for(int i=0; i<4; i++) {
    bb_ul[i] = Wire.read(); // receive a byte as character
  }
  *ul = constrain(*ul, V_MIN, V_MAX);

  // read in ur
  for(int i=0; i<4; i++) {
    bb_ur[i] = Wire.read(); // receive a byte as character
  }
  *ur = constrain(*ur, V_MIN, V_MAX);
  
}

void write_i2c(float val)
{
  byte* b = (byte *)&val;
  Wire.write(b,4);
}

//---------------------------- Mathematical functions ---------------------------------------------------------------------------

/* function called on timer interrupt */
void next_step()
{
  one_step(state_vars, inputs);
  constraints(state_vars);
  
  theta = state_vars[0];
  omega = state_vars[1];
  ul    = inputs[0];
  ur    = inputs[1];
  Troll = inputs[2];
}

/* init: initializes the state vars */
void init(float *state_vars)
{
    state_vars[0] = -0.4;
    state_vars[1] = 0.1;

}
/* one_step: performs one step of the Runge-Kutta4 algorithm */
void one_step(float *state_vars, float *inputs)
{
    size_t N = sizeof(state_vars);
    float k1[N], k2[N], k3[N], k4[N];
    float temp[N];
    
    // step 1 
    deriv(state_vars, k1, inputs);
    multiply(k1, h);
    
    // step 2 
    for (int i=0; i<N; i++) {
        temp[i] = state_vars[i] + k1[i]/2.0;
    }
    deriv(temp, k2, inputs);
    multiply(k2, h);
    
    
    // step 3 
    for (int i=0; i<N; i++) {
        temp[i] = state_vars[i] + k2[i]/2.0;
    }
    deriv(temp, k3, inputs);
    multiply(k3, h);
    
    // step 4 
    for (int i=0; i<N; i++) {
        temp[i] = state_vars[i] + k3[i];
    }
    deriv(temp, k4, inputs);
    multiply(k4, h);
    
    // update state_vars 
    for (int i=0; i<N; i++) {
        state_vars[i] = state_vars[i] + 1.0/6 * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i]);
    }
}

/* multiply: multiply array inline with float */
void multiply(float* arr, float h)
{
  for (int i=0; i<sizeof(arr); i++) {arr[i] = h*arr[i]; }
}


/* deriv: computes first-order derivative for all state variables */
void deriv(float *state_vars, float *state_vars_dot, float *inputs)
{   
    float ul    = inputs[0];
    float ur    = inputs[1];
    float Troll = inputs[2];
    
    state_vars_dot[0]  = state_vars[1];                                             // theta_dot = omega
    state_vars_dot[1] =  -6./(M_BEAM*L_BEAM) * ((KL*ul*ul) - (KR*ur*ur)) + Troll;   // omega_dot 
    
}

/* constraints: impose constraints on state variables */
void constraints(float *state_vars)
{
    /* 1. if theta < -PI/6 or theta > PI/6 => beam bounces back */
    if (state_vars[0] < -PI/6.0) {
        state_vars[0] = -PI/6.0;
        state_vars[1] = 0.0;
    }
    if (state_vars[0] >  PI/6.0) {
        state_vars[0] =  PI/6.0;
        state_vars[1] = 0.0;
    }       
}

 
