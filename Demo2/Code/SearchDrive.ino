/*
Controls Coders: Caden Nubel & Joel Shorey & Hunter Burnham
Start Date: 3/25/24
Completion Date: 4/5/24

Objective: Search for the marker and then approach it getting within one foot.

Explaination: This code uses communication between a Pi and Arudino using i2c and a P-I-controller for position and P-controller for velocity from encoders. 
A state machine is used in order to move from state to state and it is moved forward either by the Pi detecting a marker or by finishing a case. 
The cases include searching, in which the robot turns in 30 degree intervals and pauses to allow detection by Pi, this is broken if the Pi detects an aruco marker.
Then the robot moves into the center case where the Pi sends the angle from center and we use the P-I-D-conroller to center the robot that number of degrees. 
The drive case uses the P-I-D-controller to make the robot drive a little less than 7 feet forward. The stop case does nothing.

Setup: Pins 2 & 3 should be going to the motor encoder A pins on motor 1 & 2. The arduino should power the enoders on the
motors using the 5V pin and GND and then the Motor PWM pins should be connected to pins 9 & 10 for the assciated motors 1 & 2.
Pins 7 & 8 should go to the direction pin of the motors to set the direction of the motor velocity.

For i2c: connect pins A6,A5, and GND to i2c connections on pi
*/
#define MY_ADDR 8
#define MARKER_FOUND 1
#define REQUEST_FOUND 1
#define REQUEST_ANGLE 2
#define REQUEST_DISTANCE 3
#define PI_ADDR 8
#define BUFFER_SIZE 4

#include <Wire.h>
#include <string.h>

float pi = 3.1416; //Necessary value for moving circularly
float diameter = 0.15; //Wheel diameter
float radius = diameter/2; //Wheel radius
double wheelbase = 0.3537; //Measured distacne between wheels
const int APIN1 = 2;  // A pin on motor 1
const int APIN2 = 3;  // A pin on motor 2
const int BPIN1 = 5;  // B pin on motor 1
const int BPIN2 = 6;  // B pin on motor 2

long motorCount[2] = {0,0}; //These are the encoder counts on the motors 1 & 2
unsigned int pwm_duty_cycle[2] = {0,0}; //Duty cycle for motor 1

//Anti-windup min/max values
double max_phi_dot = 0.8;
double max_rho_dot = 1;
double min_phi_dot = -0.8;
double min_rho_dot = -1;

//Values for the rho and phi values and voltages.
double phi_dot_desired;
double rho_dot_desired; 
double rho_error, phi_error;
double rho_dot_error, phi_dot_error;
double rho_dot, phi_dot;
double rho_actual, phi_actual;
double add_voltage, delta_voltage;
double phi_error_integral, rho_error_integral;

//Set parameters
double desired_feet = 0;
double desired_degrees = 0;
double rho_desired = 0.3048 * desired_feet;
double phi_desired = desired_degrees * pi / 180;
double circle_radius_feet = 2.1;
double desired_circle_distance_feet = 2 * pi * circle_radius_feet;
double circle_percentage;
double desired_orientation_degrees = 0;
double rho_time_stamp, start_degrees;
double phi_actual_after_turn;

//p controller gain values for speed
double Kp_phi_dot = 3; 
double Kp_rho_dot = 3;
//PI values for outer controller
double Kp_phi = 60;
double Kp_rho = 30;
double Ki_phi = 0.1;
double Ki_rho = 0.1;

//Set parameters
float battery_voltage = 8.2;
int total = 0;

//Motor velocity, position, and voltage variables.
float pos_before_rad[2] = {0,0};
float pos_after_rad[2] = {0,0};
float delta_pos_rad[2] = {0,0};
float pos_velo_rad_s[2] = {0,0};
float delta_velo_rad_s = 0;
float Voltage[2] = {battery_voltage * pwm_duty_cycle[0] / 255, battery_voltage * pwm_duty_cycle[1] / 255};

//Variables to keep track of the time elapsed and setup time
unsigned long desired_Ts_ms = 5; // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time = 0;
unsigned long desired_Ts_ms_2 = 1000; // desired sample time in milliseconds
unsigned long last_time_ms_2;

//comunication variables
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
volatile uint8_t command = 0;
volatile uint8_t offset = 0;
volatile uint8_t reply = 0;
uint8_t marker = 0;
float angle = 0;
float distance = 0;
float angle1;

//recieve flag, true if data has been recieved
bool recieved = false;
bool recieveEnabled = false;

//state machine flags
bool check = true;

//define state class
enum class state {
    IDLE,
    SEARCH,
    CENTER,
    DRIVE,
    STOP
};
state machineState = state::IDLE;

// Motor functions in order to get encoder value.
void motorOneInterrupt() {
  int thisA1 = digitalRead(APIN1);
  int thisB1 = digitalRead(BPIN1);
  if (thisA1 == thisB1) motorCount[0] -= 2;
  else motorCount[0] +=2;
}
void motorTwoInterrupt() {
  int thisA2 = digitalRead(APIN2);
  int thisB2 = digitalRead(BPIN2);
  if (thisA2 == thisB2) motorCount[1] += 2;
  else motorCount[1] -=2;
}

void setup() {
  //Initialize pins as outputs for motor driver and set encoder pins to pullup-resistor.
  Serial.begin(115200);
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(APIN1, INPUT_PULLUP);
  pinMode(BPIN1, INPUT_PULLUP);
  pinMode(APIN2, INPUT_PULLUP);
  pinMode(BPIN2, INPUT_PULLUP);
  digitalWrite(4, HIGH);//Enables Tri-State Buffer
  digitalWrite(7, HIGH);//Set Motor Direction
  digitalWrite(8, HIGH);//Set Motor Direction
  last_time_ms = millis(); // set up sample time variable
  start_time_ms = last_time_ms;
  //Enable the interrupts and attach them to correct functions.
  attachInterrupt(digitalPinToInterrupt(APIN1), motorOneInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(APIN2), motorTwoInterrupt, CHANGE);
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onReceive(receive);
  Wire.onRequest(request);
}

void loop() {
    static state machineState = state::IDLE;

    //state machine to control different parts of task
    switch (machineState) {
        case state::IDLE:
          //check flag controls wether or not robot will stop halfway and recenter
          Serial.print("Searching\n");
          machineState = state::SEARCH;
          break;

        case state::SEARCH:
          //turn in 30 degree increments until aruco marker is found
          recieveEnabled = true;
          velocities_positions();
          max_phi_dot = 1;
          phi_desired = desired_degrees * pi / 180;
          if (phi_desired - 0.1 < phi_actual && phi_actual < 0.1 + phi_desired) {
            //Prepare to search another 30 degree interval
            desired_degrees = desired_degrees + 30;
            phi_desired = desired_degrees * pi / 180;
            //Turn motors off between searches
            analogWrite(9,0);
            analogWrite(10,0);
            //Pause between searches
            while (millis() < last_time_ms_2 + desired_Ts_ms_2) {} 
            last_time_ms_2 = millis();
          }
          //Call controller functions for motors
          p_i_controller_distance();
          p_controller_velocity();
          
          //check recieve flag
          if (recieved == true) {
            //Turn off motors for state switch
            analogWrite(9,0);
            analogWrite(10,0);
            //marker found
            if (marker == 1) {
              machineState = state::CENTER;
              angle1 = angle; //set angle to adjust for center state
              //check if correction is needed, it is not needed for small angles off
              if(angle1 > 1.8 || angle1 < -1.8) {
                desired_degrees = angle1 + desired_degrees;
                marker = 0;
                recieveEnabled = false;
              }
              //subtract the 30 degrees that were added, because the marker was found
              //update desired angle for controller functions
              desired_degrees = desired_degrees - 30;
              phi_desired = desired_degrees * pi / 180;
            }
            recieved = false;
            msgLength = 0;
          }
          break;

        case state::CENTER:
          recieveEnabled = true;
          //update gains and anti-windup for the center case to prevent overshoot
          Ki_phi = 0.1;
          Kp_phi = 60;
          max_phi_dot = 0.1;
          min_phi_dot = -0.1;
          //only continue if data was received, flag set in recieve ISR
          recieveEnabled = false;
          //turn angle to center robot
          velocities_positions();
          p_i_controller_distance();
          p_controller_velocity();
          msgLength = 0;
          recieved = false;
          //state switch condition is if the angle is within 0.01 radians of desired angle
          if (phi_desired - 0.01 < phi_actual && phi_actual < 0.01 + phi_desired) {
              machineState = state::DRIVE;
              //set robot to move 6.5 feet forward towards the marker
              rho_desired = rho_actual + (6.5 * 0.3048);
              //reset gains and anti-windup 
              max_phi_dot = 0.8;
              max_rho_dot = 1;
              min_phi_dot = -0.8;
              min_rho_dot = -1;
              Ki_phi = 0.1;
          }

          break;

        case state::DRIVE:
          //update gains and anti-windup for drive state to move very quickly toward the marker
          max_rho_dot = 2.5;
          min_rho_dot = -2.5;
          max_phi_dot = 0.8;
          min_phi_dot = -0.8;
          recieveEnabled = true;
          //call controller functions to drive 6.5 feet
          velocities_positions();
          p_i_controller_distance();
          p_controller_velocity();
          //set flags for I2C communication
          recieveEnabled = false;
          recieved = false;
          msgLength = 0;
          //stopping condition is wif the robot is within 0.05 meters of the set desired position
          if (rho_desired - 0.05 < rho_actual && rho_actual < 0.05 + rho_desired) {
            machineState = state::STOP;
            //turn motors off for stop state
            analogWrite(9,0);
            analogWrite(10,0);
          }
          break;

        case state::STOP:
          //make sure the motors are turned off in the stop state
          analogWrite(9,0);
          analogWrite(10,0);
          //don't leave the stop state
          machineState = state::STOP;
          break;

    }
}

//This function takes measurements of the robot's physical characteristics using motor encoders and interrupts.
//The function loops twice through to measure the position/velocity of both motors
void velocities_positions() {
  int i;
  for(i=0;i<2;i++){
    //variables for velocity and position of each motor and the timer.
    current_time = (float)(last_time_ms-start_time_ms)/1000; //gets the current time
    pos_before_rad[i] = 2 * pi * (float)motorCount[i]/3200; //the postion before 10ms timer
    while (millis() < last_time_ms + desired_Ts_ms) {} 
    last_time_ms = millis(); //10ms waiting period
    pos_after_rad[i] = 2 * pi * (float)motorCount[i]/3200;
    delta_pos_rad[i] = pos_after_rad[i] - pos_before_rad[i];
    pos_velo_rad_s[i] = delta_pos_rad[i] / 0.005; //gets instantaneous velocity of position compared to 10ms before
    //average[i] = average[i] + Voltage[i];
    } 

    //variables for actual speed and positions
    rho_dot = radius * (pos_velo_rad_s[0] + pos_velo_rad_s[1])/2;
    phi_dot = radius * (pos_velo_rad_s[0] - pos_velo_rad_s[1])/ wheelbase;
    rho_actual = radius * ( 2*pi*(float)motorCount[0]/3200 +  2*pi*(float)motorCount[1]/3200)/2;
    phi_actual = radius * (pos_after_rad[0] - pos_after_rad[1]) / wheelbase;
}


//This function implements proportional control of the velocity of both wheels
void p_controller_velocity(){
     //p inner controller
    rho_dot_error = rho_dot_desired - rho_dot;
    phi_dot_error = phi_dot_desired - phi_dot;
    delta_voltage = Kp_phi_dot * phi_dot_error;
    add_voltage = Kp_rho_dot * rho_dot_error;
    //Two outputs for controller to get output voltage.
    Voltage[1] = (add_voltage - delta_voltage)/2;
    Voltage[0] = (add_voltage + delta_voltage)/2;
    voltage_output();
}

//This function implements proportional and integral control of the angle and forward velocity of the robot as a whole.
void p_i_controller_distance() {
  //measure errors between the desired and actual quantities measured
    rho_error = rho_desired - rho_actual; 
    phi_error = phi_desired - phi_actual;
    rho_error_integral = rho_error_integral + rho_error * ((double)desired_Ts_ms /1000);
    phi_error_integral = phi_error_integral + phi_error * ((double)desired_Ts_ms /1000);
    
    //set new desired values for speed from PI controller of position
    if(rho_dot_desired > 0){
      if(rho_dot_desired > max_rho_dot) rho_dot_desired = max_rho_dot;
      else{rho_dot_desired = Kp_rho * rho_error + Ki_rho * rho_error_integral;}
    }
    else{
      if(rho_dot_desired < min_rho_dot) rho_dot_desired = min_rho_dot;
      else{rho_dot_desired = Kp_rho * rho_error + Ki_rho * rho_error_integral;}
    }
    
    //anti-windup on phi in both negative and positive directions
    if(phi_dot_desired > 0){
      if(phi_dot_desired > max_phi_dot) phi_dot_desired = max_phi_dot;
      else{phi_dot_desired = Kp_phi * phi_error + Ki_phi * phi_error_integral;}
    }
    else{
      if(phi_dot_desired < min_phi_dot) phi_dot_desired = min_phi_dot;
      else{phi_dot_desired = Kp_phi * phi_error + Ki_phi * phi_error_integral;}
    }
}

//This function outputs voltages to each motor after they are set/updated in the controller functions.
void voltage_output() {
  int i;
  //Select direction for the Voltages
  for(i=0;i<2;i++){
    if (Voltage[i] > 0) digitalWrite(7+i,HIGH);
    else digitalWrite(7+i,LOW);
    //Turn the voltage into a PWM
    pwm_duty_cycle[i] = 255 * abs(Voltage[i])/battery_voltage;
    analogWrite(i+9,min(pwm_duty_cycle[i],255));
  }
}



void receive(){
  if (recieveEnabled) {
    // Set the offset, this will always be the first byte.
    offset = Wire.read();
    // If there is information after the offset, it is telling us more about the command.
    while (Wire.available()) {
      instruction[msgLength] = Wire.read();
      msgLength++;
    }
    marker = instruction[0];
    byte angleBuffer[BUFFER_SIZE] = {0};
    byte distanceBuffer[BUFFER_SIZE] = {0};
    for(int i = 0; i < 4; i++){
      angleBuffer[i] = instruction[4-i];
    }
    for(int i = 0; i < 4; i++){
      distanceBuffer[i] = instruction[8-i];
    }
    memcpy(&distance, distanceBuffer, sizeof(float));
    memcpy(&angle, angleBuffer, sizeof(float));
    recieved = true;
  }
}


void request() {
  // According to the Wire source code, we must call write() within the requesting ISR
  // and nowhere else. Otherwise, the timing does not work out. See line 238:
  // https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/Wire.cpp
  Wire.write(reply);
  //Serial.println("Request Recieved");
}

//print recieved data for debugging purposes only
void printReceived() {
  // Print on serial console
  Serial.print("Offset received: ");
  Serial.println(offset);
  Serial.print("Message Length: ");
  Serial.println(msgLength);
  Serial.print("Message: ");
  for(int i = 0; i < msgLength; i++){
    Serial.print(char(instruction[i]));
  }
  Serial.println();
  Serial.print("Instruction received: ");
  for (int i=0;i<msgLength;i++) {
      Serial.print(String(instruction[i])+"\t");
    }
    Serial.println("");
}
