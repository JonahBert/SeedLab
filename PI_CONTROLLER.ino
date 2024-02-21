#include <Wire.h>
#define MY_ADDR 8
// Global variables to be used for I2C communication
volatile uint8_t offset = 0;

volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
volatile uint8_t reply = 0;

float pi = 3.1415; // This is PI

const int APIN1 = 2;  // A pin on motor 1
const int APIN2 = 3;  // A pin on motor 2
const int BPIN1 = 5;  // B pin on motor 1
const int BPIN2 = 6;  // B pin on motor 2
int desired_output = 0;

//The variables below should be set based on the battery voltage and desired postion.
float battery_voltage = 7.78; // Battery voltage
float desired_pos[2] = {0,0}; // desired motor position


//initialize 2 array variables for each motor,
//includes, position, velocity, desired position and velocity as well as variables for the 
//PI controller, including errors and proportional and integral. 
int motorCount[2] = {0,0}; //These are the encoder counts on the motors 1 & 2
unsigned int pwm_duty_cycle[2] = {0,0}; //Duty cycle for motor 1 & 2
float pos_velo_rad_s[2] = {0,0};//velocity array for motors 
float pos_error[2] = {0,0}; //error for position controller
float error[2] = {0,0}; //error for velocity controller
float integral_error[2] = {0,0};//integral error for position
float desired_velocity[2] = {0,0}; //desired velocity variable for both motors
float pos_before_rad[2] = {0,0};//Position before the sample interval
float pos_after_rad[2] = {0,0};//Position after sample interval
float delta_pos_rad[2] = {0,0};//Change in postion over sample interval
float Voltage[2] = {battery_voltage * pwm_duty_cycle[0] / 255, battery_voltage * pwm_duty_cycle[1] / 255}; // Voltage set by the PWM frequency

//PI controller values for position and P controller value for velocity controller
float Kp[2] = {3, 3.5}; //Proportional Values for Velocity Controller
float Kp_pos[2] = {30,30}; //Proportional Values for position Controller
float Ki_pos[2] = {1, 1}; //Integral Values for position Controller

//Variables to keep track of the time elapsed and setup time for mili() function.
unsigned long desired_Ts_ms = 10; // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time = 0;

//Motor interrupts to read of the encoder value.
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

  //Send and Receive Setup
  //We want to control the built-in LED (pin 13)
  pinMode(LED_BUILTIN, OUTPUT);
  //Initialize I2C
  Wire.begin(MY_ADDR);
  //Set callbacks for I2C interrupts
  Wire.onReceive(receive);
  Wire.onRequest(request);

}

void loop() {

  //If there is data on the buffer, read it
  if (msgLength > 0) {
    if (offset==1) {
    digitalWrite(LED_BUILTIN,instruction[0]);
    }  
    printReceived();
    msgLength = 0;
  }

  desired_output = instruction[0]; //Find desired output
//Case statement to parse through the value sent from the PI and update the desired postion according to that.
  switch (desired_output) {
    case 0:
      desired_pos[0] = 0;
      desired_pos[1] = 0;
      break;
    case 1:
      desired_pos[0] = 0;
      desired_pos[1] = pi;
      break;
    case 2:
      desired_pos[0] = pi;
      desired_pos[1] = 0;
      break;
    case 3:
      desired_pos[0] = pi;
      desired_pos[1] = pi;
      break;
  }
  int i;
  // for loop to loop through both motors
  for(i=0;i<2;i++){
    current_time = (float)(last_time_ms-start_time_ms)/1000; //gets the current time
    pos_before_rad[i] = 2*pi*(float)motorCount[i]/3200; //the postion before 10ms timer
    while (millis()<last_time_ms + desired_Ts_ms) {} 
    last_time_ms = millis();//10ms waiting period
    pos_after_rad[i] = 2*pi*(float)motorCount[i]/3200; //position after 10ms sample
    delta_pos_rad[i] = pos_after_rad[i] - pos_before_rad[i];
    pos_velo_rad_s[i] = delta_pos_rad[i] / 0.01; //gets instantaneous velocity of position compared to 10ms before

    //error for the feedback loop the desired - actual velocity.  
  //PI controller code using the P controller for velocity.
    pos_error[i] = desired_pos[i] - pos_after_rad[i]; //The error of the position
    integral_error[i] = integral_error[i] + pos_error[i] * ((float)desired_Ts_ms/1000); accumulation of position error
    desired_velocity[i] = Kp_pos[i] * pos_error[i] + Ki_pos[i] * integral_error[i]; // Desired velocity for P controller set by PI controller on position.
    error[i] = desired_velocity[i] - pos_velo_rad_s[i]; //error for P controller
    Voltage[i] = Kp[i] * error[i]; // Voltage set from error and P controller.

    //If the voltage is less than 0 then motor direction is changed. 
    if (Voltage[i] > 0) digitalWrite(7+i,HIGH);
    else digitalWrite(7+i,LOW);

    //Sets the PWM duty cycle based on the desired voltage above.
    pwm_duty_cycle[i] = 255 * abs(Voltage[i])/battery_voltage;
    //writes the PWM to the motors to get desired velocity.
    analogWrite(i+9,min(pwm_duty_cycle[i],255));
  }   
}
