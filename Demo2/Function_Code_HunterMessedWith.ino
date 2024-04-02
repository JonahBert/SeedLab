#define MY_ADDR 8
#define MARKER_FOUND 1
#define REQUEST_FOUND 0x01 
#define REQUEST_ANGLE 0x02
#define REQUEST_DISTANCE 0x03
#define PI_ADDR 8
#include <Wire.h>
#include <string.h>
float pi = 3.1416; // This is PI
float diameter = 0.15;
float radius = diameter/2;
double wheelbase = 0.3537; 
const int APIN1 = 2;  // A pin on motor 1
const int APIN2 = 3;  // A pin on motor 2
const int BPIN1 = 5;  // B pin on motor 1
const int BPIN2 = 6;  // B pin on motor 2



long motorCount[2] = {0,0}; //These are the encoder counts on the motors 1 & 2
unsigned int pwm_duty_cycle[2] = {0,0}; //Duty cycle for motor 1

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
double desired_degrees = 90;
double rho_desired = 0.3048 * desired_feet;
double phi_desired = desired_degrees * pi / 180;



//p controller values for speed
double Kp_phi_dot = 3; 
double Kp_rho_dot = 3;
//PI values for outer controller
double Kp_phi = 60;
double Kp_rho = 30;
double Ki_phi = 0.1;
double Ki_rho = 0.1;


//Set parameters
float battery_voltage = 8.2;
double average[2] = {0,0};
int total = 0;

//Motor velocity and position variables.
float pos_before_rad[2] = {0,0};
float pos_after_rad[2] = {0,0};
float delta_pos_rad[2] = {0,0};
float pos_velo_rad_s[2] = {0,0};//velocity array for motors 
float delta_velo_rad_s = 0;

float Voltage[2] = {battery_voltage * pwm_duty_cycle[0] / 255, battery_voltage * pwm_duty_cycle[1] / 255};



//Variables to keep track of the time elapsed and setup time
unsigned long desired_Ts_ms = 5; // desired sample time in milliseconds
unsigned long last_time_ms;
unsigned long start_time_ms;
float current_time = 0;

//comunication variables
volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
volatile uint8_t command = 0;
volatile uint8_t offset = 0;
float angle = 0;
float distance = 0;

//recieve flag, true if data has been recieved
bool recieved = false;

//state machine flags
bool check = true;

//define state class
enum class state {
    IDLE,
    SEARCH,
    CENTER,
    DRIVE,
    CIRCLE,
    STOP
};



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

void loop(){
    //initialize  state on first exexution, so declare as static
    static state machineState = state::IDLE;

    //velocitiesPositions();
    //PIControllerDistance();
    //pControllerVelocity();

    //state machine to control different parts of task
    switch (machineState){
        case state::IDLE:
            //check flag controls wether or not robot will stop halfway and recenter
            check = true;
            machineState = state::SEARCH;
            break;

        case state::SEARCH:
            //following requests data from pu
            Wire.beginTransmission(PI_ADDR);
            Wire.write(REQUEST_FOUND);
            Wire.endTransmission();
            //Pause idk how to do
            if (recieved == true){
                //output recieved message for debugging purposes
                printReceived();
                if (instruction[0] == MARKER_FOUND){
                    //marker found
                    machineState = state::CENTER;
                }
                else{
                    //turn 30 degrees
                }
                recieved = false;
            }
            break;

        case state::CENTER:
            //short pause to allow PI to get correct angle
            Wire.beginTransmission(PI_ADDR);
            Wire.write(REQUEST_ANGLE);
            Wire.endTransmission();
            //only continue if data was received, flag set in recieve ISR
            if (recieved == true){
                //output recieved message for debugging purposes
                printReceived();
                angle = data_to_float();
                //turn robot desired angle idk how to do
                machineState = state::DRIVE;
                recieved == false;
            }
            break;

        case state::DRIVE:
            Wire.beginTransmission(PI_ADDR);
            Wire.write(REQUEST_DISTANCE);
            Wire.endTransmission();
            if(recieved == true){
                //output recieved message for debugging purposes
                printReceived();
                distance = data_to_float();
                if(check == true){
                    //drive distance/2
                    machineState = state::CENTER;
                    check = false;
                }
                else{
                    //drive distance to marker - 1ft
                    machineState = state::STOP;
                }
                recieved = false;
            }
            break;

        case state::CIRCLE:
            //drive in circle, eventuallly this would circle until marker detected again i think for final demo
            machineState = state::IDLE;
            break;
        
        case state::STOP:
            //do nothing
            break;

    }
    //make sure msgLength set to 0
    msgLength = 0;
}


void velocitiesPositions(){
  int i;
  for(i=0;i<2;i++){
    //variabls for velocity and position of each motor and the timer.
    current_time = (float)(last_time_ms-start_time_ms)/1000; //gets the current time
    pos_before_rad[i] = 2*pi*(float)motorCount[i]/3200; //the postion before 10ms timer
    while (millis()<last_time_ms + desired_Ts_ms) {} 
    last_time_ms = millis();//10ms waiting period
    pos_after_rad[i] = 2*pi*(float)motorCount[i]/3200;
    delta_pos_rad[i] = pos_after_rad[i] - pos_before_rad[i];
    pos_velo_rad_s[i] = delta_pos_rad[i] / 0.005; //gets instantaneous velocity of position compared to 10ms before
    average[i] = average[i] + Voltage[i];
    } 

    //variables for actual speed and positions
    rho_dot = radius * (pos_velo_rad_s[0] + pos_velo_rad_s[1])/2;
    phi_dot = radius * (pos_velo_rad_s[0] - pos_velo_rad_s[1])/ wheelbase;
    rho_actual = radius * ( 2*pi*(float)motorCount[0]/3200 +  2*pi*(float)motorCount[1]/3200)/2;
    phi_actual = radius * (pos_after_rad[0] - pos_after_rad[1]) / wheelbase;
}



void pControllerVelocity(){
     //p inner controller
    rho_dot_error = rho_dot_desired - rho_dot;
    phi_dot_error = phi_dot_desired - phi_dot;
    delta_voltage = Kp_phi_dot * phi_dot_error;
    add_voltage = Kp_rho_dot * rho_dot_error;
    //Two outputs for controller to get output voltage.
    Voltage[1] = (add_voltage - delta_voltage)/2;
    Voltage[0] = (add_voltage + delta_voltage)/2;
    voltageOutput();
}

void PIControllerDistance(){
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

void voltageOutput(){
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

void receive() {
  // Set the offset, this will always be the first byte.
  offset = Wire.read();
  // If there is information after the offset, it is telling us more about the command.
  recieved = true;
  while (Wire.available()) {
    instruction[msgLength] = Wire.read();
    msgLength++;
  }
}

//converts string recieved from PI to float
float data_to_float(){
    String data = "";
    for(int i = 0; i < msgLength; i++){
        data += (instruction[i]);
    }
    return data.toFloat();
}

void request() {
  // According to the Wire source code, we must call write() within the requesting ISR
  // and nowhere else. Otherwise, the timing does not work out. See line 238:
  // https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/Wire.cpp
  Wire.write(0);
  Serial.println("Request Recieved");
}

//print recieved data for debugging purposes
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
      Serial.print(string(instruction[i])+"\t");
    }
    Serial.println("");
}