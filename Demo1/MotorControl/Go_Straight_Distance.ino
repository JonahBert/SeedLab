/*
Caden Nubel & Joel Shorey
Start Date:2/19/24
Completion Date:3/8/24
Demo 1 Prompt: Move forward in a straight line and stop after a specified distance in feet (between 1 and 10 feet).
Setup: Pins 2 & 3 should be going to the motor encoder A pins on motor 1 & 2. The arduino should power the enoders on the
motors using the 5V pin and GND and then the Motor PWM pins should be connected to pins 9 & 10 for the assciated motors 1 & 2.
Pins 7 & 8 should go to the direction pin of the motors to set the direction of the motor velocity.
Description of Code: This code utulizes a two input two output system which has the input of the desired forward and rotational velocities, 
these are then run to a P controller which gives the two motors a voltage that will move the motors at the desired speeds. Then in order to controll the 
distance we integrate this and use a PI controller on the desired forward position and rotational position. This then gives a specified desired velocity to the 
P controller. 

*/
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

double max_phi_dot = 1;
double max_rho_dot = 1;

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
double desired_feet = 10;
double desired_degrees = 0;
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
}

void loop() {
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



  
    //outer loop PI controller
    rho_error = rho_desired - rho_actual;
    phi_error = phi_desired - phi_actual;
    rho_error_integral = rho_error_integral + rho_error * ((double)desired_Ts_ms /1000);
    phi_error_integral = phi_error_integral + phi_error * ((double)desired_Ts_ms /1000);
    
    //set new desired values for speed from PI controller of position
    if(rho_dot_desired > max_rho_dot) rho_dot_desired = max_rho_dot;
    else{rho_dot_desired = Kp_rho * rho_error + Ki_rho * rho_error_integral;}
    //Anti Windup so that it does not overshoot.
    if(phi_dot_desired > max_phi_dot) phi_dot_desired = max_phi_dot;
    else{phi_dot_desired = Kp_phi * phi_error + Ki_phi * phi_error_integral;}
    
   
   //p inner controller
    rho_dot_error = rho_dot_desired - rho_dot;
    phi_dot_error = phi_dot_desired - phi_dot;
    delta_voltage = Kp_phi_dot * phi_dot_error;
    add_voltage = Kp_rho_dot * rho_dot_error;


   //Print statements to find error
    Serial.print(rho_actual);
    Serial.print(" \t ");
    Serial.print(phi_actual);
    Serial.print(" \t ");
    Serial.println(phi_error);
   

  //Two outputs for controller to get output voltage.
    Voltage[1] = (add_voltage - delta_voltage)/2;
    Voltage[0] = (add_voltage + delta_voltage)/2;
  //Select direction for the Voltages
    for(i=0;i<2;i++){
    if (Voltage[i] > 0) digitalWrite(7+i,HIGH);
    else digitalWrite(7+i,LOW);
  //Turn the voltage into a PWM
    pwm_duty_cycle[i] = 255 * abs(Voltage[i])/battery_voltage;
    analogWrite(i+9,min(pwm_duty_cycle[i],255));
    }
}

