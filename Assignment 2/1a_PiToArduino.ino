// 1a_PiToArduino.ino: Arduino Side of Assignment 1a code
// Authors: Hunter Burnham, Joseph Kirby
// Resources: N/A
// Date Created: 1/25/2024
// Date Completed: 2/8/2024 
// Description: 

// include <Wire.h>
// define MY_ADDR 8
// Global variables to be used for I2C communication
volatile uint8_t offset = 0;

volatile uint8_t instruction[32] = {0};
volatile uint8_t msgLength = 0;
volatile uint8_t reply = 0;

void setup() {
  Serial.begin(115200);
  // We want to control the built-in LED (pin 13)
  pinMode(LED_BUILTIN, OUTPUT);
  // Initialize I2C
  Wire.begin(MY_ADDR);
  // Set callbacks for I2C interrupts
  Wire.onReceive(receive);
}

void loop() {
  // If there is data on the buffer, read it
  if (msgLength > 0) {
    if (offset==1) {
    digitalWrite(LED_BUILTIN,instruction[0]);
    }
  
  printReceived();
  msgLength = 0;
  }
}

// printReceived helps us see what data we are getting from the leader
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

// function called when an I2C interrupt event happens
void receive() {
  // Set the offset, this will always be the first byte.
  offset = Wire.read();
  // If there is information after the offset, it is telling us more about the command.
  while (Wire.available()) {
    instruction[msgLength] = Wire.read();
    msgLength++;
  }
}
