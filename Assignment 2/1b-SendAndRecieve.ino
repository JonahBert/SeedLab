#include <Wire.h>
#define MY_ADDR 8
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
  Wire.onRequest(request);
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
  Serial.print("Recieved: ");
  Serial.println(instruction[0]);
  Serial.print("Reply: ");
  reply = int(instruction[0]) + 100;
  Serial.println(reply);
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

void request() {
  // According to the Wire source code, we must call write() within the requesting ISR
  // and nowhere else. Otherwise, the timing does not work out. See line 238:
  // https://github.com/arduino/ArduinoCore-avr/blob/master/libraries/Wire/src/Wire.cpp
  Wire.write(reply);
  Serial.println("Request Recieved");
  reply = 0;
}
