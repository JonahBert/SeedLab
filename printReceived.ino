// printReceived helps us see what data we are getting from the leader
void printReceived() {
  Serial.print("Recieved: ");
  Serial.println(instruction[0]);
  Serial.print("Reply: ");
  reply = int(instruction[0]) + 100;
  Serial.println(reply);
}