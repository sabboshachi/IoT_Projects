// Slave code for Uno

void setup() {
  Serial.begin(9600); // Initialize serial communication with the master
}

void loop() {
  if (Serial.available() > 0) {
    int receivedNumber = Serial.read(); // Read the number sent by the master
    int incrementedNumber = receivedNumber + 1; // Increment the number
    delay(1000); // Small delay to ensure master is ready
    Serial.write(incrementedNumber); // Send the incremented number back
  }
}
