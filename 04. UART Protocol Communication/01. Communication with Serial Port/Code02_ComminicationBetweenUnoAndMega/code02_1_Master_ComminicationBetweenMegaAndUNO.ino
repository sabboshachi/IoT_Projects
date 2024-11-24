// Master code for Mega

void setup() {
  Serial.begin(9600);       // Initialize serial communication with PC
  Serial1.begin(9600);      // Initialize serial communication with the slave (hardware serial)
  Serial.println("Master ready. Send a number:");
}

void loop() {
  if (Serial.available() > 0) {
    int number = Serial.parseInt(); // Read the number from Serial Monitor

    Serial1.write(number); // Send the number to the slave
    delay(1000);            // Allow time for the slave to process

    if (Serial1.available() > 0) {
      int incrementedNumber = Serial1.read(); // Read the incremented number from the slave
      Serial.print("Incremented number from slave: ");
      Serial.println(incrementedNumber);
    }
  }
}
