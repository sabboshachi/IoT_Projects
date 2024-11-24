//  It receives a number via the serial interface, increments it by 1, and sends the incremented number back to the sender.

void setup() {
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
  while (!Serial) {
    // Wait for the serial port to be available
  }
  Serial.println("Send a number:");
}

void loop() {
  if (Serial.available() > 0) {
    // Read the incoming data as a string
    String receivedData = Serial.readStringUntil('\n');
    receivedData.trim(); // Remove any extra spaces or newline characters

    // Convert the received string to an integer
    int number = receivedData.toInt();

    // Increment the number
    int incrementedNumber = number + 1;

    // Send the incremented number back
    Serial.print("Incremented number: ");
    Serial.println(incrementedNumber);
  }
}

