// Define pin numbers and variables
const int interruptPin = 2;  // Example interrupt pin (change according to your setup)

// Interrupt Service Routine (ISR) function
void handleInterrupt() {
  if (digitalRead(interruptPin) == HIGH) {
    Serial.println("HIGH")
  } else {
    Serial.println("LOW")
  }
}

void setup() {
  // Set up interrupt pin
  pinMode(interruptPin, INPUT);

  // Attach interrupt to the pin and specify ISR function
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, CHANGE);

  // Other setup code
  Serial.begin(9600);
}

void loop() {
  // Your main code here
}
