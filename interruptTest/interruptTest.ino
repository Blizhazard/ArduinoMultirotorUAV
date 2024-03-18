// Define pin numbers and variables
const int interruptPin = 2;  // Example interrupt pin (change according to your setup)

<<<<<<< HEAD
// Define debounce period in milliseconds
const unsigned long debouncePeriod = 50; // Adjust as needed

// Variables to store last trigger time and interrupt state
volatile unsigned long lastTriggerTime = 0;
int interruptState = 1800;

// Interrupt Service Routine (ISR) function
void handleInterrupt() {
  unsigned long currentTime = millis();
  // Check if it's been longer than the debounce period since the last trigger
  if (currentTime - lastTriggerTime >= debouncePeriod) {
    // Record the current time as the last trigger time
    lastTriggerTime = currentTime;

    // Check the current state of the interrupt pin
    bool currentState = digitalRead(interruptPin);

    // Only process if the state has changed
    if ((pulseIn(interruptPin, HIGH) - 1800) > 0) {
      interruptState = currentState;
      Serial.println("Interrupt!");
      if (pulseIn(interruptPin, HIGH)>1800) {
        Serial.println("HIGH");
      } else {
        Serial.println("LOW");
      }
    }
=======
// Interrupt Service Routine (ISR) function
void handleInterrupt() {
  if (digitalRead(interruptPin) == HIGH) {
    Serial.println("HIGH")
  } else {
    Serial.println("LOW")
>>>>>>> cf5a32ff51ecef69e1674fa778c44f97cd1bf349
  }
}

void setup() {
  // Set up interrupt pin
  pinMode(interruptPin, INPUT);

  // Attach interrupt to the pin and specify ISR function
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, CHANGE);

  // Other setup code
<<<<<<< HEAD
  Serial.begin(115200);
=======
  Serial.begin(9600);
>>>>>>> cf5a32ff51ecef69e1674fa778c44f97cd1bf349
}

void loop() {
  // Your main code here
<<<<<<< HEAD
  Serial.println("runtime");
  delay(10);
=======
>>>>>>> cf5a32ff51ecef69e1674fa778c44f97cd1bf349
}
