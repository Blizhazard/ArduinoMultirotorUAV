volatile unsigned long lastTriggerTime = 0;
volatile double dTime;

void setup() {
  // put your setup code here, to run once:
  pinMode(2,INPUT);
  Serial.begin(115200);
  // Attach interrupt to the pin and specify ISR function
  attachInterrupt(digitalPinToInterrupt(2), handleInterrupt, CHANGE);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  //double input = pulseIn(2, HIGH);
  //double input = analogRead(2);
  
  //Serial.println(digitalRead(2));
  Serial.println(dTime);
}

// Interrupt Service Routine (ISR) function
void handleInterrupt() {
  
  unsigned long currentTime = millis();
  dTime = currentTime - lastTriggerTime;
  lastTriggerTime = currentTime;
}
