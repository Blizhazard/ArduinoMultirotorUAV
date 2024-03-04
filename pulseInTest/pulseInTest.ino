void setup() {
  // put your setup code here, to run once:
  pinMode(6,INPUT);
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  double input = pulseIn(6, HIGH);
  //double input = analogRead(6);
  Serial.println(input);
}
