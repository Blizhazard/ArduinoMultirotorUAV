// Arduino code to communicate with SPRacingF3 flight controller using MSP protocol

#define MSP_HEADER '$'
#define MSP_IDENT 100
#define MSP_ATTITUDE 108

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);  // Make sure this baud rate matches the one set in the flight controller
}

void loop() {

  delay(10);

  //Serial.println("Start Loop");

  uint8_t datad = 0;
  uint8_t *data = &datad;

  sendMSP(MSP_ATTITUDE, data, 0);



  // Request attitude data from the flight controller
  //sendMSPRequest(MSP_ATTITUDE);

  // Wait for response
  //delay(10);  // Adjust delay as necessary
  byte count = 0;

  int16_t roll;
  int16_t pitch;
  int16_t yaw;

  int8_t buf[12];

  // Read response from flight controller
  while (Serial.available()) {
    byte c = Serial.read();
    if (count < 12) {
      buf[count] = c;
    }
    count += 1;
  }

  Serial.println("Bytes read: " + String(count));

  if (count<12) {
    Serial.println("missing bytes");
    return;
  } else if (buf[0] != '$' || buf[1] != 'M' || buf[2] != '>') {
    char *chars = (char *) buf;
    Serial.println("header mismatch- expected: '$M>', got: '" + String(chars[0])+String(chars[1])+String(chars[2]) + "'");
    return;
  } else {
    int8_t chsum = 0;
    for (int i = 3; i<12; ++i) {
      chsum = chsum ^ buf[i];
    }
    if (chsum) {
      Serial.println("checksum mismatch");
      return;
    }
  }

  for (int i = 0; i < 12; ++i) {
    int8_t c = buf[i];
    switch (i+1) {
      case 6:
        roll = c;
        break;
      case 7:
        roll <<= 8;
        roll += c;
        roll = (roll & 0xFF00) >> 8 | (roll & 0x00FF) << 8;  // Reverse the order of bytes
        break;
      case 8:
        pitch += c;
        break;
      case 9:
        pitch <<= 8;
        pitch += c;
        pitch = (pitch & 0xFF00) >> 8 | (pitch & 0x00FF) << 8;  // Reverse the order of bytes
        break;
      case 10:
        yaw += c;
        break;
      case 11:
        yaw <<= 8;
        yaw += c;
        yaw = (yaw & 0xFF00) >> 8 | (yaw & 0x00FF) << 8;  // Reverse the order of bytes
        break;
    }
  }


  
  //Serial.println("Roll: " + String(((roll/10.0)+180)/2));
  //Serial.println(" Pitch: " + String((pitch/10.0)+90));
  //Serial.println(" Yaw: " + String(yaw));
  Serial.println("\t" + String(((roll/10.0)+180)/2) + "\t" + String((pitch/10.0)+90) + "\t" + String(yaw));
}

void sendMSP(uint8_t cmd, uint8_t *data, uint8_t n_bytes) {

  uint8_t checksum = 0;

  Serial.write((byte *)"$M<", 3);
  Serial.write(n_bytes);
  checksum ^= n_bytes;

  Serial.write(cmd);
  checksum ^= cmd;

  Serial.write(checksum);
}