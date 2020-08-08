void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial1.begin(38400);
  pinMode(5,OUTPUT);
  pinMode(13,OUTPUT);
}


void loop() {
    if (Serial.available()>0) {
      digitalWrite(5,HIGH);
      char c = Serial.read();
      Serial1.write(c);
      Serial1.flush();
      digitalWrite(5,LOW);
    } else if (Serial1.available()>0) {
      Serial.write(Serial1.read());
      Serial.flush();
    }
}
