// connected pins
const int PIN_PHOTOCELL = 5;  // the cell and 10K pulldown are connected to a5

void setup() {
  Serial.begin(9600);
  
  Serial.print("Monitoring values of photocell connected to pin ");
  Serial.println(PIN_PHOTOCELL);
}

void loop() {
  int value = analogRead(PIN_PHOTOCELL);
  Serial.println(value);
  delay(250);
}
