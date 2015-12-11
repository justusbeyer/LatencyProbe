const int PIN_TRIGGER  = 13; // We choose 13 as this is also connected to an on-board LED

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_TRIGGER, OUTPUT);
  digitalWrite(PIN_TRIGGER, LOW);

  // Do not start triggering right after power up
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(PIN_TRIGGER, HIGH);
  delay(200);
  
  digitalWrite(PIN_TRIGGER, LOW);
  delay(random(1500,3000));
  //delay(random(700,1300));
}
