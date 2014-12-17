// CONFIGURATION
const int PIN_BUTTON  = 2;
const int PIN_SERVO   = 8;
const int SERVO_POSITION_PRESS = 20;
const int SERVO_POSITION_RELEASE = 29;

#include <Servo.h> 

Servo servo;
int current_servo_position = -1;

void setup()
{
  pinMode(PIN_BUTTON, INPUT);
  
  servo.attach(PIN_SERVO);
  set_servo(SERVO_POSITION_RELEASE);
}

void loop()
{
  int button_state = digitalRead(PIN_BUTTON);
  
  set_servo((button_state == HIGH) ? SERVO_POSITION_PRESS : SERVO_POSITION_RELEASE);
}

void set_servo(int servo_position)
{
  if (servo_position != current_servo_position)
  {
    current_servo_position = servo_position;
    servo.write(servo_position);
  }
}

