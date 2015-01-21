// CONFIGURATION
const int PIN_PHOTOCELL = 5;        // the cell and 10K pulldown are connected to a5
const int PIN_BUTTON  = 7;
const int PIN_SERVO   = 8;
const int PIN_CHANGE_SIGNAL_LED = 2; // This lights up when a change is detected.

const int SERVO_POSITION_PRESS = 20;
const int SERVO_POSITION_RELEASE = 29;

const int SAMPLE_FREQUENCY = 50000;  // nr of samples per second
const int WINDOW_WIDTH = 8;        // Nr of samples that are looked at as part of the change determination
const int DETECTION_THRESHOLD = 100;

const unsigned int sample_interval = (long)1000000 / SAMPLE_FREQUENCY;
// Data types
enum CALIBRATION_STATE
{
  LS_CALIBRATING = 0,
  LS_CALIBRATED  = 1
} state;

// Data
unsigned short sensor_values[WINDOW_WIDTH] = { 0 };
unsigned short next_sample_pos = 0;
unsigned long  timestamp_button_press = 1; // 0: Ready to perform measurements, 1: awaiting calibration, millis(): timestamp of button press

#include <Servo.h>
Servo servo;
int current_servo_position = -1;

void setup()
{
  pinMode(PIN_CHANGE_SIGNAL_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT);
  
  servo.attach(PIN_SERVO);
  set_servo(SERVO_POSITION_RELEASE);
  
  Serial.begin(9600);
  
  Serial.print("Detecting change of light intensity measured by photocell connected to pin ");
  Serial.println(PIN_PHOTOCELL);
  
  state = LS_CALIBRATING;
}

void sample()
{
  sensor_values[next_sample_pos] = analogRead(PIN_PHOTOCELL);
  next_sample_pos = (next_sample_pos + 1) % WINDOW_WIDTH;
}

float calculate_stddev()
{
    float mean=0.0, sum_deviation=0.0;
    int i;
    for(i=0; i<WINDOW_WIDTH; i++)
        mean += sensor_values[i];
        
    mean=mean / WINDOW_WIDTH;
    
    for(i=0; i < WINDOW_WIDTH; i++)
      sum_deviation += (sensor_values[i] - mean) * (sensor_values[i]-mean);
    return sqrt(sum_deviation/WINDOW_WIDTH);           
}

// Fill the sensor_values array with meaningful data
bool calibrate()
{
  sample();
  return (next_sample_pos == 0);
}

bool detect_change()
{
  sample();
  return (calculate_stddev() > DETECTION_THRESHOLD);
}

void set_servo(int servo_position)
{
  if (servo_position != current_servo_position)
  {
    current_servo_position = servo_position;
    servo.write(servo_position);
  }
}

void print_values()
{
  Serial.println("Measured values:");
  for (int offset=WINDOW_WIDTH; offset > 0; offset--)
  {
    char buf[5];
    sprintf(buf, "%04d", sensor_values[(next_sample_pos-1-offset) % WINDOW_WIDTH]);
    Serial.print(buf);
    Serial.print(":");
  }
  Serial.println();   
}

void loop()
{
  switch(state) {  
    case LS_CALIBRATING:
      if(calibrate())
      {
        Serial.println("Calibrated, measuring...");
        state = LS_CALIBRATED;
        
        // Turn of signal LED
        digitalWrite(PIN_CHANGE_SIGNAL_LED, LOW);
        
        // ready for new measurements
        timestamp_button_press = 0;
      }
      break;
      
    case LS_CALIBRATED:
      if(detect_change()) {
        // change detected
        unsigned long measured_delay = 0;
        
        // If the button was pressed and the true measurement is running,
        // the observed change should correspond to our input action and
        // we can measure the delay now.
        if (timestamp_button_press > 1)
          measured_delay = millis() - timestamp_button_press;
        
        // Turn on signal LED
        digitalWrite(PIN_CHANGE_SIGNAL_LED, HIGH);        
        
        Serial.println("Change detected.");
        print_values();
        
        if (measured_delay > 0) {
          Serial.print("Seen delay: ");
          Serial.print(measured_delay);
          Serial.println("ms.");
        }
        
        // Recalibrate
        state = LS_CALIBRATING;
        next_sample_pos = 0;
      }
      break;
  }
  
  // Move servo depending on button input
  int button_state = digitalRead(PIN_BUTTON);
  set_servo((button_state == HIGH) ? SERVO_POSITION_PRESS : SERVO_POSITION_RELEASE);
  if ((button_state == HIGH) && (timestamp_button_press == 0)) {
    timestamp_button_press = millis();
  }
  else if ((button_state == LOW) && (timestamp_button_press > 1)) {
    timestamp_button_press = 0;
  }
  
  delayMicroseconds(sample_interval);
}
