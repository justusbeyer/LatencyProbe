// CONFIGURATION
const int PIN_PHOTOCELL = 1;        // the cell and 10K pulldown are connected to a5
const int PIN_BUTTON  = 7;
const int PIN_SERVO   = 8;
// const int PIN_CHANGE_SIGNAL_LED = 2; // This lights up when a change is detected.

const int PIN_LED_MEASUREMENT_RUNNING = 20;

// TFT
const int PIN_TFT_DC = 9;
const int PIN_TFT_CS = 10;

const int SERVO_POSITION_PRESS = 20;
const int SERVO_POSITION_RELEASE = 29;

const unsigned long SAMPLE_FREQUENCY = 3000;  // nr of samples per second
const int WINDOW_WIDTH = 8;        // Nr of samples that are looked at as part of the change determination
const int DETECTION_THRESHOLD = 100;

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

const unsigned int sample_interval = (unsigned long)1000000 / SAMPLE_FREQUENCY;
unsigned long  timestamp_last_sample = 0; // 0: not sample yet since calibration
unsigned long  measured_sample_interval = 0; // 0: no valid measurement (yet)
unsigned long  sample_delay = sample_interval; // This is the time in microseconds the main loop waits after each execution

unsigned int detection_theshold = DETECTION_THRESHOLD; // initial threshold

#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"
#include <Servo.h>

Servo servo;
int current_servo_position = -1;

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(PIN_TFT_CS, PIN_TFT_DC);

void print(char* msg)
{
  //Serial.print(msg);
  tft.print(msg);
}

void println(char* msg)
{
  //Serial.println(msg);
  tft.println(msg);
}

void print(long unsigned int val)
{
  //Serial.print(val);
  tft.print(val);
}

void println(long unsigned int val)
{
  //Serial.println(val);
  tft.println(val);
}

void clearScreen()
{
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
}

void setup()
{
  delay(2000);
  
  // pinMode(PIN_CHANGE_SIGNAL_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT/*_PULLUP*/);
  pinMode(PIN_LED_MEASUREMENT_RUNNING, OUTPUT);
  digitalWrite(PIN_LED_MEASUREMENT_RUNNING, LOW);
  
  servo.attach(PIN_SERVO);
  set_servo(SERVO_POSITION_RELEASE);
  
  Serial.begin(115200);

  tft.begin();
  clearScreen();
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(2);
  
  Mouse.begin();
  
  print("Photosensor@PIN: A");
  println(PIN_PHOTOCELL);
  
  state = LS_CALIBRATING;
}

void sample()
{
  sensor_values[next_sample_pos] = analogRead(PIN_PHOTOCELL);
  
  // Calculate time since last sample
  unsigned long now = micros();
  if (timestamp_last_sample > 0)
    measured_sample_interval = now - timestamp_last_sample;
  timestamp_last_sample = now;
  
  next_sample_pos = (next_sample_pos + 1) % WINDOW_WIDTH;
}

unsigned int calculate_stddev()
{
  unsigned long window_sum = 0;
  unsigned int sum_deviation=0;
  
  int i;
  for(i=0; i<WINDOW_WIDTH; i++)
      window_sum += sensor_values[i];
      
  unsigned short mean = window_sum / WINDOW_WIDTH;
  
  for(i=0; i<WINDOW_WIDTH; i++)
    sum_deviation += sq(sensor_values[i] - mean);
    
  return sum_deviation;
    /*float mean=0.0, sum_deviation=0.0;
    int i;
    for(i=0; i<WINDOW_WIDTH; i++)
        mean += sensor_values[i];
        
    mean=mean / WINDOW_WIDTH;
    
    for(i=0; i < WINDOW_WIDTH; i++)
      sum_deviation += (sensor_values[i] - mean) * (sensor_values[i]-mean);
    return sqrt(sum_deviation/WINDOW_WIDTH);*/
}

// Fill the sensor_values array with meaningful data
bool calibrate()
{
  // Before we (re)calibrate wait 1s to make sure we're not catching the same change multiple times
  if (next_sample_pos == 0)
  {
    delay(200);
    timestamp_last_sample = 0;
  }
  
  sample();
  
  // Check if we've filled the whole array
  if (next_sample_pos == 0)
  {
    // Set the detection threshold using the observed fluctuations in the signal
    detection_theshold = 2 * calculate_stddev() + DETECTION_THRESHOLD;
    return true;
  }

  return false;
}

bool detect_change()
{
  sample();
  return (calculate_stddev() > detection_theshold);
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
  println("Measured values:");
  for (int i=0; i < WINDOW_WIDTH; i++)
  {
    char buf[5];
    sprintf(buf, "%04d", sensor_values[(next_sample_pos+i) % WINDOW_WIDTH]);
    print(buf);
    print(":");
  }
  println("");   
}

void loop()
{
  switch(state) {  
    case LS_CALIBRATING:
      if(calibrate())
      {
        print("Calibrated. ");
        state = LS_CALIBRATED;
        timestamp_last_sample = 0;
        
        // Turn of signal LED
        // digitalWrite(PIN_CHANGE_SIGNAL_LED, LOW);
        
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
        if (timestamp_button_press > 1) {
          measured_delay = timestamp_last_sample/1000 - timestamp_button_press;
        }
        
        // Turn on signal LED
        // digitalWrite(PIN_CHANGE_SIGNAL_LED, HIGH);        
        
        println("Change detected.");        
        
        if (measured_delay > 0) {
          // Turn measurement signal LED off
          digitalWrite(PIN_LED_MEASUREMENT_RUNNING, LOW);

          // Reset measurement timestamp
          timestamp_button_press = 0;
          
          // Print out result
          clearScreen();
          tft.setTextSize(4);
          print("d: ");
          print(measured_delay);
          Serial.println(measured_delay);
          println("ms");
          tft.setTextSize(1);
          print("sr: ");
          print(1000000L / measured_sample_interval);
          println("Hz");
          /*print("Confed SI: ");
          print(sample_interval);
          println("us");*/
          /*print("Sample Delay: ");
          print(sample_delay);
          println("us");*/
          print_values();
          // delay(100);
        }
        
        // Recalibrate
        state = LS_CALIBRATING;
        next_sample_pos = 0;
      }
      break;
  }
  
  // Move servo depending on button input
  int button_state = digitalRead(PIN_BUTTON);
  // set_servo((button_state == HIGH) ? SERVO_POSITION_PRESS : SERVO_POSITION_RELEASE);
  if ((button_state == HIGH) && (timestamp_button_press == 0) && (state == LS_CALIBRATED)) {
    //Mouse.click();
    //Mouse.press();
    timestamp_button_press = millis();
    digitalWrite(PIN_LED_MEASUREMENT_RUNNING, HIGH);
    //delay(40);
    //Mouse.release();
  }
  else if ((button_state == LOW) && (timestamp_button_press > 1) && (timestamp_last_sample/1000 - timestamp_button_press > 1000 /* max delay */)) {
    timestamp_button_press = 0;
    digitalWrite(PIN_LED_MEASUREMENT_RUNNING, LOW);
  }
  
  // Compute the right sleep time to achieve the desired sampling frequency
  /*if (measured_sample_interval > 0 &&
      abs(measured_sample_interval - sample_interval) > 0)
  {
    long correction = ((long)sample_interval - (long)measured_sample_interval) / 2L;
    if (-correction > sample_delay)
      sample_delay = 0;
    else
      sample_delay += correction;
  }*/
  if (measured_sample_interval > 0)
  {
    if (measured_sample_interval > sample_interval && sample_delay > 0)
      sample_delay--;
    else
      sample_delay++;
  }
  
  delayMicroseconds(sample_delay);
}
