// CONFIGURATION
const int PIN_PHOTOCELL = 1;        // the cell and 10K pulldown are connected to a5
const int PIN_BUTTON  = 7;
const int PIN_LED_MEASUREMENT_RUNNING = 20;

// TFT
const int PIN_TFT_DC = 9;
const int PIN_TFT_CS = 10;

const unsigned long SAMPLE_FREQUENCY = 3000;  // nr of samples per second
const int WINDOW_WIDTH = 8;        // Nr of samples that are looked at as part of the change determination (i.e. when the system under test responds)
const int DETECTION_THRESHOLD = 100;

// Data types
enum CALIBRATION_STATE
{
  LS_CALIBRATING = 0,
  LS_CALIBRATED  = 1
} state;

// Data
unsigned short      sensor_values[WINDOW_WIDTH] = { 0 };
unsigned short      next_sample_pos = 0;
unsigned long       timestamp_button_press = 1; // 0: Ready to perform measurements, 1: awaiting calibration, millis(): timestamp of button press

const unsigned int  sample_interval = (unsigned long)1000000 / SAMPLE_FREQUENCY;
unsigned long       timestamp_last_sample = 0; // 0: not sample yet since calibration
unsigned long       measured_sample_interval = 0; // 0: no valid measurement (yet)
unsigned long       sample_delay = sample_interval; // This is the time in microseconds the main loop waits after each execution

unsigned int        detection_theshold = DETECTION_THRESHOLD; // initial threshold

#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#include <Mouse.h>

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(PIN_TFT_CS, PIN_TFT_DC);

void print(char* msg)
{
  tft.print(msg);
}

void println(char* msg)
{
  tft.println(msg);
}

void print(long unsigned int val)
{
  tft.print(val);
}

void println(long unsigned int val)
{
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
  
  pinMode(PIN_BUTTON, INPUT/*_PULLUP*/);
  pinMode(PIN_LED_MEASUREMENT_RUNNING, OUTPUT);
  digitalWrite(PIN_LED_MEASUREMENT_RUNNING, LOW);
  
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
          print_values();
        }
        
        // Recalibrate
        state = LS_CALIBRATING;
        next_sample_pos = 0;
      }
      break;
  }
  
  // Poll for the trigger (button) to start measuring
  int button_state = digitalRead(PIN_BUTTON);
  if ((button_state == HIGH) && (timestamp_button_press == 0) && (state == LS_CALIBRATED)) {
    // Time to start the test.

    // If you want the Arduino to create Mouse / Keyboard / whatever events, now is the time:
    
    // To initiate a click, uncomment the following line:
    //Mouse.click();

    // To initiate a longer-lasting click uncomment the following line instead and do not forget to uncomment the delay+release line below as well.
    //Mouse.press();

    // Rember this the timestamp of this moment for later computation of the delay
    timestamp_button_press = millis();

    // Signal using the connected LED that a measurement is now running
    digitalWrite(PIN_LED_MEASUREMENT_RUNNING, HIGH);

    // The following line is just necessary when you used the 'manual' click method with press() and release():
    // delay(40); Mouse.release();
  }
  else if ((button_state == LOW) && (timestamp_button_press > 1) && (timestamp_last_sample/1000 - timestamp_button_press > 1000 /* max delay */)) {
    // Stop measuring
    timestamp_button_press = 0;
    digitalWrite(PIN_LED_MEASUREMENT_RUNNING, LOW);
  }
  
  // Compute the right sleep time to achieve the desired sampling frequency
  if (measured_sample_interval > 0)
  {
    if (measured_sample_interval > sample_interval && sample_delay > 0)
      sample_delay--;
    else
      sample_delay++;
  }
  
  delayMicroseconds(sample_delay);
}

