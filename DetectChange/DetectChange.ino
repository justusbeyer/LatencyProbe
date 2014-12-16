// CONFIGURATION
const int PIN_PHOTOCELL = 5;        // the cell and 10K pulldown are connected to a5
const int SAMPLE_FREQUENCY = 50000;  // nr of samples per second
const int WINDOW_WIDTH = 100;        // Nr of samples that are looked at as part of the change determination
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

void setup()
{
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

void loop()
{
  switch(state) {  
    case LS_CALIBRATING:
      if(calibrate())
      {
        Serial.println("Calibrated, measuring...");
        state = LS_CALIBRATED;
      }
      break;
      
    case LS_CALIBRATED:
      if(detect_change()) {
        Serial.println("Change detected, recalibrating...");       
        state = LS_CALIBRATING;
        next_sample_pos = 0;
      }
      break;
  }

  delayMicroseconds(sample_interval);
}
