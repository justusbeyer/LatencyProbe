/* Cloudgaming motion sensor

   - arduino button is pressed and starts servo
   - servo is pressing a key (on keyboard to move character) -> start timer
   - action is processed by cloud gaming plattform
   - the photo sensor detects changes in light -> end timer
   - time difference is real latency for gamer
 
  created 20 Nov 2014
  by Richard Varbelow
*/
#include <Servo.h> 

// connected pins
const int buttonPin = 2;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin
const int photocellPin = 5;  // the cell and 10K pulldown are connected to a5
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
int pos = 0;    // variable to store the servo position

// pushbutton status
int buttonState = 0;
//int lastState = 0;

// time differences
unsigned long time;
unsigned long differenceDetectedTime;
int measurementCycles = 25;
int cycleCounter = 0;
unsigned long maximumWaitingDelay = 1000; // waiting for cloud gaming processing and servor movements
unsigned long servoResetDelay = 100; // waiting for cloud gaming processing and servor movements
unsigned long averagedDelay = 0;

// photo sensor
int photocellReading;
int photocellLastValue = 0;
const int photoSensitivity = 15;   // sensitivity to sensor changes
boolean motionDetectionAborted = false;

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);      
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  
  // attaches the servo on pin 8 to the servo object 
  myservo.attach(8);
  myservo.write(0);
  
  // set comparison value for photo sensor
//  photocellLastValue = analogRead(photocellPin);
  
  // prepare serial output
  Serial.begin(9600);
  Serial.println("Time stamps are relatively to program start");
}

void loop(){
  // TODO use two servos to go back and forth, make bigger delays to let the player stop his movement
  // TODO tidy up code
  
  // read the state of button and sensor:
  buttonState = digitalRead(buttonPin);
  
  if(buttonState == HIGH){
    cycleCounter = measurementCycles;
  }
  
  if(cycleCounter != 0){
    if(cycleCounter == measurementCycles){
      averagedDelay = 0;
    }
    digitalWrite(ledPin, HIGH);
    photocellReading = analogRead(photocellPin);
    myservo.write(20);
    time = millis(); // measure time after servo is set to compensate inertia of servo (at least a little)
//    Serial.print("starting time: ");
//    Serial.println(time);

    // read data first before checking condition
    do{
      photocellLastValue = photocellReading;
      photocellReading = analogRead(photocellPin);
      // keep for debugging
      // WARNING: due to a hardwired 20ms delay in the rxtx library used in java, using the serial output slows
      // down and therefore distorts the measurements! (see http://neophob.com/2011/04/serial-latency-teensy-vs-arduino/) 
//       Serial.print("Analog reading: last value = ");
//       Serial.print(photocellLastValue);
//       Serial.print("; current value = ");
//       Serial.println(photocellReading);

      // break loop when delay becomes too big and would be unrealistic in cloud gaming
      if((time+maximumWaitingDelay) < millis()){
//        Serial.println(time+maximumWaitingDelay);
        motionDetectionAborted = true;
        break;
      }
      
    }
    // read photo cell data as long as there is no difference
    // +- value of sensitivity compared to last value --> motion
    while(!((photocellReading > (photocellLastValue + photoSensitivity))
         || (photocellReading < (photocellLastValue - photoSensitivity))));
    if(!motionDetectionAborted){
      differenceDetectedTime = millis();
        Serial.println("---------------motion detected---------------");
//          Serial.print("difference time: ");
//          Serial.print(differenceDetectedTime);
//          Serial.print(" / time: ");
//          Serial.println(time);
        Serial.print("photo data: last value=");
        Serial.print(photocellLastValue);
        Serial.print(", current value=");
        Serial.println(photocellReading);
        Serial.print("time difference: ");
        Serial.print(differenceDetectedTime - time);
        Serial.print("ms, cycles: ");
        Serial.print(measurementCycles-cycleCounter+1);
        Serial.print("/");
        Serial.println(measurementCycles);
        Serial.println("---------------------------------------------");
        averagedDelay += (differenceDetectedTime - time);
        cycleCounter--;
        if(cycleCounter == 0){
          Serial.print("--> averaged cloud gaming delay: ");
          Serial.print(averagedDelay/measurementCycles); 
          Serial.println("ms");
        }
    } else{
       Serial.println(">motion detection aborted (too high latency)<");
       Serial.println(">-----------------try again-----------------<");
       motionDetectionAborted = false;
    }
    // either way, set back the servo
    myservo.write(0);
    delay(servoResetDelay);
    
  } else{
    digitalWrite(ledPin,LOW);
  }
}
