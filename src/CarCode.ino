/**
  Group 9
  Arduino code for the car

*/



#include <NewPing.h>

#include <Servo.h>

#include <Smartcar.h>

#define SONAR1_TRIG 35

#define SONAR1_ECHO 34

#define SONAR2_TRIG 41

#define SONAR2_ECHO 40

#define IR_REAR              3

#define IR_REAR_RIGHT        5

#define IR_FRONT_RIGHT       4

#define CMD            (byte)0x00  // Command byte, values of 0 being sent with write have to be masked as a byte to stop

// them being misinterpreted as NULL this is a bug with arduino 1.0

#define RANGEBYTE            0x02        // Byte for start of ranging data

//lights

#define NUM_LEDS             8

#define led_lights           11


//definitions for the movement

#define NEUTRAL              1400

#define FULL                 1500

#define BREAK                1000

#define NEUTRALREVERSE       1200



byte highByte              = 0x00;                             // Stores high byte from ranging

byte lowByte               = 0x00;                              // Stored low byte from ranging


//sensors declarations

unsigned char usFront      = 0;          // Calls a function to get range

unsigned char usFrontRight = 0;

unsigned char irFrontRight = 0;

unsigned char irRear       = 0;

unsigned char irRearRight  = 0;



int straight               = 70;

int speedValue             = NEUTRAL;

int steerValue             = straight;

int speedValueRev          = NEUTRALREVERSE;

bool joystick              = false;

bool controller            = false;

char userInput;



int steerCarOutput         = 9 ;

int SpeedCarOutput         = 10;



unsigned char inputValues[1];
unsigned char serialString[15];
unsigned char oldInputValue = 0x00;

unsigned int inputSpeed = 0;

int inputSteer = 0;



float angle = 90;

float speeds = 0;


int counterClock; //clock for the reading of sensors
String serialSensor; //for sending the string to the serial



NewPing sonar1(SONAR1_TRIG, SONAR1_ECHO, 45);

NewPing sonar2(SONAR2_TRIG, SONAR2_ECHO, 45);



//Movementdeclaration

Servo EscServo;

Servo SteerServo;

Odometer encoder;



const int encoderPinA = 2;
const int encoderPinB = 4;
//Initialize Variables
long counts = 0; //counts the encoder counts. The encoder has ~233counts/rev


/*
  reads a byte from serial port, current speed and angle , reads them as bytes and tranforms them into floats. used when you get info from the proxy
*/

struct Vehicle {

  unsigned char carSpeed : 2;

} s;



/*
   Method created in order to get the range from the infrared.
*/

unsigned char getInfraredRange(int sensor) {

  unsigned char range = 0;
  int reading = analogRead(sensor);

  if (reading > 100 && reading < 490) {
    range = 2076 / (reading - 20);

  } else {
    range = 0;

  }

  return range;

}


/**
  Method created in order to read sensors.
*/

void readSensors() {


  usFront = sonar1.ping_cm();

  delay(2);

  usFrontRight = sonar2.ping_cm();

  delay(2);

  irFrontRight = getInfraredRange(IR_FRONT_RIGHT);
  //
  irRear =  getInfraredRange(IR_REAR);
  //
  irRearRight = getInfraredRange(IR_REAR_RIGHT);

  serialSensor = "";

  serialSensor.concat('{');
  serialSensor.concat(irFrontRight);
  serialSensor.concat(' ');
  serialSensor.concat(irRear);
  serialSensor.concat(' ');
  serialSensor.concat(irRearRight);
  serialSensor.concat(' ');
  serialSensor.concat(usFront);
  serialSensor.concat(' ');
  serialSensor.concat(usFrontRight);
  serialSensor.concat(' ');
  serialSensor.concat(counts);
  serialSensor.concat(' ');
  serialSensor.concat('}');

  Serial.print(serialSensor);

}



void setup() {

  Serial.begin(115200);

  delay(100);

  pinMode(5, INPUT);

  pinMode(6, INPUT); //steering

  pinMode(led_lights, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);



  //control

  pinMode(9, OUTPUT);

  pinMode(12, OUTPUT);



  delay(100);

  pinMode(encoderPinA, INPUT); //initialize Encoder Pins
  pinMode(encoderPinB, INPUT);

  digitalWrite(encoderPinA, LOW); //initialize Pin States
  digitalWrite(encoderPinB, LOW);

  attachInterrupt(0, readEncoder, CHANGE); //attach interrupt to PIN 2

  EscServo.attach(12); //12 speed pin

  SteerServo.attach(9); //steering

  delay(100);  // Waits to make sure everything is powered up

}



void loop() {

  /*
     READ SENSORS
  */
  if (counterClock == 10) {
    readSensors();
    counterClock = 0;

  }
  counterClock++;




  Serial.flush();


  if (Serial.available()) {

    //Timeout
    Serial.setTimeout(50);
    //Clean the buffer
    Serial.flush();

    //Method to read only one byte
    Serial.readBytes(inputValues, 1);

    Serial.setTimeout(50);

    //Guard to avoid Redundancy
    if (oldInputValue != inputValues[0]) {

      Serial.flush();


      /*
         Speed Command
      */
      s.carSpeed = inputValues[0];
      int tmpInt = 0;
      //The speeds from can be 0,1,2 and 3
      //0 is stop
      //1 and 2 is forward
      //3 is backwards
      if (s.carSpeed < 3) {
        tmpInt = (s.carSpeed * 50) + 1400;
      } else {
        tmpInt = 1200;
      }

      EscServo.writeMicroseconds( tmpInt);


      /*
         Steering Command
      */
      //For writing to Steering. We shift the bits 2 to the right and multiply by two.
      inputSteer = (inputValues[0] >> 2);

      //Used for debugging. check debug in proxy and compare steering from both sides.
      //Serial.println(inputSteer);
      inputSteer *= 2;

      delay(15);

      SteerServo.write((int)inputSteer);

      delay(15);


      //we use this for avoiding redundancy in the steering.
      inputValues[0] = oldInputValue;

    }//end of if

  }//end of serial.available()

}//end of loop



void readEncoder() //this function is triggered by the encoder CHANGE, and increments the encoder counter
{
  counts = counts + 1;

}
