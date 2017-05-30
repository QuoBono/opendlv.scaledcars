/**

  Group 9

  Arduino code for the car

*/



#include <NewPing.h>

//#include <Wire.h>

//#include <SoftwareSerial.h>

#include <Servo.h>

//#include <FastLED.h>

#include <Smartcar.h>

//#include <SharpIR.h>



//#include <RcCar.h>



//definitions of the sensors

//#define SRF_ADDRESS1         0x71

//#define SRF_ADDRESS2         0x70        // Address of the SRF08



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

//CRGB leds[NUM_LEDS];



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

unsigned int inputSpeed = 0;

int inputSteer = 0;



float angle = 90;

float speeds = 0;

//RcCar* car ;



NewPing sonar1(SONAR1_TRIG, SONAR1_ECHO, 45);

NewPing sonar2(SONAR2_TRIG, SONAR2_ECHO, 45);



//Movementdeclaration

Servo EscServo;

Servo SteerServo;

//Odometer encoder;



/*

  reads a byte from serial port, current speed and angle , reads them as bytes and tranforms them into floats. used when you get info from the proxy

*/

struct Vehicle {

  unsigned char carSpeed : 2;

} s;





void byteDecode(char byteIn, float *speed, float * angle) {

  float tempAng = *angle;

  *angle = byteIn >> 3;

}



/**

   Method created in order to get the range from the infrared.

*/

unsigned char getInfraredRange(int sensor) {

  unsigned char range = 0;

  int reading = analogRead(sensor);



  if (reading > 100 && reading < 490) {

    range = 2076 / (reading - 20);

  }

  else

  {

    range = 0;

  }



  return range;

}







/**

  Method created in order to read sensors.

*/

void readSensors() {

  //  usFront = getUltrasonicRange(1);          // Calls a function to get range

  usFront = sonar1.ping_cm();

  //  delay(2);

  //  usFrontRight = getUltrasonicRange(2);

  usFrontRight = sonar2.ping_cm();

  //  delay(2);

  irFrontRight = getInfraredRange(IR_FRONT_RIGHT);

  irRear =  getInfraredRange(IR_REAR);

  irRearRight = getInfraredRange(IR_REAR_RIGHT);



  unsigned char startTransmissionVal = 255;

  unsigned char endTransmissionVal = 254;




    Serial.print("{");



    Serial.print(irFrontRight);

    Serial.print(" ");



    //  Serial.write(irRear);

    Serial.print(irRear);

    Serial.print(" ");



    //  Serial.write(irRearRight);

    Serial.print(irRearRight);

    Serial.print(" ");



    //  Serial.write(usFront);

    Serial.print(usFront);

    Serial.print(" ");



    //  Serial.write(usFrontRight);

    Serial.print(usFrontRight);

    // Serial.print(" ");



    Serial.print("}");



    //  Serial.write(endTransmissionVal);

    //  Serial.print(endTransmissionVal);

    //  Serial.print("\n");

 

}



void setup() {



  //  speeds = 0; //speed of vehicle

  //  angle = 90; //steering angle



  Serial.begin(115200);

  //  Serial.begin(250000);



  // Wire.begin();

  delay(100);



  //joystick

  pinMode(5, INPUT);

  pinMode(6, INPUT); //steering

  pinMode(led_lights, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);



  //control

  pinMode(9, OUTPUT);

  pinMode(10, OUTPUT);



  delay(100);



  EscServo.attach(10); //11 speed pin

  SteerServo.attach(9); //steering



  delay(100);                                     // Waits to make sure everything is powered up



}



void loop() {



readSensors();

  if (pulseIn(5, HIGH, 27000) > 800 && pulseIn(6, HIGH, 27000) > 800) {

    joystick = true;

  }

  else

  {

    joystick = false;

  }



  if (joystick) {

    if (!controller) {

      speedValue = NEUTRAL;

      speedValueRev = NEUTRALREVERSE;

      steerValue = straight;



      SteerServo.write(straight);

      EscServo.writeMicroseconds(NEUTRAL);

      controller = true;



    }





    int i = pulseIn(5, HIGH); //Get speed

    int j = pulseIn(6, HIGH); //Get steering



    j = j - 275;

    delay(5);



    if (i > FULL) {

      i = FULL;

    }



    if (i < BREAK) {

      i = BREAK;

    }



    EscServo.writeMicroseconds(i); //speed

    SteerServo.write(j); //steering

  }

  else

  {



    if (controller) {

      speedValue = NEUTRAL;



      speedValueRev = NEUTRALREVERSE;

      steerValue = straight;

      controller = false;

    }



    if (Serial.available()) {

      //Read the byte.
      Serial.setTimeout(1);

      Serial.readBytes(inputValues, 1);
      

      //For writing the speed

      s.carSpeed = inputValues[0];

      Serial.println(inputValues[0]);

      Serial.println(s.carSpeed);

      int tmpInt = (s.carSpeed * 50) + 1400;

      EscServo.writeMicroseconds( tmpInt);



      //For writing to Steering

      inputSteer = (inputValues[0] >> 2);

      inputSteer *= 2;

      Serial.println(inputSteer);

      SteerServo.write((int)inputSteer);

      Serial.flush();


    }



  }



  if (speedValue > FULL) {

    speedValue = FULL;

  }

  if (speedValue < BREAK) {

    speedValue = BREAK;

  }



  delay(30);

}
