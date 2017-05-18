/**
  Group 9
  Arduino code for the car
*/


//the includes
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <FastLED.h>
#include <Smartcar.h>
#include <SharpIR.h>

//definitions of the sensors
#define SRF_ADDRESS1         0x70
#define SRF_ADDRESS2         0x71        // Address of the SRF08
#define IR_REAR              3
#define IR_REAR_RIGHT        5
#define IR_FRONT_RIGHT       4
#define CMD            (byte)0x00  // Command byte, values of 0 being sent with write have to be masked as a byte to stop
// them being misinterpreted as NULL this is a bug with arduino 1.0
#define RANGEBYTE            0x02        // Byte for start of ranging data

//lights
#define NUM_LEDS             16
#define led_lights           11
CRGB leds[NUM_LEDS];

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

//Movementdeclaration
Servo EscServo;
Servo SteerServo;
Odometer encoder;

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
  Method created in order to get the range from the ultrasound.
*/
unsigned char getUltrasonicRange(int sensor) {                                  // This function gets a ranging from the SRF08
  unsigned char range = 0;

  //Start communticating with SRF08
  if (sensor == 1) {
    Wire.beginTransmission(SRF_ADDRESS1);
  }
  if (sensor == 2) {
    Wire.beginTransmission(SRF_ADDRESS2);
  }
  Wire.write(CMD);                                 // Send Command Byte
  Wire.write(0x51);                                // Send 0x51 to start a ranging
  Wire.endTransmission();

  delay(100);                                      // Wait for ranging to be complete

  // start communicating with SRFmodule
  if (sensor == 1) {
    Wire.beginTransmission(SRF_ADDRESS1);          // Start communticating with SRF08
  }
  if (sensor == 2) {
    Wire.beginTransmission(SRF_ADDRESS2);          // Start communticating with SRF08
  }
  Wire.write(RANGEBYTE);                           // Call the register for start of ranging data
  Wire.endTransmission();

  // Request 2 bytes from SRF module
  if (sensor == 1) {
    Wire.requestFrom(SRF_ADDRESS1, 2);
  }
  if (sensor == 2) {
    Wire.requestFrom(SRF_ADDRESS2, 2);
  }

  if (2 <= Wire.available()) {
    //highByte = Wire.read();                          // Get high byte
    //lowByte = Wire.read();
    range = Wire.read();
    range = range << 8;
    range |= Wire.read();

    if (range > 50) {
      range = 0;
    }

    return (range);
  }

  if (range > 40) {
    range = 0;
  }
  return (0);                                  // Returns Range
}


/*
    Method created in order to read a byte from serial port, current speed and angle.
    Iy converts them into floats.
*/
void byteDecode(char byteIn, float *speedIn, float * angle) {
  float tempAng = *angle;
  *angle = ((byteIn & 31) - 15) * 4;
  //Serial2.println("qwdwe  "+String(((float)((byteIn & 224) >> 5)-3)));
  //Serial2.println("abba  "+String(((float)((byteIn & 224) >> 5)-3)/2));
  *speedIn = ((float)((byteIn & 224) >> 5) - 3) / 2;
}


/**
  Method created in order to read sensors.
*/
void readSensors() {
  usFront = getUltrasonicRange(1);          // Calls a function to get range
  usFrontRight = getUltrasonicRange(2);
  irFrontRight = getInfraredRange(IR_FRONT_RIGHT);
  irRear =  getInfraredRange(IR_REAR);
  irRearRight = getInfraredRange(IR_REAR_RIGHT);

  unsigned char startTransmissionVal = 255;
  unsigned char endTransmissionVal = 254;

  //Serial.write(startTransmissionVal);   //for sending the bytes
  Serial.print(startTransmissionVal);     //debugging
  Serial.print(" ");            //debbuging

  //Serial.write(usFront);
  Serial.print(usFront);
  Serial.print(" ");

  //Serial.write(usFrontRight);
  Serial.print(usFrontRight);
  Serial.print(" ");

  //Serial.write(irFrontRight);
  Serial.print(irFrontRight);
  Serial.print(" ");

  //Serial.write(irRear);
  Serial.print(irRear);
  Serial.print(" ");

  //Serial.write(irRearRight);
  Serial.print(irRearRight);
  Serial.print(" ");

  //Serial.write(endTransmissionVal);
  Serial.print(endTransmissionVal);
  Serial.print("\n");
}


void setup() {

  Serial.begin(230400);
  Wire.begin();
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
  SteerServo.write(straight);
  EscServo.writeMicroseconds(NEUTRAL);

  delay(100);                                     // Waits to make sure everything is powered up

  FastLED.addLeds<WS2811, led_lights, RGB>(leds, NUM_LEDS);
}

void loop() {

  //FOR THE LEDS
  //for (int i; i < NUM_LEDS; i++) {
  //  leds[i] = CRGB::Blue;
  //  FastLED.show();
  //}
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

      //delay(20);
      Serial.println("remotel");
    }
    Serial.println("Joystick");

    int i = pulseIn(5, HIGH); //Get speed
    int j = pulseIn(6, HIGH); //Get steering
    j = j - 275;

    /*
       This delay makes sure that is doesn't receive too much information the servo
    */
    delay(10);

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

      SteerServo.write(straight);
      EscServo.writeMicroseconds(NEUTRAL);

    }
    unsigned char readSignal = Serial.read();
    if (readSignal == 31) {
      readSignal = Serial.read();
      unsigned char angleSignal = readSignal;
      angleSignal &= ~(1 << 7);
      angleSignal &= ~(1 << 6);
      angleSignal &= ~(1 << 5);

      unsigned char speedSignal = readSignal;
      speedSignal = speedSignal >> 5;

      //int steering = words[1].toInt();
      SteerServo.write(angleSignal);

      if (speedSignal == 0) {
        EscServo.writeMicroseconds(1400);
      }
      if (speedSignal > 0 && speedSignal < 6) {
        EscServo.writeMicroseconds((speedSignal * 50) + 1400);
      }
      if (speedSignal > 5) {
        EscServo.writeMicroseconds(1150);
      }
    }
  }

  if (speedValue > FULL) {
    speedValue = FULL;
  }
  if (speedValue < BREAK) {
    speedValue = BREAK;
  }

  delay(1);
  FastLED.stop();
}
