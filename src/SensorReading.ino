#include <Wire.h>
#include <SoftwareSerial.h>

#define SRF_ADDRESS1        0x70
#define SRF_ADDRESS2        0x71                                   // Address of the SRF08
#define IR_REAR             3
#define IR_REAR_RIGHT       5
#define IR_FRONT_RIGHT      4
#define CMD                 (byte)0x00                             // Command byte, values of 0 being sent with write have to be masked as a byte to stop them being misinterpreted as NULL this is a bug with arduino 1.0
#define RANGEBYTE           0x02                                   // Byte for start of ranging data

byte highByte = 0x00;                             // Stores high byte from ranging
byte lowByte = 0x00;                              // Stored low byte from ranging

void setup(){
  
  Serial.begin(9600);
  Wire.begin();                               
  delay(100);                                     // Waits to make sure everything is powered up
}

void loop(){

  int ultrasonicFF = getUltrasonicRange(1);          // Calls a function to get range
  int ultrasonicFR = getUltrasonicRange(2);
  int infraredFR = getInfraRedRange(IR_FRONT_RIGHT);
  int infraredR =  getInfraRedRange(IR_REAR);
  int infraredRR = getInfraRedRange(IR_REAR_RIGHT);
  String sensorData = "", tmp = "";
  tmp = String(infraredFR);
  tmp = String(tmp + " ");
  sensorData = String(sensorData + tmp);
  tmp = String(infraredR);
  tmp = String(tmp + " ");
  sensorData = String(sensorData + tmp);
  tmp = String(infraredRR);
  tmp = String(tmp + " ");
  sensorData = String(sensorData + tmp);
  tmp = String(ultrasonicFF);
  tmp = String(tmp + " ");
  sensorData = String(sensorData + tmp);
  tmp = String(ultrasonicFR);
  tmp = String(tmp + "\n");
  sensorData = String(sensorData + tmp);
  
  //sensorData = String(infraredFR + " " + infraredR + " " + infraredRR + " " + ultrasonicFF + " " + ultrasonicFR);
  
  Serial.print(sensorData);

 delay(100);                                      // Wait before looping
}

int getInfraRedRange(int sensor){
  int range = 0;
  int reading = analogRead(sensor);

  if(reading > 100 && reading < 490){
    range = 2076/(reading - 20);
  } else {
    range = 0;
  }

  return range;
}

int getUltrasonicRange(int sensor){                                   // This function gets a ranging from the SRF08
  
  int range = 0; 

  // Start communticating with SRF08
  if(sensor == 1){
    Wire.beginTransmission(SRF_ADDRESS1);
  }
  if(sensor == 2){
    Wire.beginTransmission(SRF_ADDRESS2);             
  }
  Wire.write(CMD);                                 // Send Command Byte
  Wire.write(0x51);                                // Send 0x51 to start a ranging
  Wire.endTransmission();
  
  delay(100);                                      // Wait for ranging to be complete

  // start communicating with SRFmodule
  if(sensor == 1){
    Wire.beginTransmission(SRF_ADDRESS1);          // Start communticating with SRF08
  }
  if(sensor == 2){
    Wire.beginTransmission(SRF_ADDRESS2);          // Start communticating with SRF08             
  }
  Wire.write(RANGEBYTE);                           // Call the register for start of ranging data
  Wire.endTransmission();

  // Request 2 bytes from SRF module
  if(sensor == 1){
    Wire.requestFrom(SRF_ADDRESS1, 2);                
  }
  if(sensor == 2){
    Wire.requestFrom(SRF_ADDRESS2, 2);
  }
  
  while(Wire.available() < 2);                     // Wait for data to arrive
  highByte = Wire.read();                          // Get high byte
  lowByte = Wire.read();                           // Get low byte

  range = (highByte << 8) + lowByte;               // Put them together

  if(range > 40){
    range = 0;
  }
  return(range);                                   // Returns Range
}
