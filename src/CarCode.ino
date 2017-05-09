  /*author Group9
   */
  
  
  #define NEUTRAL 1400
  #define FULL 1500
  #define BREAK 1000
  #define NEUTRALREVERSE 1200
  //#define RIGHT 80
  //#define LEFT 60
  
  
  
  #include <Servo.h>
  #include <Smartcar.h>
  
  int speedCarInput = 5; //speed
  int steerCarInput = 6; //steering
  int steerCarOutput = 9 ;
  int SpeedCarOutput = 10;
  
  int straight = 70;
  bool joystick = false;
  char userInput;
  
  
  int speedValue = NEUTRAL;
  int right = RIGHT;
  int left = LEFT;
  int speedValueRev = NEUTRALREVERSE;
  bool controller = false;
  int steerValue = straight;
  
  Car car(useServo(9), useESC(10)); //use the servo motor connected at pin 8 and the ESC connected to pin 9
  
  Servo EscServo;
  Servo SteerServo;
  
  void setup(){
      Serial.begin(9600);
      
      pinMode(speedCarInput, INPUT);
      pinMode(steerCarInput, INPUT); //steering
      pinMode(9, OUTPUT);
      pinMode(10, OUTPUT);
      
      EscServo.attach(10); //11 speed pin
      SteerServo.attach(9); //steering
      SteerServo.write(straight);
      EscServo.writeMicroseconds(NEUTRAL);
  }
  
  void loop(){
      
      if(pulseIn(speedCarInput, HIGH, 27000) > 800 && pulseIn(steerCarInput, HIGH, 27000) > 800){
          joystick = true;
      }else{
          joystick = false;
      }
      
      if(joystick) {
          //Serial.println(controller);
          
          if(!controller){
              speedValue = NEUTRAL;
              right = RIGHT;
              left = LEFT;
              speedValueRev = NEUTRALREVERSE;
              steerValue = straight;
              
              SteerServo.write(straight);
              EscServo.writeMicroseconds(NEUTRAL);
              controller = true;
              
              delay(20);
              Serial.println("remotel");
          }
          
          
          Serial.println("Joystick");
          
          int i = pulseIn(speedCarInput, HIGH); //Get speed
          int j = pulseIn(steerCarInput, HIGH); //Get steering
          j = j - 275;
          
          /*
           * This delay makes sure that is doesn't receive too much information the servo
           */
          
          delay(50);
          
          if(i > FULL){
              i = FULL;
          }
          
          if(i < BREAK){
              i = BREAK;
          }
          
          
          EscServo.writeMicroseconds(i); //speed
          SteerServo.write(j); //steering
          
          
          
      } else if (!joystick) {
          
          if(controller){
              speedValue = NEUTRAL;
              right = RIGHT;
              left = LEFT;
              speedValueRev = NEUTRALREVERSE;
              steerValue = straight;
              controller = false;
              
              SteerServo.write(straight);
              EscServo.writeMicroseconds(NEUTRAL);
              delay(20);
              Serial.println("serial");
          }
          userInput = Serial.read();
          
          switch(userInput){
              case 'w':
                  if(speedValue < NEUTRAL){
                      EscServo.writeMicroseconds(NEUTRAL);
                      delay(100);
                      speedValueRev = NEUTRALREVERSE;
                      speedValue = NEUTRAL;
                  }
                  speedValue = speedValue+50;
                  break;
                  
              case 's':
                  speedValue = NEUTRAL;
                  steerValue = straight;
                  right = RIGHT;
                  left = LEFT;
                  speedValueRev = NEUTRALREVERSE;
                  break;
                  
              case 'd':
                  right = right + 10;
                  steerValue = right;
                  break;
                  
              case 'a':
                  
                  preset();
                  
                  //left  = left - 10;
                  //steerValue = left;
                  break;
                  
              case 'x':
                  if(speedValue > NEUTRAL){
                      EscServo.writeMicroseconds(NEUTRAL);
                      delay(100);
                      speedValue = NEUTRALREVERSE;
                  }
                  speedValueRev = speedValueRev - 50;
                  speedValue = speedValueRev;
                  break;
  
           
            
          }
          if (speedValue > FULL) {
              speedValue = FULL;
          }
          if (speedValue < BREAK){
              speedValue = BREAK;
          }
          if(right >= 120){
              steerValue = 120;
              right = 120;
          }
          if(left <= 0){
              steerValue = 0;
              left = 0;
          }
  
          
         
     

          String phrase = Serial.readString();
          String words[20];
          
          int steering = words[1].toInt();
          SteerServo.write(steering);

          int speeding = words[2].toInt();
          speeding = speeding * 50;
         
          EscServo.writeMicroseconds(speeding+1400);
          
          
          
          
          
          
      }
      
    
      
      
      delay(10);
      
  }
  
  
  void preset(){
  
    SteerServo.write(70);
    EscServo.writeMicroseconds(NEUTRAL);
    delay(200);
  
     SteerServo.write(90);
    EscServo.writeMicroseconds(1500);
    delay(200);
  
     SteerServo.write(110);
    EscServo.writeMicroseconds(1500);
    delay(200);
  
     SteerServo.write(90);
    EscServo.writeMicroseconds(1500);
    delay(200);
  
     SteerServo.write(70);
    EscServo.writeMicroseconds(NEUTRAL);
    delay(200);
  
     SteerServo.write(50);
    EscServo.writeMicroseconds(1500);
    delay(200);
  
     SteerServo.write(30);
    EscServo.writeMicroseconds(1500);
    delay(200);
  
     SteerServo.write(10);
    EscServo.writeMicroseconds(1500);
    delay(200);
  
     SteerServo.write(30);
    EscServo.writeMicroseconds(1500);
    delay(200);
     SteerServo.write(50);
    EscServo.writeMicroseconds(1500);
    delay(200);
  
     SteerServo.write(70);
    EscServo.writeMicroseconds(NEUTRAL);
    delay(200);
  
  }
