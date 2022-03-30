/********************************************************
  Group42Lab4LineFollowOnOff (v1.4)

 
  Original by H. Fernando, 25/04/2021
  Modified by B. Surgenor, 19/03/2022


This is a basic line following code using On/Off control on the
differential drive robot and two line-following sensors.
*********************************************************/

 

#include <Servo.h>  
Servo leftWheel;
Servo rightWheel;

 

// Pin Assignments
int RED = 10; int GRN = 9; int YLW = 5;         
int BUTTON = 7;         //pushbutton Pin
int MOTOR_R = 3;       // right motor signal
int MOTOR_L = 4;       // left motor signal
int RSENSOR = A1; // Right Sensor on Analog Pin 1
int LSENSOR = A2; // Left Sensor on Analog Pin 2

 


//int MOTOR_R = 3;        // right motor signal pin 
//int MOTOR_L = 4;        // left motor signal pin 

 


// global constants
const int STOP_SPEED = 148;  //stop speed for motors (nominal = 150)
const int DELTA = 12;        //DELTA is nominal speed
const int DIFFER = 8;        //DIFFER is speed differential 
boolean leftLineDetected = false;
boolean rightLineDetected = false;
int lvalue = 0;  //left sensor value 
int rvalue = 0;  //right sensor value 

 

                        
// Set-up Routine
void setup() {
                
// Initialize pins and wheels
  pinMode(GRN, OUTPUT); pinMode(YLW, OUTPUT); pinMode(RED, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(LSENSOR, INPUT); pinMode(RSENSOR, INPUT);
  leftWheel.attach(MOTOR_L);
  rightWheel.attach(MOTOR_R);

 

// Initialize serial and motors
  Serial.begin(9600);   // default 9600, or try 115200

 


  runMotors(0,0);      // make sure motors stopped
  do {
    toggleLED(GRN);         //motors stopped, Green LED flashing
  } while(digitalRead(BUTTON)== LOW);
}

 


// Main Routine
void loop() {

 

   
      
      //read the sensor value 
      lvalue = analogRead(LSENSOR); 
      rvalue = analogRead(RSENSOR); 
      
      //map the values into millivolts (assuming 3000 mV reference voltage) 
      lvalue = map(lvalue,0,1023,0,3000); 
      rvalue = map(rvalue,0,1023,0,3000);

 

      

 

      
  //reset LEDs
  digitalWrite(GRN, LOW); digitalWrite(RED, LOW); digitalWrite(YLW, LOW);
  leftLineDetected = isLine(LSENSOR);
  rightLineDetected = isLine(RSENSOR);
  
  //Condition Red: left sensor on line, right not on line, turn left
  if(leftLineDetected && !rightLineDetected){
    turnOnLED(RED);
    runMotors(DELTA-DIFFER, DELTA); }
    
  //Condition Yellow: right sensor on line, left not on line, turn right
   else if(!leftLineDetected && rightLineDetected){
    turnOnLED(YLW);
    runMotors(DELTA, DELTA-DIFFER); }
    
  //Condition Green: both sensors not on line, go straight
   else if(!leftLineDetected && !rightLineDetected){
     turnOnLED(GRN);
     runMotors(DELTA, DELTA); }

 


  //Condition if both sensors are on the line 
   else if(leftLineDetected && rightLineDetected){
    turnOnLED(YLW);
    delay(20); 
    runMotors(-DELTA, -DELTA);
    delay(100); 
    runMotors(DELTA, 0); 
    delay(250);
   }
   
     
  // Condition Green/Red/Yellow: both sensors on line, stop
  /*else{ 
    turnOnLED(GRN);turnOnLED(RED); turnOnLED(YLW);
    runMotors(0, 0);  
    }    */
}   

 

//********** Functions (subroutines) ******************

 

boolean isLine(int sensor){
  int THRESHOLD = 1100;  //white/black threshold in mV
  int value = analogRead(sensor);
  int valueMv = map(value,0,1023,0,3300);
  if (valueMv > THRESHOLD){
    return true;
  }
  else{
    return false;
  }
}
 

void runMotors(int delta_L, int delta_R)
{
  int pulse_L = (STOP_SPEED + delta_L)*10;   
  int pulse_R = (STOP_SPEED + delta_R)*10;
  leftWheel.writeMicroseconds(pulse_L);
  rightWheel.writeMicroseconds(pulse_R); 
}

 

// Turn on a single LED and turn others off
void turnOnLED(int COLOUR)
{
  digitalWrite(GRN, LOW);
  digitalWrite(YLW, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(COLOUR, HIGH);
}

 

//Toggle an LED on/off
void toggleLED(int colour){
  digitalWrite(colour, HIGH);
  delay(250);
  digitalWrite(colour, LOW);
  delay(250); 
}
