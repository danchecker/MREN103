/********************************************************
Group14Lab4LineFollowBetter (v1.4)

Original by H. Fernando, 25/04/2021
Modified by Naser Al-Obediat and Daniel Tcherkezian, 03/17/2022

This code is designed to have robot follow a black line of width 1.9cm on a white surface.

*********************************************************/

//include libraries
#include <Servo.h>

//call upon both wheels
Servo leftWheel;
Servo rightWheel;



// Pin Assignments
const int RED = 10;           //red LED Pin
const int GRN = 9;           //green LED Pin
const int YLW = 5;           //yellow LED Pin
const int BUTTON = 7;        //pushbutton Pin
int MOTOR_R = 3;        // right motor signal pin
int MOTOR_L = 4;        // left motor signal pin
int SHARP = A3;     // Sharp Sensor on Analog Pin 3

// Delta = speed above (+) or below (-) stop speed (must be positive)

const int stopPulse = 149;  // stop speed for motors (default = 150))
const int delta =   12;       // pulse differential (default = 15)
const float offset = 0;       // offset, slows left wheel (default = 0)



const int LSENSOR = A2; // Left Sensor on Analog Pin 1
const int RSENSOR = A1; // Right Sensor on Analog Pin 2
const boolean PLOT = false;  //true=plot sensor reading; false=serial monitor output.



//global variables

int lvalue = 0;  //left sensor value
int rvalue = 0;  //right sensor value
int value = 0;
int mv_value = 0;


// Set-up Routine

void setup() {



// Initialize led pins as outputs.

  pinMode(GRN, OUTPUT);
  pinMode(YLW, OUTPUT);
  pinMode(RED, OUTPUT);



// Initialize button pins as inputs

  pinMode(BUTTON, INPUT);

//initialize motor control pins as servo pins

  leftWheel.attach(MOTOR_L);
  rightWheel.attach(MOTOR_R);

// Initialize line following sensor pins as inputs

  pinMode(LSENSOR, INPUT);
  pinMode(RSENSOR, INPUT);
  pinMode(SHARP, INPUT);

  //flash green LED
  do{

             digitalWrite(GRN, HIGH);
             delay(125);
             digitalWrite(GRN, LOW);
             delay(125);
          }while(digitalRead(BUTTON) == LOW);

}



// Main Routine

void loop() {

      //read the sensor value
      lvalue = analogRead(LSENSOR);
      rvalue = analogRead(RSENSOR);

      //map the values into millivolts (assuming 3000 mV reference voltage)
      lvalue = map(lvalue,0,1023,0,4000);
      rvalue = map(rvalue,0,1023,0,4000);
      value = analogRead(SHARP);
      mv_value = map(value,0,1023,0,3300); //convert AtoD count to millivolts

      if(lvalue < 2000 && rvalue > 2000){
        digitalWrite(YLW, HIGH);
        digitalWrite(GRN, LOW);
        digitalWrite(RED, LOW);
        runMotors(delta, 0);
      }
      else if(lvalue > 2000 && rvalue < 2000){
        digitalWrite(RED, HIGH);
        digitalWrite(YLW, LOW);
        digitalWrite(GRN, LOW);
        runMotors(0, delta);
      }
      else if(mv_value > 1100){
        runMotors(0, 0);
      }
      else if(lvalue > 2000 && rvalue > 2000){
        turnOnLED(YLW);

        // runMotors(-DELTA, -DELTA);
        // delay(100);

        runMotors(0,0);
        delay(250);
        runMotors(DELTA, -DELTA);
        delay(250);
      }

      else{
        digitalWrite(GRN, HIGH);
        digitalWrite(RED, LOW);
        digitalWrite(YLW, LOW);
        runMotors(delta,delta);
      }

}



//********** Functions (subroutines) ******************
// run robot wheels
void runMotors(int deltaL, int deltaR)
{
  int pulseL = (stopPulse + deltaL)*10;    //length of pulse in microseconds
  int pulseR = (stopPulse + deltaR)*10;
  leftWheel.writeMicroseconds(pulseL);
  rightWheel.writeMicroseconds(pulseR);
}



// Turn on a single LED and turn others off
void turnOnLED(int COLOUR)
{
  digitalWrite(GRN, LOW);
  digitalWrite(YLW, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(COLOUR, HIGH);
}
