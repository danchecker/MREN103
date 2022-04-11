/********************************************************
Group19Stage1Navigation (v1.4)
Original by H. Fernando, 25/04/2021
Modified by Naser Al-Obediat and Daniel Tcherkezian, 03/17/2022
This was our super successful stage 1 naviation code.
*********************************************************/
//include libraries
#include <Servo.h>
Servo myServoA;  // Makes a servo object to control servo A
Servo myServoB; //Makes a servo object to control servo B
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
int servoPinA = 11;     // Bucket servomotor #1 pin
int servoPinB = 12; // Bucket servomotor #2 pin
int myAngleA1 = 160; //160;    // initial angle, bucket lifts off ground if too high
int myAngleB1 = 80; //inital angle of bucket
int posA = myAngleA1;   // if set to 180, bucket lifts robot off of ground
int posB = myAngleB1;
int myAngleA2 = 95;// highest angle (lift), puts almost straight, set to 110,     still bent (i.e. not as high)
int myAngleB2 = 40;
int myAngleB3 = 120;
int myAngleA3 = 135;


// Delta = speed above (+) or below (-) stop speed (must be positive)
const int stopPulse = 149;  // stop speed for motors (default = 150))
const int delta =   10;       // pulse differential (default = 15)
const float offset = 0;       // offset, slows left wheel (default = 0)

const int LSENSOR = A2; // Left Sensor on Analog Pin 1
const int RSENSOR = A1; // Right Sensor on Analog Pin 2
//global variables
int lvalue = 0;  //left sensor value
int rvalue = 0;  //right sensor value
int value = 0;
int mv_value = 0;
int counter=0;

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
  // Set-up servo motors
  myServoA.write(posA);         // Servo A starting position
  myServoA.attach(servoPinA);   // Attaches the servo to the servo object
  myServoB.write(posB);
  myServoB.attach(servoPinB);
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
  else if(lvalue > 2000 && rvalue > 2000){
    turnOnLED(YLW);
    runMotors(0,0);
    delay(250);
    runMotors(delta, -delta);
    delay(150);
  }
  else if(mv_value > 1200 && counter==0){
    runMotors(0,0);
    delay(200);
    runMotors(-delta, -delta);
    delay(800);
    runMotors(-delta, delta);
    delay(800);
    while(lvalue < 2000){
      runMotors((1.2*-delta),delta);
      delay(250);
    }
    runMotors(-delta+offset,-delta);
    delay(1800);
    for (posB = myAngleB1; posB >= myAngleB2; posB--){
        myServoB.write(posB);
        delay(20);
      }
    delay (500);           // A couple seconds to stand back
    runMotors(0,0);
    delay(100);
    for (posA = myAngleA1; posA >= myAngleA3; posA--) { // Lift action
      myServoA.write(posA);
      delay(20);
    }
    runMotors(delta-offset, delta);
    delay(2500);
    counter++;
  }
  else if(mv_value > 1800 && counter==1){
    runMotors(0,0);
    delay(200);
    runMotors(-delta,-delta);
    delay(2000);
    runMotors(delta, delta);
    delay(500);
    runMotors(0,0);
    delay(1000);
    runMotors(-delta,delta);
    delay(1700);
    runMotors(-delta,-delta);
    delay(1900);
    runMotors(0,0);
    delay(1000);
    counter--;
    for (posB = myAngleB2; posB <= myAngleB3; posB++){
      myServoB.write(posB);
      delay(20); }
    for (posB = myAngleB1; posB >= myAngleB1; posB--){
        myServoB.write(posB);
        delay(20);}
    runMotors(delta,delta);
    delay(2500);
    for (posA = myAngleA3; posA <= myAngleA1; posA++) {  // Drop action
    myServoA.write(posA);
    delay(20);
    }
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
