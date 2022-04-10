//Open Loop Stage 2 code///
/***************************************************
Group19Lab6LiftandCurl (v1.2)

  Original by D. Ells 27/8/2021
  Revised by B. Surgenor, 25/04/2021
  Modified by Naser Al-Obediat and Daniel Tcherkezian, 23/03/2022


Start with bucket resting (at an angle) touching ground, drive
link parallel to the ground. Lift the bucket, uncurl, recurl and then lower the bucket once.
runMotors(delta,-delta);
delay(1800);
*****************************************************/
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
int myAngleB1 = 75; //inital angle of bucket
int posA = myAngleA1;   // if set to 180, bucket lifts robot off of ground
int posB = myAngleB1;
int myAngleA2 = 110;// highest angle (lift), puts almost straight, set to 110,     still bent (i.e. not as high)
int myAngleB2 = 40;
int myAngleB3 = 120;
int myAngleA3 = 135;


// Delta = speed above (+) or below (-) stop speed (must be positive)
const int stopPulse = 149;  // stop speed for motors (default = 150))
const int delta =   15;       // pulse differential (default = 15)
const float offset = 0;       // offset, slows left wheel (default = 0)
const int LSENSOR = A2; // Left Sensor on Analog Pin 1
const int RSENSOR = A1; // Right Sensor on Analog Pin 2

//global variables
int lvalue = 0;  //left sensor value
int rvalue = 0;  //right sensor value
int value = 0;
int mv_value = 0;
const boolean PLOT = false;  //true=plot sensor reading; false=serial monitor output.

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

}// Main routine
void loop() {
  //flash green LED
  do{
             digitalWrite(GRN, HIGH);
             delay(125);
             digitalWrite(GRN, LOW);
             delay(125);
          }while(digitalRead(BUTTON) == LOW);

runMotors(-delta+offset,-delta);
delay(4000);
  for (posB = myAngleB1; posB >= myAngleB2; posB--){
      myServoB.write(posB);
      delay(20);
    }
  delay (500);           // A couple seconds to stand back
  for (posA = myAngleA1; posA >= myAngleA3; posA--) { // Lift action
    myServoA.write(posA);
    delay(20);
  }
  runMotors(-delta, -delta);
  delay(200);
  runMotors(delta,-delta);
  delay(1300);
  runMotors(-delta-offset,-delta);
  delay(4000);
  runMotors(0,0);
  delay(1000);
  //runMotors(delta,-delta);
  //delay(1600);
  //runMotors(0, 0);
  for (posB = myAngleB2; posB <= myAngleB3; posB++){
      myServoB.write(posB);
      delay(20);
    }
  delay(250);
  runMotors(delta, -delta);
  delay(1600);
  runMotors(0,0);
  for (posA = myAngleA3; posA <= myAngleA1; posA++) {  // Drop action
    myServoA.write(posA);
    delay(20);
  }
  delay(500);
  for (posB = myAngleB1; posB >= myAngleB1; posB--){
      myServoB.write(posB);
      delay(20);}
}
//********************* Functions (subroutines)*****************
//Toggle an LED on/off
void toggleLED(int colour){
  digitalWrite(colour, HIGH);
  delay(250);
  digitalWrite(colour, LOW);
  delay(250);
}


 // run robot wheels
void runMotors(int deltaL, int deltaR)
{
  int pulseL = (stopPulse + deltaL)*10;    //length of pulse in microseconds
  int pulseR = (stopPulse + deltaR)*10;
  leftWheel.writeMicroseconds(pulseL);
  rightWheel.writeMicroseconds(pulseR);
}


// Turn on a single LED and turn others off
void turnOnLED(int COLOUR) {
  digitalWrite(GRN, LOW);
  digitalWrite(YLW, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(COLOUR, HIGH);
}
