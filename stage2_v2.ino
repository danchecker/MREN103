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
#include <Servo.h>  // Includes the library

Servo myServoA;  // Makes a servo object to control servo A
Servo myServoB; //Makes a servo object to control servo B
Servo leftWheel;
Servo rightWheel;

int MOTOR_R = 3;        // right motor signal pin
int MOTOR_L = 4;        // left motor signal pin
int SHARP = A3;     // Sharp Sensor on Analog Pin 3

const int stopPulse = 149;  // stop speed for motors (default = 150))
const int delta =   10;       // pulse differential (default = 15)
const float offset = 20;       // offset, slows left wheel (default = 0)
int mv_value = 0;

// Pin Assignments
int GRN = 9;            // Green LED Pin
int YLW = 5;            // Yellow LED Pin
int RED = 10;           // Red LED Pin
int BUTTON = 7;         // Pushbutton Pin
int servoPinA = 11;     // Bucket servomotor #1 pin
int servoPinB = 12; // Bucket servomotor #2 pin
int myAngleA1 = 160; //160;    // initial angle, bucket lifts off ground if too high
int myAngleB1 = 80; //inital angle of bucket
int posA = myAngleA1;   // if set to 180, bucket lifts robot off of ground
int posB = myAngleB1;
int myAngleA2 = 120;// highest angle (lift), puts almost straight, set to 110,     still bent (i.e. not as high)
int myAngleB2 = 170;
int myAngleB3 = 100;


// Set-up routine
void setup() {

// Set-up LED pins as outputs
  pinMode(GRN, OUTPUT);
  pinMode(YLW, OUTPUT);
  pinMode(RED, OUTPUT);

// Set-up button pin as input
  pinMode(BUTTON, INPUT);

// Set-up servo motors
  myServoA.write(posA);         // Servo A starting position
  myServoA.attach(servoPinA);   // Attaches the servo to the servo object
  myServoB.write(posB);
  myServoB.attach(servoPinB);
//initialize motor control pins as servo pins
  leftWheel.attach(MOTOR_L);
  rightWheel.attach(MOTOR_R);

  turnOnLED(GRN);
}

// Main routine
void loop() {


  do {
      toggleLED(GRN);                     // Toggle green LED on
  } while(digitalRead(BUTTON)== LOW);     // Press button to start

  runMotors(-delta,-delta);
  delay(5000);
  runMotors(0,0);
  delay(1000);
           // A couple seconds to stand back

if(mv_value<470){


  runMotors(0,0);

  for (posA = myAngleA1; posA >= myAngleA2; posA--) { // Lift action
    myServoA.write(posA);
    delay(20);}
  for (posB = myAngleB1; posB <= myAngleB2; posB++){
      myServoB.write(posB);
      delay(20);
    }
  }


}

  runMotors(delta,-delta);
  delay(2000);
  runMotors(-delta,-delta);
  delay(4450);
  runMotors(0,0);
  delay(500);
  delay(500);

    }

  runMotors(delta,-delta);
  delay(2300);
  runMotors(0,0);

  for (posB = myAngleB2; posB >= myAngleB1; posB--) {
  myServoB.write(posB);
  delay(20);
    }

  delay(1000);
  for (posA = myAngleA2; posA <= myAngleA1; posA++) {  // Drop action
  myServoA.write(posA);
  delay(20);

  }
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
