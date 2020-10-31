#include <math.h>
#include <Wire.h> 
#define ADDRESS 0x08
int nD2 = 4; // Motor Tristate
int motor1Dir = 7; //Better realized as Voltage Sign
int motor2Dir = 8; //Better reawlized as Voltage Sign
int motor1Speed = 9; //Better realized as Voltage of M1
int motor2Speed = 10; //Better realized as Voltage of M2
int motor1A = 2;
int motor1B = 6;
int motor2A = 3;
int motor2B = 4;
int nSF = 12; //Status Flag Indicator

long oldPosition  = -999;
const float pi = 3.1415926535898;


// Desired linear velocity for the motors
double desiredVelocity = 1;
double KpV = .34651664;
double KiV = 0.00038853330385174;;
double KdV = 2.5256354;
double deltaError = 0;
double totalVelocityError = 0;
double previousVelocityError = 0;
double newVelocityError = 0;
double controlVelocitySignal = 0;

//variable to keep track of the counts for each wheel (encoder)
int countRight = 0;
int countLeft = 0;

//Counts per rotation
//The encoder has 1600 'clicks' or counts for one revolution
const int countsPerRot = 1600;

//Radius of Wheels (meters)
double radius = 0.073;

//Wheel Baseline (meters)
double d = 0.247;

// Desired linear position for the motors
double desiredAngleDegrees = 180;
double desiredAngleRadians = (desiredAngleDegrees / 360) * (2 * pi) ;
double desiredPosition = desiredAngleRadians * (.247/2) * .3048;
double KpD = 4.165965794;
double KiD = 2.652362528;
double totalError = 0;


double currentPosition = 0;

int data = 0;
int toSend = 0;





//angular postion of the Right wheel
double rightWheelPosNew = 0;
double rightWheelPosOld = 0;

//angular postion of the Left wheel
double leftWheelPosNew = 0;
double leftWheelPosOld = 0;

//Time Variables for Right Wheel - these keep track of when the POSITIONS are read.
unsigned long rightTimeNew;
unsigned long rightTimeOld;
double rightDeltaT;

//Time Variables for Left Wheel  - these keep track of when the POSITIONS are read.
unsigned long leftTimeNew;
unsigned long leftTimeOld;
double leftDeltaT;

//Velocity of Right Wheel
double velocityRight;

//Velocity of Left Wheel
double velocityLeft;

//Angular velocity of the Right Wheel
double angVelocRight;

//Angular Velocity of the Left Wheel
double angVelocLeft; 

//New and Old Position of Robot, x, y, and phi coordinates, also a new delta t
double wheelXPos_new;

double wheelXPos_old;

double wheelYPos_new;

double wheelYPos_old;

double phi_new;

double phi_old;

double deltaT;

double timeOld;

//void sendData(){
//Wire.write(data);
//}

int moveHappen = 0;


unsigned long currentTime = 0;
int samplePeriod = 10;

unsigned long oldTime;
void setup() {  

  //Right Wheel Enocoder Setup
  pinMode(motor1A, INPUT);
  pinMode(motor2B, INPUT);

  //Left Wheel Encoder Setup
  pinMode(motor2A, INPUT);
  pinMode(motor2B, INPUT);

  //Set Up Right Wheel Interrupt
  attachInterrupt(digitalPinToInterrupt(motor1A), rightWheelCount, CHANGE);
  //Set Up Left Wheel Interrupt
  attachInterrupt(digitalPinToInterrupt(motor2A), leftWheelCount, CHANGE);

  
  //Pins 4 Digital 4 - nD2 - triState
  pinMode(nD2, OUTPUT);
  
  //7,8 - Voltage Sign
  pinMode(motor1Dir, OUTPUT);
  pinMode(motor2Dir, OUTPUT);

  //9, 10 Motor Voltage
  pinMode(motor1Speed, OUTPUT);
  pinMode(motor2Speed, OUTPUT);
  
  //Pins 12 for input - nSF - Status flag indicator
  pinMode(nSF, INPUT);

  digitalWrite(nD2, HIGH);
  digitalWrite(motor1Dir, HIGH); //High for CW
  digitalWrite(motor2Dir, LOW); //Low for CCW

  //analogWrite(motor1Speed, HIGH);
  //analogWrite(motor2Speed, HIGH);

  Serial.begin(9600);

}

void loop() {

  double time = millis() / pow(10, 3);
  double Vbar_a = 8;
  desiredVelocity = rotationDistanceController(currentPosition);
  int motorSpeed = double((rotationController(-1*velocityRight) / 15.6)) * 255;
  if(motorSpeed > 255){
    motorSpeed = 255;
  }else if(motorSpeed < 0){
    motorSpeed = 0;
  }
  
  if(moveHappen == 0){
    delay(2000);
    int delayCount = (desiredAngleDegrees * 6.6071) + 57.5;
    digitalWrite(motor2Dir, HIGH); //High for CW
    analogWrite(motor2Speed, 100);
    analogWrite(motor1Speed, 100);
    delay(delayCount);
    analogWrite(motor2Speed, 0);
    analogWrite(motor1Speed, 0);
    moveHappen = 1;
  }else if(moveHappen == 1){
    delay(500);
    digitalWrite(motor2Dir, LOW); //High for CW
    analogWrite(motor2Speed, 97);
    analogWrite(motor1Speed, 100);
    delay(918);
    analogWrite(motor2Speed, 0);
    analogWrite(motor1Speed, 0);
    moveHappen = 999; 
  }
  
  
  


  double rhoDot = -1 * velocityRight;

  

  

 // Serial.print(time);
 // Serial.print("\t");
 // Serial.print(rhoDot);
//  Serial.print("\t");
  
//  Serial.print(velocityLeft);
//  Serial.print("\t");
//  Serial.print(angVelocRight);
//  Serial.print("\t");
//  Serial.print(velocityRight * -1);
//  Serial.print("\t");
//  Serial.print(angVelocRight * -1);
//  Serial.print("\t");
 // Serial.println();
 
}

//ISR for the right wheel/encoder
void rightWheelCount(){

rightTimeNew = micros();

//Current Right Wheel Position
rightWheelPosNew = (2*pi*(double)countRight) / countsPerRot;
currentPosition = (-1*rightWheelPosNew) * radius;


//Right wheel is moving CCW
if (digitalRead(motor1B) == digitalRead(motor1A)){
  countRight -= 1;
}

//Right wheel is moving CW
if (digitalRead(motor1B) != digitalRead(motor1A)){
  countRight += 1;
}

//Calcuate Time spent inside the ISR, and then calculate angular velocity and velocity

rightDeltaT = (double)rightTimeNew - (double)rightTimeOld;

if (rightDeltaT > 0.001){
  
 angVelocRight = ((rightWheelPosNew - rightWheelPosOld) / ((double)rightDeltaT) * pow(10,6));
 velocityRight = angVelocRight * radius;
 rightTimeOld = rightTimeNew;
 
 rightWheelPosOld = rightWheelPosNew;
  
}

}

////////////////////////////////////////////////////////////

//ISR for the left wheel/encoder
void leftWheelCount(){

leftTimeNew = micros();

//Current Left Wheel Position

leftWheelPosNew = (2*pi*(double)countLeft) / countsPerRot;

//Left wheel is moving CCW
if (digitalRead(motor2B) == digitalRead(motor2A)){
  countLeft -= 1;
}

//Left wheel is moving CW
if (digitalRead(motor2B) != digitalRead(motor2A)){
  countLeft += 1;
}

leftDeltaT = ((double)leftTimeNew - (double)leftTimeOld);

//Calcuate Time spent inside the ISR, and then calculate angular velocity and velocity

if (leftDeltaT > 0.001){
  angVelocLeft = (leftWheelPosNew - leftWheelPosOld) / ((double)leftDeltaT * pow(10,6));
  velocityLeft = angVelocLeft * radius;
  leftTimeOld = leftTimeNew;
  leftWheelPosOld = leftWheelPosNew;
}

}

double rotationController(double velocityRight){
  // New error is used in the proportional term
  float newVelocityError = desiredVelocity - velocityRight;
  totalVelocityError += newVelocityError;
  deltaError = newVelocityError - previousVelocityError;
  
  double derivativePart;
  if(rightDeltaT < .001){
    derivativePart = 0;
  }else{
    derivativePart = (KdV*deltaError) / ((rightDeltaT+10)/1000000);
  }
  double controlVelocitySignal = (KpV*newVelocityError) + (KiV*(rightDeltaT/1000000)*totalVelocityError) + derivativePart;
  return controlVelocitySignal;
  
  
}

double rotationDistanceController(double currentPosition){
  // New error is used in the proportional term
  float newDistanceError = (desiredPosition - currentPosition);
  // Total error is used in the integration term 
  totalError += newDistanceError;
  // Calculates the new value based on the current and desired position
  double controlSignal = (KpD*newDistanceError) + (KiD*(((double)rightTimeNew - (double)rightTimeOld)/1000000)*totalError);
  return controlSignal;
}
