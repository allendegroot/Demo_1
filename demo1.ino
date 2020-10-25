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

int data = 0;
int toSend = 0;


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
  double Vbar_a = 7.6;

  int motorSpeed = double((Vbar_a / 15.2)) * 255;
  
  analogWrite(motor2Speed, motorSpeed);
  analogWrite(motor1Speed, motorSpeed);
  


  double rhoDot = velocityRight;

  

  

  Serial.print(time);
  Serial.print("\t");
  Serial.print(rhoDot);
  
//  Serial.print(velocityLeft);
//  Serial.print("\t");
//  Serial.print(angVelocLeft);
//  Serial.print("\t");
//  Serial.print(velocityRight * -1);
//  Serial.print("\t");
//  Serial.print(angVelocRight * -1);
//  Serial.print("\t");
  Serial.println();
 
}

//ISR for the right wheel/encoder
void rightWheelCount(){

rightTimeNew = micros();

//Current Right Wheel Position
rightWheelPosNew = (2*pi*(double)countRight) / countsPerRot;


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

double linearController(double velocityRight){
  
}
 
