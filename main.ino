//ENGR 290 Self-Driving Hovercraft Code
//Submitted: April 18th, 2023
//Team 9

//Download library MPU6050 by Electronic Cats
//Trig for forward US is PB3
//Trig for side US is PB5
//Echo for forward US is PD2
//Echo for side sensor is PD3
//Servo attached to PD6

#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
#include <Wire.h>

Servo myservo;
MPU6050 mpu;

#define LIFT_FAN PD7
#define THRUST_FAN PD5
#define FS_ADDR 0x01 //DEPRECATED?
#define INT_PIN 2
#define THRESH 80 //deg              TEST1 
#define TURN_DIST_FRONT 50 //cm     //IDEAL:50 DO NOT SET PAST 50 OR PERISH
#define TURN_DIST_SIDE 40 //cm      //IDEAL:40 DO NOT SET PAST 60 OR PERISH AGAIN
#define TRIG_PIN_SIDE 13   //PB5
#define TRIG_PIN_FRONT 11  //PB3
#define ECHO_PIN_FRONT 2   //PD2
#define ECHO_PIN_SIDE 3    //PD3
#define SERVO_PIN PD6
#define N 10 //dimless Number of samples to take to get an average US reading
#define TURN_TIMEOUT 2.0 //s time from a completed turn until the HC is allowed to check for the next turn opportunity
#define TURN_RANGE 60 //deg from forwards for turn fan deflection. larger values lead to a bigger snap.
#define TURN_FRONT_RANGE 20 //cm TURN_DIST_FRONT +- TURN_FRONT_RANGE is the range in cm that the HC is allowed to turn in
#define TURN_SIDE_RANGE 10 //cm DEPRECATED
#define TURN_RAMP_STEP 300 //ms time between steps in the leeway section 200?
#define PORT_OFFSET 20 //deg makes it snap harder when turning to PORT - cancels transverse thrust and angular accel from fans making craft drift and rotate to STBD
#define LIFT_FAN_PWR 245 //dimless [0, 255] DEPRECATED
#define TURN_PWR 200 //dimless [0, 255] 
#define SPINDOWN_TIME 1.5 //s time it takes to deflate the skirt and come to a complete stop
#define HARD_DELAY 50 //ms time that the end of the leeway step lasts 100?
#define PORT_ADD_DELAY 0 //ms - see "PORT_OFFSET" for justification DEPRECATED
#define DRIFT_OFFSET 50 //ms - see "PORT_OFFSET" for justification, compensates for IMU drift leading into the first turn
#define SNAP false
#define PORT_SPECIAL 70 //deg distance to snap when making a port turn. see "PORT_OFFSET" for justification

int ThrustFanSpeed,LiftFanSpeed; //DEPRECATED
int InitialYaw; //DEPRECATED
int counter = 0; //DEPRECATED
 
int pos = 0;    // variable to store the servo position

//All related to the IMU library used
uint16_t packetSize;
uint8_t fifoBuffer[64];
float ypr[3];
Quaternion quat;
VectorFloat grav;

float tempAngle = 0; //DEPRECATED
int forwardAngle = 0; //Represents the forwards direction
int servoRebase = 0; //DEPRECATED
float yaw = 0; //DEPRECATED
int y, deltaTheta; //y is the current frame's yaw reading, deltaTheta is the frame's unsigned angular difference from forwardAngle
bool dmpReady = false; //Related to IMU lib
bool firstRun = true; //Used to calibrate the forwardDirection
bool d = false; //DEPRECATED ?
bool canCheck = true; //Controls if the HC is allowed to check for a turn via a timer
bool firstTurnCheck = true; //True for the first time the HC turns, not deprecated but useless outside testing
bool stopTurnRamp = false; //Stops the turn ramping
bool startingTurn = true; //DEPRECATED
bool firstTurn = true; //See "firstTurnCheck"
unsigned long t; //DEPRECATED
float thetaDiff = 0; //DEPRECATED

long duration=0; //Used to get US readings
long dist=0; //Used to get US readings

//Setup queue for front and side sensor data
long front_measurements[N];
long side_measurements[N];

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
 
void setup() {   
  //Serial.begin(38400);
  pinMode(INT_PIN, INPUT);
  myservo.attach(SERVO_PIN);  
  myservo.write(90);
  ThrustFanSpeed=200; //DEPRECATED
  analogWrite(LIFT_FAN, 255);
  analogWrite(THRUST_FAN, 255);
  
  Wire.begin();
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), dmpDataReady, RISING);
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();

  pinMode(TRIG_PIN_FRONT, OUTPUT);
  pinMode(TRIG_PIN_SIDE, OUTPUT);
  pinMode(ECHO_PIN_FRONT, INPUT);
  pinMode(ECHO_PIN_SIDE, INPUT);
  delay(1000); // IMPORTANT!
}
 
void loop() {
  //Get the frame's yaw value
  y = pollYaw();

  //Calibrate the IMU and get forwardDirection
  if (firstRun)
  {
    forwardAngle = y;
    thetaDiff = y; //DEPRECATED
    firstRun = false;
  }

  //Check the differnce between "forward" and present
  deltaTheta = (forwardAngle - y + 540) % 360 - 180;
  deltaTheta = constrain(deltaTheta, -90, 90);
  myservo.write(deltaTheta + 90);

  //Attempt to turn every TURN_TIMEOUT seconds
  if (pollTurnConditions(d) && canCheck)
  {
    turn(d);
    t = millis();
  }
  if (((millis() - t) > TURN_TIMEOUT * 1000) || firstTurnCheck)
  {
    canCheck = true;
  }
  else
  {
    canCheck = false;
  }
}

//@param dir False = port, True = starboard
void turn(bool dir)
{
  while (true)
  {
    //Lower fan power to make turn more gentle
    analogWrite(THRUST_FAN, TURN_PWR);

    //Get new dTheta for the reversed "forward" direction
    y = pollYaw();
    deltaTheta = (forwardAngle - y + 540 + 180) % 360 - 180;

    //END OF TURN
    if (abs(deltaTheta) < THRESH)
    {
      END:
      //Stop the HC to kill angular momentum
      analogWrite(LIFT_FAN, 0);
      delay(SPINDOWN_TIME * 1000);
      analogWrite(LIFT_FAN, 255);
      myservo.write(90);
      forwardAngle = (forwardAngle + 180) % 360;
      analogWrite(THRUST_FAN, 255);
      firstTurnCheck = false;
      stopTurnRamp = false;
      startingTurn = true;
      firstTurn = false;
      return;
    }
    //Ramp all the way
    if (!stopTurnRamp)
    {
      //Leeway section
      if (!dir)
      {
        //Snap me on left turn only, false = left maybe
        myservo.write(90 - PORT_SPECIAL);
        delay(1000); //Enough delay for the fan to swing around and for the turn
        //This is actually, unironcally really bad but I'm really tired by now
        goto END;
      }
      for (int i = 0; i < TURN_RANGE / 10; i++)
      {
        myservo.write(dir ? 180 - (10 * i) : 10 * i);
        delay(TURN_RAMP_STEP);
      }
      //Correct for IMU drift during calibration
      //After the first turn, it's kinda the wild west so this kind of precision is redundant
      if (firstTurn)
      {
        delay(DRIFT_OFFSET);
      }
      if (SNAP)
      {
        delay(HARD_DELAY);
      }
      
      stopTurnRamp = true;
      //Snap to finish turn after making leeway by ramp
      //This also finishes off the turn with a very aggressive angular accel.
      if (SNAP)
      {
        myservo.write(dir ? 180 : 0);
      }
      // ?
    }
    //New method? try creating a ton of leeway to get the HC flush into a corner and then snap 180 in the opposite direction. dTheta should update accordingly?
    //myservo.write(dir ? 100 : 80);
    //delay(5000);
    //myservo.write(dir ? 0 : 180);
    //delay(100);
    //Toby Smith writes worst code ever, asked to leave ENGR 290 team
    //goto END;
  }
}

bool pollTurnConditions(bool &dir)
{
  //if can turn, return true and set dir to turning direction
  int fd = avgPollFwd();
  int sd = avgPollSide();
  
  if (fd < TURN_DIST_FRONT && fd > TURN_DIST_FRONT - TURN_FRONT_RANGE)
  {
    if (sd < TURN_DIST_SIDE && sd > 0)
    {
      //Turn away from sensor (to STBD)
      dir = true;
      return true;
    }
    //Turn toward sensor (to PORT)
    dir = false;
    return true;
  }
  return false;
}

//Get the current yaw reading in deg
int pollYaw()
{
  mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
  mpu.dmpGetQuaternion(&quat, fifoBuffer);
  mpu.dmpGetGravity(&grav, &quat);
  mpu.dmpGetYawPitchRoll(ypr, &quat, &grav);
  yaw = ypr[0] * 180 / 3.141592 + 180;
  return (int)yaw;
}

//Return the Front US reading in cm
int pollFwd()
{
  digitalWrite(TRIG_PIN_FRONT,LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN_FRONT,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_FRONT,LOW);
  duration=pulseIn(ECHO_PIN_FRONT,HIGH);
  //If not too far for a good read
  if (duration<13000){    
  dist = duration * 0.034 / 2;
  }
  else return 0;
  return dist;
}

//Return the Side US reading in cm
int pollSide()
{
  digitalWrite(TRIG_PIN_SIDE,LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_SIDE,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_SIDE,LOW);
  duration=pulseIn(ECHO_PIN_SIDE,HIGH);
  //If not too far for a good read
  if (duration<13000){
  dist = duration * 0.034 / 2;
  }
  else return 0;
  //Serial.print("Side:  ");
  //Serial.println(dist);
  return dist;
}

//Get the average of N reads fromn FWD sensor
int avgPollFwd(){
  int sum = 0;
  int temp = 0;
  for (int i = 0; i < N; i++)
  {
    temp = pollFwd();
    sum = sum + temp;
    delay(1);
  }
  return sum / N;

}

//Get the average of N reads from the SIDE sensor
int avgPollSide(){
  int sum = 0;
  int temp = 0;
  for (int i = 0; i < N; i++)
  {
    temp = pollSide();
    sum = sum + temp;
    delay(1);
  }
  return sum / N;
  
}

//DEPRECATED
   void unstuck() {
     Serial.println("WE ARE STUCK!");
   }
