#ifndef Motion_h
#define Motion_h

#include "MyPID.h"
#include "DualVNH5019MotorShield.h"
#include "math.h"

#define Pi 3.1428

#define M1k  0.424535
#define M1ts  0.002
#define M1td  0.05

#define M2k  0.447004
#define M2ts  0.0036
#define M2td  0.0687
#define errorCtrl  2.0

/* Class to control all motion with respect to bot inclusive of 
1. Move Forward
2. Move Backward
3. Turn Right
4. Turn Left */
class Motion {
  private:
    float rpm;
    float setInitialMotorSpeed1, setInitialMotorSpeed2;
    int pinM1, pinM2;
    MyPID *motorOne, *motorTwo;
    DualVNH5019MotorShield md;
    float runningTime(float distance);
    float rotateTime(float angle);
    void startMotion(bool forwardMotorOne, bool forwardMotorTwo, long totalTime);
    void stopMotion();

  public:
    Motion(float _rpm, float _setInitialMotorSpeed1, int _pinM1, float _setInitialMotorSpeed2, int _pinM2, DualVNH5019MotorShield _md);
    void moveForward(float distance);
    void moveBackward(float distance);
    void turnLeft(float angle);
    void turnRight(float angle);

};

/* Constructor to itialialise all private variable of object
motorOne and motorTwo are PID values or rather MyPID object required to update speeds of the motors with respect to current speed*/
Motion::Motion(float _rpm, float _setInitialMotorSpeed1, int _pinM1, float _setInitialMotorSpeed2, int _pinM2, DualVNH5019MotorShield _md) {

  motorOne = new MyPID(M1k, M1td, M1ts, errorCtrl);
  motorTwo = new MyPID(M2k, M2td, M2ts, errorCtrl);

  rpm = _rpm;
  md = _md;

  pinM1 = _pinM1;
  pinM2 = _pinM2;

  setInitialMotorSpeed1 = _setInitialMotorSpeed1;
  setInitialMotorSpeed2 = _setInitialMotorSpeed2;
}

/* Calculates time required to move forward or backward by the bot in milliseconds. 
Note: in order to change forward travelling distance, simply change wheelRadius and offset */
float Motion::runningTime(float distance) {
  float wheelRadius = 2.95;//2.92
  float offset = 0;
  
  float Speed = (rpm * 2 * Pi * wheelRadius) / 60000.0; // Speed in cm/millisecond
  float runTime = distance / Speed;
  
  return runTime + offset;
}

/* Calculates time required rotate right or left by the bot in milliseconds. 
Note: in order to change rotate angle, simply change radius (Bot Radius), wheelRadius and offset */
float Motion::rotateTime(float angle) {
  float radius = 8.25;//.52;
  float wheelRadius = 2.89;
  float offset = 5;
  
  float distance = (2 * Pi * radius * angle) / 360.0;   // 2*pi*R*(angleOfRotation/360)
  float Speed = (rpm * 2 * Pi * wheelRadius) / 60000.0; // Speed in cm/millisecond
  float rotateTime = distance / Speed;
  
  return rotateTime + offset;
}

/* Apply breaks to bot motors */
void Motion::stopMotion() {
  md.setM1Brake(400);
  md.setM2Brake(367);
}

/* Function to set motor speed based on required direction and run for required time in microseconds (for precision)
i.e. function to handles all motion of bot eg. move Forward, turn Left, etc.
forwardMotorOne is set to true implies motorOne will rotate such that bot moves forwards
whereas if forwardMotorOne is set to false implies motorOne will rotate such that bot moves backward*/
void Motion::startMotion(bool forwardMotorOne, bool forwardMotorTwo, long totalTime) {

  //    Serial.print("Run Time : ");
  //    Serial.println(totalTime);

  long count = 0;
  float setMotorSpeed1 = forwardMotorOne ? setInitialMotorSpeed1 : -setInitialMotorSpeed1;
  float setMotorSpeed2 = forwardMotorTwo ? setInitialMotorSpeed2 : -setInitialMotorSpeed2;

  motorOne->MyPIDReboot();
  motorTwo->MyPIDReboot();

  long startTime = micros();
  long stopTime = micros();

  md.setSpeeds(setMotorSpeed1, setMotorSpeed2);

  while (stopTime - startTime < totalTime) {    //Keep track of time elapsed

    if (count == 2000) {  // Update set speed every 2000 counts (in order not to over or under sample)

      setMotorSpeed1 = motorOne->setWheelSpeed(setMotorSpeed1, pinM1, rpm, forwardMotorOne);  // Find new set speed of motor 1
      setMotorSpeed2 = motorTwo->setWheelSpeed(setMotorSpeed2, pinM2, rpm, forwardMotorTwo);  // Find new set speed of motor 2

      md.setM1Speed(setMotorSpeed1);
      md.setM2Speed(setMotorSpeed2);

      count = 0;
    }

    count++;
    stopTime = micros();
  }

  stopMotion();

}

void Motion::moveForward(float distance) {

  long totalTime  = round(runningTime(distance) * 1000);
  startMotion(true, true, totalTime );
}

void Motion::moveBackward(float distance) {

  long totalTime  = round(runningTime(distance) * 1000);
  startMotion(false, false, totalTime );
}

void Motion::turnLeft(float angle) {

  long totalTime = round(rotateTime(angle) * 1000);
  startMotion(true, false, totalTime );
}

void Motion::turnRight(float angle) {

  long totalTime  = round(rotateTime(angle) * 1000);
  startMotion(false, true, totalTime );
}

#endif
