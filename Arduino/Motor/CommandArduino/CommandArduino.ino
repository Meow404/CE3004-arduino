#include "DualVNH5019MotorShield.h"
#include "Sensors.h"
#include "math.h"

DualVNH5019MotorShield md;

const long Pi = 3.1328;
long E2A = 3;
long E2B = 5;
long E1A = 11;
long E1B = 13;
float M1k = 0.371285;
double M1ts = 0.00375;//78,0.00565
double M1td = 0.057;//0.047;//0.07625;
float M1k1, M1k2, M1k3;
float M2k = 0.401039;
float M2ts = 0.0043;//77;
float M2td = 0.0585;
float M2k1, M2k2, M2k3;
bool update1, update2;
String sensorData;
float errorCtrl = 2;
float error1[3] = {0, 0, 0};
float error2[3] = {0, 0, 0};
float setSpeed1, setSpeed2, angle = 90;
float rpm1 = 105, rpm2 = 105;
long startTime, stopTime;
float calibrationOffset = -30;
float turnOffset = 25;
#define ledPin 13
int timer1_counter;
float dist[3] = {30, 30, 30};


void setup() {
  Serial.begin(115200);
  //  Serial.println("Dual VNH5019 Motor Shield");
  pinMode(E1A, INPUT);
  pinMode(E1B, INPUT);
  pinMode(E2A, INPUT);
  pinMode(E2B, INPUT);
  PIDGearController1();
  PIDGearController2();

  //  Serial.print("Speed set : ");
  //  Serial.println(rpm1);

  setSpeed1 = 0;
  setSpeed2 = 0;

  md.init();           // enable all interrupts
}
//
//void loop() {
//  avoidObstacle(200);
////  moveForward(10);
////  delay(4000);
////  turnRight(180);
////  delay(4000);
////  moveForward(120);
////  delay(4000);
////  turnLeft(180);
////  delay(4000);
//}


void loop() {

  while (Serial.available() > 0) //RPi to Arduino
  {
    String input = Serial.readStringUntil('~'); //use this if char reading fails
    String firstVal = input;
    int secondVal = 10;

    for (int i = 0; i < input.length(); i++) {
      if (input.substring(i, i + 1) == ":") {
        firstVal = input.substring(0, i);
        secondVal = input.substring(i + 1).toInt();
        break;
      }
    }

    char instructions = firstVal[0];
    switch (instructions) {
      //botstart88
      case 'S':
        //calibrateLeftSide();
        sensorData = returnSensorData();
        Serial.println("X_SENDATA" + sensorData);
        break;
      //moveforward
      case '0':
        //Serial.println("X Bot forward");
        moveForward(secondVal);
        sensorData = returnSensorData();
        Serial.println("X_SENDATA" + sensorData);
        break;
      //turn 90 deg left
      case '1':
        //Serial.println("X Bot turn left");
        turnLeft(90);
        sensorData = returnSensorData();
        Serial.println("X_SENDATA" + sensorData);
        break;
      //turn 90 deg right
      case '2':
        //Serial.println("X Bot turn right");
        turnRight(90);
        sensorData = returnSensorData();
        Serial.println("X_SENDATA" + sensorData);
        break;
      //bot backward
      case '3':
        //Serial.println("X Bot stopped");
        moveBackward(secondVal);
        sensorData = returnSensorData();
        Serial.println("X_SENDATA" + sensorData);
        break;
      //turn 180 deg leftt
      case '4':
        //Serial.println("X Bot turn 180 deg left");
        turnLeft(180);
        sensorData = returnSensorData();
        Serial.println("X_SENDATA" + sensorData);
        break;
      //turn 180 deg right
      case '5':
        //Serial.println("X Bot turn 180 deg right");
        turnRight(180);
        sensorData = returnSensorData();
        Serial.println("X_SENDATA" + sensorData);
        break;
      //botstop
      case 'E':
        //Serial.println("X Bot stopped");
        md.setM1Speed(0);
        md.setM2Speed(0);
        sensorData = returnSensorData();
        Serial.println("X_SENDATA" + sensorData);
        break;
      //calibration
      case 'C':
        Serial.println("FYI! Bot calibrating");
        if (secondVal == 0)
          calibrateDistance();
        else if (secondVal == 1)
          calibrateLeftSide();
        Serial.println("X_CALIBRATIONDONE");
        break;
    }
  }
}


void calibrateLeftSide() {
  float frontSensor = SR4.getDistance(false) + 2;
  float backDistance = SR5.getDistance(false) + 2;
  while (fabs(frontSensor - backDistance) != 5) {
    if (frontSensor - backDistance > 5) {
      md.setSpeeds(350, -350);
      delay(10);
      md.setM1Brake(400);
      md.setM2Brake(400);
      delay(30);
    }
    if (frontSensor - backDistance < 5) {
      md.setSpeeds(-350, 350);
      delay(10);
      md.setM2Brake(400);
      md.setM1Brake(400);
      delay(30);
    }
    frontSensor = SR4.getDistance(false) + 2;
    backDistance = SR5.getDistance(false) + 2;
  }
  md.setSpeeds(-350, 350);
      delay(10);
      md.setM2Brake(400);
      md.setM1Brake(400);
      delay(30);
}
//
//void calibrateRightSide() {
//  float distance[3] = {0, 0, 0};
//  distance[1] = LR1.getDistance(false) + 10;
//  md.setSpeeds(350, -350);
//  delay(25);
//  md.setBrakes(400);
//  delay(50);
//  float distance[0] = LR1.getDistance(false) + 10;
//  md.setSpeeds(350, -350);
//  delay(25);
//  md.setBrakes(400);
//  delay(50);
//  float distance[3] = LR1.getDistance(false) + 10;
//  while (fabs(frontSensor - backDistance) != 6) {
//    if (frontSensor - backDistance > 6) {
//      md.setSpeeds(350, -350);
//      delay(25);
//      md.setBrakes(400);
//      delay(50);
//    }
//    if (frontSensor - backDistance < 6) {
//      md.setSpeeds(-350, 350);
//      delay(25);
//      md.setM2Brake(400);
//      md.setM1Brake(400);
//      delay(50);
//    }
//    frontSensor = SR4.getDistance(false) + 2;
//    backDistance = SR5.getDistance(false) + 2;
//  }
//}

void calibrateFront() {
  float rightDistance = SR1.getDistance(false) + 2;
  float leftDistance = SR3.getDistance(false) + 2;
  while (fabs(rightDistance - leftDistance) != 0) {
    if (rightDistance - leftDistance < 0) {
      md.setSpeeds(350, -350);
      delay(25);
      md.setM1Brake(400);
      md.setM2Brake(400);
      delay(50);
    }
    if (rightDistance - leftDistance > 0) {
      md.setSpeeds(-350, 350);
      delay(25);
      md.setM2Brake(400);
      md.setM1Brake(400);
      delay(50);
    }
    rightDistance = SR1.getDistance(false) + 2;
    leftDistance = SR3.getDistance(false) + 2;
  }
}

void calibrateDistance(){

  float rightDistance = SR1.getDistance(false) + 2;
  float leftDistance = SR3.getDistance(false) + 2;

  if(fabs(rightDistance - leftDistance) != 0){
    calibrateFront();
    }

    rightDistance = SR1.getDistance(false) + 2;
    leftDistance = SR3.getDistance(false) + 2;

    float setM1, setM2;
    
  while (rightDistance!=13 && leftDistance!=13) {
    
    if (rightDistance < 13) 
      setM1 = -350;
    else if(rightDistance > 13)
      setM1 =350;
    else 
      setM1 = 0;
    
    if (leftDistance < 13) 
      setM2 = -350;
    else if(leftDistance > 13)
      setM2 =350;
    else 
      setM2 = 0;

    md.setSpeeds(setM1, setM2);
      delay(25);
      md.setM2Brake(400);
      md.setM1Brake(400);
      delay(50);
     
    rightDistance = SR1.getDistance(false) + 2;
    leftDistance = SR3.getDistance(false) + 2;
  }
  
  }

void avoidObstacle(float distance) {
  long runningTime = round(runTime(rpm1, distance));
  Serial.print("Run Time : ");
  Serial.println(runningTime);
  startTime = millis();
  stopTime = millis();
  int count = 0;
  while (stopTime - startTime < runningTime) {
    if (count % 15 == 0) {
      //      dist[2] = dist[1];
      //      dist[1] = dist[0];
      dist[0] = SR2.getDistance(false) + 2;
      Serial.println(dist[0]);
      //      float avgDist = (dist[0] + dist[1] + dist[2]) / 3.0;
      //      Serial.print("Average Distance : ");
      //      Serial.println(avgDist);
      if ( dist[0] <= 15) {
        Serial.println(dist[0]);
        stopMovement();
        float collisionDistance = returnSrDist (20 , SR2);
        float travelDistance = sqrt(pow(collisionDistance, 2) + 256) + 25 ;
        Serial.println(travelDistance);
        delay(500);
        turnRight(45);
        delay(500);
        moveForward(travelDistance );
        delay(500);
        turnLeft(90);
        delay(500);
        moveForward(travelDistance-5);
        delay(500);
        turnRight(55);
        delay(500);
      }
    } count++;
    setSpeed1 = setWheelSpeed1(rpm1, true);
    setSpeed2 = setWheelSpeed2(rpm2, true);
    if (update1)
      md.setM1Speed(setSpeed1);
    if (update2)
      md.setM2Speed(setSpeed2);
    stopTime = millis();
  }
  md.setM1Speed(0);
  md.setM2Speed(0);
}

float caluclateExtraSpeed(long collisionTime, float collisionDistance) {
  float botRadius = 9;
  float angle = atan(15 / collisionDistance);
  float turnDistance = botRadius * angle;
  return turnDistance / collisionTime;

}

void moveForward(float distance) {
  update1 = true;
  update2 = true;
  long runningTime = round(runTime(rpm1, distance));
  //  Serial.print("Run Time : ");
  //  Serial.println(runningTime);
  startTime = millis();
  stopTime = millis();
  while (stopTime - startTime < runningTime) {
    setSpeed1 = setWheelSpeed1(rpm1, true);
    setSpeed2 = setWheelSpeed2(rpm2, true);
    if (update1)
      md.setM1Speed(setSpeed1);
    if (update2)
      md.setM2Speed(setSpeed2);
    stopTime = millis();
  }
  md.setM1Brake(400);
  md.setM2Brake(400);
}

void moveBackward(float distance) {
  long runningTime = round(runTime(rpm1, distance));
  //  Serial.print("Run Time : ");
  //  Serial.println(runningTime);
  startTime = millis();
  stopTime = millis();
  while (stopTime - startTime < runningTime) {
    setSpeed1 = setWheelSpeed1(rpm1, false);
    setSpeed2 = setWheelSpeed2(rpm2, false);
    if (update1)
      md.setM1Speed(setSpeed1);
    if (update2)
      md.setM2Speed(setSpeed2);
    stopTime = millis();
  }
  md.setM1Brake(400);
  md.setM2Brake(400);
}

void stopMovement() {
  md.setM2Brake(400);
  md.setM1Brake(400);
}

void turnLeft(float angle) {
  long rotateTime = round(rotateArduino(rpm1, angle));
  //  Serial.print("Rotation Time : ");
  //  Serial.println(rotateTime);
  startTime = millis();
  stopTime = millis();
  while (stopTime - startTime < rotateTime) {
    setSpeed1 = setWheelSpeed1(rpm1, true);
    setSpeed2 = setWheelSpeed2(rpm2, false);
    if (update1)
      md.setM1Speed(setSpeed1);
    if (update2)
      md.setM2Speed(setSpeed2);
    stopTime = millis();
  }
  stopMovement();
}

void turnRight(float angle) {
  long rotateTime = round(rotateArduino(rpm1, angle));
  //  Serial.print("Rotation Time : ");
  //  Serial.println(rotateTime);
  startTime = millis();
  stopTime = millis();
  while (stopTime - startTime <  rotateTime) {
    setSpeed1 = setWheelSpeed1(rpm1, false);
    setSpeed2 = setWheelSpeed2(rpm2, true);
    if (update1)
      md.setM1Speed(setSpeed1);
    if (update2)
      md.setM2Speed(setSpeed2);
    stopTime = millis();
  }
  stopMovement();



  
}

float runTime(float rpm, float distance) {
  float wheelRadius = 3;
  float Speed = (rpm * 2 * Pi * wheelRadius) / 60000.0; // Speed in cm/millisecond
  float runTime = distance / Speed;
  return runTime + 1462.5 ; //705.375 ;
}


float rotateArduino(float rpm, float angle) {
  float radius = 9.665;
  float wheelRadius = 3;
  float distance = (2 * Pi * radius * angle) / 360.0;
  float Speed = (rpm * 2 * Pi * wheelRadius) / 60000.0; // Speed in cm/millisecond
  float rotateTime = distance / Speed;
  return rotateTime + 1415 ;//1700; //
}

float setWheelSpeed1(float rpm, bool forward) {

  float motorSpeed1 = getSpeed (pulseIn(E1A, HIGH));

  //  Serial.print(motorSpeed1);
  //  Serial.print(" | ");

  if (motorSpeed1 > 150)
    motorSpeed1 = 0;

  errorUpdate(error1);
  error1[0] = rpm - motorSpeed1 ;

  if (fabs(error1[0]) > errorCtrl ) {
    update1 = true;
    if (motorSpeed1 < 150)
      setSpeed1 = correctedSpeed(fabs(setSpeed1), error1[0], error1[1], error1[2], 1);
    else
      error1[0] = 0;
    return forward ? setSpeed1 : -setSpeed1;
  }

  update1 = false;
}

float setWheelSpeed2(float rpm, bool forward) {

  float motorSpeed2 = getSpeed (pulseIn(E2B, HIGH));
  //  Serial.println(motorSpeed2);
  if (motorSpeed2 > 150)
    motorSpeed2 = 0;

  errorUpdate(error2);
  error2[0] = rpm - motorSpeed2 ;


  if (fabs(error2[0]) > errorCtrl ) {
    update2 = true;
    if (motorSpeed2 < 150)
      setSpeed2 = correctedSpeed(fabs(setSpeed2), error2[0], error2[1], error2[2], 2);
    else
      error2[0] = 0;

    return forward ? setSpeed2 : -setSpeed2;
  }
  update2 = false;
}

float getSpeed (long pulseInTime) {
  long timePeriod = pulseInTime * 2;
  float rps = 1000000 / (timePeriod * 562.25);
  float rpm = rps * 60;
  return rpm;
}

void errorUpdate(float error[]) {
  error[2] = error[1];
  error[1] = error[0];
}

void PIDGearController1() {

  float Kc =  (1.2 * M1ts) / (M1k * M1td);
  float Ti =  2 * M1td;
  float Td = 0.5 * M1td;
  //  float Td = 0;

  //  Serial.print("Kc : ");
  //  Serial.print(Kc);
  //  Serial.print(" Ti : ");
  //  Serial.print(Ti);
  //  Serial.print(" Td : ");
  //  Serial.println(Td);

  float Kp =  Kc;
  float Ki = Kc / Ti;
  //  float Ki = 0;
  //  float Kd = 0;
  float Kd =  Kc * Td;

  M1k1 = Kp + Ki + Kd;
  M1k2 = -Kp - 2 * Kd;
  //  M1k2 = 0;
  M1k3 = Kd;
  //
  //  Serial.print("K1 : ");
  //  Serial.print(M1k1);
  //  Serial.print(" K2 : ");
  //  Serial.print(M1k2);
  //  Serial.print(" K3 : ");
  //  Serial.println(M1k3);
}

void PIDGearController2() {
  float Kc =  (1.2 * M2ts) / (M2k * M2td);
  float Ti = 2 * M2td;
  float Td = 0.5 * M2td;

  float Kp =  Kc;
  float Ki = Kc / Ti;
  //  float Ki = 0;
  //  float Kd = 0;
  float Kd =  Kc * Td;

  M2k1 = Kp + Ki + Kd;
  M2k2 = -Kp - 2 * Kd;
  M2k3 = Kd;

  //  Serial.print("K1 : ");
  //  Serial.print(M2k1);
  //  Serial.print(" K2 : ");
  //  Serial.print(M2k2);
  //  Serial.print(" K3 : ");
  //  Serial.println(M2k3);
}

float correctedSpeed(float originalSpeed, float error1, float error2, float error3, int motor) {

  if (motor == 1) {
    //    Serial.print(" Additional Speed : ");
    //    Serial.println(error1 * M1k1 + error2 * M1k2 + error3 * M1k3);
    float newSpeed = originalSpeed + error1 * M1k1 + error2 * M1k2 + error3 * M1k3;
    if (newSpeed > 350)
      newSpeed = 350;

    return newSpeed;
  }
  else {
    //    Serial.print(" Additional Speed : ");
    //    Serial.print(error1 * M2k1);
    float newSpeed = originalSpeed + error1 * M2k1 + error2 * M2k2 + error3 * M2k3;
    if (newSpeed > 400)
      newSpeed = 350;

    return newSpeed;
  }
}
