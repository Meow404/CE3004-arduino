#include "DualVNH5019MotorShield.h"
#include "Sensors.h"
#include "math.h"

DualVNH5019MotorShield md;

#define Pi 3.1428

#define E2A  3
#define E2B  5
#define E1A  11
#define E1B  13

#define M1k  0.43483
#define M1ts  0.0037//78,0.00565
#define M1td  0.07//12//066//0.047;//0.07625;

#define M2k  0.437393
#define M2ts  0.0035 //77;
#define M2td  0.048//116 // 0.8 //Less turn right, more urn left

#define errorCtrl  2

float M1k1, M1k2, M1k3;
float M2k1, M2k2, M2k3;

bool update1, update2;

String sensorData;


float error1[3] = {0, 0, 0};
float error2[3] = {0, 0, 0};

float setSpeed1, setSpeed2, angle = 90;

float rpm1 = 100, rpm2 = 100;

long startTime, stopTime;
float calibrationOffset = -30;
float turnOffset = 25;
#define ledPin 13
int timer1_counter;
float dist[3] = {30, 30, 30};


void setup() {
  Serial.begin(115200);

  pinMode(E1A, INPUT);
  pinMode(E1B, INPUT);
  pinMode(E2A, INPUT);
  pinMode(E2B, INPUT);
  PIDGearController1();
  PIDGearController2();

  md.init();           // enable all interrupts
}


void loop() {
  int secondVal = 10;

  if (Serial.available() > 0) //RPi to Arduino
  {
    int instructions = Serial.parseInt(); //use this if char reading fails
    controlBot(instructions, 10);
  }
}

void controlBot (int instruction, int secondVal) {
  switch (instruction) {
    case 1:
      Serial.println("X_BOTREADY");
      break;
    case 2:
      sensorData = returnSensorData(8);
      Serial.println("X_SENDATA" + sensorData);
      break;
    case 3:
      moveForward(secondVal);
      Serial.println("X_BOTDONE");
      break;
    case 4:
      turnLeft(90);
      Serial.println("X_BOTDONE");
      break;
    case 5:
      turnRight(90);
      Serial.println("X_BOTDONE");
      break;
    //bot backward
    case 6:
      moveBackward(secondVal);
      Serial.println("X_BOTDONE");
      break;
    case 7:
      turnLeft(180);
      Serial.println("X_BOTDONE");
      break;
    case 8:
      turnRight(180);
      Serial.println("X_BOTDONE");
      break;
    case 9:
      md.setM1Speed(0);
      md.setM2Speed(0);
      Serial.println("X_BOTDONE");
      break;
    case 10:
      { turnLeft(90);
        delay(100);
        calibrateDistance();
        calibrateDistance();
        delay(100);
        turnRight(90);
      }
      Serial.println("X_CALIBRATIONDONE");
      break;
    case 11 :
      calibrateDistance();
      Serial.println("X_CALIBRATIONDONE");
      break;
    case 12 :
      calibrateLeftSide();
      Serial.println("X_CALIBRATIONDONE");
      break;
    case 13 :
      avoidObstacle(150);
      break;
    case 14 :
      String fastest_path = getFastestPath();
      fastestPath(fastest_path);
      Serial.println("X_FASTESTPATHDONE");
      break;
  }
}

String getFastestPath() {
  String fastestPath = "";
  Serial.read();
  while (fastestPath.equals("")) {
    fastestPath = Serial.readStringUntil('~');
  }
  return fastestPath;
}

void fastestPath(String fastest_path_code) {
  String instruction = "";
  bool getInstruction = false;
  bool getValue = false;
  int value = 0;

  do {
    for (int i = 0; i <= fastest_path_code.length(); i++) {
      if (fastest_path_code.substring(i, i + 1) == ":" || fastest_path_code.length() == i) {
        if (!getInstruction) {
          instruction = fastest_path_code.substring(0, i);
          fastest_path_code =  fastest_path_code.substring(i + 1);
          getInstruction = true;
          break;
        }
        else if (!getValue) {
          value = fastest_path_code.substring(0, i).toInt();
          fastest_path_code =  fastest_path_code.substring(i + 1);
          getValue = true;
          break;
        }
      }
    }
    if (getValue && getInstruction) {
      controlBot(instruction.toInt(), value);
      getInstruction = false;
      getValue = false;
      delay(100);
    }
  } while (!fastest_path_code.equals(""));

}

void calibrateLeftSide() {
  float frontSensor = SR4.getDistance(false) + 2;
  float backDistance = SR5.getDistance(false) + 2;
  while (fabs(frontSensor - backDistance) != 4) {
    if (frontSensor - backDistance < 4) {
      md.setSpeeds(350, -350);
      delay(10);
      md.setM1Brake(400);
      md.setM2Brake(400);
      delay(30);
    }
    if (frontSensor - backDistance > 4) {
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

// CHANGE BOT 10 CM DISTANCE HERE
float runTime(float rpm, float distance) {
  float wheelRadius = 2.65;
  float Speed = (rpm * 2 * Pi * wheelRadius) / 60000.0; // Speed in cm/millisecond
  float runTime = distance / Speed;
  return runTime + 22.5; // + 35; // + 1415;//1462.5 ; //705.375 ;
}

// CHANGE ROTATION HERE
float rotateArduino(float rpm, float angle) {
  float radius = 9.5;//9.665;
  float wheelRadius = 3;
  float distance = (2 * Pi * radius * angle) / 360.0;
  float Speed = (rpm * 2 * Pi * wheelRadius) / 60000.0; // Speed in cm/millisecond
  float rotateTime = distance / Speed;
  return rotateTime + 19;//36;//1700; //
}

void moveForward(float distance) {

  update1 = true;
  update2 = true;

  long runningTime = round(runTime(rpm1, distance));
  //    Serial.print("Run Time : ");
  //    Serial.println(runningTime);
  startTime = millis();
  stopTime = millis();
  md.setSpeeds(250, 250);
  while (stopTime - startTime < runningTime) {
    setSpeed1 = setWheelSpeed1(rpm1, true);
    setSpeed2 = setWheelSpeed2(rpm2, true);
    if (update1)
      md.setM1Speed(setSpeed1);
    if (update2)
      md.setM2Speed(setSpeed2);
    //delayMicroseconds(10);
    stopTime = millis();
  }
  stopMovement();
}

void moveBackward(float distance) {

  update1 = true;
  update2 = true;

  long runningTime = round(runTime(rpm1, distance));
  //  Serial.print("Run Time : ");
  //  Serial.println(runningTime);
  startTime = millis();
  stopTime = millis();
  md.setSpeeds(-250, -250);
  while (stopTime - startTime < runningTime) {
    setSpeed1 = setWheelSpeed1(rpm1, false);
    setSpeed2 = setWheelSpeed2(rpm2, false);
    if (update1)
      md.setM1Speed(setSpeed1);
    if (update2)
      md.setM2Speed(setSpeed2);
    stopTime = millis();
  }
  stopMovement();
}

void stopMovement() {
  md.setM1Brake(400);
  md.setM2Brake(400);
}

void turnLeft(float angle) {

  update1 = true;
  update2 = true;

  long rotateTime = round(rotateArduino(rpm1, angle));
  //  Serial.print("Rotation Time : ");
  //  Serial.println(rotateTime);
  startTime = millis();
  stopTime = millis();
  md.setSpeeds(250, -250);
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

  update1 = true;
  update2 = true;

  long rotateTime = round(rotateArduino(rpm1, angle));
  //  Serial.print("Rotation Time : ");
  //  Serial.println(rotateTime);
  startTime = millis();
  stopTime = millis();
  md.setSpeeds(-250, 250);
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

void calibrateFront() {
  float rightDistance = SR1.getDistance(false) + 2;
  float leftDistance = SR3.getDistance(false) + 2;
  while (fabs(rightDistance - leftDistance) != 0) {
    if (rightDistance - leftDistance < 0) {
      md.setSpeeds(350, -350);
      delay(10);
      md.setM1Brake(400);
      md.setM2Brake(400);
      delay(30);
    }
    if (rightDistance - leftDistance > 0) {
      md.setSpeeds(-350, 350);
      delay(10);
      md.setM2Brake(400);
      md.setM1Brake(400);
      delay(30);
    }
    rightDistance = SR1.getDistance(false) + 2;
    leftDistance = SR3.getDistance(false) + 2;
  }
}

void calibrateDistance() {

  float rightDistance = SR1.getDistance(false) + 2;
  float leftDistance = SR3.getDistance(false) + 2;

  if (fabs(rightDistance - leftDistance) != 0) {
    calibrateFront();
  }

  rightDistance = SR1.getDistance(false) + 2;
  leftDistance = SR3.getDistance(false) + 2;

  float setM1, setM2;

  do {

    if (leftDistance < 12)
      setM1 = -350;
    else if (leftDistance > 12)
      setM1 = 350;
    else
      setM1 = 0;

    if (rightDistance < 12)
      setM2 = -350;
    else if (rightDistance > 12)
      setM2 = 350;
    else
      setM2 = 0;

    md.setSpeeds(setM1, setM2);
    delay(10);
    md.setM2Brake(400);
    md.setM1Brake(400);
    delay(30);

    rightDistance = SR1.getDistance(false) + 2;
    leftDistance = SR3.getDistance(false) + 2;
  }  while (!(rightDistance == 12 && leftDistance == 12));

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
      if ( dist[0] <= 22) {
        Serial.println(dist[0]);
        stopMovement();
        float collisionDistance = returnSrDist (20 , SR2);
        float travelDistance = sqrt(pow(collisionDistance, 2) + 256) + 20 ;
        Serial.println(travelDistance);
        delay(500);
        turnRight(45);
        delay(500);
        moveForward(travelDistance );
        delay(500);
        turnLeft(90);
        delay(500);
        moveForward(travelDistance - 5);
        delay(500);
        turnRight(50);
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


float setWheelSpeed1(float rpm, bool forward) {

  float motorSpeed1 = getSpeed (pulseIn(E1A, HIGH));

  //      Serial.print(motorSpeed1);
  //      Serial.print(" | ");

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
  //      Serial.println(motorSpeed2);
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

  float Kp =  Kc;
  float Ki = Kc / Ti;
  float Kd =  Kc * Td;

  M1k1 = Kp + Ki + Kd;
  M1k2 = -Kp - 2 * Kd;
  M1k3 = Kd;
}

void PIDGearController2() {
  float Kc =  (1.2 * M2ts) / (M2k * M2td);
  float Ti = 2 * M2td;
  float Td = 0.5 * M2td;

  float Kp =  Kc;
  float Ki = Kc / Ti;
  float Kd =  Kc * Td;

  M2k1 = Kp + Ki + Kd;
  M2k2 = -Kp - 2 * Kd;
  M2k3 = Kd;
}

float correctedSpeed(float originalSpeed, float error1, float error2, float error3, int motor) {

  if (motor == 1) {
    float newSpeed = originalSpeed + error1 * M1k1 + error2 * M1k2 + error3 * M1k3;
    if (newSpeed > 350)
      newSpeed = 350;

    return newSpeed;
  }
  else {
    float newSpeed = originalSpeed + error1 * M2k1 + error2 * M2k2 + error3 * M2k3;
    if (newSpeed > 400)
      newSpeed = 350;

    return newSpeed;
  }
}
