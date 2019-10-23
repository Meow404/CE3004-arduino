
#include "DualVNH5019MotorShield.h"
#include "TimerOne.h"

DualVNH5019MotorShield md;
long E2A = 3;
long E2B = 5;
long E1A = 11;
long E1B = 13;
float avgSpeed1 = 0;
float avgSpeed2 = 0;
float avgSpeeds1[25];
float avgSpeeds2[25];
int count1 = 0;
int count2 = 0;
int timer1_counter;

#define M1k  0.43483
#define M1ts  0.0037//78,0.00565
#define M1td  0.12//066//0.047;//0.07625;

#define M2k  0.437393
#define M2ts  0.0035 //77;
#define M2td  0.1

float M1k1, M1k2, M1k3;
float M2k1, M2k2, M2k3;
float errorCtrl = 2;
float error1[3] = {0, 0, 0};
float error2[3] = {0, 0, 0};
float setSpeed1, setSpeed2, rpm = 100;
float totalCount;
long errorCount;
#define ledPin 13

void stopIfFault()
{
  if (md.getM1Fault())
  {
    Serial.println("M1 fault");
    while (1);
  }
  if (md.getM2Fault())
  {
    Serial.println("M2 fault");
    while (1);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Dual VNH5019 Motor Shield");
  pinMode(E1A, INPUT);
  pinMode(E1B, INPUT);
  pinMode(E2A, INPUT);
  pinMode(E2B, INPUT);
  PIDGearController1();
  PIDGearController2();

  Serial.print("Speed set : ");
  Serial.println(rpm);

  setSpeed1 = 250;
  setSpeed2 = 250;

  errorCount = 0;
  totalCount = 0;

  md.init();

  md.setM1Speed(setSpeed1);
  md.setM2Speed(setSpeed2);
}

void findSpeed(void)
{
  float speed_E1A, speed_E2A, speed_E1B, speed_E2B;
  speed_E1A = getSpeed(pulseIn(E1A, HIGH));
  speed_E1B = getSpeed(pulseIn(E1B, HIGH));
  speed_E2A = getSpeed(pulseIn(E2A, HIGH));
  speed_E2B = getSpeed(pulseIn(E2B, HIGH));

  Serial.print(millis());
  Serial.print(", ");
  //  Serial.print("M1 : ");
  Serial.print((speed_E1B + speed_E1A) / 2);
  Serial.print(", ");
  //  Serial.print("M2 : ");
  Serial.println((speed_E2B + speed_E2A) / 2);

}


void loop() {

  //  Serial.print("Set Speed 1 : ");
  //  Serial.print(setSpeed1);
  //  Serial.print("Set Speed 2 : ");
  //  Serial.println(setSpeed2);
  if(totalCount == 100){
    Serial.println();
    Serial.println("~~~~~");
    Serial.print("Error Rate : ");
    Serial.println(errorCount/totalCount);
    Serial.println("~~~~~");
    Serial.println();
    errorCount = 0;
    totalCount = 0;
    }
totalCount+=1;

  delay(50);

  float motorSpeed1 = getSpeed (pulseIn(E1B, HIGH));
  float motorSpeed2 = getSpeed (pulseIn(E2A, HIGH));
//    Serial.print(", Motor Speed 1 : ");
  Serial.print(motorSpeed1);
  Serial.print(" | ");
////    Serial.print(", Motor Speed 2 : ");
  Serial.print(motorSpeed2);
  Serial.print(" | ");
//  Serial.print(motorSpeed2 - motorSpeed1);

  if(fabs(motorSpeed2 - motorSpeed1)>2*errorCtrl)
  {
  errorCount++;
  Serial.println(" ~~~");}
  else
  Serial.println();

  if (motorSpeed1 > 150)
    motorSpeed1 = 0;

  if (motorSpeed2 > 150)
    motorSpeed2 = 0;

  errorUpdate(error1);
  error1[0] = rpm - motorSpeed1 ;
  errorUpdate(error2);
  error2[0] = rpm - motorSpeed2 ;

  if (fabs(error1[0]) > errorCtrl ) {
    //    Serial.print(millis());

    if (motorSpeed1 < 150)
      setSpeed1 = correctedSpeed(setSpeed1, error1[0], error1[1], error1[2], 1);
    else
      error1[0] = 0;

    md.setM1Speed(setSpeed1);
    //      Serial.print("New Set Speed 1 : ");
    //      Serial.println(setSpeed1);
    //      Serial.println();
  }

  if (fabs(error2[0]) > errorCtrl) {
    //      Serial.print(millis());

    if (motorSpeed2 < 150)
      setSpeed2 = correctedSpeed(setSpeed2, error2[0], error2[1], error2[2], 2);
    else
      error2[0] = 10;

    md.setM2Speed(setSpeed2);
//          Serial.print(", New Set Speed 2 : ");
//          Serial.println(setSpeed2);
  }

  delayMicroseconds(500);

}

//void loop() {
//  Serial.println("Speed set : 250");
//  md.setM1Speed(250);
//  md.setM2Speed(250);
//  for (int i = 0; i < 16000; i++)
//    if (i % 100 == 0)
//    { findSpeed();
//      delay(10);
//    }
//  Serial.print(millis());
//  Serial.print(", ");
//  Serial.println("Speed set : 300");
//  md.setM1Speed(300);
//  md.setM2Speed(300);
//  for (int i = 0; i < 16000; i++)
//    if (i % 100 == 0)
//    { findSpeed();
//      delay(10);
//    }
//  Serial.println("Speed set : 0");
//  md.setM1Speed(0);
//  md.setM2Speed(0);
//  delay(100000);
//}

//void loop()
//{ float speed_E1A, speed_E2A, speed_E1B, speed_E2B;
//  for (int j = 5; j <= 40; j++) {
////    resetAvgSpeed(1);
//    resetAvgSpeed(2);
//    long speed = j * 10;
//    for (int i = 0; i <= 2000; i++)
//    {
////      md.setM1Speed(speed);
//      md.setM2Speed(speed);
//      stopIfFault();
//      if (i % 200 == 100)
//      {
////        speed_E1A = getSpeed(pulseIn(E1A, HIGH));
////        speed_E1B = getSpeed(pulseIn(E1B, HIGH));
////        addAvgSpeed(1, speed_E1A);
////        addAvgSpeed(1, speed_E1B);
//
//        speed_E2A = getSpeed(pulseIn(E2A, HIGH));
//        speed_E2B = getSpeed(pulseIn(E2B, HIGH));
//        addAvgSpeed(2, speed_E2A);
//        addAvgSpeed(2, speed_E2B);
//      }
//      delay(2);
//    }
//    Serial.print(speed);
////    Serial.print(", ");
////    Serial.print(getAvgSpeed(1));
////    Serial.print(", ");
////    Serial.println(sortAndAverage(avgSpeeds1, count1));
//    Serial.print(", ");
//    Serial.print(getAvgSpeed(2));
//    Serial.print(", ");
//    Serial.println(sortAndAverage(avgSpeeds2, count2));
//  }
//
//}

//void calibrate(){
//  for(i=0;i<10;i++){
//    md.setSpeeds(350,350);
//    delay(1000);
//    md.setBrakes(400,400);
//    delay(1000);
//  }
//}
void errorUpdate(float error[]) {
  error[2] = error[1];
  error[1] = error[0];
}

//float setSpeedMotor1 (float rpm) {
//  return (rpm + 9.156) / 0.37082;
//}
//
//
//float setSpeedMotor2 (float rpm) {
//  return (rpm + 12.133) / 0.36246;
//}

float getSpeed (long pulseInTime) {
  long timePeriod = pulseInTime * 2;
  float rps = 1000000 / (timePeriod * 562.25);
  float rpm = rps * 60;
  return rpm;
}

float addAvgSpeed (int motor, float rpm) {
  if (motor == 1)
  {
    avgSpeed1 += rpm;
    avgSpeeds1[count1] = rpm;
    count1++;
  }
  else if (motor == 2)
  {
    avgSpeed2 += rpm;
    avgSpeeds2[count2] = rpm;
    count2++;
  }
  else
    Serial.print("Invalid input of motor");
}

float getAvgSpeed (int motor) {
  if (motor == 1)
    return avgSpeed1 / count1;
  else if (motor == 2)
    return avgSpeed2 / count2;
  else
    Serial.print("Invalid input of motor");
}

float resetAvgSpeed (int motor) {
  if (motor == 1)
  {
    avgSpeed1 = 0;
    count1 = 0;
  }
  else if (motor == 2)
  {
    avgSpeed2 = 0;
    count2 = 0;
  }
  else
    Serial.print("Invalid input of motor");
}

float sortAndAverage(float values[], int count) {
  sort(values, count);
  int startCount = 2 * count / 5;
  int stopCount = 4 * count / 5;
  float sum = 0;
  for (int i = startCount; i <= stopCount; i++)
    sum += values[i];
  return sum / (stopCount - startCount + 1);
}

void sort(float values[], int count) {
  float temp;
  for (int i = 0; i < count; i++)
    for (int j = i + 1; j <= count - 1 - i; j++)
      if (values[i] > values[j])
      { temp = values[i];
        values[i] = values[j];
        values[j] = temp;
      }
}

void PIDGearController1() {

  Serial.print("M1k : ");
  Serial.print(M1k);
  Serial.print(" M1ts : ");
  Serial.print(M1ts);
  Serial.print(" M1td : ");
  Serial.println(M1td);

  float Kc =  (1.2 * M1ts) / (M1k * M1td);
  float Ti =  2 * M1td;
  float Td = 0.5 * M1td;
  //  float Td = 0;

  Serial.print("Kc : ");
  Serial.print(Kc);
  Serial.print(" Ti : ");
  Serial.print(Ti);
  Serial.print(" Td : ");
  Serial.println(Td);

  float Kp =  Kc;
  float Ki = Kc / Ti;
  //  float Ki = 0;
  //  float Kd = 0;
  float Kd =  Kc * Td;

  Serial.print("Kp : ");
  Serial.print(Kp);
  Serial.print(" Ki : ");
  Serial.print(Ki);
  Serial.print(" Kd : ");
  Serial.println(Kd);

  M1k1 = Kp + Ki + Kd;
  M1k2 = -Kp - 2 * Kd;
  //  M1k2 = 0;
  M1k3 = Kd;

  Serial.print("K1 : ");
  Serial.print(M1k1);
  Serial.print(" K2 : ");
  Serial.print(M1k2);
  Serial.print(" K3 : ");
  Serial.println(M1k3);
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

  Serial.print("K1 : ");
  Serial.print(M2k1);
  Serial.print(" K2 : ");
  Serial.print(M2k2);
  Serial.print(" K3 : ");
  Serial.println(M2k3);
}

float correctedSpeed(float originalSpeed, float error1, float error2, float error3, int motor) {
  //  Serial.print(" original Speed : ");
  //  Serial.print(originalSpeed);
  //  Serial.print(" Error 1 : ");
  //  Serial.print(error1);
  //  Serial.print(" Error 2 : ");
  //  Serial.print(error2);
  //  Serial.print(" Error 3 : ");
  //  Serial.print(error3);
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
