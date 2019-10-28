#ifndef Sensor_h
#define Sensor_h

#include "SharpIR.h"
#include "ArduinoSort.h"

SharpIR SR1(SharpIR::GP2Y0A21YK0F, A4);
SharpIR SR2(SharpIR::GP2Y0A21YK0F, A3);
SharpIR SR3(SharpIR::GP2Y0A21YK0F, A2);
SharpIR SR4(SharpIR::GP2Y0A21YK0F, A5);
SharpIR LR1(SharpIR::GP2Y0A02YK0F, A1);
SharpIR SR5(SharpIR::GP2Y0A21YK0F, A0);

class Sensor {
  private:
    void printArray(String msg, int* arraySR1);
  public:
    float returnLrDist (int count, SharpIR sensor) ;
    float returnSrDist (int count, SharpIR sensor);
    String returnSensorData(int count);
};

/* Prints all obtained sensor values */
void printArray(String msg, int* arraySR1) {
  Serial.print(msg);
  for (int j = 0; j < 20; j++) {
    Serial.print(arraySR1[j]);
    Serial.print(" , ");
  }
  Serial.println();
}

/* 1. Finds COUNT number of Long Range sensor values
   2. Adds an offset to each sensor values
   3. Arrages values in ascending order
   4. takes median (or middle values) for must accurate data */
   
float returnLrDist (int count, SharpIR sensor) {
  int arraySR1[count];

  for (int i = 0; i < count; i++) {
    arraySR1[i] = sensor.getDistance(false) + 10; //When it comes to LR sensor, add a +10 due to unknown offset
  }
  sortArray(arraySR1, count);

  float final = round(arraySR1[count / 2]);
  return final;
}

/* 1. Finds COUNT number of Short Range sensor values
   2. Adds an offset to each sensor values
   3. Arrages values in ascending order
   4. takes median (or middle values) for must accurate data */
   
float returnSrDist (int count, SharpIR sensor) {
  int arraySR1[count];

  for (int i = 0; i < count; i++) {
    arraySR1[i] = sensor.getDistance(false) + 2;
  }

  sortArray(arraySR1, count);

  float final = arraySR1[count / 2];
  return final;
}

/* Returns sensor data in string consisting of required sensor values by calling each sensor value individually*/
String returnSensorData(int count) {

  float SR1_distance = returnSrDist(count, SR1);
  float SR2_distance = returnSrDist(count, SR2);
  float SR3_distance = returnSrDist(count, SR3);
  float SR4_distance = returnSrDist(count, SR4);
  float LR1_distance = returnLrDist(count, LR1);
  float SR5_distance = returnSrDist(count, SR5);

  return ' ' + String(SR1_distance) + ' ' + String(SR2_distance) + ' ' + String(SR3_distance) + ' ' + String(SR4_distance) + ' ' + String(SR5_distance) + ' ' + String(LR1_distance);
}

#endif
