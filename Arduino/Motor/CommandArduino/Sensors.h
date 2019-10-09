#ifndef Sensor_h
#define Sensor_h

#include "SharpIR.h"
#include "ArduinoSort.h"

SharpIR SR1(SharpIR::GP2Y0A21YK0F, A4);
SharpIR SR3(SharpIR::GP2Y0A21YK0F, A2);
SharpIR SR4(SharpIR::GP2Y0A21YK0F, A5);
SharpIR LR1(SharpIR::GP2Y0A02YK0F, A0);
SharpIR SR5(SharpIR::GP2Y0A21YK0F, A1);

void printArray(String msg, int* arraySR1) {
  Serial.print(msg);
  for (int j = 0; j < 20; j++) {
    Serial.print(arraySR1[j]);
    Serial.print(" , ");
  }
  Serial.println();
}

float returnLrDist (int x, SharpIR y){
  int arraySR1[x];
  for (int i = 0; i < x; i++) {
    arraySR1[i] = y.getDistance(false) + 10; //When it comes to LR sensor, add a +10 due to unknown offset
  }

  // Not sorted
//  printArray("Not sorted:", arraySR1);

  // Sort normally
  sortArray(arraySR1, 20);
//  printArray("sortArray:", arraySR1);


  float final;
  final = (arraySR1[9] + arraySR1[10]) / 2.0;
  return final;
  }

float returnSrDist (int x, SharpIR y){
    int arraySR1[x];
    for (int i = 0; i < x; i++) {
      arraySR1[i] = y.getDistance(false) + 2;
    }
    // Not sorted
    //printArray("Not sorted:", arraySR1);
    // Sort normally
    sortArray(arraySR1, 20);
    //printArray("sortArray:", arraySR1);
    
    float final = (arraySR1[9] + arraySR1[10]) / 2.0;
    return final;
  }
String returnSensorData(){
  float SR1_distance = returnSrDist(20,SR1);
  float SR2_distance = 0;
  float SR3_distance = returnSrDist(20,SR3);
  float SR4_distance = returnSrDist(20,SR4);
  float LR1_distance = returnLrDist(20,LR1);
  float SR5_distance = returnSrDist(20,SR5);

  return ' '+String(SR1_distance)+' '+String(SR2_distance)+' '+String(SR3_distance)+' '+String(SR4_distance)+' '+String(SR5_distance)+' '+String(LR1_distance);
  }

#endif
