#include "SharpIR.h"
#include "ArduinoSort.h"

SharpIR SR1(SharpIR::GP2Y0A21YK0F, A0);
SharpIR SR2(SharpIR::GP2Y0A21YK0F, A1);
SharpIR SR3(SharpIR::GP2Y0A21YK0F, A2);
SharpIR SR4(SharpIR::GP2Y0A21YK0F, A3);
SharpIR LR1(SharpIR::GP2Y0A02YK0F, A4);

void setup() {
  Serial.begin(9600);
}

void loop() {


  //returnLrDist(20, LR1);
  //returnSrDist(20,SR1); //PROB
  returnSrDist(20,SR2);
  //returnSrDist(20,SR3);
  //returnSrDist(20,SR4);
  
  delay(2000);
}

void returnLrDist (int x, SharpIR y){
  int arraySR1[x];
  for (int i = 0; i < x+1; i++) {
    arraySR1[i] = y.getDistance(false) + 10; //When it comes to LR sensor, add a +10 due to unknown offset
  }

  // Not sorted
  printArray("Not sorted:", arraySR1);

  // Sort normally
  sortArray(arraySR1, 20);
  printArray("sortArray:", arraySR1);


  float final;
  final = (arraySR1[9] + arraySR1[10]) / 2.0;
  Serial.print("Final value is ");
  Serial.println(final);
  }

void returnSrDist (int x, SharpIR y){
    int arraySR1[x];
    for (int i = 0; i < x+1; i++) {
      arraySR1[i] = y.getDistance(false) + 2;
    }
  
    // Not sorted
    //printArray("Not sorted:", arraySR1);
  
    // Sort normally
    sortArray(arraySR1, 20);
    //printArray("sortArray:", arraySR1);
  
  
    float final;
    final = (arraySR1[9] + arraySR1[10]) / 2.0;
    Serial.print("Final value is ");
    Serial.println(final);
  }

void printArray(String msg, int* arraySR1) {
  Serial.print(msg);
  for (int j = 0; j < 20; j++) {
    Serial.print(arraySR1[j]);
    Serial.print(" , ");
  }
  Serial.println();
}
