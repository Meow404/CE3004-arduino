#include "DualVNH5019MotorShield.h"
#include "math.h"
#include "Sensors.h"
#include "Motion.h"
#include "Calibration.h"

/* Object to control motor shield and in turn control the motors, additionaly information found at https://github.com/pololu/dual-vnh5019-motor-shield */
DualVNH5019MotorShield md; 

/* Definition of A & B lines of Encoder 1 (right) and Encoder 2 (left) */
#define E2A  3
#define E2B  5
#define E1A  11
#define E1B  13

/*Definition of target or required RPM by motors, calibration speed and starting set Speed
  ~ Staring set Speed is set to a mid range value below 80 rpm in order to not allow the PID
  controlled to start from 0 rpm which tend to cause a delay in response time*/
#define targetRPM 80
#define initialSetSpeed1 210
#define initialSetSpeed2 220
#define calibrationSetSpeed1 315
#define calibrationSetSpeed2 350

String sensorData;

Motion* bot;  // Motion object to control bot movements such as "Move Forward", "Turn Right", etc.
Calibration* calibrateBot; // Calibration object to control calibration techniques such as "Calibrate with Front sensors", "Calibrate with Left sensors", etc.


void setup() {
  Serial.begin(115200);

  pinMode(E1A, INPUT);
  pinMode(E1B, INPUT);
  pinMode(E2A, INPUT);
  pinMode(E2B, INPUT);

  md.init();                                                                            //Initialisation of motor shield
  bot = new Motion(targetRPM, initialSetSpeed1, E1A, initialSetSpeed2, E2A, md);        //Construction of Motion object
  calibrateBot = new Calibration(calibrationSetSpeed1, calibrationSetSpeed2, md);       //Construction of Calibration object
}


void loop() {
  
  int secondVal = 10; // Offset of 10 to let bot travel by 10 cm in forward and backward movement by default

  if (Serial.available() > 0)                   //RPi to Arduino
  {
    int instructions = Serial.parseInt();       //Integer parsing is more efficient and has a faster response time than string reading i.e Serial.read(), Serial.readStringUntil(), etc.
    controlBot(instructions, secondVal);
  }
}

/* Function to control bot functions such as return sensor data, turn left, calibrate front, etc.
The function additionally passes a message back to RPI via serial port when required action is complete
In order to see bot functionalities simply use serial monitor to input bot functionalities
1. Check If arduino is responding (redundant)
2. Return Sensor data
3. Move Forward
4. Turn Left
5. Turn Right
6. Move Backward
7. Turn Left 180
8. Turn Right 180
9. Stop bot (helps unlock wheels after using break)
10. Turn left, Calibrate with Front sensors, and turn back to original position (left wall hugging)
11. Calibrate with Front sensors
12. Calibrate with Left sensors
13. Calibrate using front sensors at a staircase
14. Get ready to receive fastest path string */

void controlBot (int instruction, int secondVal) {  

  switch (instruction) {
    case 1:  // Check If arduino is responding 
      Serial.println("X_BOTREADY");
      break;
    case 2:  // Return Sensor data
      sensorData = returnSensorData(8);
      Serial.println("X_SENDATA" + sensorData);
      break;
    case 3:  // Move Forward
      bot->moveForward(secondVal);
      Serial.println("X_BOTDONE");
      break;
    case 4:  // Turn Left
      bot->turnLeft(90);
      Serial.println("X_BOTDONE");
      break;
    case 5:  // Turn Right
      bot->turnRight(90);
      Serial.println("X_BOTDONE");
      break;
    case 6:  // Move Backward
      bot->moveBackward(secondVal);
      Serial.println("X_BOTDONE");
      break;
    case 7:  // Turn left 180 degrees
      bot->turnLeft(180);
      Serial.println("X_BOTDONE");
      break;
    case 8:  // Turn right 180 degrees
      bot->turnRight(180);
      Serial.println("X_BOTDONE");
      break;
    case 9:  // Unbreak wheels
      md.setM1Speed(0);
      md.setM2Speed(0);
      Serial.println("X_BOTDONE");
      break;
    case 10:  // Turn Left and Calibrate with front sensors
      { bot->turnLeft(90);
        delay(100);
        calibrateBot->CalibrateFront();
        delay(100);
        bot->turnRight(90);
        delay(200);
      }
      Serial.println("X_CALIBRATIONDONE");
      break;
    case 11 :  // Calibrate with front sensors
      calibrateBot->CalibrateFront();
      Serial.println("X_CALIBRATIONDONE");
      break;
    case 12 :  // Calibrate with left sensors
      calibrateBot->CalibrateLeft();
      Serial.println("X_CALIBRATIONDONE");
      break;
    case 13 :  // Calibrate with respect to steps
      calibrateBot->CalibrateStep();
      Serial.println("X_CALIBRATIONDONE");
      break;
    case 14 :  //Get ready for fastest path string
      String fastest_path = getFastestPath();
      fastestPath(fastest_path);
      Serial.println("X_FASTESTPATHDONE");
      break;
  }
}

/* Function that waits for fastest path string of the format (action):(distance):....:(action):(distance)~
Note : String must end with '~'
eg. 3:10:4:x:11:x~ implies 
1. Move forward by 10 cm
2. Turn left by 90
3. Calibrate with front sensors

x implies that the function does not require a second values, hence an arbituary value may be passed (MUST be passed)
eg =  3:10:4:90:10:20~ */

String getFastestPath() {
  String fastestPath = "";
  Serial.read();
  while (fastestPath.equals("")) {
    fastestPath = Serial.readStringUntil('~');
  }
  return fastestPath;
}

/* Function breaks down fastest path string in order to control bot movements, direction and callibration */
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
      
      fastestPathCalibration();
      
      getInstruction = false;
      getValue = false;
      
      delay(100);
    }
  } while (!fastest_path_code.equals(""));

}
/* Function does manual bot calibration if possible using sensor values after each bot movement */
void fastestPathCalibration() {
  
  float rightDistance = SR1.getDistance(false) + 2;
  float leftDistance = SR3.getDistance(false) + 2;
  float frontDistance = SR4.getDistance(false) + 2;
  float backDistance = SR5.getDistance(false) + 2;

  if (frontDistance <= 20 && backDistance <= 17) {
    bot->turnLeft(90);
    delay(100);
    calibrateBot->CalibrateFront();
    delay(100);
    bot->turnRight(90);
    delay(200);
  }

  if (rightDistance <= 16 && leftDistance <= 16)
    calibrateBot->CalibrateFront();

}
