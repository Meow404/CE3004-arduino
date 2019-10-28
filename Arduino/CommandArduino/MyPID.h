#ifndef MyPID_h
#define MyPID_h

/* Class to handle all PID related functions including calculation of new set speed based on error and previous values of set speed */
class MyPID
{
  private:
    double K1, K2, K3;
    float error[3];
    float errorCtrl;
    float correctedSpeed(float originalSpeed);
    float getSpeed (long pulseInTime);
    void errorUpdate();

  public:
    MyPID(double k, double td, double ts, float _errorCtrl);
    float setWheelSpeed(float setMotorSpeed, int pinNo, float rpm, boolean forward);
    void MyPIDReboot();
};

/* Constructor to calculate and initialise all variables of class i.e. Kp, Ki and Kd values for PID controller
errorCtrl defines the margin of error for each motor speed reading from required value*/
MyPID::MyPID(double k, double td, double ts, float _errorCtrl) {

  error[0] = 0;
  error[1] = 0;
  error[2] = 0;

  errorCtrl = _errorCtrl;

  double Kc, Ti, Td;
  double Kp, Ki, Kd;

  Kc =  (1.2 * ts) / (k * td);
  Ti =  2 * td;
  Td = 0.5 * td;

  Kp =  Kc;
  Ki = Kc / Ti;
  Kd =  Kc * Td;

  K1 = Kp + Ki + Kd;
  K2 = -Kp - 2 * Kd;
  K3 = Kd;
}

/* Updates error values before calculating new error values */
void MyPID::errorUpdate() {
  error[2] = error[1];
  error[1] = error[0];
}

/* Calculates new setSpeed using previous setSpeed, previous errors and direction of wheels*/
float MyPID::setWheelSpeed(float setMotorSpeed, int pinNo, float rpm, boolean forward) {

  float motorSpeed = getSpeed (pulseIn(pinNo, HIGH));  // Finds RPM using pulseIn function which returns the time period between two pulses

  //  Serial.println(motorSpeed);

  if (motorSpeed > 150)
    motorSpeed = 0;

  errorUpdate();
  error[0] = rpm - motorSpeed ;                       // Finds new error betweek required rpm and current speed

  if (fabs(error[0]) > errorCtrl ) {                  // If error greater than error control, fine new set speed
    if (motorSpeed < 150)
      setMotorSpeed = correctedSpeed(fabs(setMotorSpeed));
    else
      error[0] = 0;
    return forward ? setMotorSpeed : -setMotorSpeed;  // Add direction to new set speed value
  }
  return setMotorSpeed;
}

/* Function to find new set speed using calculated PID values on initialisation*/
float MyPID::correctedSpeed(float originalSpeed) {
  float newSpeed = 0;
  newSpeed = originalSpeed + error[0] * K1 + error[1] * K2 + error[2] * K3;
  if (newSpeed > 350)
    newSpeed = 350;
  return newSpeed;
}

/* Initialises all errors to 0 for next motion*/
void MyPID::MyPIDReboot() {
  error[0] = 0;
  error[1] = 0;
  error[2] = 0;
}

/* Calculated rpm from PWM line using time persiod between two high pulses*/
float MyPID::getSpeed (long pulseInTime) {
  long timePeriod = pulseInTime * 2;
  float rps = 1000000 / (timePeriod * 562.25);
  float rpm = rps * 60;
  return rpm;
}

#endif
