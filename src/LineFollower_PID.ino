#include <QTRSensors.h>

// Motor Driver Pin Definitions
#define AIN1 5
#define AIN2 6
#define BIN1 9
#define BIN2 10

// Speed Configuration
int MAX_SPEED = 210;  // PWM Upper Limit (0-255)
int BASE_SPEED = 190; // Nominal cruising speed

QTRSensors qtr;

const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];
int threshold[SensorCount];

// PID Control Constants
float Kp = 3.63; // Proportional: Corrects current error
float Ki = 0.0;  // Integral: Corrects accumulated steady-state error 
float Kd = 6.77; // Derivative: Dampens oscillations and predicts future error

uint8_t multiP = 1, multiI = 1, multiD = 1;
float Pvalue, Ivalue, Dvalue;

uint16_t position;
int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
uint16_t lastPosition = 0;
bool onLine = true;

void setup() {
  // Initialize Motor Control Pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  // Configure QTR Sensor Array
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A1, A2, A3, A4, A5}, SensorCount);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Calibration Routine: Rotate/move robot over the line during this phase
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW);

  // Calculate dynamic threshold for each sensor
  for (uint8_t i = 0; i < SensorCount; i++) {
    threshold[i] = (qtr.calibrationOn.minimum[i] + qtr.calibrationOn.maximum[i]) / 2;
  }

  delay(1000);
}

void loop() {
  robot_control();
}

void robot_control() {
  position = readLine();
  // Setpoint is 2000 for a 5-sensor array (range 0-4000)
  error = 2000 - position;
  PID_Linefollow(error);
}

/**
 * Calculates line position and handles "off-line" scenarios
 */
uint16_t readLine() {
  uint16_t position = qtr.readLineBlack(sensorValues);
  onLine = false;
  bool allOnBlack = true;
  
  for (uint8_t i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > threshold[i]) {
      onLine = true;
    } else {
      allOnBlack = false;
    }
  }

  // Out of bounds handling: Keep turning in the last known direction
  if (!onLine) {
    if (lastPosition == (SensorCount - 1) * 1000 / 2)
      return lastPosition;
    
    if (lastPosition < (SensorCount - 1) * 1000 / 2) {
      return 0; // Turn sharp left
    } else {
      return (SensorCount - 1) * 1000; // Turn sharp right
    }
  }

  // Handle intersections/full black marks
  if (allOnBlack) {
    return lastPosition; 
  }

  lastPosition = position;
  return position;
}

/**
 * PID Controller implementation for motor speed adjustment
 */
void PID_Linefollow(int error) {
  P = error;
  I += error; // Accumulate integral error
  D = error - previousError; // Rate of change

  Pvalue = (Kp / pow(10, multiP)) * P;
  Ivalue = (Ki / pow(10, multiI)) * I;
  Dvalue = (Kd / pow(10, multiD)) * D;

  PIDvalue = Pvalue + Ivalue + Dvalue;
  previousError = error;

  // Differential drive logic
  lsp = BASE_SPEED - PIDvalue;
  rsp = BASE_SPEED + PIDvalue;

  // Default behavior if line is lost
  if (!onLine) {
    lsp = BASE_SPEED;
    rsp = BASE_SPEED;
  }

  motor_drive(lsp, rsp);
}

/**
 * Low-level motor control with speed constraints
 */
void motor_drive(int left, int right) {
  // Constrain speeds to predefined limits
  left = constrain(left, -MAX_SPEED, MAX_SPEED);
  right = constrain(right, -MAX_SPEED, MAX_SPEED);

  // Left Motor control
  analogWrite(AIN1, left > 0 ? left : 0);
  analogWrite(AIN2, left < 0 ? -left : 0);

  // Right Motor control
  analogWrite(BIN1, right > 0 ? right : 0);
  analogWrite(BIN2, right < 0 ? -right : 0);
}
