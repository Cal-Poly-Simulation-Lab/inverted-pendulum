#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// State feedback controller variables
float theta = 0;
float theta_dot = 0;
float u = 0;
float pwmOutput = 0;
float voltage;

// Motor constants
const int R = 6;
const int Kt = 0.0343;

// Motor control pins
const int motorDirectionPin = 12;
const int motorSpeedPin = 3;

// Motor setup
float motorMaxVoltage = 6;

// LQR Gains GAINS FROM MATLAB SIM
float K[3] = {-162.1, -17.6168, -2.0346}; 

Adafruit_BNO055 bno = Adafruit_BNO055();

// set up IMU and motor 
void setup() {
 Serial.begin(57600);
  Wire.begin();

  if (!bno.begin()) {
  Serial.print("IMU not detected!");
  while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  pinMode(motorDirectionPin, OUTPUT);
  pinMode(motorSpeedPin, OUTPUT);
}

void loop() {
  // Read sensor data
  readIMU(&theta, &theta_dot);

  // Calculate control input
  calculateControlInput(K, theta, theta_dot, &u);

  // Control the motor
  controlMotor(u);

  // Print data to the Serial Monitor
  printSerialMonitor(theta, theta_dot, u);
}

void readIMU(float *theta, float *theta_dot) {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  *theta = euler.x() * DEG_TO_RAD;
  *theta_dot = gyro.x() * DEG_TO_RAD;
}

void calculateControlInput(float K[], float theta, float theta_dot, float *u) {
    float states[2][1] = {{theta}, {theta_dot}};
    *u = 0;
  
  for (int i = 0; i < 2; i++) {
    *u += -K[i] * states[i][0];
  }
}

void controlMotor(float u) {
  // convert torque to voltage
  float voltage = u * R / Kt; 
  
  // clamp the voltage output
  voltage = constrain(voltage, -motorMaxVoltage, motorMaxVoltage);
  int pwmOutput = (int) map(abs(voltage), 0, motorMaxVoltage, 0, 255);
  
  // setting the motor direction
  if (voltage >= 0) {
    digitalWrite(motorDirectionPin, LOW);
  } else {
    digitalWrite(motorDirectionPin, HIGH);
  }

  // output to control the motor
  analogWrite(motorSpeedPin, pwmOutput);
}

void printSerialMonitor(float theta, float theta_dot, float u) {
  Serial.print("Angle: ");
  Serial.print(theta*RAD_TO_DEG);
  Serial.print(" rad\t");
  Serial.print("Angular Velocity: ");
  Serial.print(theta_dot*RAD_TO_DEG);
  Serial.print(" rad/s\t");
  Serial.print("Control Input: ");
  Serial.print(u);
  Serial.print(" V\t");
  Serial.print(" Voltage");
  Serial.print(voltage);
  Serial.print(" V\t");
  Serial.println();
}
