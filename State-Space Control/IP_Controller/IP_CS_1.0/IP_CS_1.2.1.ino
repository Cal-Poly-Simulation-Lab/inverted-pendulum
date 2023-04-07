#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

const double m_wheel = 0.35;
const double m_length = 0.048;
const double m_motor = 0.11;
const double m_pend = m_motor + m_length;
const double r_wheel = 0.0875;
const double wt = 0.01;
const double radius_motor = 0.01;
const double L = 0.16;
const double g = 9.81;
const double Im = 2/5*m_motor*pow(radius_motor, 2) + m_motor*pow((L+radius_motor), 2);
const double Is = 1/3*m_length*pow(L, 2) + Im;
const double Ir = 1/2*m_wheel*(pow(r_wheel, 2) + pow((r_wheel - wt), 2));

// Motor parameters
float rated_voltage = 10;
float rated_speed = 100 * 2 * PI / 60;
float rated_torque = 0.85 * 0.09806;
float stall_torque = 4.2 * 0.09806;
float I_stall = 1;

// Motor constants
float Resistor = 1;
float R = rated_voltage / I_stall;
float Kt = stall_torque * Resistor / rated_voltage;
float Kv = rated_speed / rated_voltage;

// LQR Gains 
float K[2] = {0.005, 0.05}; 

// State feedback controller variables
float theta = 0;
float theta_dot = 0;
float u = 0;
float pwmOutput = 0;
float voltage;


// Motor control pins
const int motorDirectionPin = 12;
const int motorSpeedPin = 3;

// Motor setup
float motorMaxVoltage = 6;

// State-space matrices (global scope)
float A[3][3] = {{0, 1, 0}, {m_pend*g*L/Is, 0, Kt*Kt/(R*Is)}, {0, 0, -Kt*Kt /(R*Ir)}};
float B[3][1] = {{0}, {-Kt/(R*Is)}, {Kt/(R*Ir)}};
float C[1][3] = {{1, 0, 0}};
float D = 0;


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

  *theta = euler.y() * DEG_TO_RAD;
  *theta_dot = gyro.y() * DEG_TO_RAD;
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
  float voltage = u * R / Kt; // u is torque here
  
  // clamp the voltage output
  voltage = constrain(voltage, -motorMaxVoltage, motorMaxVoltage);
  int pwmOutput = (int) map(abs(voltage), 0, motorMaxVoltage, 0, 255);
  
  // Set motor direction
  if (voltage >= 0) {
    digitalWrite(motorDirectionPin, HIGH);
  } else {
    digitalWrite(motorDirectionPin, LOW);
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