#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// State feedback controller variables
float theta = 1;
float theta_dot = 0;
float theta_dot_old = 0;
float theta_double_dot = 0;
float u = 0;
float omega = 0;
float omega_old = 0;
float pwmOutput = 0;
float voltage;

// Define Time
float dt = 0;
unsigned long prev_time = 0;

// Motor constants
const int R = 12;       // motor resistance 
const int Kt = 0.0343;  // toruqe constant 
const int Kv = 0.0343;  // EMF constant
const float I = 0.001;  // momement of inerita of motor+rxn wheel
const float b = 0.0001; // friction damping coeff.

// Motor control pins
const int motorDirectionPin = 12;
const int motorSpeedPin = 3;

// Motor setup
float motorMaxVoltage = 12;

// LQR Gains GAINS FROM MATLAB SIM
float K[4] = {-200, -129.6168, -40, -1.0346}; 

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
  readIMU(&theta, &theta_dot, &theta_double_dot);

  // Calculate control input
  calculateControlInput(K, theta, theta_dot, theta_double_dot, omega, &u);

  // Control the motor
  controlMotor(u);

  // Print data to the Serial Monitor
  printSerialMonitor(theta, theta_dot, u);
}

void readIMU(float *theta, float *theta_dot, float *theta_double_dot) {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  *theta = euler.y() * DEG_TO_RAD;
  *theta_dot = gyro.y() * DEG_TO_RAD;

  // Note: The BNO055 sensor measures raw accel & linear accel data.
  // This means we can't get the true angular accel of the pendulum from the IMU.
  // Thus, include a time sample and take a derivative... 
  // ...however, this will likely add noise into the system. May need to add a filter.

  // calculate time difference
  unsigned long current_time = micros();
  dt = (current_time - prev_time) / 1000000.0f; // convert to seconds

  // calculate angular acceleration
  *theta_double_dot = (*theta_dot - theta_dot_old) / dt;

  // update previous values
  theta_dot_old = *theta_dot;
  prev_time = current_time;

}

void calculateControlInput(float K[], float theta, float theta_dot, float theta_double_dot, float omega, float *u) {

    float states[4][1] = {{theta}, {theta_dot}, {theta_double_dot}, {omega}};
    *u = 0;
    for (int i = 0; i < 4; i++) {
    *u += -K[i] * states[i][0];
    }
    *u = *-u; // correct?

    // Note: calculate the velocity of the rxn wheel. Need a relationship between voltage (u) & wheel veloctiy (omega)...
    // can use a first-order transfer function in the form: G(s) = Kt/(Ls + R)*J +b... we can prob simplify it though.
    // b=damping coeff. due to friction of motor+rxn, s=laplace variable
    
    // motor transfer function gives us rxn wheel angular accel
    float omega_dot = (Kt / R) * (*u) - (b / I) * omega; // not sure what L is for our motor

    // get omega by using Euler's method
    omega += omega_dot * dt;

    // update previous values
    omega_old = omega;
}

void controlMotor(float u) {
  // convert torque to voltage
  // float voltage = u * R / Kt;
  float voltage = (u / Kt) + (omega * Kv); 
  
  // clamp the voltage output
  voltage = constrain(voltage, -motorMaxVoltage, motorMaxVoltage);
  int pwmOutput = (int) map(abs(voltage), 0, motorMaxVoltage, 0, 255);
  
  // motor direction
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
  Serial.println();
}

