#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_PWMServoDriver.h>

// system parameters
const float m_wheel = 0.35;                     
const float m_length = 0.048;                  
const float m_motor = 0.11;                    
const float m_pend = m_motor + m_length;       
const float wt = 0.01;                         
const float radius_motor = 0.01;                
const float g = 9.81;  
const float Im = 2/5*m_motor*pow(radius_motor, 2) + m_motor*pow((L+radius_motor), 2); //motor mass moment of inertia
const float Is = 1/3*m_length*pow(L, 2) + Im;                                         //pendulum mass moment of inertia
const float Ir = 1/2*m_wheel*(pow(r_wheel, 2) + pow((r_wheel - wt), 2));              //wheel mass moment of inertia

// motor parameters
const float rated_voltage = 12;                 
const float rated_speed = 120 * 2 * PI / 60;    
const float rated_torque = 0.85 * 0.09806;      
const float stall_torque = 4.2 * 0.09806;       
const float I_stall = 1;                        

// motor constants
const float R = rated_voltage / I_stall;        
const float Kt = stall_torque / rated_voltage;  
const float Kv = rated_speed / rated_voltage;   

// LQR Gains ~ need to define values
float K1 = 0.2;
float K2 = 0.2;

const float K[2] = {K1, K2}; // k matrice

// state feedback controller variables
float theta = 0;
float theta_dot = 0;

// kalman filter variables
float angle_estimate = 0;
float angle_velocity_estimate = 0;
float angle_variance = 1;
float angle_velocity_variance = 1;
float angle_measurement_variance = 0.01;    // need to adjust
float angle_velocity_measurement_variance = 0.01; // need to adjust 
float angle_kalman_gain = 0;
float angle_velocity_kalman_gain = 0;

// motor setup
int motorPin = 1; // 
float motorMaxVoltage = 12; // our maximum voltage will be 6 or 12volts

// IMU setup
Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // initialize IMU
  if (!bno.begin()) {
    Serial.print("IMU not detected!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  
  // initialize motor control pin
  pinMode(motorPin, OUTPUT);
  analogWrite(motorPin, 0);

}

void loop() {
 
  // read the IMU data
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  float angle_measurement = euler.x() * DEG_TO_RAD;
  float angle_velocity_measurement = gyro.x() * DEG_TO_RAD;
  float u = 0;
  
  // kalman filter for angle
  angle_kalman_gain = angle_variance / (angle_variance + angle_measurement_variance);
  angle_estimate = angle_estimate + angle_kalman_gain * (angle_measurement - angle_estimate);
  angle_variance = (1 - angle_kalman_gain) * angle_variance;
  
  // kalman filter for angular velocity
  angle_velocity_kalman_gain = angle_velocity_variance / (angle_velocity_variance + angle_velocity_measurement_variance);
  angle_velocity_estimate = angle_velocity_estimate + angle_velocity_kalman_gain * (angle_velocity_measurement - angle_velocity_estimate);
  angle_velocity_variance = (1 - angle_velocity_kalman_gain) * angle_velocity_variance;

  // update state variables with kalman filtered values
  theta = angle_estimate;
  theta_dot = angle_velocity_estimate;

  // calculate control input
  float states[2][1] = {{theta},{theta_dot}};
  
  for (int i = 0; i < 2; i++) {
    u += -K[i] * states[i][0];
  }
  
  // convert torque to voltage
  float voltage = u * R / Kt;

  // clamp the voltage output ~ simulation should tell us this value 
  voltage = constrain(voltage, -motorMaxVoltage, motorMaxVoltage);
  
  int pwmValue = (int) map(voltage, -motorMaxVoltage, motorMaxVoltage, 0, 255);
  
  // output to control the motor
  
  analogWrite(motorPin, pwmValue);
  
  // add delay?
  delay(10);
}
