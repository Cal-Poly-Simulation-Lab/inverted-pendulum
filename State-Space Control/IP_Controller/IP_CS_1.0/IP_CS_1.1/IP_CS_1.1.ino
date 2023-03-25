#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_PWMServoDriver.h>

// system parameters
const double m_wheel = 0.35;                     
const double m_length = 0.048;                  
const double m_motor = 0.11;                    
const double m_pend = m_motor + m_length;
const double r_wheel = 0.0875;        
const double wt = 0.01;                         
const double radius_motor = 0.01;
const double L = 0.16;                 
const double g = 9.81;
const double Im = 2/5*m_motor*pow(radius_motor, 2) + m_motor*pow((L+radius_motor), 2); //motor mass moment of inertia
const double Is = 1/3*m_length*pow(L, 2) + Im;                                         //pendulum mass moment of inertia
const double Ir = 1/2*m_wheel*(pow(r_wheel, 2) + pow((r_wheel - wt), 2));              //wheel mass moment of inertia

// motor parameters ~ from data sheet
const float rated_voltage = 12;                 
const float rated_speed = 120 * 2 * PI / 60;    
const float rated_torque = 0.85 * 0.09806;      
const float stall_torque = 4.2 * 0.09806;       
const float I_stall = 1;                        

// motor constants
const float Resistor = 1;
const float R = rated_voltage / I_stall;        
const float Kt = stall_torque * Resistor / rated_voltage;  
const float Kv = rated_speed / rated_voltage;   

// LQR Gains ~ need to define values
float K1 = 0.2;
float K2 = 0.2;
float K[2] = {K1, K2}; // k matrice

// state feedback controller variables
float theta = 0;
float theta_dot = 0;
float u = 0;

// motor setup
int motorPin = 1;
float motorMaxVoltage = 12; // our maximum voltage will be 6 or 12v

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  Serial.begin(57600);
  Wire.begin();

  if (!bno.begin()) { // initialize IMU
    Serial.print("IMU not detected!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  pinMode(motorPin, OUTPUT); // initialize motor
  analogWrite(motorPin, 0);
 
  // matrices
  float A[3][3] = {{0, 1, 0}, {m_pend*g*L/Is, 0, Kt*Kt/(R*Is)},{0, 0, -Kt*Kt /(R*Ir)}};
  float B[3][1] = {{0},{-Kt/(R*Is)},{Kt/(R*Ir)}};
  float C[1][3] = {{1, 0, 0}};
  float D = 0;
}

void loop() {
  // read the IMU data
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  float theta = euler.x() * DEG_TO_RAD;
  float theta_dot = gyro.x() * DEG_TO_RAD;
  
  // calculate control input using LQR gain matrix
  float states[2][1] = {{theta},{theta_dot}};
  for (int i = 0; i < 2; i++) {
    u += -K[i] * states[i][0];
  }

  // convert torque to voltage
  float voltage = u * R / Kt;

  // clamp the voltage output ~ simulation should tell us this value 
  voltage = constrain(voltage, -motorMaxVoltage, motorMaxVoltage);
  int pwmOutput = (int) map(voltage, -motorMaxVoltage, motorMaxVoltage, 0, 180);
  
  // output to control the motor
  analogWrite(motorPin, pwmOutput);
  
  // print to serial monitor
  Serial.print("Angle: ");
  Serial.print(theta);
  Serial.print(" rad\t");
  Serial.print("Angular Velocity: ");
  Serial.print(theta_dot);
  Serial.print(" rad/s\t");
  Serial.print("Control Input: ");
  Serial.print(u);
  Serial.print(" V\t");
  Serial.println();

}
