
#include<Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *M1 = AFMS.getMotor(1);
// You can also make another motor on port M2
Adafruit_DCMotor *M2 = AFMS.getMotor(2);

const int MPU=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  Serial.begin(9600);

  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
}

void loop(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  //  Serial.print("AcX = "); 
  //  Serial.print(AcX);
  //  Serial.print(" | AcY = "); 
  //  Serial.print(AcY);
  //Serial.print(" | AcZ = "); 
  //Serial.println(AcZ);
  //  Serial.print(" | Tmp = "); 
  //  Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  //  Serial.print(" | GyX = "); 
  //  Serial.print(GyX);
  //  Serial.print(" | GyY = "); 
  //  Serial.print(GyY);
  //  Serial.print(" | GyZ = "); 
  //  Serial.println(GyZ);
  //delay(333);

  int v = AcZ/8+GyY;
  if(v > 0) {
    M1->run(FORWARD);
    M2->run(FORWARD);
    v = min(255, v);
  }
  else if(v < 0) {
    M1->run(BACKWARD);
    M2->run(BACKWARD);
    v = min(255, -v);
  }
  else {
    M1->run(RELEASE);
    M2->run(RELEASE);
  }
  M1->setSpeed(v);  
  M2->setSpeed(v);  
}


