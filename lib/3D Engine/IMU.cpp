#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Functions.h>
#include <Matrix.h>
#include "IMU.h"

#include <vector>
using namespace std;

extern Adafruit_SSD1306 oled;

IMU::IMU()
{
  rotationMatrix = Matrix3x3::identity;

  position = Vector3<float>();
  rotation = Vector3<float>();
  velocity = Vector3<float>();
  angularVelocity = Vector3<float>();

  prevPosition = Vector3<float>();
  prevRotation = Vector3<float>();
  prevVelocity = Vector3<float>();
  prevAngularVelocity = Vector3<float>();

  accelZeroOffset = Vector3<float>();
  gyroZeroOffset = Vector3<float>();
}

IMU::~IMU(){}

void IMU::Reset()
{
  //mpu.reset();
  rotationMatrix = Matrix3x3::identity;

  position = Vector3<float>();
  rotation = Vector3<float>();
  velocity = Vector3<float>();
  angularVelocity = Vector3<float>();

  prevPosition = Vector3<float>();
  prevRotation = Vector3<float>();
  prevVelocity = Vector3<float>();
  prevAngularVelocity = Vector3<float>();
}

void IMU::Calibrate()
{
  const int nSamples = 1000;
  const int delayMillis = 4;// 250Hz sample rate. 1000ms/250 = 4ms
  Vector3<float> accelSamples = Vector3<float>(0.0, 0.0, 0.0);
  Vector3<float> gyroSamples = Vector3<float>(0.0, 0.0, 0.0);
  Vector3<float> accelError;
  Vector3<float> gyroError;

  Serial.println("Calibrating...");
  oled.clearDisplay();
  oled.println("Calibrating...");
  oled.display();

  Reset();
  
  mpu.getEvent(&a, &g, &temp);

  for (size_t i = 0; i < nSamples; i++) 
  {
    if (mpu.getEvent(&a, &g, &temp))
    { 
      accelSamples += a.acceleration.v;
      gyroSamples += g.gyro.v;
    }
    delay(delayMillis);
  }

  // Vector average calculated from sampled vector sum
  accelZeroOffset.x = accelSamples.x / (float) nSamples;
  accelZeroOffset.y = accelSamples.y / (float) nSamples;
  accelZeroOffset.z = accelSamples.z / (float) nSamples;

  gyroZeroOffset.x = gyroSamples.x / (float) nSamples;
  gyroZeroOffset.y = gyroSamples.y / (float) nSamples;
  gyroZeroOffset.z = gyroSamples.z / (float) nSamples;
  //error = Vector3<float>(abs(avg.x), abs(avg.y), abs(avg.z-9.81));

  oled.clearDisplay();
  print(accelZeroOffset, "Zero Offset Accel: ");
  print(gyroZeroOffset, "Zero Offset Gyro: ");
}

bool IMU::Init()
{
  if (!mpu.begin())
  {
    Serial.println("MPU not detected...");
    oled.clearDisplay();
    oled.println("MPU not detected...");
    oled.display();

    return false;
  }

  Serial.println("MPU detected!");
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);// Sensitivity Scale Factor: raw 2,048 = 1g = (2^15)/16
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);// Sensitivity Scale Factor: raw ~65.5 = 1 deg/s = (2^15)/500
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.getAccelerometerSensor()->printSensorDetails();
  mpu.getGyroSensor()->printSensorDetails();
  switch (mpu.getCycleRate())
  {
  case MPU6050_CYCLE_40_HZ:
    Serial.println("MPU6050_CYCLE_40_HZ");
    break;
  case MPU6050_CYCLE_20_HZ:
    Serial.println("MPU6050_CYCLE_20_HZ");
    break;
  case MPU6050_CYCLE_5_HZ:
    Serial.println("MPU6050_CYCLE_5_HZ");
    break;
  case MPU6050_CYCLE_1_25_HZ:
    Serial.println("MPU6050_CYCLE_1_25_HZ");
    break;
  default:
    Serial.println("Cycle: ...");
    break;
  }
  return true;
}

void IMU::Update() 
{
  //static const int MAX_SIZE_MPU_BUFFER = 10;
  //static std::vector<Rigidbody> buffer;//heap
  static unsigned long delayRead = 4000;// 1/250 = 0.004s
  static unsigned long lastStartTime = 0;
  static unsigned long mpuLastTimeSampled = 0;

  if (micros() - lastStartTime > delayRead) {
    lastStartTime = micros();

    if (mpu.getEvent(&a, &g, &temp)) 
    {
      //===================================================
      //                    PHYSICS
      //===================================================
      // Time difference between now and last sample
      float mpuDeltaTime = ((float)(millis() - mpuLastTimeSampled))/1000.0;
      mpuLastTimeSampled = millis();

      prevPosition = position;
      prevVelocity = velocity;
      prevRotation = rotation;
      prevAngularVelocity = angularVelocity;   

      // =============== GYRO RAD/S ================
      Vector3<float> gyroAngularVel = Vector3<float>(g.gyro.v) - gyroZeroOffset;// rad/s

      rotationMatrix *= YPR(gyroAngularVel.x * mpuDeltaTime, gyroAngularVel.y * mpuDeltaTime, gyroAngularVel.z * mpuDeltaTime);

      angularVelocity.x = RadToDeg(gyroAngularVel.x);
      angularVelocity.y = RadToDeg(gyroAngularVel.y);
      angularVelocity.z = RadToDeg(gyroAngularVel.z);
      
      rotation += angularVelocity * mpuDeltaTime; // delta rotate n degrees
      
      // =============== ACCELEROMETER M/S^2 ================
      Vector3<float> accel = (Vector3<float>(a.acceleration.v) - accelZeroOffset); // m/s/s
      
      velocity += accel * mpuDeltaTime;
      position += accel * (mpuDeltaTime * mpuDeltaTime);
    }
  }
}