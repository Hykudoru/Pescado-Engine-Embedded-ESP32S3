#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Vector.h>
#include <Matrix.h>
#ifndef IMU_H
#define IMU_H

class Rigidbody
{
    public:
        float mass;
        float momentOfInertia;
        Vector3<float> position;
        Vector3<float> rotation;
        Vector3<float> velocity;
        Vector3<float> angularVelocity;
        ulong timestamp;
        Rigidbody(){};
        ~Rigidbody(){};
};

class IMU
{
    Adafruit_MPU6050 mpu;
    sensors_event_t a, g, temp;
public:
    Matrix3x3 rotationMatrix;

    Vector3<float> position;
    Vector3<float> rotation;
    Vector3<float> velocity;
    Vector3<float> angularVelocity;

    Vector3<float> prevPosition;
    Vector3<float> prevRotation;
    Vector3<float> prevVelocity;
    Vector3<float> prevAngularVelocity;

    Vector3<float> accelZeroOffset;
    Vector3<float> gyroZeroOffset;
    
    IMU();
    ~IMU();
    bool Init();
    void Update();
    void Calibrate();
    void Reset();
};

#endif
