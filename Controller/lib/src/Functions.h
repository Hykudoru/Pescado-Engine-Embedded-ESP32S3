#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MotorShield.h>
#include <utility/Adafruit_MS_PWMServoDriver.h>
#include <Vector.h>

// template <typename T>
// void Print(Vector3<T> vec, String header = "");
byte clamp(byte &val, byte min, byte max);
int clamp(int &val, int min, int max);
float clamp(float &val, float min, float max);

void print(Vector3<float> vec, char header[]);
void duelPrint(Vector3<int> vec, String header = "");
void duelPrint(Vector3<float> vec, String header = "");