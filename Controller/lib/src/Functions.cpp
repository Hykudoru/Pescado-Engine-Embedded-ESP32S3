
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
#include "Functions.h"

extern Adafruit_SSD1306 oled;

byte clamp(byte &val, byte min, byte max) {
  if (val > max) 
  {
    val = max;
  }
  else if (val < min) {
    val = min;
  }
  
  return val;
}

int clamp(int &val, int min, int max) {
  if (val > max) 
  {
    val = max;
  }
  else if (val < min) {
    val = min;
  }
  
  return val;
}

float clamp(float &val, float min, float max) {
  if (val > max) 
  {
    val = max;
  }
  else if (val < min) {
    val = min;
  }
  
  return val;
}

void print(Vector3<float> vec, char header[])
{
    Serial.println("");

    Serial.print(header); 
    Serial.print(vec.x); Serial.print(", ");
    Serial.print(vec.y); Serial.print(", ");
    Serial.println(vec.z); 
}


void duelPrint(Vector3<int> vec, String header)
{
    Serial.println("");

    Serial.print(header); 
    Serial.print(vec.x); Serial.print(", ");
    Serial.print(vec.y); Serial.print(", ");
    Serial.println(vec.z); 

    oled.print(header); 
    oled.print(vec.x); oled.print(", ");
    oled.print(vec.y); oled.print(", ");
    oled.println(vec.z); 

    oled.display();
}
void duelPrint(Vector3<float> vec, String header)
{
    Serial.println("");

    Serial.print(header); 
    Serial.print(vec.x); Serial.print(", ");
    Serial.print(vec.y); Serial.print(", ");
    Serial.println(vec.z); 

    oled.print(header); 
    oled.print(vec.x); oled.print(", ");
    oled.print(vec.y); oled.print(", ");
    oled.println(vec.z); 

    oled.display();
}
