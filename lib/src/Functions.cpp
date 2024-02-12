
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Vector.h>
#include "Functions.h"

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

void Pause(char resumeKey) 
{ 
  if (Serial.available()) 
  { 
    Serial.println("Paused... Press 'p' to resume.");
    while (Serial.read() != resumeKey) 
    {
        delay(2);
    }
  }
}

float RadToDeg(float radians)
{
  float degrees = radians * 180.0 / PI;
  return degrees;
}