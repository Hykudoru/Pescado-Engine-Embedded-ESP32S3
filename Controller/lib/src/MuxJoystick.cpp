
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_Qwiic_Joystick_Arduino_Library.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Vector.h>
#include <MuxJoystick.h>

QWIICMUX mux;
JOYSTICK rawJoystick;

void MuxJoystick::Start()
{
  //----- MUX joystick setup -------
  rawJoystick.begin();
  Wire.begin();
  mux.begin();
  //force read for potentially bad init readings
  Read();
}

Vector3<float> MuxJoystick::Read(int distanceRadius)
{
  static const int joystickADCResolution = 1024;
  static int rawMidpoint = joystickADCResolution / 2;
  static int rawAbsErrorOffset = 15;
  static unsigned long t = 0;
  Vector3<float> vec3;

  // Check frame to determine if joystick values are still current; 
  if (t == millis())
  {
    return vec3;
  }

  // New frame
  t = millis();

  mux.enablePort(muxPort);

  uint16_t rawX = rawJoystick.getHorizontal();// 0 - 1023
  uint16_t rawY = rawJoystick.getVertical();// 0 - 1023
  
  // Fix/reverse x axis
  if (rawX != rawMidpoint)
  {
    rawX = 1023 - rawX;
  }

  // Map X-axis Range [-100, 100] 
  if (rawX < (rawMidpoint - rawAbsErrorOffset))
  {
    // Left
    vec2.x = -map(rawX, 0, (rawMidpoint - rawAbsErrorOffset), distanceRadius, 0);

  }
  else if (rawX > (rawMidpoint + rawAbsErrorOffset))
  {
    // Right
    vec2.x = map(rawX, (rawMidpoint + rawAbsErrorOffset), 1023, 0, distanceRadius);
  }
  else {
    vec2.x = 0;
  }

  // Map Y-axis Range [-100, 100] 
  if (rawY < (rawMidpoint - rawAbsErrorOffset))
  {
    // Down
    vec2.y = -map(rawY, 0, (rawMidpoint - rawAbsErrorOffset), distanceRadius, 0);

  }
  else if (rawY > (rawMidpoint + rawAbsErrorOffset))
  {
    // Up
    vec2.y = map(rawY, (rawMidpoint + rawAbsErrorOffset), 1023, 0, distanceRadius);
  }
  else {
    vec2.y = 0;
  }
  
  // Invert axes if physically upside down
  if (invertH) {
    vec2.x *= -1.0;
  }
  if (invertV)
  {
    vec2.y *= -1.0;
  }
  
  vec3 = vec2;

  //Invert button so that pressed state means 1 = true else 0;
  isPressed = !rawJoystick.getButton();
  vec3.z = isPressed;

  Serial.println(String("Joystick_")+muxPort
  +" <x:"+vec3.x+", y:"+vec3.y+">"+"  pressed:"+(int)(isPressed)
  +" \t raw: <x:"+rawX+", y:"+rawY+">  pressed:"+rawJoystick.getButton());
  
  mux.disablePort(muxPort);
  
  return vec3;
};