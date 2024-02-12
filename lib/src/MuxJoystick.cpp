
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SparkFun_Qwiic_Joystick_Arduino_Library.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <Functions.h>
#include <Vector.h>
#include <MuxJoystick.h>

QWIICMUX mux;
JOYSTICK rawJoystick;
int joystickCount = 0;

void MuxJoystick::Start()
{
  static byte count = 0;

  //----- MUX joystick setup -------
  bool jsConnected = rawJoystick.begin();//(Wire, 0x20);
  Wire.begin();
  bool muxBegin = mux.begin();

  // if (!jsConnected) {
  //   joystickCount = 0;
  //   Serial.println("WARNING: No joysticks connected!");
  // }
  // else if (!mux.isConnected()) {
  //   joystickCount = 1;
  //   Serial.println("Only one joystick connected.");
  // } 
  // else if (!mux.enablePort(muxPort)) {
  //   Serial.println(String("WARNING: No Joystick on Mux port ") + muxPort + " enabled.");
  // } else {
  //   joystickCount++;
  //   Serial.println(String("Joystick on port ") +muxPort+" enabled.");
  // }

  Serial.println(String("---------")+muxPort+"----------");
  
  if (jsConnected) {
    if (++count == 1) {
      joystickCount = 1;
    }
    else if (mux.enablePort(muxPort)) {
      joystickCount++;
    }
    Serial.println("jsConnected = true");
  } else {
    Serial.println("jsConnected = false");
  }

  if (!muxBegin) {
    Serial.println("muxBegin == false");
  } else {
    Serial.println("muxBegin == true");
  }

  mux.setPort(muxPort);
  mux.enablePort(muxPort);
  Serial.println(mux.getPort());

  if (!mux.enablePort(muxPort)) {
    Serial.println(String("mux.enablePort(")+muxPort+")  == false");
  } else {
    Serial.println(String("mux.enablePort(")+muxPort+")  == true");
  }

  if (!mux.isConnected()) {
    Serial.println("mux.isConnected() == false");
  } else {
    joystickCount++;
    Serial.println("mux.isConnected() == true");
  }
  
  Serial.println(String("Joysticks: ")+joystickCount);
  Serial.println("-------------------");
  mux.disablePort(muxPort);

  Read();//force read for potentially bad init readings                                                       
}

Vector3<float> MuxJoystick::Read(int distanceRadius)
{
  static const int joystickADCResolution = 1024;
  static int rawMidpoint = joystickADCResolution / 2;
  static int rawAbsErrorOffset = 15;
  Vector3<float> vec3;

  // // return zero vector incase disconnected since values may still read false readings.
  if (!mux.isConnected() || !mux.enablePort(muxPort)) 
  {
    vec3 = Vector3<float>(0,0,0);
    return vec3;
  }

  if (mux.isConnected())
  {
    mux.enablePort(muxPort);
  }

  uint16_t rawX = rawJoystick.getHorizontal();// 0 - 1023
  uint16_t rawY = rawJoystick.getVertical();// 0 - 1023
  
  //------------ Fix reversed rawX axis to match left(-) right(+) standards for cartesian coords. --------------
  if (rawX != rawMidpoint)
  {
    rawX = 1023 - rawX;
  }

  // ==================================
  //            HORIZONTAL
  // ==================================
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

  // ==================================
  //              VERTICAL
  // ==================================
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

  //Always inverted so that pressed = 1, else 0;
  isPressed = !rawJoystick.getButton();
  vec3.z = isPressed;

  Serial.println(String("Joystick_")+muxPort
  +" <x:"+vec3.x+", y:"+vec3.y+">"+"  pressed:"+(int)(isPressed)
  +" \t raw: <x:"+rawX+", y:"+rawY+">  pressed:"+rawJoystick.getButton());
  
  if (mux.isConnected())
  {
    mux.disablePort(muxPort);
  }
  
  return vec3;
};