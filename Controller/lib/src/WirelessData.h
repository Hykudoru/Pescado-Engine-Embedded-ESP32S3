#ifndef DATA_H
#define DATA_H
#include <Vector.h>

typedef struct WirelessData
{
  public:
  int ID;
};
//ESPNOW limits 0-250 bytes
typedef struct DroneData: public WirelessData
{
  public:
  Vector3<float> Acceleration;
  Vector3<float> AngularVelocity;
};
//ESPNOW limits 0-250 bytes
typedef struct JoystickControllerData: public WirelessData
{
  public:
  float leftJoystick[2] = {0, 0};
  float rightJoystick[2] = {0, 0};
  bool leftJoystickIsPressed;
  bool rightJoystickIsPressed;
  uint16_t Potentiometer;
};
#endif