#include <Vector.h>
#ifndef MUXJOYSTICK_H
#define MUXJOYSTICK_H

class MuxJoystick
{
  //int x = 0;// [-100, 100]
  //int y = 0;// [-100, 100]
  bool isPressed = 0; // 1
  Vector2<float> vec2;
public:  
  int muxPort;
  bool invertH; 
  bool invertV;
  MuxJoystick(int muxPort, bool invertHorizontal = true, bool invertVertical = true)
  {
    this->muxPort = muxPort;
    invertH = invertHorizontal;
    invertV = invertVertical;
    vec2 = Vector2<float>();
  }
  ~MuxJoystick() {}
  void Start();
  Vector3<float> Read(int distanceRadius = 512);
};
#endif