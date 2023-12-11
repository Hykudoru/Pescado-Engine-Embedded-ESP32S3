#pragma once
#ifndef INPUT_H
#define INPUT_H
#include <Graphics.h>
#include <Physics.h>
#include <MuxJoystick.h>

extern MuxJoystick leftJoystick;
extern MuxJoystick rightJoystick;
//-----------------Input----------------------
float sensitivity = .1;
Camera* cam;


/*
void RotateCamera(double x, double y)
{
    Vec3 joystick = rightJoystick.Read(100);
    static float rad = PI / 180;
    double xAngle = rad * sensitivity * x;// *deltaTime;
    double yAngle = 0.0000001 + rad * mouseSensitivity * y;// * deltaTime; // 0.0000001 so no gimbal lock
    if (CameraSettings::outsiderViewPerspective) {
        Camera::projector->rotation *= YPR(xAngle, yAngle, 0);
    }
    else {
        Camera::main->rotation *= YPR(xAngle, yAngle, 0);
    }
    //Camera::main->rotation = Matrix3x3::RotX(rotateSpeed * -deltaMouseY) * Camera::main->rotation * Matrix3x3::RotY((0.00001 + rotateSpeed) * -deltaMouseX);
}*/

static void CameraControl(Camera* cam)
{
    
    Vec2 move = leftJoystick.Read(100);
    Vec2 rotate = rightJoystick.Read(100);
    
    moveDir = Vec3(0, 0, 0);
    //----------Camera Controls-------
    // FORWARD
    if (move.y > 0) {
        moveDir += cam->Forward();
    }
    // BACK
    if (move.y < 0) {
        moveDir += cam->Back();
    }
    // LEFT
    if (move.x < 0) {
        moveDir += cam->Left();
    }
    // RIGHT
    if (move.x > 0) {
        moveDir += cam->Right();
    }

    moveDir.Normalize();
    rotate.Normalize();
    float rate = rotateSpeed * deltaTime;
    // LOOK UP
    if (rotate.y > 0) {
        cam->rotation *= Matrix3x3::RotX(rate);
    }
    // LOOK DOWN
    if (rotate.y < 0) {
        cam->rotation *= Matrix3x3::RotX(-rate);
    }
    // TURN LEFT
    if (rotate.x < 0) {
        cam->rotation *= Matrix3x3::RotY(rate);
    }
    // TURN RIGHT
    if (rotate.x > 0) {
        cam->rotation *= Matrix3x3::RotY(-rate);
    }
    
    // Speed 
    if (leftJoystick.isPressed ^ rightJoystick.isPressed)
    {
        accel = defaultAcceleration * 5;
    } else {
        accel = defaultAcceleration;
    }
}

void OnClickButton1() 
{
  moveDir += cam->Up();
}

void OnClickButton2() 
{
  moveDir += cam->Down();
}

static void Input()
{
    if (CameraSettings::outsiderViewPerspective)
    {
        cam = Camera::projector;
        CameraControl(Camera::projector);
    }
    else 
    {
        cam = Camera::main;
        CameraControl(Camera::main);
    }

    if (leftJoystick.isPressed)
    {
        //FOV(fieldOfViewDeg+5);
    }
    if (rightJoystick.isPressed)
    {
        //FOV(fieldOfViewDeg-5);
    }
}

#endif