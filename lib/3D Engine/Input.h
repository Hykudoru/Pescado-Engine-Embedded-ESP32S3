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
        cam->localRotation *= Matrix3x3::RotX(rate);
    }
    // LOOK DOWN
    if (rotate.y < 0) {
        cam->localRotation *= Matrix3x3::RotX(-rate);
    }
    // TURN LEFT
    if (rotate.x < 0) {
        cam->localRotation *= Matrix3x3::RotY(rate);
    }
    // TURN RIGHT
    if (rotate.x > 0) {
        cam->localRotation *= Matrix3x3::RotY(-rate);
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

    
//---------SERIAL INPUT-------------
  if(Serial.available())
  {
    char key = Serial.read();
    Serial.println(key);
    if (key == ' ')
    {
        return;
    }
    switch (key)
    {
    //Reset Camera
    case '0':
        Camera::main->localRotation = Matrix3x3::identity;
        Camera::main->localPosition = Vec3();
        break;
    //Switch Cameras
    case '1':
      Camera::main = Camera::cameras[1];
      break;
    case '2':
        Camera::main = Camera::cameras[2];
        break;
    case '3':  //Outsider perspective cam
        CameraSettings::outsiderViewPerspective = !CameraSettings::outsiderViewPerspective;
        Camera::projector->localRotation = Matrix3x3::identity;
        Camera::projector->localPosition = Vec3();
        break;
    //------------------Spawn-------------------
    case '9':  //Cube
       { 
            Mesh* mesh = nullptr;
            try {
                mesh = new CubeMesh();//LoadMeshFromOBJFile("Objects/Sphere.obj");
            }
            catch(const std::exception& e)
            {
                Serial.println(e.what());
            }
            if (mesh != nullptr) {
                mesh->localPosition = Camera::main->localPosition + (Camera::main->Forward() * 10);
                mesh->localRotation = Camera::main->localRotation;
            }

       }
        break;
    case '8':  //Sphere
        {
            Mesh* mesh = LoadMeshFromOBJFile("Sphere.obj");
            mesh->localPosition = Camera::main->localPosition + (Camera::main->Forward() * 10);
            mesh->localRotation = Camera::main->localRotation;
            break;
        }
    case '7':  //Diamond
        {
            Mesh* mesh = LoadMeshFromOBJFile("Diamond.obj");
            mesh->localPosition = Camera::main->localPosition + (Camera::main->Forward() * 10);
            mesh->localRotation = Camera::main->localRotation;
            mesh->localScale *= 0.1;
            mesh->color = Color::red;
        }
      break;
    case '6':  //Icosahedron
        {
            Mesh* mesh = LoadMeshFromOBJFile("Icosahedron.obj");
            mesh->localPosition = Camera::main->localPosition + (Camera::main->Forward() * 10);
            mesh->localRotation = Camera::main->localRotation;
            mesh->localScale *= 0.1;
            mesh->color = Color::purple;
        }
      break;
    //------------------Physics-------------------
    //Toggle Momentum
    case 'x': 
        velocity = Vec3(0, 0, 0);//Reset every toggle state
        isKinematic = !isKinematic;
        break;
    //Toggle Inertial Dampeners
    case 'z':  
        dampenersActive = !dampenersActive;
        break;
    //Toggle Collision Detection
    case 'p':  
        Physics::collisionDetection = !Physics::collisionDetection;
        break;
    //Toggle Gravity
    case 'g':  
        Physics::gravity = !Physics::gravity;
        break;
    //-------------------Debugging------------------------
    case 'i':
        Graphics::invertNormals = !Graphics::invertNormals;  
        break;
    case 'n':
        Graphics::debugNormals = !Graphics::debugNormals;
        break;
    case 'v': 
        Graphics::backFaceCulling = !Graphics::backFaceCulling;
        break;
    case 'f': 
        Graphics::fillTriangles = !Graphics::fillTriangles;
        break;
    case 'm': 
        Graphics::displayWireFrames = !Graphics::displayWireFrames;
        break;
    case ',': 
        Graphics::debugAxes = !Graphics::debugAxes;
        break;
    case 'l': 
        Graphics::lighting = !Graphics::lighting;
        break;
    case '4': 
        Graphics::vfx = !Graphics::vfx;
        break;
    case '5': 
        Graphics::matrixMode = !Graphics::matrixMode;
        break;
    default:
      break;
    }
  }
}

#endif