#pragma once
#ifndef INPUT_H
#define INPUT_H
#include <MuxJoystick.h>
#include <Graphics.h>
#include <Physics.h>
#include <esp_now.h>
#include <WiFi.h>
#include <WirelessData.h>
const int LEFT_JOYSTICK_MUX_PORT = 0;
const int RIGHT_JOYSTICK_MUX_PORT = 3;
MuxJoystick leftJoystick(LEFT_JOYSTICK_MUX_PORT, false, false);
MuxJoystick rightJoystick(RIGHT_JOYSTICK_MUX_PORT, false, false);
#include <IMU.h>
IMU hmd = IMU();//Head Mounted Display
bool hmdAttached = false;
//==================================================================
//                  ESPNOW WIRELESS COMMUNICATION  
//==================================================================
//Original Drone MAC address = 94:B9:7E:5F:51:40
// Original controller MAC address 0x0C, 0xDC, 0x7E, 0xCA, 0xD2, 0x34
// Lilygo 84:FC:E6:64:2E:68
const int MAX_DATA_BUFFER_SIZE = 10;
const unsigned long INCOMING_DATA_LIFETIME = 50UL;
uint8_t selfMACAddress[] {0x84, 0xFC, 0xE6, 0x64, 0x2E, 0x68}; 
uint8_t broadcastMACAddress[] {0x94, 0xB9, 0x7E, 0x5F, 0x51, 0x40}; 
esp_now_peer_info_t peerInfo;
//DroneData outgoingData = DroneData();
JoystickControllerData incomingData = JoystickControllerData();
JoystickControllerData* ptrInput = &incomingData;
//WirelessData incomingDataBuffer[MAX_DATA_BUFFER_SIZE];
unsigned int outgoingSuccessCount = 0;
unsigned int outgoingFailCount = 0;
unsigned int outgoingCount = 0;
unsigned int incomingCount = 0;
unsigned long timeSinceLastIncoming = 0;
bool newInputReceived = false;

void OnDataReceived(const uint8_t *mac, const uint8_t *data, int length)
{
  // static int count = -1;
  // count = (count + 1) > MAX_DATA_BUFFER_SIZE ? 0 : count + 1;
  
  memcpy(&incomingData, data, length);
  ptrInput = &incomingData;
  
  timeSinceLastIncoming = 0;
  newInputReceived = true;
  incomingCount++;
  //Serial.println("------INCOMING------");
 }

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS) {
    outgoingSuccessCount++;
  }
  else if (status == ESP_NOW_SEND_FAIL) {
    outgoingFailCount++;
  }
}

void SetupESPNOW()
{
  WiFi.mode(WIFI_MODE_STA);
  Serial.println("MAC Address: "+WiFi.macAddress());
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP_NOW failed to init");
    return;
  }
  esp_now_register_recv_cb(OnDataReceived);
  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastMACAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  switch (esp_now_add_peer(&peerInfo))
  {
  case ESP_ERR_ESPNOW_EXIST:
    Serial.println("ESP_NOW Error: Peer already existed!");
    delay(2000);
  break;
  case ESP_OK:
    Serial.println("ESP_NOW: Peer added!");
    delay(2000);
    break;
  default:
    break;
  }
  // if (esp_now_add_peer(&peerInfo) != ESP_OK)
  // {
  //   Serial.println("ESP_NOW failed to add peer");
  //   return;
  // }
}
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
    Vec3 move = leftJoystick.Read(100) + Vec3(incomingData.leftJoystick[0], incomingData.leftJoystick[1], incomingData.leftJoystickIsPressed);
    Vec3 rotate = rightJoystick.Read(100) + Vec3(incomingData.rightJoystick[0], incomingData.rightJoystick[1], incomingData.rightJoystickIsPressed);
    
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
    cam->rotation *= Matrix3x3::RotY(-rotate.x * rate) * Matrix3x3::RotX(rotate.y * rate);
    // // LOOK UP
    // if (rotate.y > 0) {
    //     cam->rotation *= Matrix3x3::RotX(rate);
    // }
    // // LOOK DOWN
    // if (rotate.y < 0) {
    //     cam->rotation *= Matrix3x3::RotX(-rate);
    // }
    // // TURN LEFT
    // if (rotate.x < 0) {
    //     cam->rotation *= Matrix3x3::RotY(rate);
    // }
    // // TURN RIGHT
    // if (rotate.x > 0) {
    //     cam->rotation *= Matrix3x3::RotY(-rate);
    // }
    
    // Speed 
    
    if ((move.z == 1 || incomingData.leftJoystick[2] == 1) ^ (rotate.z || incomingData.rightJoystick[2] == 1))
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
      //----------REMOTE INPUT-------------
  // Checks for stale data. Simulates continuous input if the last valid input state becomes stale/old from waiting too long for new incoming data.
  timeSinceLastIncoming += deltaTimeMillis;
  if (timeSinceLastIncoming >= INCOMING_DATA_LIFETIME) 
  {
    ptrInput->leftJoystick[0] = 0;
    ptrInput->leftJoystick[1] = 0;
    ptrInput->rightJoystick[0] = 0;
    ptrInput->rightJoystick[1] = 0;
    timeSinceLastIncoming = 0;
  }


  if (CameraSettings::outsiderViewPerspective) {
      cam = Camera::projector;
  }
  else {
      cam = Camera::main;
  }

  if (hmdAttached)
  {
    hmd.Update();
    cam->rotation = hmd.rotationMatrix;
    CameraControl(cam);
    hmd.rotationMatrix = cam->rotation;
  }
  else {
    CameraControl(cam);
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
        Camera::main->rotation = Matrix3x3::identity;
        Camera::main->position = Vec3();
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
        Camera::projector->rotation = Matrix3x3::identity
;
        Camera::projector->position = Vec3();
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
                mesh->position = Camera::main->position + (Camera::main->Forward() * 10);
                mesh->rotation = Camera::main->rotation;
            }

       }
        break;
    case '8':  //Sphere
        {
            Mesh* mesh = LoadMeshFromOBJFile("Sphere.obj");
            mesh->position = Camera::main->position + (Camera::main->Forward() * 10);
            mesh->rotation = Camera::main->rotation;
            break;
        }
    case '7':  //Diamond
        {
            Mesh* mesh = LoadMeshFromOBJFile("Diamond.obj");
            mesh->position = Camera::main->position + (Camera::main->Forward() * 10);
            mesh->rotation = Camera::main->rotation;
            mesh->scale *= 0.1;
            mesh->color = &RGB::red;
        }
      break;
    case '6':  //Icosahedron
        {
            Mesh* mesh = LoadMeshFromOBJFile("Icosahedron.obj");
            mesh->position = Camera::main->position + (Camera::main->Forward() * 10);
            mesh->rotation = Camera::main->rotation;
            mesh->scale *= 0.1;
            mesh->color = &RGB::purple;
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
        GraphicSettings::invertNormals = !GraphicSettings::invertNormals;  
        break;
    case 'n':
        GraphicSettings::debugNormals = !GraphicSettings::debugNormals;
        break;
    case 'v': 
        GraphicSettings::backFaceCulling = !GraphicSettings::backFaceCulling;
        break;
    case 'f': 
        GraphicSettings::fillTriangles = !GraphicSettings::fillTriangles;
        break;
    case 'm': 
        GraphicSettings::displayWireFrames = !GraphicSettings::displayWireFrames;
        break;
    case ',': 
        GraphicSettings::debugAxes = !GraphicSettings::debugAxes;
        break;
    case 'l': 
        GraphicSettings::lighting = !GraphicSettings::lighting;
        break;
    case '4': 
        GraphicSettings::vfx = !GraphicSettings::vfx;
        break;
    case '5': 
        GraphicSettings::matrixMode = !GraphicSettings::matrixMode;
        break;
    default:
      break;
    }
  }
}

#endif