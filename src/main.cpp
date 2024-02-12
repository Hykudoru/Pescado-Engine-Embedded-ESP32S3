#include <FS.h>
#include <SPIFFS.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <U8g2lib.h>
#include <rm67162.h>
#include <TFT_eSPI.h>


#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <math.h>

#include <esp_now.h>
#include <WiFi.h>
#include <WirelessData.h>
//Alex Lib
#include <Vector.h>
#include <MuxJoystick.h>
#include <3D Engine.h>
#include <Matrix.h>
#include <Graphics.h>
#include <Physics.h>
#include <Input.h>
using namespace std;

const int BAUD_RATE = 115200;
//#define BUTTON_1 32//27
//#define BUTTON_2 4
#define POTENTIOMETER_1 0b100111// A3 (39)
#define POTENTIOMETER_2 0b100100// A4 (36)
const uint16_t ADC_RESOLUTION = 4095; // 0 - 4095
#define I2C_SCL 46
#define I2C_SDA 45

//pfunc can be reasigned at runtime to change the desired procedure invoked inside the default loop function.
typedef void (*pointerFunction)(void);
pointerFunction ptrMode;

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite display = TFT_eSprite(&tft);
Adafruit_SSD1306 oled = Adafruit_SSD1306(128, 32, &Wire);
extern MuxJoystick leftJoystick;
extern MuxJoystick rightJoystick;
extern unsigned int incomingCount;

// Checking if DEBUGGING true in other scripts before using cout also ensures readable slow incremental output.
bool DEBUGGING = false;
void Debug()
{
    //debugging
    DEBUGGING = false;
    static double coutTimer = 0;
    coutTimer += deltaTime;
    if (coutTimer > 0.25 && coutTimer < 0.25 + deltaTime)
    {
        DEBUGGING = true;
        coutTimer = 0;
    }

    if (DEBUGGING)
    {
        std::cout << "--------GRAPHICS-------" << endl;
        std::cout << "FPS:" << fps << std::endl;
        std::cout << "Frame Time:" << 1.0 / (double)fps << std::endl;
        std::cout << "Meshes:" << Mesh::count << std::endl;
        std::cout << "Triangles Drawn:" << Mesh::worldTriangleDrawCount << std::endl;
        std::cout << "FOV:" << fieldOfViewDeg << std::endl;
    }
}

Mesh* textHelloWorld;
Mesh* planet;
Mesh* spaceShip;
Mesh* spaceShip2;
Mesh* spaceShip3;
Mesh* spaceShip4;
CubeMesh* obj1;
CubeMesh* obj2;
CubeMesh* obj3;
CubeMesh* obj4;
static CubeMesh cube2 = CubeMesh(1, Vec3(-5, -5, 10));
static CubeMesh cube3 = CubeMesh(1, Vec3(-5, 5, -20));  
static CubeMesh cube4 = CubeMesh(1, Vec3(5, 5, -30));

void Init()
{
  GraphicSettings::matrixMode = true;
  GraphicSettings::lighting = false;
  GraphicSettings::displayWireFrames = true;
  GraphicSettings::fillTriangles = false;
  GraphicSettings::debugAxes = true;
  Physics::collisionDetection = false;
  FOV(60);
  cube2.color = &RGB::green;
  cube3.color = &RGB::red;
  cube4.color = &RGB::yellow;
  //Mesh* cube0 = new CubeMesh(1, Vec3(0, 0, 0));
  //Mesh* cube1 = new CubeMesh(1, Vec3(0, 0, -10));
   // CubeMesh* cube = new CubeMesh(1, Vec3());
   // cube->position += Direction::forward;

/*
    planet = LoadMeshFromOBJFile("Sphere.obj");
    planet->scale = Vec3(500, 500, 500);
    planet->position += Direction::forward * 1000;
    planet->color = &RGB::white;
    
    textHelloWorld = LoadMeshFromOBJFile("Hello3DWorldText.obj");
    textHelloWorld->scale = Vec3(2, 2, 2);
    textHelloWorld->position = Vec3(0, 0, -490);
    textHelloWorld->color = &RGB::green;
    
    
    spaceShip = LoadMeshFromOBJFile("SpaceShip_2.2.obj");
    spaceShip->position = Direction::left * 30 + Direction::forward * 10;

    spaceShip2 = LoadMeshFromOBJFile("SpaceShip_3.obj");
    spaceShip2->position = Direction::right * 40 + Direction::forward * 100;
    spaceShip2->rotation = Matrix3x3::RotY(PI);

    spaceShip3 = LoadMeshFromOBJFile("SpaceShip_5.obj");
    spaceShip3->position = Direction::right * 20 + Direction::up * 10;
    */

  spaceShip4 = LoadMeshFromOBJFile("SpaceShip_2.2.obj");
  spaceShip4->position = Direction::forward * 10;
/*
 
    Mesh* parent = new CubeMesh();
    Mesh* child = new CubeMesh();
    Mesh* grandchild = new CubeMesh();
    child->parent = parent;
    grandchild->parent = child;

    obj1 = new CubeMesh(1, Vec3(0, 10, 50), Vec3(0, 90, 0));
    obj2 = new CubeMesh(2, Vec3(0, 0, -2), Vec3(0, 90, 0));
    obj3 = new CubeMesh(2, Vec3(0, 0, -2), Vec3(0, 90, 0));
    obj4 = new CubeMesh(2, Vec3(0, 0, -2), Vec3(0, 90, 0));
    obj2->SetParent(obj1);
    obj3->SetParent(obj2);
    obj4->SetParent(obj3);
    obj1->color = &RGB::red;
    obj2->color = &RGB::orange;
    obj3->color = &RGB::yellow;
    obj4->color = &RGB::green;

*/
    //Mesh* guitar = LoadMeshFromOBJFile("Objects/Guitar.obj");
    //guitar->position += (Camera::main->Forward() * 10) + Camera::main->Right();
    //Mesh* chair = LoadMeshFromOBJFile("Objects/Chair.obj");
    //chair->position += (Camera::main->Forward() * 10) + Camera::main->Left();

    //Plane* plane = new Plane(1, Vec3(0, 0, 0), Vec3(0, 0, 0));
    /*for (size_t i = 1; i < Camera::cameras.size(); i++)//starts at 1 to avoid projector camera
    {
       Mesh* cameraMesh = LoadMeshFromOBJFile("Objects/Camera.obj");
       cameraMesh->SetParent(Camera::cameras[i]);
    }*/
}

void Update()
{
   float rotationSpeed = (PI / 2) * deltaTime;
   if (spaceShip4)
   {
    spaceShip4->rotation = Matrix3x3::RotY(-(PI / 10) * deltaTime) * spaceShip4->rotation;// *spaceShip2->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
   }
    cube2.rotation = Matrix3x3::RotX(rotationSpeed) * cube2.rotation;// *spaceShip2->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
    cube3.rotation = Matrix3x3::RotY(-rotationSpeed) * Matrix3x3::RotZ(rotationSpeed) * cube3.rotation;// *spaceShip2->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
    cube4.rotation = Matrix3x3::RotY(2*rotationSpeed) * Matrix3x3::RotZ(rotationSpeed) * cube4.rotation;// *spaceShip2->rotation;// MatrixMultiply(YPR(angle * ((screenWidth / 2)), angle * -((screenWidth / 2)), 0), Mesh.meshes[1].rotation);
}

TaskHandle_t Core0Task;
TaskHandle_t Core1Task;

void Core0(void * param)
{
  for(;;) 
  {
    Time();
    Input();
    Physics();
    Update();
  }
}

void Core1(void * param)
{
  for(;;) 
  {
    Time2();
    display.fillSprite(TFT_BLACK);
    Draw();
    display.drawString(String("Core 0 [")+fps+" fps]", 20,20,4);
    display.drawString(String("Core 1 [")+fps2+" fps]", 20,50,4);
    display.drawString(String("Packets Incoming: ")+incomingCount, 20,110,4);
    display.drawString(String("X:")+incomingData.leftJoystick[0]+ String(" Y:")+incomingData.leftJoystick[1], 20,170,4);
    display.drawString(String("X:")+incomingData.rightJoystick[0] + String(" Y:")+incomingData.rightJoystick[1], 20,200,4);
    lcd_PushColors(0, 0, 536, 240, (uint16_t*)display.getPointer());
  }
}

extern IMU hmd;
extern bool hmdAttached;

void setup() 
{
  Serial.begin(BAUD_RATE);

  if (!SPIFFS.begin())
  {
    Serial.println("Failed while mounting SPIFFS.");
  }
  
  // ============ INPUT SETUP ===========

  Wire.begin(I2C_SDA, I2C_SCL);
  SetupESPNOW();

  leftJoystick.Start();
  rightJoystick.Start();
  
  if(hmd.Init())
  {
    hmdAttached = true;
    //hmd.Calibrate();
    //Camera::main->rotation = hmd.rotationMatrix;
  }
  // INPUT_PULLUP button MUST be connected to GND
  // INPUT_PULLDOWN button MUST be connected to VCC
  //pinMode(BUTTON_A, INPUT_PULLUP);
  //pinMode(BUTTON_B, INPUT_PULLUP);
  //pinMode(BUTTON_C, INPUT_PULLUP);
 // pinMode(BUTTON_1, INPUT_PULLDOWN);
 // pinMode(BUTTON_2, INPUT_PULLDOWN);
  // RISING & FALLING are reversed if INPUT_PULLDOWN
  //attachInterrupt(BUTTON_1, OnClickButton1, RISING);
  //attachInterrupt(BUTTON_2, OnClickButton2, RISING);
  
  rm67162_init();
  lcd_setRotation(1);
  display.createSprite(536, 240);
  display.setSwapBytes(1);

  Init();
  
  xTaskCreatePinnedToCore(
    Core0, // TaskFunction_t
    "Core 0 Task", // name of task
    10000, // stack size in words
    NULL, // task params
    17, //Priority
    &Core0Task, // TaskHandle_t
    0 // core
  );

  xTaskCreatePinnedToCore(
    Core1, // TaskFunction_t
    "Core 1 Task", // name of task
    10000, // stack size in words
    NULL, // task params
    17, //Priority
    &Core1Task, // TaskHandle_t
    1 // core
  );
 }

// Compile
// Plug in USB
// (While program running) Hold Boot, Hold Reset, Release Reset, Release Boot
// Upload
// Press Reset
void loop() 
{  
}