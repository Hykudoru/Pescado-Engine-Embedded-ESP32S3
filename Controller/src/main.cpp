#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>
//Alex Lib
#include <Functions.h>
#include <Vector.h>
#include <MuxJoystick.h>
#include <WirelessData.h>

#if defined(ESP32)
  const int BAUD_RATE = 115200;
  #define BUTTON_A 15
  #define BUTTON_B 32
  #define BUTTON_C 14
  #define BUTTON_1 27
  #define BUTTON_2 4
  #define POTENTIOMETER_1 0b100111// A3 (39)
  #define POTENTIOMETER_2 0b100100// A4 (36)
  const uint16_t ADC_RESOLUTION = 4095; // 0 - 4095
#endif
#if defined(__AVR_ATmega32U4__)
  const int BAUD_RATE = 9600;
  #define BUTTON_A 9
  #define BUTTON_B 6
  #define BUTTON_C 5
#endif
const int LEFT_JOYSTICK_MUX_PORT = 0;
const int RIGHT_JOYSTICK_MUX_PORT = 3;
MuxJoystick leftJoystick(LEFT_JOYSTICK_MUX_PORT, false, false);
MuxJoystick rightJoystick(RIGHT_JOYSTICK_MUX_PORT, false, false);
/*
uint16_t rawPot1Value;
uint16_t rawPot2Value;
unsigned long* ptrPot1;
unsigned long* ptrPot2;
*/
//pfunc can be reasigned at runtime to change the desired procedure invoked inside the default loop function.
typedef void (*pointerFunction)(void);
pointerFunction ptrMode;


Adafruit_SSD1306 oled = Adafruit_SSD1306(128, 32, &Wire);

unsigned long timeDelay = 0;
unsigned long deltaTimeMillis = 0.0;//time difference (in milliseconds) between each loop;
unsigned long deltaTimeMicros = 0.0;//time difference (in microseconds) between each loop

void Time() {
  
  static unsigned long prevMillisTime = millis();
  static unsigned long prevMicrosTime = micros();

  deltaTimeMillis = (millis() - prevMillisTime);
  prevMillisTime = millis();

  deltaTimeMicros = (micros() - prevMicrosTime);
  prevMicrosTime = micros();
}

//================ ESPNOW WIRELESS COMMUNICATION DATA VARS ================

uint8_t selfMACAddress[] {0x94, 0xB9, 0x7E, 0x5F, 0x51, 0x40};//Controller
uint8_t broadcastMACAddress[] {0x84, 0xFC, 0xE6, 0x64, 0x2E, 0x68};//HMD
esp_now_peer_info_t peerInfo;
JoystickControllerData outgoingData;
const int MAX_DATA_BUFFER_SIZE = 10;
DroneData incomingData;
DroneData incomingDataBuffer[255];
int outgoingSuccessCount = 0;
int outgoingFailCount = 0;
int outgoingCount = 0;
int incomingCount = 0;
unsigned long sendDelay = 0;

void OnDataReceived(const uint8_t *mac, const uint8_t *data, int length)
{
    Serial.println("Received: "+ (++incomingCount));
    memcpy(&incomingData, data, sizeof(data));
}

void OnDataSent(const uint8_t *mac, esp_now_send_status_t status)
{
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Sent: "+ (++outgoingSuccessCount));
  }
  else if (status == ESP_NOW_SEND_FAIL) {
    Serial.println("Sent: "+ (++outgoingFailCount));
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

  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    Serial.println("ESP_NOW failed to add peer");
    delay(5000);
    return;
  }
}
bool bindingSet = false;

void DisplayMode1() 
{
  oled.clearDisplay();
  // oled.setCursor(0, 0);
  // oled.println(String("Left Joystick (")+(bool)outgoingData.LeftJoystick.z+")");
  // oled.println(String("")+"X:"+outgoingData.LeftJoystick.x+" Y:"+outgoingData.leftJoystickY;
  // oled.println(String("Right Joystick (")+(bool)outgoingData.RightJoystick.z+")");
  // oled.println(String("")+"X:"+outgoingData.rightJoystickX+" Y:"+outgoingData.rightJoystickY);
  oled.display();
}

void DisplayMode2() 
{
  // if (!ptrPot1 || ptrPot1 != &sendDelay) {
  //   bindingSet = false;
  // }
  // if (outgoingData.RightJoystick.z)
  // {
  //   bindingSet = !bindingSet;
  //   ptrPot1 = bindingSet ? &sendDelay : NULL;
  // }
  oled.clearDisplay();
  oled.setCursor(0, 0);
  oled.setTextColor(WHITE, BLACK);
  if (bindingSet) {
    oled.setTextColor(BLACK, WHITE);
  }
  oled.println(String("Send rate:")+1000.0f/(float)sendDelay+"Hz");
  oled.setTextColor(WHITE, BLACK);
  oled.println(String("Outgoing:")+outgoingCount);
  oled.println(String("Success:")+outgoingSuccessCount+", Failed:"+outgoingFailCount);
  oled.println(String("Incoming:")+incomingCount);
  oled.display();
}



void DisplayMode3() 
{
  // oled.clearDisplay();
  // oled.setCursor(0, 0);
  // oled.println(String("Potentiometer(1): ")+rawPot1Value);
  // oled.println(String("Potentiometer(2): ")+rawPot2Value);
  // oled.display();
}

void DisplayMode4() 
{
  //static bool bindingSet = false;
  // if (!ptrPot1 || ptrPot1 != &timeDelay) {
  //   bindingSet = false;
  // }
  // if (outgoingData.RightJoystick.z)
  // {
  //   bindingSet = !bindingSet;
  //   ptrPot1 = bindingSet ? &timeDelay : NULL;
  // }

  // oled.clearDisplay();
  // oled.setCursor(0, 0);
  // oled.setTextColor(WHITE, BLACK);
  // if (bindingSet) {
  //   oled.setTextColor(BLACK, WHITE);
  // }
  // oled.println(String("Display rate:")+1000.0f/(float)timeDelay+"Hz");
  // oled.setTextColor(WHITE, BLACK);

  // oled.display();
}

static pointerFunction modes[] = {DisplayMode1, DisplayMode2, DisplayMode3, DisplayMode4};
static int modeIndex = 0;
static int prevModeIndex = 0;
/*
void OnClickButton1() 
{
  modeIndex = clamp(--modeIndex, 0, 3);
  ptrMode = modes[modeIndex];
  Serial.println("Button Pressed");
}

void OnClickButton2() 
{
  modeIndex = clamp(++modeIndex, 0, 3);
  ptrMode = modes[modeIndex];
  Serial.println("Button 2 Pressed");
}
*/
void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(BAUD_RATE);
 
  // ============ OLED DISPLAY SETUP ===========

  oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  oled.display();//displays initial adafruit image
  oled.clearDisplay();//clears initial adafruit image
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.println("Setup...");
  oled.display();
  delay(100);
  oled.clearDisplay();

  ptrMode = &DisplayMode1;

  // ============ INPUT SETUP ===========

  //ptrPot1 = &sendDelay;
  leftJoystick.Start();
  rightJoystick.Start();

  // INPUT_PULLUP button MUST be connected to GND
  // INPUT_PULLDOWN button MUST be connected to VCC
  /*
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  pinMode(BUTTON_1, INPUT_PULLDOWN);
  pinMode(BUTTON_2, INPUT_PULLDOWN);
  // RISING & FALLING are reversed if INPUT_PULLDOWN
  attachInterrupt(BUTTON_1, OnClickButton1, RISING);
  attachInterrupt(BUTTON_2, OnClickButton2, RISING);
*/
  // ============ WIRELESS COMMUNICATION SETUP ===========
  SetupESPNOW();
  
  oled.display();
 }

bool paused = false;

void loop() 
{
  if (paused) {
    Serial.println("Paused...");
    return;
  }
  
  // check for mode changes
  if (modeIndex != prevModeIndex)
  {
    prevModeIndex = modeIndex;
  
  //  ptrPot1 = NULL;
  }

  static unsigned long lastTimeStamp = millis();
  static unsigned long lastTimePacketSent = millis();
/*
  rawPot1Value = analogRead(POTENTIOMETER_1);
  rawPot2Value = analogRead(POTENTIOMETER_2);
  if (ptrPot1) {
    *ptrPot1 = map(rawPot1Value, 0, ADC_RESOLUTION, 1000, 0); //sendDelay = map(rawPotentiometer, 0, ADC_RESOLUTION, 1000, 0);
  }
*/
  // Update values
  outgoingData.ID = (outgoingData.ID + 1) > sizeof(outgoingData.ID) ? 0 : outgoingData.ID + 1;
  Vector3<float> leftJS = leftJoystick.Read(100);//.Normalize();
  Vector3<float> rightJS = rightJoystick.Read(100);//.Normalize();

  outgoingData.leftJoystick[0] = leftJS.x;
  outgoingData.leftJoystick[1] = leftJS.y;

  outgoingData.rightJoystick[0] = rightJS.x;
  outgoingData.rightJoystick[1] = rightJS.y;

  outgoingData.leftJoystickIsPressed = leftJS.z;
  outgoingData.rightJoystickIsPressed = rightJS.z;

  //time += millis();
  if (millis() - lastTimePacketSent > sendDelay)
  {
    lastTimePacketSent = millis();
    // Send data
    esp_now_send(broadcastMACAddress, (uint8_t *) &outgoingData, sizeof(outgoingData));
    outgoingCount++;
  }

  if (millis() - lastTimeStamp > timeDelay)
  {
    lastTimeStamp = millis();
    
    if (ptrMode) {
      (*ptrMode)();
    }
  }

  //Wait 2 milliseconds for analog-to-digital converted to settle after last reading 
  delay(2);//delay(20);
}