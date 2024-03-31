A 3D Graphics Engine for the ESP32.

<h2>----HMD Experimental version-----</h2>

<h2>Hardware</h2>

- HMD/Board/Display: LILYGO T-Display-S3 ESP32-S3 --> https://www.amazon.com/dp/B0C2T6DP5J?ref=ppx_yo2ov_dt_b_product_details&th=1
- x2 SparkFun Qwiic Joysticks --> https://www.sparkfun.com/products/15168
- x1 SparkFun Qwiic Mux Breakout - 8 Channel (TCA9548A) --> https://www.sparkfun.com/products/16784
- x1 Adafruit MPU-6050 6-DoF Accel and Gyro Sensor - STEMMA QT Qwiic--> https://www.adafruit.com/product/3886

<h2>Instructions/Gotchas</h2>

1. HMD: Make sure to select "Upload Filesystem Image" from the PlatformIO extension in VS Code. This will upload all the data/*.obj files to your ESP32 board. 
2. HMD: Copy and paste this code in setup() and run the program once. Comment it out or remove it after it succeeds for the first time.

    <code>if (!SPIFFS.begin(true))
    {
     Serial.println("Failed while mounting SPIFFS.");
     return;
    }</code>

3. HMD: Connect MPU6050 to LILYGO T-Display-S3.
4. Controller: Wire joysticks to a different ESP32 board (I'm using https://www.adafruit.com/product/3591) and update the mac addresses in code for ESP-NOW communication. 