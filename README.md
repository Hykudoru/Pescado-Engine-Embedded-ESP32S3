A 3D graphics engine for the ESP32. Board/Display: LILYGO T-Display-S3 ESP32-S3. 

-----Instructions/Gotchas-----
1. Make sure to select "Upload Filesystem Image" from the PlatformIO extension in VS Code. This will upload all the data/*.obj files to your ESP32 board. 
2. Copy and paste this code in setup() and run the program once. Comment it out or remove it after it succeeds for the first time.

    <code>if (!SPIFFS.begin(true))
    {
     Serial.println("Failed while mounting SPIFFS.");
     return;
    }</code>
