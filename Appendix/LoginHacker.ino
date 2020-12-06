/**
 * This example turns the ESP32 into a Bluetooth LE keyboard that writes the words, presses Enter, presses a media key and then Ctrl+Alt+Delete
 */
#include <BleKeyboard.h>
#include <M5StickC.h>
BleKeyboard bleKeyboard;
int Fs =150;
int txFreq = 30;
unsigned long microsPerReading, microsPrevious;
String password="";

float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;  


void setup() {
  M5.begin();
  M5.Lcd.setCursor(0, 0, 2);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.printf("Authentication Hacker");  
  M5.update();
  
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  bleKeyboard.begin();
  M5.IMU.Init();
  microsPerReading = 1000000 / Fs;
  microsPrevious = micros();
}

void loop() {
  if(bleKeyboard.isConnected() && (micros() - microsPrevious >= microsPerReading)){

    M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
    M5.IMU.getAccelData(&accX,&accY,&accZ);

    if(abs(accX) > 2.5 || abs(accY)>2.5 || abs(accZ) >2.5){ 
      Serial.println("Sending Enter key...");
      bleKeyboard.write(KEY_RETURN);
      delay(500);
      
      bleKeyboard.print(password);
      delay(100);
      bleKeyboard.releaseAll();
    }
  }
  microsPrevious = microsPrevious + microsPerReading;
}
