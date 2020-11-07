#include <M5StickC.h>
float accX = 0, accY = 0, accZ = 0;
float accOffsetX = -0.01, accOffsetY = -0.01, accOffsetZ = 0.09;
float gyroX = 0, gyroY = 0, gyroZ = 0;  
float gyroOffsetX = 3.93, gyroOffsetY = -14.56, gyroOffsetZ = 2.48;  
float pitch = 0.0,roll=0.0,yaw=0.0;
float pitchRef=-11.5 , rollRef=1.1 , yawRef=0;
//float pitchOffset=1.1 , rollOffset=-11.5 , yawOffset=0;
float pitchOffset=0 , rollOffset=0 , yawOffset=0;
float dt;
float velX = 0, velY = 0, velZ = 0;
float posX = 0, posY = 0, posZ = 0;
float gyroVelX = 0, gyroVelY = 0, gyroVelZ = 0;  

#define INTERVAL 3
float accXs[INTERVAL],accYs[INTERVAL],accZs[INTERVAL]; 
unsigned long acci = 0;
float gyroXs[INTERVAL],gyroYs[INTERVAL],gyroZs[INTERVAL]; 
unsigned long gyroi = 0;

//#include <MadgwickAHRS.h>
//Madgwick filter;
//#include "mahony/MahonyAHRS.h"
//Mahony filter;

//#include <SensorFusion.h>
//SF filter;
//float deltat;

//CLock
int Fs =150;
int txFreq = 75;
unsigned long microsPerReading, microsPrevious;
unsigned long txcount=0;

//BLE
String mode;
#include <BleMouse.h>  // https://github.com/T-vK/ESP32-BLE-Mouse]
BleMouse bleMouse;
int skillSelectCount = 0;
int skillExecCount = 0;
int skillSelectTxTimes = 20;//ゲームパッドが繋がっていない場合、最低20回必要
int skillExecTxTimes = 20;//ゲームパッドが繋がっていない場合、最低20回必要
int skillStopCount = 0;
boolean doneSkill = 0;
boolean scrolledUp = 0;
boolean scrolledDown = 0;
boolean skillSwitch = 0;
boolean skillSwitchPrevious = 0;
int skillSelectWait = 30; //BLEゲームパッドが繋がっていない場合50ms必要
boolean G36Switch = 0;
boolean G36SwitchPrevious = 0;

String skillMode;
int skillMotionCount=0;
String initialMotion;
int waitCount = 20;

//Unwrapping Yaw
int Nwrap=0;
float preYaw = 10000;
float unwrapYaw;

//3D mouse
float x = 0;
float y = 0;
float z = 0;
float xRef = 0;
float yRef = 0;
int viewSpeed=13;
int viewSpeed2=3;
int viewSpeedGyro =1;
boolean viewSwitch = 0;
boolean viewSwitchPrevious = 0;
String viewMode = "low";
String viewMode2 = "gyro";
String handMode = "single";
//boolean viewFlag = 1;
//long txMoveCnt=0;

//Joystick
#include "Wire.h"
#define JOY_ADDR 0x52
uint8_t x_data;
uint8_t y_data;
uint8_t button_data;
String contMode = "FEZ";


//Debug
int output= 0;
String output_data= "rpy_diff";
boolean DEBUG = 1;

void calibrateMPU6886(){
  float gyroSumX,gyroSumY,gyroSumZ;
  float accSumX,accSumY,accSumZ;
  int calibCount = 1000;

  Serial.println("Calibrating...");
  digitalWrite(10, LOW);
  delay(2000);
  digitalWrite(10, HIGH);
  delay(100);
  digitalWrite(10, LOW); 
  for(int i = 0; i < calibCount; i++){
    M5.IMU.getGyroData(&gyroX,&gyroY,&gyroZ);
    M5.IMU.getAccelData(&accX,&accY,&accZ);
      gyroSumX += gyroX;
      gyroSumY += gyroY;
      gyroSumZ += gyroZ;
      accSumX += accX;
      accSumY += accY;
      accSumZ += accZ;
      delay(2);
      //Serial.printf("%6.2f, %6.2f, %6.2f\r\n", accX, accY, accZ);
  }
  gyroOffsetX = gyroSumX/calibCount;
  gyroOffsetY = gyroSumY/calibCount;
  gyroOffsetZ = gyroSumZ/calibCount;
  accOffsetX = accSumX/calibCount;
  accOffsetY = accSumY/calibCount;
  accOffsetZ = (accSumZ/calibCount) - 1.0;//重力加速度1G、つまりM5ボタンが上向きで行う想定
  //accOffsetZ = (accSumZ/calibCount) + 1.0;//重力加速度1G、つまりM5ボタンが下向きで行う想定
  //accOffsetZ = (accSumZ/calibCount);//
  Serial.println("Calibrating...OK");
  Serial.printf("%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f", accOffsetX, accOffsetY, accOffsetZ, gyroOffsetX , gyroOffsetY , gyroOffsetZ);
  Serial.printf("%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f", accOffsetX, accOffsetY, accOffsetZ, gyroOffsetX , gyroOffsetY , gyroOffsetZ);
  /*M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 30);
  M5.Lcd.printf("%6.2f %6.2f %6.2f %6.2f %6.2f %6.2f", accOffsetX, accOffsetY, accOffsetZ, gyroOffsetX , gyroOffsetY , gyroOffsetZ);
  */
  digitalWrite(10, HIGH);
}

void LED_ONOFF(int num){
  int i;
  while(i<=num){
      digitalWrite(10, HIGH);
      delay(1);
      digitalWrite(10, LOW);
    i++;
  }
}

float  WRAPPING_YAW() { // Determines if a phase wrap/unwrap occured and accounts for it
  if(preYaw == 10000){}
  else if ((preYaw ) > (180) && (yaw) < (100)) {
    // Compare the previous theat to the current yaw to see if a wrap occured.
    Nwrap = Nwrap + 1; // If so, add 1 to the number of wraps.
  }
  else if ((preYaw) < (100) && (yaw) > (180)) {
    // If an unwrap occured (a phase wrap in the other direction)...
    Nwrap = Nwrap - 1; // ...then subtract 1 from the number of wraps.
  }
  preYaw = yaw; // Store the current yaw for the next cycle
  unwrapYaw = yaw + Nwrap*360.0;
  
  return unwrapYaw;
}

float  WRAPPING_YAW_M5AHRS() { // Determines if a phase wrap/unwrap occured and accounts for it
  if(preYaw == 10000){}
  else if ((preYaw  >= 0) && (yaw <= -90)) {
    // Compare the previous theat to the current yaw to see if a wrap occured.
    Nwrap = Nwrap + 1; // If so, add 1 to the number of wraps.
  }
  else if ((preYaw) < (0) && (yaw >= 90)) {
    // If an unwrap occured (a phase wrap in the other direction)...
    Nwrap = Nwrap - 1; // ...then subtract 1 from the number of wraps.
  }
  preYaw = yaw; // Store the current yaw for the next cycle
  unwrapYaw = yaw + (float)Nwrap*360.0;
  
  return unwrapYaw;
}



void setup() {
    pinMode(10,   OUTPUT); //LED
    pinMode(26,   INPUT_PULLUP); //半PD 電圧が1V程にプルダウン？されている。Pin36を ONにするとノイズが発生し、GNDとの判定で誤判定となる。3.3Vとのショート判定する必要がある。
    pinMode(0,   INPUT_PULLUP); //PU 起動モード設定のためか、常にプルアップされており、ソフト制御できない。スイッチでGNDに落として判定する必要あり。
    pinMode(36,   INPUT); //3.3V入力をスイッチとして利用可能
    M5.begin();
    M5.Lcd.setCursor(0, 0, 2);
    M5.Lcd.fillScreen(BLACK);
    Serial.begin(115200);
    Wire.begin();

    M5.IMU.Init();
    calibrateMPU6886();
    
    //filter.begin(Fs);  // 10Hz  filterを初期化する
    microsPerReading = 1000000 / Fs;
    microsPrevious = micros();

    bleMouse.begin();
    //bleGamepad.begin();w
        
    digitalWrite(10, HIGH);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("FEZ Motion Controller Mouse. Set up...");
    digitalWrite(10, HIGH);
}



void loop() {
    if ((bleMouse.isConnected() && (micros() - microsPrevious >= microsPerReading))) {
/* ---Roll/Pitch/Yawの計算---*/
      M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);
      M5.IMU.getAccelData(&accX, &accY, &accZ);
      M5.IMU.getAhrsData(&pitch,&roll,&yaw);
      accX -= accOffsetX;
      accY -= accOffsetY;
      accZ -= accOffsetZ;
      gyroX -=gyroOffsetX;
      gyroY -=gyroOffsetY;
      gyroZ -=gyroOffsetZ;       
      pitch -=pitchOffset; 
      roll -= rollOffset;
      yaw -= yawOffset;
      
      //deltat = filter.deltatUpdate();
      //filter.updateIMU(gyroX/0.49, gyroY/0.49, gyroZ/0.49, accX, accY, accZ);
      //filter.updateIMU(gyroX, gyroY, gyroZ, accX, accY, accZ);
      //filter.MahonyUpdate(gyroX, gyroY, gyroZ, accX, accY, accZ,deltat);
      
      //平滑化なし
      
      //roll = filter.getRoll();
      //pitch = filter.getPitch();
      //yaw   = filter.getYaw();
      
      unwrapYaw = WRAPPING_YAW_M5AHRS();
      //unwrapYaw = WRAPPING_YAW();
      
      //M5.Lcd.fillScreen(BLACK);
      //M5.Lcd.setCursor(0, 50);
      //M5.Lcd.printf("%6.2f %6.2f %6.2f", roll, pitch, yaw);
      //Serial.printf("%d %d %6.2f, %6.2f, %6.2f,%6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f  \r\n",txcount,skillMotionCount,roll,pitch,yaw, accX, accY, accZ,gyroX,gyroY,gyroZ);
      //Serial.printf("%d %s %4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f \r\n",txcount%10000,skillMode,roll,pitch,yaw, accX, accY, accZ,gyroX,gyroY,gyroZ);
      Serial.printf("%d %s %d,%d,%d,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f \r\n",txcount%10000,skillMode,skillSwitch,G36Switch,viewSwitch,roll,pitch,yaw, accX, accY, accZ,gyroX,gyroY,gyroZ);

      //=====スキル判定・実行======
      skillSwitchPrevious = skillSwitch;
      skillSwitch = !digitalRead(26);
      G36SwitchPrevious = G36Switch;
      G36Switch = digitalRead(36);
      if(skillSwitch || G36Switch){
        digitalWrite(10, LOW);
      }else{
         digitalWrite(10, HIGH);       
      }

  if(contMode == "FEZ"){
    //====方法１：姿勢判定====
    if(skillSwitch == 1 && G36Switch ==0){
      if(doneSkill == 0){
        //スキル選択
        if(skillMotionCount == 0){
          /*if(abs(roll) >=0 && abs(roll) <= 40){
            skillMode = "1";//アサルト
          }
          else*/ 
          if(abs(roll) < 50 && (pitch >= -20 && pitch < 60)){
            skillMode = "2";//アサルト
            while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_MIDDLE); skillSelectCount++;}
          }
          else if((pitch < -20 && pitch >-70) && (roll >= 30 && roll <130)){
            skillMode = "7";//スマ
            while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_FORWARD); skillSelectCount++;}
            //delay(skillSelectWait);
          }
          else if(pitch <= -20 && (roll<30 && roll>-120)){
            skillMode = "4";//ヘビスマ
            while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_BACK); skillSelectCount++;}
            bleMouse.move(0,0,1);
          }
          else if(abs(roll) >=130 && abs(pitch) <=40){
            skillMode = "3";//ランペ
            while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_MIDDLE); skillSelectCount++;}
            bleMouse.move(0,0,-1);            
          }
          else if(pitch >= 50 ){
            skillMode = "5";//ストスマ
            while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_BACK); skillSelectCount++;}
          }
          else if(roll >60 && roll <= 140 && abs(pitch) < 40 ){
            skillMode = "8";//
            while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_FORWARD); skillSelectCount++;}
            bleMouse.move(0,0,-1);
          }
          /*else if(roll >=40 && roll <= 140 && abs(pitch)<40 ){
            skillMode = "5or6or7or8";//
          }*/

        }
        skillMotionCount++;
 
       
        /*if(skillMode=="1" && gyroY > 1500){//クランブル
          if(DEBUG)Serial.println("accZ > 3, skill1");
          while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_MIDDLE); skillSelectCount++;}
          bleMouse.move(0,0,1);
          delay(skillSelectWait);
          while(skillExecCount<skillExecTxTimes){bleMouse.click(MOUSE_LEFT); skillExecCount++;}
          doneSkill = 1;
        }*/
        if(skillMode=="2" && (accX >= 1.2 || accZ < -1.2 |gyroZ < -1000)){//アサルト
          if(DEBUG)Serial.println("accX < -3, skill2");
          //while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_MIDDLE); skillSelectCount++;}
          //delay(skillSelectWait);
          while(skillExecCount<skillExecTxTimes){bleMouse.click(MOUSE_LEFT); skillExecCount++;}
          doneSkill = 1;
        }
        else if(skillMode=="3" && (gyroZ < -1000 || accX > 1.2)){//ランペ
          if(DEBUG)Serial.println("accZ < -3,skill3");
          //while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_MIDDLE); skillSelectCount++;}
          //bleMouse.move(0,0,-1);
          //delay(skillSelectWait);
          while(skillExecCount<skillExecTxTimes){bleMouse.click(MOUSE_LEFT); skillExecCount++;}
          doneSkill = 1;
        }
        else if(skillMode=="4" && gyroZ <= -1000 || accX >1.2){//ヘビスマ
          if(DEBUG)Serial.println("accX > 3,skill4");
          //while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_BACK); skillSelectCount++;}
         // bleMouse.move(0,0,1);
         // delay(skillSelectWait);
          while(skillExecCount<skillExecTxTimes){bleMouse.click(MOUSE_LEFT); skillExecCount++;}
          doneSkill = 1;
        }
       else if(skillMode=="5" && (accX > 1.2 || gyroZ <= -1000)){//ストスマ
          if(DEBUG)Serial.println("accY > 3,skill5");
          //while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_BACK); skillSelectCount++;}
          //delay(skillSelectWait);
          while(skillExecCount<skillExecTxTimes){bleMouse.click(MOUSE_LEFT); skillExecCount++;}
          doneSkill = 1;
        }
        /*else if(skillMode=="5or6or7or8" && accY > 3){
          if(DEBUG)Serial.println("accY > 3,skill6");
          while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_BACK); skillSelectCount++;}
          bleMouse.move(0,0,-1);
          delay(skillSelectWait);
          while(skillExecCount<skillExecTxTimes){bleMouse.click(MOUSE_LEFT); skillExecCount++;}
          doneSkill = 1;
        }*/
        else if(skillMode=="7" && gyroZ < -1000 || accX > 1.3){
          if(DEBUG)Serial.println("gyroZ < -1500,skill7");
          //while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_FORWARD); skillSelectCount++;}
          //delay(skillSelewactWait);
          while(skillExecCount<skillExecTxTimes){bleMouse.click(MOUSE_LEFT); skillExecCount++;}
          doneSkill = 1;
        } 
        else if(skillMode=="8" && gyroX > 1500){
          if(DEBUG)Serial.println("gyroX < -1500,skill8");
          //while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_FORWARD); skillSelectCount++;}
          //bleMouse.move(0,0,-1);
          //delay(skillSelectWait);
          while(skillExecCount<skillExecTxTimes){bleMouse.click(MOUSE_LEFT); skillExecCount++;}
          doneSkill = 1; 
        }
        /*else if(skillMode=="8" && (accX > 1.0 || gyroZ <= -1000)){//ストスマ
          if(DEBUG)Serial.println("accY > 3,skill5");
          while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_BACK); skillSelectCount++;}
          //delay(skillSelectWait);
          while(skillExecCount<skillExecTxTimes){bleMouse.click(MOUSE_LEFT); skillExecCount++;}
          doneSkill = 1;
        }*/
      }
    }
    //===方法２：加速度・ジャイロ判定===
    //スキル選択⇒スキルホイール選択⇒スキル実行の順に行う。スキルによってはスキルホイール選択が不要。
    //具体例では、　bleMouse.click(MOUSE_BACK) => bleMouse.move(0,0,-1); => bleMouse.click(MOUSE_LEFT)。
    //スキル選択後、skillMotionCountが10になるまで何もせず、skillMotionCountが10になったときにスキル実行するといったカウント待機時間を設けている。
    //理由は、bleMouseでFEZにスキルホイール選択すると、最大３０ｍｓ？程度のランダム時間を要するため。
    //また、スキル選択、スキル実行に限り、失敗する可能性があるため、skillSelectTxTimes=20、skillExecTxTimes=20のように送信回数を20回にしている。
    //尚、スキルホイール選択は必ず成功する。
    //下記コードは、実行周期６ms(≒1/150)、移動周期33ms(≒1/(150/30))の際に98%程成功する。
    else if(skillSwitch == 1 && G36Switch == 1 ){
      if(doneSkill == 0){
        if(accX > 2.5 || initialMotion == "accX+"){
            if(skillMotionCount==0){while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_MIDDLE); skillSelectCount++;}}
            else if(skillMotionCount==1){bleMouse.move(0,0,1);}
            else if(skillMotionCount==waitCount){while(skillExecCount<skillExecTxTimes){bleMouse.click(MOUSE_LEFT); skillExecCount++;}doneSkill = 1;}
            skillMotionCount++;
            initialMotion = "accX+";
            skillMode = "1";
          }
        else if(accZ > 2.5 || initialMotion == "accZ+"){
            if(skillMotionCount==0){while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_MIDDLE); skillSelectCount++;}}
            else if(skillMotionCount==1){}
            else if(skillMotionCount==waitCount){while(skillExecCount<skillExecTxTimes){bleMouse.click(MOUSE_LEFT); skillExecCount++;}doneSkill = 1;}
            skillMotionCount++;
            initialMotion = "accZ+";
            skillMode = "2";
        }
        else if(accX < -2.5 || initialMotion == "accX-"){
            if(skillMotionCount==0){while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_MIDDLE); skillSelectCount++;}}
            else if(skillMotionCount==1){bleMouse.move(0,0,-1);}
            else if(skillMotionCount==waitCount){while(skillExecCount<skillExecTxTimes){bleMouse.click(MOUSE_LEFT); skillExecCount++;}doneSkill = 1;}
            skillMotionCount++; 
            initialMotion = "accX-";
            skillMode = "3";     
        }
        else if(accZ < -2.5 || initialMotion == "accZ-"){
            if(skillMotionCount==0){while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_MIDDLE); skillSelectCount++;}}
            else if(skillMotionCount==1){bleMouse.move(0,0,-1);bleMouse.move(0,0,-1);}
            else if(skillMotionCount==waitCount){while(skillExecCount<skillExecTxTimes){bleMouse.click(MOUSE_LEFT); skillExecCount++;} doneSkill = 1;}
            skillMotionCount++;
            initialMotion = "accZ-";
            skillMode = "4";
        }
      }
    }
    else if(G36Switch == 1){
      if(doneSkill == 0){
        if(accX > 2.5 || initialMotion == "accX+"){
            if(skillMotionCount==0){while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_BACK); skillSelectCount++;}}
            else if(skillMotionCount==1){}
            else if(skillMotionCount==waitCount){while(skillExecCount<skillExecTxTimes){bleMouse.click(MOUSE_LEFT); skillExecCount++;}doneSkill = 1;}
            skillMotionCount++;
            initialMotion = "accX+";
            skillMode = "5";
          }
        else if(accZ > 2.5 || initialMotion == "accZ+"){
            if(skillMotionCount==0){while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_BACK); skillSelectCount++;}}
            else if(skillMotionCount==1){bleMouse.move(0,0,-1);}
            else if(skillMotionCount==waitCount){while(skillExecCount<skillExecTxTimes){bleMouse.click(MOUSE_LEFT); skillExecCount++;}doneSkill = 1;}
            skillMotionCount++;
            initialMotion = "accZ+";
            skillMode = "6";
        }
        else if(accX < -2.5 || initialMotion == "accX-"){
            if(skillMotionCount==0){while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_FORWARD); skillSelectCount++;}}
            else if(skillMotionCount==1){}
            else if(skillMotionCount==waitCount){while(skillExecCount<skillExecTxTimes){bleMouse.click(MOUSE_LEFT); skillExecCount++;}doneSkill = 1;}
            skillMotionCount++; 
            initialMotion = "accX-";
            skillMode = "7";     
        }
        else if(accZ < -2.5 || initialMotion == "accZ-"){
            if(skillMotionCount==0){while(skillSelectCount<skillSelectTxTimes){bleMouse.click(MOUSE_FORWARD); skillSelectCount++;}}
            else if(skillMotionCount==1){bleMouse.move(0,0,-1);}
            else if(skillMotionCount==waitCount){while(skillExecCount<skillExecTxTimes){bleMouse.click(MOUSE_LEFT); skillExecCount++;} doneSkill = 1;}
            skillMotionCount++;
            initialMotion = "accZ-";
            skillMode = "8";
        }
      }
    }
    
    //===方法３：Bボタンでスキル選択・実行===
    else if(doneSkill == 0){
        //1秒押下で選択スキル連続実行
        if(M5.BtnB.pressedFor(1000)){
          bleMouse.click(MOUSE_LEFT);
        }
        
        //1秒押下で選択スキル実行
        if(M5.BtnB.wasReleasefor(500)){
          while(skillExecCount<10){bleMouse.click(MOUSE_LEFT); skillExecCount++;}
        }
        
        //200m秒押下で選択スキルを上へ移動。これは一回入力で100%成功する。
        else if(M5.BtnB.wasReleasefor(200)){
          Serial.println("Scroll Up");
          bleMouse.move(0,0,1);
          //scrolledUp = 1;
        }
    
        //200m秒以下押下で選択スキルを下へ移動。これは一回入力で100%成功する。
        else if(M5.BtnB.wasReleasefor(1)){
          Serial.println("Scroll Down");
          bleMouse.move(0,0,-1);
          //scrolledDown = 1;      
        }
    }

/*
    if(skillSwitch==0){
        skillSelectCount = 0;
        skillExecCount = 0; 
        doneSkill = 0;  
        skillStopCount=0;
        skillMotionCount = 0;
    }*/
      
    //スキル実行後、0.3秒間スキル実行停止. チャタリング防止
    //if(doneSkill == 1 ||(skillSwitch==0 && skillSwitchPrevious==1) || (G36Switch==0 && G36SwitchPrevious==1)){
        //skillStopCount++;
        //if(skillStopCount > Fs/3.0){
    if((skillSwitch==0 && skillSwitchPrevious==1) || (G36Switch==0 && G36SwitchPrevious==1)){      
      skillSelectCount = 0;
      skillExecCount = 0; 
      doneSkill = 0;  
      skillStopCount=0;
      skillMotionCount = 0;
      //G36SwitchCnt=0;
      initialMotion ="";
    }
  }else if(contMode=="FPS"){
    if(skillSwitch == 1 && G36Switch ==0){
      bleMouse.click(MOUSE_LEFT);
    }else if(skillSwitch == 0 && G36Switch ==1){
      bleMouse.click(MOUSE_RIGHT); //長押しできないため、スコープズーム不可。キーボードで代用が良い。
    }
  }else if(contMode=="Mouse"){
    if(skillStopCount == 0){
      if(skillSwitch == 1 && G36Switch ==0){
        bleMouse.click(MOUSE_LEFT);
        skillStopCount++;
      }else if(skillSwitch == 0 && G36Switch ==1){
        bleMouse.click(MOUSE_RIGHT); //長押しできないため、スコープズーム不可。キーボードで代用が良い。
        skillStopCount++;
      }
    }else if(skillStopCount > Fs/5.0){
      skillStopCount = 0;
    }else if((skillSwitch==0 && skillSwitchPrevious==1) || (G36Switch==0 && G36SwitchPrevious==1)){
       skillStopCount = 0;
    }
  }
      
    //視点の変化量を変更 ホームボタンと右(B)ボタン押下
    if(M5.BtnA.isPressed() && M5.BtnB.wasPressed() ){
      viewSpeed = (viewSpeed + 2) % 20;
      viewSpeedGyro = (viewSpeedGyro + 1) % 5;
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setCursor(0, 5, 2);
      M5.Lcd.printf("ViewSp %d",viewSpeed);
      M5.Lcd.setCursor(0, 20, 2);
      M5.Lcd.printf("ViewSp2 %d",viewSpeedGyro);
    }else if(M5.BtnB.wasPressed()){
    }

  int axpButton = M5.Axp.GetBtnPress();
  if ( axpButton == 1 ) {
    // 1秒以上電源ボタンを押している
    Serial.println("M5.Axp.GetBtnPress() == 1, MouseBack");
    output = (output + 1) % 4;
    if(output == 0){
      output_data = "gyroVel";
    }else if(output == 1){
      output_data = "rpy";
    }else if(output == 2){
      output_data = "rpy_diff";
    }else if(output == 3){
      output_data = "acc";
    }
  }
  else if ( axpButton == 2 ) {
    // 1秒未満電源ボタンを押して離した
    if(contMode == "FEZ"){
      contMode = "FPS";
      txFreq = 75;
    }else if(contMode == "FPS"){
      contMode = "Mouse";
      txFreq = 75;
    }else if(contMode == "Mouse"){
      contMode = "FEZ";
      txFreq = 75;
    }
  }
  
/* ---Aボタンの動作---*/ 
  if(M5.BtnA.wasReleasefor(5000)){
    calibrateMPU6886();

  }else if(M5.BtnA.wasReleasefor(2000)){
  }else if(M5.BtnA.wasReleasefor(1000)){
    if(handMode == "single"){
      handMode = "double";
      skillSelectTxTimes = 1;//ゲームパッドが接続していれば、１回送信で良い。原因不明。
      skillExecTxTimes = 1;  
    }else if(handMode == "double"){
      handMode = "single2";
      skillSelectTxTimes = 50;//ゲームパッドが接続されていない場合、最低20回必要
      skillExecTxTimes = 50;
    }else if(handMode == "single2"){
      handMode = "single";
      skillSelectTxTimes = 20;//ゲームパッドが接続されていない場合、最低20回必要
      skillExecTxTimes = 20;
    }
  } else if(M5.BtnA.wasReleasefor(10)){
    if(viewMode == "low"){ viewMode = "middle";}
    else if(viewMode == "middle"){viewMode ="high";} 
    else if(viewMode == "high"){viewMode ="low";} 
    //M5.Lcd.fillScreen(BLACK);
  }


  
/* ---視点移動のための慣性データの平滑化・処理---*/ 
    viewSwitchPrevious = viewSwitch;
    viewSwitch = !digitalRead(0);
    //if(viewSwitch == 0 && viewSwitchPrevious ==1){
    if(viewSwitch == 0){
      //Serial.println("------viewSwitchOFF------");
      x=0;y=0;z=0;
      posX=0; posY=0; posZ=0;
      velX=0;velY=0; velZ=0;
      gyroVelX=0; gyroVelY=0; gyroVelZ=0;
      //viewMode = 1;
    }
    
    /*else if(viewSwitch == 1){
      viewMode = 2;
    }*/
    
  
    //加速度の平滑化(値にばらつきがあるため)
    accXs[acci % INTERVAL] = accX;  accYs[acci % INTERVAL] = accY;  accZs[acci % INTERVAL] = accZ;
    acci = acci++;
    for(int i=0; i< INTERVAL; i++){
      accX += accXs[i]; accY += accYs[i]; accZ += accZs[i];
    }
    accX/=INTERVAL;  accY/=INTERVAL;  accZ/=INTERVAL;

    //ジャイロの平滑化
    gyroXs[gyroi % INTERVAL] = gyroX;  gyroYs[gyroi % INTERVAL] = gyroY;  gyroZs[gyroi % INTERVAL] = gyroZ;
    gyroi = gyroi++;
    for(int i=0; i< INTERVAL; i++){
      gyroX += gyroXs[i]; gyroY += gyroYs[i]; gyroZ += gyroZs[i];
    }
    gyroX/=INTERVAL;  gyroY/=INTERVAL;  gyroZ/=INTERVAL;

    if(output_data=="pos" || output_data=="vel" || output_data=="gyro" ||output_data=="gyroVel"){
      dt = (float)microsPerReading/1000000.0;

      //if(abs(accX) < 0.2)
        posX = posX + (velX * dt + (accX * dt * dt)/2.0) ;
        velX = velX + accX * dt;       
      //if(abs(accY) < 0.2)
        posY = posY + velY * dt + accY * dt * dt/2.0 ;
        velY = (velY + accY * dt);
      //if(abs(accZ) < 0.2)
        posZ = posZ + velZ * dt + (accZ * dt * dt)/2.0 ;
        velZ = (velZ + accZ * dt);

       gyroVelX = gyroVelX + gyroX * dt;
       gyroVelY = gyroVelY + gyroY * dt;
       gyroVelZ = gyroVelZ + gyroZ * dt;  
    }
    //}
    
/*---シリアル出力データの選択---*/
  if(txcount%(Fs/txFreq)== 0){
    if(output_data=="rpy"){
      if(viewSwitchPrevious == 0 && viewSwitch == 1){
        yawRef = unwrapYaw; pitchRef = pitch;   
      }
      x = -(unwrapYaw - yawRef)*2.5; y= (pitch - pitchRef)*2; 
      //x = x + -(unwrapYaw - yawRef); y= y + (roll - rollRef); 
      //x = pitch; y=-roll;  z=yaw; 
      //Serial.printf("%6.2f, %6.2f, %6.2f\r\n", roll, pitch, yaw);
    }else if(output_data=="rpy_diff"){
      if(viewSwitchPrevious == 0 && viewSwitch == 1){
        yawRef = unwrapYaw; pitchRef = pitch;   
      }
      x = -(unwrapYaw - yawRef)*100;  y= (pitch - pitchRef)*50;
      yawRef = unwrapYaw;  pitchRef = pitch;
            
      //Serial.printf("%6.2f, %6.2f, %6.2f\r\n", roll, pitch, yaw);
    }else if(output_data=="acc"){   
      x = -accZ*30*(float)viewSpeed; y=accX*30*(float)viewSpeed/3.0; 
      Serial.printf("%6.2f, %6.2f, %6.2f\r\n", accX, accY, accZ);
    }else if(output_data=="vel"){
      x=velX*100; y =-velZ*100; //z=velZ*100;
      Serial.printf("%6.2f, %6.2f, %6.2f\r\n", x, y, z);
    }else if(output_data=="pos"){
      x=posX*100; y =posY*100; z=posZ*100;
      Serial.printf("%6.2f, %6.2f, %6.2f\r\n", x, y, z);
    }else if(output_data=="gyro"){
       z=-gyroZ*viewSpeedGyro;
       x=-gyroY*viewSpeedGyro;
       y=gyroX*viewSpeedGyro*2;
      //Serial.printf("%d %6.2f, %6.2f, %6.2f,%6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f  \r\n",txcount, x, y, z,roll,pitch,yaw,gyroX,gyroY,gyroZ);
    }else if(output_data=="gyroVel"){
        z=-gyroVelZ*viewSpeedGyro; //未使用
        x=-gyroVelY*viewSpeedGyro*10;
        y=+gyroVelX*viewSpeedGyro*10 /3;
      //Serial.printf("%d %6.2f, %6.2f, %6.2f,%6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f  \r\n",txcount, x, y, z,roll,pitch,yaw,gyroX,gyroY,gyroZ);
    }else if(output_data=="none"){
    }
  }

/* ---視点移動判定---*/ 
    if (txcount%(Fs/txFreq)== 0 &&  M5.BtnB.isReleased()) {
      if(contMode=="FEZ" && (G36Switch == 1 || skillSwitch==1)){}
      else{
         //視点スピード調整
         if (viewMode == "low"){
           x *= 0.33; y *=0.3;
         }else if (viewMode == "middle"){
           x *= 0.5; y *=0.5;
         }else if(viewMode = "high"){  
            x *= 1; y *=1;
         }
        if(viewSwitch == 1){
          if(output_data =="gyroVel"){
            //xまたはyが128以上だと座標が反転してしまうため、xまたはyを127以下にする。
            if((abs(x) > 2 || abs(y) >2)){
              if(x>127) x=127;
              else if(x <-127) x=-127;
              if(y>127) y=127;
              else if(y<-127) y=-127;
              bleMouse.move(x, y);
            }
          }else if(output_data == "rpy"){
            if((abs(x) > 2 || abs(y) >2)){
              if(x>127) x=127;
              else if(x <-127) x=-127;
              if(y>127) y=127;
              else if(y<-127) y=-127;
              bleMouse.move(x, y);
            }
          }else if(output_data == "rpy_diff"){
            //x = -(unwrapYaw - yawRef)*1000; y= (pitch - pitchRef)*200; 
            //yawRef = unwrapYaw; pitchRef = pitch;
            if((abs(x) > 1 || abs(y) > 1)){
              if(x>127) x=127;
              else if(x <-127) x=-127;
              if(y>127) y=127;
              else if(y<-127) y=-127;
              bleMouse.move(x, y);
            }
          }else{
            if(x>127) x=127;
            else if(x <-127) x=-127;
            if(y>127) y=127;
            else if(y<-127) y=-127;
            bleMouse.move(x, y);
          }
        }
      }
    }
    /*
    Wire.requestFrom(JOY_ADDR, 3);
    if(txMoveCnt%3 == 0 && Wire.available()){
      if(joystickMode=="view"){
        x_data = Wire.read();
        y_data = Wire.read();
        if(abs(int(x_data)-120)>5 || abs(int(y_data)-120)>5){bleMouse.move(-(int(x_data)-120), (int(y_data)-120)/2);}
        button_data = Wire.read();
        Serial.printf("x:%d y:%d button:%d\n", int(x_data), int(y_data), int(button_data));
      }else if(joystickMode=="view"){
      }
    }*/

/*
    x = accX*15*(float)viewSpeed; y=accY*15*(float)viewSpeed*0.66;
    //Change View
    if (M5.BtnB.isReleased()  && mouseActive ==1 && (abs(x) > 0 ||  abs(y) > 0) ) {
        //M5.Lcd.setCursor(0, 100);
        //M5.Lcd.printf("%6.2f %6.2fF", x,y);
      bleMouse.move(x, -y);
    }
*/ 
    M5.update();
    if(txcount%(Fs/2)==1){
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setCursor(0, 20, 2);
      M5.Lcd.printf("Mode:%s ",contMode);
      M5.Lcd.setCursor(0, 40, 2);
      M5.Lcd.printf("Hand:%s",handMode);
      M5.Lcd.setCursor(0, 60, 2);
      M5.Lcd.printf("SkillTx:%d",skillSelectTxTimes);
      M5.Lcd.setCursor(0, 80, 2);
      M5.Lcd.printf("Out:%s",output_data);
      M5.Lcd.setCursor(0, 100, 2);
      M5.Lcd.printf("View:%s",viewMode);
//      M5.Lcd.setCursor(0, 100, 2);
//      M5.Lcd.printf("out:%s",output_data);  
    }      
    
    txcount++;
    microsPrevious = microsPrevious + microsPerReading;
  }
}
