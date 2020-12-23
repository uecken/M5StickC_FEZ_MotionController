#include <M5StickC.h>
float accX = 0, accY = 0, accZ = 0;
float accOffsetX = -0.01, accOffsetY = -0.01, accOffsetZ = 0.09;
float gyroX = 0, gyroY = 0, gyroZ = 0;  
float gyroOffsetX = 3.93, gyroOffsetY = -14.56, gyroOffsetZ = 2.48;  
float pitch = 0.0,roll=0.0,yaw=0.0;
float pitchRef=0 , rollRef=0 , yawRef=0;
//float pitchOffset=1.1 , rollOffset=-11.5 , yawOffset=0;
float pitchOffset=0 , rollOffset=0 , yawOffset=-8.5;
float dt;
float velX = 0, velY = 0, velZ = 0;
float posX = 0, posY = 0, posZ = 0;
float gyroVelX = 0, gyroVelY = 0, gyroVelZ = 0;  

#define INTERVAL 3
float accXs[INTERVAL],accYs[INTERVAL],accZs[INTERVAL]; 
long acci = 0;
float gyroXs[INTERVAL],gyroYs[INTERVAL],gyroZs[INTERVAL]; 
long gyroi = 0;


//CLock
int Fs =150;
int txFreq = 75;
unsigned long microsPerReading, microsPrevious;
long txcount=0;

//BLE
String mode;
//#include <BleMouse.h>  // https://github.com/T-vK/ESP32-BLE-Mouse]
//BleMouse bleMouse;
#include <BleCombo.h>
int skillSelectCount = 0;
int skillExecCount = 0;
int skillSelectTxTimes = 1;//ゲームパッドが繋がっていない場合、最低20回必要。戦争中は50回がよいか？ 多すぎるとスキル発生後の姿勢推定誤差が大きい
int skillExecTxTimes = 1;//ゲームパッドが繋がっていない場合、最低20回必要
int skillStopCount = 0;
boolean doneSkill = 0;
boolean scrolledUp = 0;
boolean scrolledDown = 0;
boolean skillSwitch = 0;
boolean skillSwitchPrevious = 0;
int skillSelectWait = 30; //BLEゲームパッドが繋がっていない場合50ms必要
boolean G36Switch = 0;
boolean G36SwitchPrevious = 0;
boolean skillReset = 0;

String skillMode;
int skillMotionCount=0;
String initialMotion;
int waitCount = 10;

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
String handMode = "Daiken";
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
  else if ((preYaw < 0) && (yaw >= 90)) {
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

    //bleMouse.begin();
    //bleGamepad.begin();
    Keyboard.begin();
    Mouse.begin();
        
    digitalWrite(10, HIGH);
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.printf("FEZ Motion Controller Mouse. Set up...");
    digitalWrite(10, HIGH);
}



void loop() {
    if ((Keyboard.isConnected() && (micros() - microsPrevious >= microsPerReading))) {
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
      yaw -= yawOffset; //★最小値-188.5。原因はgyroZの補正ができていないからか？一時的に+8.5して補正する。
      
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
      Serial.printf("%ld %s %d,%d,%d,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f \r\n",txcount%10000,skillMode.c_str(),skillSwitch,G36Switch,viewSwitch,roll,pitch,yaw, accX, accY, accZ,gyroX,gyroY,gyroZ,unwrapYaw);

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
    if(skillSwitch == 1 && G36Switch ==0 && handMode == "Daiken"){
      if(doneSkill == 0){
        //スキル選択
        skillMotionCount++;
        if(skillMotionCount == 1){
          /*if(abs(roll) >=0 && abs(roll) <= 40){
            skillMode = "1";//アサルト
          }
          else*/ 
          if(abs(roll) < 50 && (pitch >= -20 && pitch < 50)){
            skillMode = "2";//アサルト
            Keyboard.press(0x32);
            //while(skillSelectCount<skillSelectTxTimes){Mouse.press(MOUSE_MIDDLE); skillSelectCount++;}
          }
          else if((pitch < -20 && pitch >-70) && (roll >= 30 && roll <130)){
            skillMode = "7";//スマ
            Keyboard.press(0x37);
            //while(skillSelectCount<skillSelectTxTimes){Mouse.press(MOUSE_FORWARD); skillSelectCount++;}
          }
          else if(pitch <= -20 && (roll<30 && roll>-120)){
            skillMode = "4";//ヘビスマ
            Keyboard.press(0x34);
            //while(skillSelectCount<skillSelectTxTimes){Mouse.press(MOUSE_BACK); skillSelectCount++;}
            //Mouse.move(0,0,1);
          }
          else if(abs(roll) >=130 && abs(pitch) <=40){
            skillMode = "3";//ランペ
            Keyboard.press(0x33);
            //while(skillSelectCount<skillSelectTxTimes){Mouse.press(MOUSE_MIDDLE); skillSelectCount++;}
            //Mouse.move(0,0,-1);            
          }/*
          else if(pitch >= 50 ){
            skillMode = "1";//クラン
            Keyboard.press(0x31);
            //while(skillSelectCount<skillSelectTxTimes){Mouse.press(MOUSE_BACK); skillSelectCount++;}
          }*/
          else if(roll >60 && roll <= 140 && abs(pitch) < 40 ){
            skillMode = "1";//
            Keyboard.press(0x31);
            //while(skillSelectCount<skillSelectTxTimes){Mouse.press(MOUSE_FORWARD); skillSelectCount++;}
            //Mouse.move(0,0,-1);
          }
        }else if(skillMotionCount > 5){
          if(skillMode=="2" && (accX >= 1.2 || accZ < -1.2 ||gyroZ < -1000)){//アサルト
            if(DEBUG)Serial.println("accX < -3, skill2");
            //while(skillSelectCount<skillSelectTxTimes){Mouse.press(MOUSE_MIDDLE); skillSelectCount++;}
            //delay(skillSelectWait);
            //while(skillExecCount<skillExecTxTimes){Mouse.press(MOUSE_LEFT); skillExecCount++;}
            Keyboard.press(0x85);
            doneSkill = 1;
          }
          else if(skillMode=="3" && (gyroZ < -1000 || accX > 1.2)){//ランペ
            if(DEBUG)Serial.println("accZ < -3,skill3");
            Keyboard.press(0x85);
            doneSkill = 1;
          }
          else if(skillMode=="4" && (gyroZ <= -1000 || accX >1.2)){//ヘビスマ
            if(DEBUG)Serial.println("accX > 3,skill4");
            Keyboard.press(0x85);
            doneSkill = 1;
          }
         else if(skillMode=="5" && (accX > 1.2 || gyroZ <= -1000)){//ストスマ
            if(DEBUG)Serial.println("accY > 3,skill5");
            Keyboard.press(0x85);
            doneSkill = 1;
          }
          else if(skillMode=="7" &&( gyroZ < -1000 || accX > 1.3)){
            if(DEBUG)Serial.println("gyroZ < -1500,skill7");
            Keyboard.press(0x85);
            doneSkill = 1;
          }
          else if(skillMode=="1" && gyroX > 500){
            if(DEBUG)Serial.println("gyroX < -1500,skill8");
            Keyboard.press(0x85);
            doneSkill = 1; 
          }
        }
      }
    }
    //===方法２：加速度・ジャイロ判定===
    else if(skillSwitch == 1 && G36Switch == 0){
      if(doneSkill == 0){
        if(accX > 2.5 || initialMotion == "accX+"){
            if(skillMotionCount==0){Keyboard.press(0x31);}
            else if(skillMotionCount==waitCount){Keyboard.press(0x85); doneSkill = 1;}
            skillMotionCount++;
            initialMotion = "accX+";
            skillMode = "1";
          }
        else if(accZ > 2.5 || initialMotion == "accZ+"){
            if(skillMotionCount==0){Keyboard.press(0x32);}
            else if(skillMotionCount==waitCount){Keyboard.press(0x85); doneSkill = 1;}
            skillMotionCount++;
            initialMotion = "accZ+";
            skillMode = "2";
        }
        else if(accX < -2.5 || initialMotion == "accX-"){
            if(skillMotionCount==0){Keyboard.press(0x33);}
            else if(skillMotionCount==waitCount){Keyboard.press(0x85); doneSkill = 1;}
            skillMotionCount++; 
            initialMotion = "accX-";
            skillMode = "3";     
        }
        else if(accZ < -2.5 || initialMotion == "accZ-"){
            if(skillMotionCount==0){Keyboard.press(0x34);}
            else if(skillMotionCount==waitCount){Keyboard.press(0x85); doneSkill = 1;}
            skillMotionCount++;
            initialMotion = "accZ-";
            skillMode = "4";
        }
      }
    }
    else if(skillSwitch == 0 && G36Switch == 1){
      if(doneSkill == 0){
        if(accX > 2.5 || initialMotion == "accX+"){
            if(skillMotionCount==0){Keyboard.press(0x35);}
            else if(skillMotionCount==waitCount){Keyboard.press(0x85); doneSkill = 1;}
            skillMotionCount++;
            initialMotion = "accX+";
            skillMode = "5";
          }
        else if(accZ > 2.5 || initialMotion == "accZ+"){
            if(skillMotionCount==0){Keyboard.press(0x36);}
            else if(skillMotionCount==waitCount){Keyboard.press(0x85); doneSkill = 1;}
            skillMotionCount++;
            initialMotion = "accZ+";
            skillMode = "6";
        }
        else if(accX < -2.5 || initialMotion == "accX-"){
            if(skillMotionCount==0){Keyboard.press(0x37);}
            else if(skillMotionCount==waitCount){Keyboard.press(0x85); doneSkill = 1;}
            skillMotionCount++; 
            initialMotion = "accX-";
            skillMode = "7";     
        }
        else if(accZ < -2.5 || initialMotion == "accZ-"){
            if(skillMotionCount==0){Keyboard.press(0x38);}
            else if(skillMotionCount==waitCount){Keyboard.press(0x85); doneSkill = 1;}
            skillMotionCount++;
            initialMotion = "accZ-";
            skillMode = "8";
        }
      }
    }else if(skillSwitch == 1 && G36Switch == 1 && viewSwitch == 1){
      //アイテムポケット3実行
      if(skillMotionCount==0){Keyboard.press(0x24);}
      else if(skillMotionCount==waitCount){Keyboard.press(0x66);}
      skillMotionCount++;
    }
    //===方法３：Bボタンでスキル選択・実行===
    else if(doneSkill == 0){
        //1秒押下で選択スキル連続実行
        if(M5.BtnB.pressedFor(1000)){
          //Mouse.press(MOUSE_LEFT);
          Mouse.click();
        }
        
        //1秒押下
        else if(M5.BtnB.wasReleasefor(500)){
          Mouse.move(0,0,1);
        }
        
        //200m秒押下で選択スキルを下へ移動。これは一回入力で100%成功する。
        else if(M5.BtnB.wasReleasefor(200)){
          Serial.println("Scroll Up");
          Mouse.move(0,0,-1);
        }
    
        //200m秒以下押下で左クリック。これは一回入力で100%成功する。
        else if(M5.BtnB.wasPressed()){
          Serial.println("Scroll Down");
          Mouse.press();
          //skillReset = 1;
        }
    }

/*
    if(skillSwitch==0){
        skillSelectCount = 0;あｓ
        skillExecCount = 0; 
        doneSkill = 0;  
        skillStopCount=0;
        skillMotionCount = 0;
    }*/
      
    //スキル実行後、0.3秒間スキル実行停止. チャタリング防止
    //if(doneSkill == 1 ||(skillSwitch==0 && skillSwitchPrevious==1) || (G36Switch==0 && G36aSwitchPrevious==1)){
        //skillStopCount++;
        //if(skillStopCount > Fs/3.0){
    if((skillSwitch==0 && skillSwitchPrevious==1) || (G36Switch==0 && G36SwitchPrevious==1) || M5.BtnB.wasReleased()){      
      skillSelectCount = 0;
      skillExecCount = 0; 
      doneSkill = 0;  
      skillStopCount=0;
      skillMotionCount = 0;
      //G36SwitchCnt=0;
      initialMotion ="";
      Mouse.release(MOUSE_LEFT);
      Mouse.release(MOUSE_BACK);
      Mouse.release(MOUSE_FORWARD);
      Mouse.release(MOUSE_MIDDLE);
      Keyboard.releaseAll();
    }
  }else if(contMode=="FPS"){
    if(skillSwitch == 1 && G36Switch ==0){
      Mouse.click(MOUSE_LEFT);
    }else if(skillSwitch == 0 && G36Switch ==1){
      Mouse.click(MOUSE_RIGHT); //長押しできないため、スコープズーム不可。キーボードで代用が良い。
    }
  }else if(contMode=="Mouse"){
    if(skillStopCount == 0 && viewSwitch == 0){
      if(skillSwitch == 1 && G36Switch ==0){
        Mouse.click(MOUSE_LEFT);
        skillStopCount++;
      }else if(skillSwitch == 0 && G36Switch ==1){
        Mouse.click(MOUSE_RIGHT);
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
      viewMode = "verylow";
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
    if(handMode == "Daiken"){
      handMode = "Normal";
    }else if(handMode == "Normal"){
      handMode = "Daiken";
    }
  } else if(M5.BtnA.wasReleasefor(10)){
    if(viewMode == "verylow"){ viewMode = "low";}
    else if(viewMode == "low"){ viewMode = "middle";}
    else if(viewMode == "middle"){viewMode ="high";} 
    else if(viewMode == "high"){viewMode ="verylow";} 
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
    acci += acci;
    for(int i=0; i< INTERVAL; i++){
      accX += accXs[i]; accY += accYs[i]; accZ += accZs[i];
    }
    accX/=INTERVAL;  accY/=INTERVAL;  accZ/=INTERVAL;

    //ジャイロの平滑化
    gyroXs[gyroi % INTERVAL] = gyroX;  gyroYs[gyroi % INTERVAL] = gyroY;  gyroZs[gyroi % INTERVAL] = gyroZ;
    gyroi += gyroi;
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
      if(contMode == "Mouse"){
        x = -(unwrapYaw - yawRef)*100;  y= (pitch - pitchRef)*100;
      }else{
        x = -(unwrapYaw - yawRef)*100;  y= (pitch - pitchRef)*50;        
      }
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
    if (txcount%(Fs/txFreq)== 0 && viewSwitch==1 && M5.BtnB.isReleased()) {
      if(contMode=="FEZ" && (G36Switch == 1 || skillSwitch==1)){}
      else if(contMode=="Mouse" && (skillSwitch == 0 && G36Switch ==1)){
        if(y < -5){ 
          Mouse.move(0,0,1);          
        }else if(y >5){
          Mouse.move(0,0,-1);          
        }
      }
      else{
         //視点スピード調整
         if(viewMode == "verylow"){
           x *= 0.2; y *=0.2;           
         }
         else if (viewMode == "low"){
           x *= 0.33; y *=0.33;
         }else if (viewMode == "middle"){
           x *= 0.5; y *=0.5;
         }else if(viewMode = "high"){  
            x *= 1; y *=1;
         }
        if(viewSwitch == 1){
          if(output_data =="gyroVel"){
            //xまたはyが128以上だと座標が反転してしまうため、xまたはyを127以下にする。
            if((abs(x) > 0 || abs(y) >0)){
              if(x>127) x=127;
              else if(x <-127) x=-127;
              if(y>127) y=127;
              else if(y<-127) y=-127;
              Mouse.move(x, y);
            }
          }else if(output_data == "rpy"){
            if((abs(x) > 0 || abs(y) >0)){
              if(x>127) x=127;
              else if(x <-127) x=-127;
              if(y>127) y=127;
              else if(y<-127) y=-127;
              Mouse.move(x, y);
            }
          }else if(output_data == "rpy_diff"){
            //x = -(unwrapYaw - yawRef)*1000; y= (pitch - pitchRef)*200; 
            //yawRef = unwrapYaw; pitchRef = pitch;
            if((abs(x) > 0 || abs(y) > 0)){
              if(x>127) x=127;
              else if(x <-127) x=-127;
              if(y>127) y=127;
              else if(y<-127) y=-127;
              Mouse.move(x, y);
            }
          }else{
            if(x>127) x=127;
            else if(x <-127) x=-127;
            if(y>127) y=127;
            else if(y<-127) y=-127;
            Mouse.move(x, y);
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
        if(abs(int(x_data)-120)>5 || abs(int(y_data)-120)>5){Mouse.move(-(int(x_data)-120), (int(y_data)-120)/2);}
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
      Mouse.move(x, -y);
    }
*/ 
    M5.update();
    if(txcount%(Fs/2)==1){
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setCursor(0, 20, 2);
      M5.Lcd.print("Mode:" + String(contMode));
      M5.Lcd.setCursor(0, 40, 2);
      M5.Lcd.print("Hand:" + handMode);
      M5.Lcd.setCursor(0, 60, 2);
      M5.Lcd.print("SkillTx:"+ String(skillSelectTxTimes));
      M5.Lcd.setCursor(0, 80, 2);
      M5.Lcd.print("Out:"+ output_data);
      M5.Lcd.setCursor(0, 100, 2);
      M5.Lcd.print("View:"+viewMode);
//      M5.Lcd.setCursor(0, 100, 2);
//      M5.Lcd.printf("out:%s",output_data);  
    }      
    
    txcount++;
    microsPrevious = microsPrevious + microsPerReading;
  }
}
