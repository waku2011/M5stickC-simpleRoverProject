/*
 *   M5StickC Controller ver 1.0  
 *   BLE Periferal on connect mode
 */

#include <M5StickC.h>
#include <utility/MahonyAHRS.h> // GyroZ補正

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SMODE  0 // stop mode
#define GMODE  1 // gesture mode
#define CMODE  2 // controler mode

#define LED_PIN 10   // M5stickCのLEDは、GPIO10の電位を下げると発光
#define LED_ON  LOW
#define LED_OFF HIGH

// mode状態変数
uint8_t mode = SMODE;  // init as Stop mode

// センサ変数
float accX = 0;
float accY = 0;
float accZ = 0;

float gyroX = 0;
float gyroY = 0;
float gyroZ = 0;
 
float pitch = 0;
float roll  = 0;
float yaw   = 0;

float temp = 0;

//[追加]GyroZのデータを蓄積するための変数
float stockedGyroZs[256];
int   stockCnt=0;
float adjustGyroZ=0;
int   stockedGyroZLength=0;

// 送信コマンド文字列
String cmd;
uint8_t cmdID = 4; // 0-9 

// BLE
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

#define SERVICE_UUID        "0001" //適当なID）
#define CHARACTERISTIC_UUID "0002" //適当なID

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup() {
  // GyroZ補正サイズ
  stockedGyroZLength=sizeof(stockedGyroZs)/sizeof(int);
  
  // M5初期化
  M5.begin();

  // LCD初期化
  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("- M5StickC -");

  // MPU6886初期化
  M5.MPU6886.Init();

  // LED初期化
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

  // Create the BLE Device
  BLEDevice::init("M5SickC_P"); //適当な名前

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x00);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
//  Serial.println("Waiting a client connection to notify...");
}

void loop() {
  
  uint8_t buf[2]; //要素数２=2byte
  
  // Buttonクラスを利用するときには必ずUpdateを呼んで状態を更新する
  M5.update();
  
  // Vbatt. 状態取得
  double vbat = M5.Axp.GetVbatData() * 1.1 / 1000;  // 本体バッテリー電圧を取得

  // IMUデータ取得:
  M5.MPU6886.getGyroData(&gyroX,&gyroY,&gyroZ); // scaled
  M5.MPU6886.getAccelData(&accX,&accY,&accZ);   // scaled
  // M5.MPU6886.getAhrsData(&pitch,&roll,&yaw);    // scaled　
  // 起動時にstockedGyroZLengthの数だけデータを貯める
  if(stockCnt<stockedGyroZLength){
    stockedGyroZs[stockCnt]=gyroZ;
    stockCnt++;
  }else{
    if(adjustGyroZ==0){
      for(int i=0;i<stockedGyroZLength;i++){
        adjustGyroZ+=stockedGyroZs[i]/stockedGyroZLength;
      }
    }
    //貯めたデータの平均値を使ってgyroZを補正する
    gyroZ-=adjustGyroZ; 
    //ここでaccelデータと補正したgyroデータを使ってpitch・roll・yawを計算する
    MahonyAHRSupdateIMU(gyroX * DEG_TO_RAD, gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, accX, accY, accZ, &pitch, &roll, &yaw);
  }
  M5.MPU6886.getTempData(&temp);                // scales
   
  // ホームボタン(BtnA)を押してmode変更
  if ( M5.BtnA.wasPressed() ) {
      mode=(mode+1) % 3;
    
      digitalWrite(LED_PIN, LED_ON);
      delay(10);
      digitalWrite(LED_PIN, LED_OFF);    
    
      M5.Lcd.setCursor(0, 0); 
      M5.Lcd.setTextSize(2);
      M5.Lcd.printf("Curr.mode: %d\r\n", mode);
      M5.Lcd.printf("Vbat: %4.2f V\r\n", vbat);
   
      // プロッタ用のタイトル出力（Serial)
      if ( mode == 0 ) {
         Serial.printf("gyroX,gyroY,gyroZ\n");
      } else if ( mode == 1 ) {
         Serial.printf("accX,accY,accZ\n");
      } else if ( mode == 2 ) {
         Serial.printf("pitch,roll,yaw\n");
      }
  }
  
  // データ出力(Serial)
  if ( mode == 0 ) {
    Serial.printf("%6.2f,%6.2f,%6.2f\n", gyroX, gyroY, gyroZ);
  } else if ( mode == 1 ) {
    Serial.printf("%5.2f,%5.2f,%5.2f\n", accX, accY, accZ);
  } else if ( mode == 2 ) {
    Serial.printf("%5.2f,%5.2f,%5.2f\n", pitch, roll, yaw);
  } 
  //  M5.Lcd.setCursor(0, 30);
  //  M5.Lcd.printf("%.2f  %.2f  %.2f", gyroX, gyroY, gyroZ);
  //  M5.Lcd.setCursor(140, 30);
  //  M5.Lcd.print("dps");
  //  
  //  M5.Lcd.setCursor(0, 45);
  //  M5.Lcd.printf("%5.3f   %5.3f   %5.3f ",accX ,accY, accZ);
  //  M5.Lcd.setCursor(140, 45);
  //  M5.Lcd.print("G");
  //   
  //  M5.Lcd.setCursor(0, 60);
  //  M5.Lcd.printf("Temperature : %.1f deg.C", temp);

  // 制御コマンドの作成
  // stop mode
  if (mode == SMODE) {
     cmd = "SS";  
     cmdID = 4;
  }
  // gesture mode
  else if  (mode == GMODE) {
     // N/A
     cmd = "NA";  
     cmdID = 9; 
  }
  // controller mode
  else if  (mode == CMODE) {
     if (pitch > 15) {
        if (roll > 15) {
           cmd = "FR"; 
           cmdID = 0;
        } else if (abs(roll) <= 15) {
           cmd = "FF"; 
           cmdID = 1;
        } else if (roll < -15) {
           cmd = "FL"; 
           cmdID = 2;
        }
     } else if (abs(pitch) <= 15) {
        if (roll > 15) {
           cmd = "RR"; 
           cmdID = 3;
        } else if (abs(roll) <= 15) {
           cmd = "SS"; 
           cmdID = 4;
        } else if (roll < -15) {
           cmd = "LL"; 
           cmdID = 5;
        }
     } else if (pitch < -15) {
        if (roll > 15) {
           cmd = "BR"; 
           cmdID = 6;
        } else if (abs(roll) <= 15) {
           cmd = "BB"; 
           cmdID = 7;
        } else if (roll < -15) {
           cmd = "BL"; 
           cmdID = 8;
        }
     }
  }
  M5.Lcd.setCursor(5, 45);
  M5.Lcd.setTextSize(2);
  M5.Lcd.printf("BLE(P): %s\r\n",cmd);
     
  // BLE通信(制御コマンドの送信Periferal--＞Central on Connect mode)
  // notify changed value
  if (deviceConnected) {
        
        M5.Lcd.setTextSize(1);
        M5.Lcd.printf(" BLE connected    ");
  
        memset(buf,0,sizeof buf);  
        buf[0]=(uint8_t)(cmdID & 0xff);
        buf[1]=(uint8_t)((cmdID >>8) & 0xff);  //2byte
        pCharacteristic->setValue(buf,sizeof buf);
        pCharacteristic->notify();
        value++;
        delay(3); // bluetooth stack will go into congestion, if too many packets are sent, in 6 hours test i was able to go as low as 3ms
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
        M5.Lcd.setTextSize(1);
        M5.Lcd.printf(" BLE disconnected ");
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
//        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
  }
  
  delay(20);
}
