/*
 *   M5StickC Controller ver 1.0  
 *   BLE Periferal on connect mode
 */

#include <M5StickC.h>
#include <utility/MahonyAHRS.h> // GyroZ calibration

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SMODE  0 // stop mode
#define GMODE  1 // gesture mode
#define CMODE  2 // controler mode

#define LED_PIN 10   
#define LED_ON  LOW
#define LED_OFF HIGH

// mode status variable
uint8_t modeA = SMODE;  // BtnA, init as Stop mode
uint8_t modeB = 0;      // BtnB

// IMU sensor vars.
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

// GyroZ calibration
float stockedGyroZs[256];
int   stockCnt           = 0;
float adjustGyroZ        = 0;
int   stockedGyroZLength = 0;

// command vars, cmdID is acutually sended via BLE
String  cmd;   
uint8_t cmdID = 4; // 0-9

// BLE
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

#define SERVICE_UUID        "0001" //any ID
#define CHARACTERISTIC_UUID "0002" //any ID

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup() {
  // GyroZ calibration
  stockedGyroZLength=sizeof(stockedGyroZs)/sizeof(int);
  
  // M5 init
  M5.begin();

  // LCD init
  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.println("- M5StickC -");

  // IMU(MPU6886) init
  M5.MPU6886.Init();

  // LED init
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

  // Create the BLE Device
  BLEDevice::init("M5SickC_P"); 

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
  
  uint8_t buf[2]; //2byte
  
  M5.update();
  
  // Vbatt. 
  double vbat = M5.Axp.GetVbatData() * 1.1 / 1000;  

  // obtain IMU data
  M5.MPU6886.getGyroData(&gyroX,&gyroY,&gyroZ); // scaled
  M5.MPU6886.getAccelData(&accX,&accY,&accZ);   // scaled
  // M5.MPU6886.getAhrsData(&pitch,&roll,&yaw);    // scaled　
  // store gyroZ data for stockedGyroZLength times to calibration 
  if(stockCnt<stockedGyroZLength){
    stockedGyroZs[stockCnt]=gyroZ;
    stockCnt++;
  }else{
    if(adjustGyroZ==0){
      for(int i=0;i<stockedGyroZLength;i++){
        adjustGyroZ+=stockedGyroZs[i]/stockedGyroZLength;
      }
    }
    //avaraging stored gyroZ data to calibrate
    gyroZ-=adjustGyroZ; 
    MahonyAHRSupdateIMU(gyroX * DEG_TO_RAD, gyroY * DEG_TO_RAD, gyroZ * DEG_TO_RAD, accX, accY, accZ, &pitch, &roll, &yaw);
  }
  M5.MPU6886.getTempData(&temp);                // scales
   
  // mode change using BtnA
  if ( M5.BtnA.wasPressed() ) {
     
      modeA=(modeA+1) % 3;
      
      digitalWrite(LED_PIN, LED_ON);
      delay(10);
      digitalWrite(LED_PIN, LED_OFF);    
    
      M5.Lcd.setCursor(0, 0); 
      M5.Lcd.setTextSize(2);
      M5.Lcd.printf("Curr.mode: %d\r\n", modeA);
      M5.Lcd.printf("Vbat: %4.2f V\r\n", vbat);

  }

  if ( M5.BtnB.wasPressed() ) {
     
      modeB=(modeB+1) % 4;
      
      // Serial output mode change
      if ( modeB == 0 ) {
         Serial.printf("gyroX,gyroY,gyroZ\n");
      } else if ( modeB == 1 ) {
         Serial.printf("accX,accY,accZ\n");
      } else if ( modeB == 2 ) {
         Serial.printf("pitch,roll,yaw\n");
      } else if ( modeB == 3 ) {
         Serial.printf("temperature\n");
      }
  }
  
  // Serial output
  if ( modeB == 0 ) {
    Serial.printf("%6.2f,%6.2f,%6.2f\n", gyroX, gyroY, gyroZ);
  } else if ( modeB == 1 ) {
    Serial.printf("%5.2f,%5.2f,%5.2f\n", accX, accY, accZ);
  } else if ( modeB == 2 ) {
    Serial.printf("%5.2f,%5.2f,%5.2f\n", pitch, roll, yaw);
  } else if ( modeB == 3 ) {
    Serial.printf("%5.2f\n", temp);
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

  
  // stop mode
  if (modeA == SMODE) {
     cmd = "SS";  
     cmdID = 4;
  }
  // gesture mode
  else if  (modeA == GMODE) {
     // N/A
     cmd = "NA";  
     cmdID = 9; 
  }
  // controller mode
  else if  (modeA == CMODE) {  
     int degNA = 15;
     if (pitch > degNA) {
        if (roll > degNA) {
           cmd = "FR"; 
           cmdID = 0;
        } else if (abs(roll) <= degNA) {
           cmd = "FF"; 
           cmdID = 1;
        } else if (roll < -degNA) {
           cmd = "FL"; 
           cmdID = 2;
        }
     } else if (abs(pitch) <= degNA) {
        if (roll > degNA) {
           cmd = "RR"; 
           cmdID = 3;
        } else if (abs(roll) <= degNA) {
           cmd = "SS"; 
           cmdID = 4;
        } else if (roll < -degNA) {
           cmd = "LL"; 
           cmdID = 5;
        }
     } else if (pitch < -degNA) {
        if (roll > degNA) {
           cmd = "BR"; 
           cmdID = 6;
        } else if (abs(roll) <= degNA) {
           cmd = "BB"; 
           cmdID = 7;
        } else if (roll < - degNA) {
           cmd = "BL"; 
           cmdID = 8;
        }
     }
  }
  M5.Lcd.setCursor(5, 45);
  M5.Lcd.setTextSize(2);
  M5.Lcd.printf("BLE(P): %s\r\n",cmd);
     
  // BLE: Periferal--＞Central on Connect mode
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
