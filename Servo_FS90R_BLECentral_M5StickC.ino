/* 
 *  Servo control for FS90R on M5StackC
 *  (BLE central)
*/

#include <M5StickC.h>

// BLE
#include "BLEDevice.h"

// The remote service we wish to connect to. (see Periferal sketch)
static BLEUUID serviceUUID("0001");
static BLEUUID    charUUID("0002");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

uint16_t val ; 

int PINL = 0;  //signal pin G0 for Left servo
int PWMCHL = 0;  // Left index
int PINR = 26; //signal pin G26 for Right servo
int PWMCHR = 1;  // Right index

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,  uint8_t* pData,  size_t length,  bool isNotify) {
    val=0;
    val=(uint16_t)(pData[1]<<8 | pData[0]); // numerous data of rechieved data from periferal
    Serial.println(val);
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }
  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());

    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");

    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic");

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      //std::string value = pRemoteCharacteristic->readValue();
      //Serial.print("The characteristic value was: ");
      //Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    connected = true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.

    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
//    if (advertisedDevice.haveServiceUUID() && advertisedDevice.getServiceUUID().equals(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks


void setup() {
  
  M5.begin();
  Serial.begin(1500000);
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");
  
  // LCD setup ... M5StickC resolution 80x160)
  M5.Axp.ScreenBreath(9);     // brightness
  M5.Lcd.setRotation(1);      // rotation
  M5.Lcd.setTextSize(2);      // font size (default = 1; 8dotW x 16dotH)
  M5.Lcd.fillScreen(BLACK);   // BLACK back color
  M5.Lcd.setCursor(8, 0, 1);  // set Cursor and font
  M5.Lcd.setTextColor(RED);   // text color
 
  // PWM setup (PWM period 50Hz=20ms, 10bit)
  pinMode(PINL, OUTPUT);
  ledcSetup(PWMCHL, 50, 10);  
  ledcAttachPin(PINL, PWMCHL);
  pinMode(PINR, OUTPUT); 
  ledcSetup(PWMCHR, 50, 10);
  ledcAttachPin(PINR, PWMCHR); 

  // BLE
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}

// servo rotation functions(from SV, duty ratio is calcrated)
// !!! This settings are for FS90R servo only !!!
int sv2duty(int SV){    //  SV:-41...0...41
   int duty = SV+77;    //  duty :36...77...118 (20ms/10bit)
   duty = max(36,min(duty, 118)); 
   return duty;
}

void setSpeed(int LR, int SV){ //  SV:-41...0...41
    if (LR == PWMCHL) {  
      int duty = sv2duty(SV);
      ledcWrite(PWMCHL, duty);      
    } else if (LR == PWMCHR){
      int duty = sv2duty(SV);
      ledcWrite(PWMCHR, duty);     
    }
}

//// basic movement functions

// SS stop
void stop(){
   setSpeed(0,0);
   setSpeed(1,0);
}

// FF forward
void forward(int SV){//  SV:-41...0...41
   setSpeed(0, SV); 
   setSpeed(1,-SV);
}

// BB backward(reverse)
void backward(int SV){//  SV:-41...0...41
   setSpeed(0,-SV); 
   setSpeed(1, SV);
}

// FL forwardLeftTurn
void turnFL(int SV){//  SV:-41...0...41
   setSpeed(0,  SV/2); 
   setSpeed(1, -SV  );
}
// FR forwardRightTurn
void turnFR(int SV){//  SV:-41...0...41
   setSpeed(0,  SV); 
   setSpeed(1, -SV/2);
}

// BL backwardLeftTurn
void turnBL(int SV){//  SV:-41...0...41
   setSpeed(0, -SV/2); 
   setSpeed(1,  SV);
}

// BR backwardRightTurn
void turnBR(int SV){//  SV:-41...0...41
   setSpeed(0, -SV); 
   setSpeed(1,  SV/2);
}

// LL leftTurn
void stopTurnL(int SV){//  SV:-41...0...41
   setSpeed(0, -SV/2); 
   setSpeed(1, -SV/2);
}

// RR RightTurn
void stopTurnR(int SV){//  SV:-41...0...41
   setSpeed(0, SV/2);
   setSpeed(1, SV/2);
}

void loop() {
    // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
/*
    String newValue = "Time since boot: " + String(millis()/1000);
    Serial.println("Setting new characteristic value to \"" + newValue + "\"");

    // Set the characteristic's value to be the array of bytes that is actually a string.
    pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
*/
  }else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just eample to start scan after disconnect, most likely there is better way to do it in arduino
  }
/*
  ledcWrite(PWMCHL, 36);  delay(1000);
  ledcWrite(PWMCHL, 66);  delay(1000);
  ledcWrite(PWMCHL, 77);  delay(1000);
  ledcWrite(PWMCHL, 87);  delay(1000);
  ledcWrite(PWMCHL, 118);  delay(1000);
  ledcWrite(PWMCHL, 77);  delay(1000);
*/

  M5.Lcd.fillScreen(BLACK);   // refesh LCD
  M5.Lcd.setCursor(0, 0);
  double vbat = M5.Axp.GetVbatData() * 1.1 / 1000;  // Vatt. for monitoring
  M5.Lcd.printf("Vbat: %4.2f V\r\n", vbat); 
  M5.Lcd.setCursor(5, 45);
  M5.Lcd.printf("BLE(C): %d",val); // val is cmdID sended from Periferal

   // Note that the duty ratio is currently constant .... :)
   // Includion of dependency to the angle of the periferal controller is future task ! 
  int SETV = 20;
  switch (val) {
     case 0:  // FR
       turnFR(SETV);
       break;
     case 1:  // FF
       forward(SETV);
       break;  
     case 2:  // FL
       turnFL(SETV);
       break;
     case 3:  // RR
       stopTurnR(SETV);
       break;
     case 4:  // SS
       stop();
       break;
     case 5:  // LL
       stopTurnL(SETV);
       break; 
     case 6:  // BR
       turnBR(SETV);
       break;
     case 7:  // BB
       backward(SETV);
       break;
     case 8:  // BL
       turnBL(SETV);
       break;
     case 9:  // NA (stop)
       stop();
       break;
  }
  
  delay(100);

}
