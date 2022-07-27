#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Wire.h>
#include <SHT2x.h>
#include <BleAdvertisedDevice.h>
#define Addr 0x40
#include <BLE2902.h>
#include <driver/adc.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
bool deviceConnected = false;
int cTemp;
int  humidity;
unsigned long lastTime = 0;
unsigned long timerDelay = 30000;
BLECharacteristic *pCharacteristic;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

void setup()
{
  
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_DB_11);
  Wire.begin();
  //serial.println("scanning the temperature and humidity")
  Serial.begin(115200);
  delay(1000);
  BLEDevice::init("ESP32-BLE-Server");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);
  // pService->addCharacteristic(&pCharacteristic);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
  pCharacteristic->addDescriptor(new BLE2902());
  //pCharacteristic->setCallbacks(new MyCallbacks());

  // pCharacteristic->setValue("Temperature in Celsius: 30 C");
  // pCharacteristic->setValue("Relative Humidity: 55 RH");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  Serial.println("Waiting a client connection to notify...");
}
int sensorP=0;
void loop()
{
  unsigned int data[2];
  if (deviceConnected) {
    if ((millis() - lastTime) > timerDelay) {
    // unsigned int data[2];

 
    Wire.beginTransmission(Addr);
    Wire.write(0xF3);
    Wire.endTransmission();
    delay(500);
    // Blynk.begin(Auth);
    Wire.requestFrom(Addr, 2);

    if (Wire.available() == 2)
    {
      data[0] = Wire.read();
      data[1] = Wire.read();
    }
    // Read temperature as Celsius (the default)
    int cTemp = (((data[0] * 256.0 + data[1]) * 175.72) / 65536.0) - 46.85;

    static char temperatureCTemp[6];
     dtostrf(cTemp, 6, 2, temperatureCTemp);
    //Set temperature Characteristic value and notify connected client
    pCharacteristic->setValue(temperatureCTemp);
    pCharacteristic->notify();
  }
  lastTime = millis();

  delay(1000);
}

    int val = adc1_get_raw(ADC1_CHANNEL_0);
 
      sensorP = analogRead(36);
    Serial.println(sensorP);
    Serial.println(val);
}
