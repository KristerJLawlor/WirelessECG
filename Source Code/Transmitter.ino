///
#include <ArduinoBLE.h>
#include <Wire.h>
#include "max86150.h"

MAX86150 max86150Sensor;

int16_t sensorSignal;

/// <Summary> Checks that the max86150 sensor is detected and begins setup </Summary>
void SensorSetup()
{
    if (max86150Sensor.begin(Wire, I2C_SPEED_FAST) == false)
    {
        //Serial.println("MAX86150 was not found. Please check wiring/power. ");
        exit(0);
    } 
  
//default 4 3 800 411
    max86150Sensor.setup(0x1F, 1, 3, 800, 71, 4096); //Configure sensor. Use 6.4mA for LED drive
}

const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";

/// <Summary> Sets up BLE and Serial </Summary>
void setup() {
  Serial.begin(57600);
  SensorSetup();
  //while (!Serial);
 
 if (!BLE.begin()) {
    Serial.println("* Starting Bluetooth® Low Energy module failed!");
    exit(1);
  }
  
  BLE.setLocalName("Nano 33 BLE (Central)"); 
  BLE.advertise();
}

void loop() {
  connectToPeripheral();
}

void connectToPeripheral(){
  BLEDevice peripheral;
  
 //Serial.println("- Discovering peripheral device...");

  do
  {
    BLE.scanForUuid(deviceServiceUuid);
    peripheral = BLE.available();
  } while (!peripheral);
  
  if (peripheral) {
   //Serial.println("* Peripheral device found!");
    //Serial.print("* Device MAC address: ");
    //Serial.println(peripheral.address());
    //Serial.print("* Device name: ");
    //Serial.println(peripheral.localName());
    //Serial.print("* Advertised service UUID: ");
    //Serial.println(peripheral.advertisedServiceUuid());
    //Serial.println(" ");
    BLE.stopScan();
    controlPeripheral(peripheral);
  }
}

void controlPeripheral(BLEDevice peripheral) {
  //Serial.println("- Connecting to peripheral device...");

  if (peripheral.connect()) {
    //Serial.println("* Connected to peripheral device!");
    //Serial.println(" ");
  } else {
    //Serial.println("* Connection to peripheral device failed!");
    //Serial.println(" ");
    return;
  }

  //Serial.println("- Discovering peripheral device attributes...");
  if (peripheral.discoverAttributes()) {
    //Serial.println("* Peripheral device attributes discovered!");
    //Serial.println(" ");
  } else {
    //Serial.println("* Peripheral device attributes discovery failed!");
    //Serial.println(" ");
    peripheral.disconnect();
    return;
  }

  BLECharacteristic pipe = peripheral.characteristic(deviceServiceCharacteristicUuid);
    
  if (!pipe.canWrite()) {
    //Serial.println("* Peripheral does not have a writable characteristic!");
    peripheral.disconnect();
    return;
  }
  
  while (peripheral.connected()) {
    if(max86150Sensor.check() > 0)
    {
				sensorSignal = (uint16_t) (max86150Sensor.getECG() >> 2); //ECG swapped to FIFO version
        //sensorSignal = (uint16_t) (max86150Sensor.getFIFORed() >> 2); //PPG
				pipe.writeValue((long)sensorSignal);
        Serial.println(sensorSignal);
    }
  }
}  