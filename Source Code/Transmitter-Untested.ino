#pragma region Includes
#include <ArduinoBLE.h>
#include <Wire.h>
#include <max86150.h>
#pragma endregion

#pragma region Variable Initializations
#pragma region BLE Variables
const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
#pragma endregion

MAX86150 max86150Sensor;
uint16_t sensorSignal;
#pragma endregion

#pragma region Setup
/// <summary>Checks that the max86150 sensor is detected and begins setup</summary>
void SensorSetup()
{
    if (max86150Sensor.begin(Wire, I2C_SPEED_FAST) == false)
    {
        Serial.println("MAX86150 was not found. Please check wiring/power. ");
        exit(0);
    } 

    max86150Sensor.setup(0x1F, 1, 3, 800, 71, 4096);
}

/// <summary>Sets up BLE and Serial</summary>
void setup() {
  #pragma region Serial Startup
  Serial.begin(57600);
  #pragma endregion
  
  #pragma region MAX86150 Startup
  SensorSetup();
  #pragma endregion
 
  #pragma region BLE Startup
  if (!BLE.begin()) {
    Serial.println("* Starting Bluetooth® Low Energy module failed!");
    exit(0);
  }
  
  BLE.setLocalName("Nano 33 BLE (Central)"); 
  BLE.advertise();
  #pragma endregion
}
#pragma endregion

#pragma region Main program
///<summary>Main program loop that searches to connect to a receiver</summary>
void loop() {
  ConnectToReceiver();
}

///<summary>Searches for a receiver, connects to it, and then runs RunReceiver</summary>
void ConnectToReceiver(){
  BLEDevice receiver;

  do
  {
    BLE.scanForUuid(deviceServiceUuid);
    receiver = BLE.available();
  } while (!receiver);
  //Searches for a receiver

  if (receiver) {
    BLE.stopScan();
    //Stops searching once a receiver is found
    RunReceiver(receiver);
  }
}

///<summary>Runs the program to communicate ECG data to the receiver</summary>
///<param name="receiver">The BLE device acting as the receiver for this program</param>
void RunReceiver(BLEDevice receiver) {

  if (!receiver.connect()) {
    Serial.println("Receiver refused to connect");
    return; //Exit method to search for new receiver
  }

  BLECharacteristic pipe = receiver.characteristic(deviceServiceCharacteristicUuid);
  
  while (receiver.connected()) {
    if(max86150Sensor.check() > 0) //checks the FIFO to see if there is an unread value
    {
				sensorSignal = (uint16_t) (max86150Sensor.getECG() >> 2); //reads the immediate value
        //sensorSignal = (uint16_t) (max86150Sensor.getRed() >> 2); //PPG
				pipe.writeValue(sensorSignal);
        Serial.println(sensorSignal);
    }
  }
}
#pragma endregion