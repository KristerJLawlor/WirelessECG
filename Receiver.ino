#pragma region Includes
#include <ArduinoBLE.h>
#include <stdlib.h>
#include <ArduinoQueue.h>
#include <FIR.h>
#pragma endregion
#pragma region Variable Initializations
#pragma region BLE Variables
const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
#pragma endregion
#pragma region FIR Variables
FIR<long, 13> fir;
long coef[13] = {-364, -103, -42, 60, 173, 262, 295, 262, 173, 60, -42, -103, -364};
#pragma endregion
#pragma region Limiting Variables
int resetCounter = 0;
int lowestValue = 0;
int highestValue = 4096;
#pragma endregion
#pragma region LowPass Variables
int previousValue = 0;
float firstValue = 0.37;
float secondValue = 0.63;
#pragma endregion
int ecgValue = 0;
#pragma endregion
#pragma region BLE Information Setup
BLEService ecgService(deviceServiceUuid); 
BLELongCharacteristic ecgCharacteristic(deviceServiceCharacteristicUuid, BLERead | BLEWrite);
#pragma endregion
#pragma region Filters
///<summary>Wrapper for the fir.processReading method</summary>
///<param name="value">The value to be filtered</param>
void FIRFilter(int* value){
  *value = fir.processReading(*value);
}

///<summary>Limits all the values to be between 0 and 4096 to be
void LimitingFilter(int* value){
  *value += 3072;

  if (*value > highestValue)
    highestValue = *value;
  *value = (int)round(*value * ((double)1500.0 / (double)highestValue));
}

///<summary>Performs a custom Low Pass Filter</summary>
///<param name="value"> The value to be filtered</param>
void LowPassFilter(int* value){
  previousValue = ((firstValue * *value) + (secondValue * previousValue));
  *value = previousValue;
}
#pragma endregion
#pragma region Main Program
///<summary>The arduino setup function that starts Serial and BLE, begins advertising, and sets the FIR coefficients</summary>
void setup() {
  #pragma region Serial Startup
  Serial.begin(57600);
  while (!Serial)
    ;
  #pragma endregion
  #pragma region BLE Startup
  if (!BLE.begin()) {
    Serial.println("- Starting Bluetooth® Low Energy module failed!");
    exit(1);
  } //BLE startup fails

  BLE.setLocalName("ECG Slave");
  BLE.setAdvertisedService(ecgService);
  ecgService.addCharacteristic(ecgCharacteristic);
  BLE.addService(ecgService);
  ecgCharacteristic.writeValue(-1);
  BLE.advertise();
  #pragma endregion
  #pragma region FIR Setup
  fir.setFilterCoeffs(coef);
  #pragma endregion
  #pragma region Pin Setup
  pinMode(A0, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A7, OUTPUT);
  analogWriteResolution(12);
  #pragma endregion
}

///<summary>The main program loop containing BLE communication, filtering, and printing values</summary>
void loop() {
  BLEDevice central = BLE.central();
  Serial.println("Looking for master");
  delay(1000);
  //loops here until a master device is found

int tempGround = 0;
  if (central) {
    //master device is found
    Serial.print("Found master");

    previousValue = 0;
    while (central.connected()) {
      if (ecgCharacteristic.written()) {
         ecgValue = ecgCharacteristic.value();
         //reads written BLE value     
         LimitingFilter(&ecgValue);
         ecgValue -= 500;
         Serial.print(ecgValue);
         Serial.print(' ');
                  int positive;
int negative;
int ground; //ground halfway between ?
//-3.3 volts = 0 positive, 4096 negative
//+3.3 volts = 4096 positive, 0 negative
positive = ecgValue * 0.3;
negative = (int) round(1500 * 0.3 - positive);
//positive += 800;
//negative += 800;

  Serial.print(positive);
         Serial.print(' ');
         Serial.print(negative);
         Serial.print(' ');

if (tempGround >= positive)
  tempGround = negative;
tempGround += 1;

analogWrite(A0, positive);
analogWrite(A4, tempGround);
analogWrite(A7, negative);
         Serial.println(tempGround);
       }
    }
    Serial.println("* Disconnected to central device!");
  }
}
//#pragma endregion