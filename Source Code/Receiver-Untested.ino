#pragma region Includes
#include <ArduinoBLE.h> //The arduino Bluetooth Low Energy library
#include <ArduinoQueue.h> //A queue (FIFO) with a generic typing
#include <FIR.h> //A library for using an FIR filter
#pragma endregion

#pragma region Variable Initializations
#pragma region BLE Variables
const char* deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
const char* deviceServiceCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
#pragma endregion

#pragma region FIR Variables
FIR<long, 13> fir;
long firCoefficients[13] = {-364, -103, -42, 60, 173, 262, 295, 262, 173, 60, -42, -103, -364};
#pragma endregion

#pragma region Limiting Filter Variables
#define maxOutput 1500.0
uint8_t resetCounter = 0;
#define resetPoint 100
uint16_t lowestValue = 0;
uint16_t highestValue = 0;
#pragma endregion

#pragma region LowPass Filter Variables
uint16_t previousValue = 0;
#define firstCoefficient 0.37
#define secondCoefficient 0.63
#pragma endregion

#pragma region ECG Output Variables
#define leftArmPin A0
#define rightArmPin A4
#define rightLegPin A7
uint16_t leftArm;
uint16_t rightArm;
uint16_t rightLeg;
#pragma endregion

uint16_t ecgValue;
#pragma endregion

#pragma region BLE Information Setup
BLEService ecgService(deviceServiceUuid); 
BLEUnsignedShortCharacteristic ecgCharacteristic(deviceServiceCharacteristicUuid, BLERead | BLEWrite);
#pragma endregion

#pragma region Signal Filters
///<summary>Wrapper for the fir.processReading method</summary>
///<param name="value">The value to be filtered</param>
void FIRFilter(uint16_t* value){
  *value = fir.processReading(*value);
}

///<summary>Performs a custom Low Pass Filter</summary>
///<param name="value"> The value to be filtered</param>
void LowPassFilter(uint16_t* value){
  previousValue = ((firstCoefficient * *value) + (secondCoefficient * previousValue));
  *value = previousValue;
}
#pragma endregion

#pragma region ECG Output
///<summary>Limits all the values to be between 0 and max
///<param name="value">The ECG value to be filtered between 0 and max</param>
///<param name="max">The max value that can be output from the filter</param>
void LimitingFilter(uint16_t* value, float max){
  if (!resetCounter++ % resetPoint){
    lowestValue = 0;
    highestValue = max;
  }
  else
    resetCounter %= resetPoint;

  if (*value < lowestValue)
    lowestValue = *value;
  if (*value > highestValue)
    highestValue = *value;

  *value -= lowestValue;
  *value = (uint16_t)round(*value * (max / (float)(highestValue - lowestValue)));
}

///<summary>Sets the output voltages for each ECG lead based on a single ECG output</summary>
///<param name="ecgValue">The input ECG value read by the MAX86150 Board</param>
///<param name="leftArm">The output variable for the left arm voltage</param>
///<param name="rightArm">The output variable for the right arm voltage</param>
///<param name="rightLeg">The input/output variable for the left leg voltage</param>
///<param name="max">The max output value for each voltage</param>
///<param name="scale"A multiplier applied to the voltages to shrink the values</param>
void VoltageOutput(uint16_t ecgValue, uint16_t* leftArm, uint16_t* rightArm, uint16_t* rightLeg, float max, float scale){
  LimitingFilter(&ecgValue, max); //limit values before calculating
  ecgValue -= 500; //sloppy, idk what it does but the monitor works best with it
  *leftArm = (uint16_t) round(ecgValue * scale);
  *rightArm = (uint16_t) round(max * scale - *leftArm);

  *rightLeg += 5; //moving ground made the monitor read the signal more consistently
  if (*leftArm > *rightArm){
    if (*rightLeg >= *leftArm)
      *rightLeg = *rightArm;
  }
  else{
    if (*rightLeg >= *rightArm)
      *rightLeg = *leftArm;
  }
}
#pragma endregion

#pragma region Main Program
///<summary>The arduino setup function that starts Serial and BLE, begins advertising, and sets the FIR coefficients</summary>
void setup() {
  #pragma region Serial Startup
  Serial.begin(57600);
  #pragma endregion

  #pragma region BLE Startup
  if (!BLE.begin()) {
    Serial.println("- Starting Bluetooth® Low Energy module failed!");
    exit(1);
  } //BLE startup fails

  BLE.setLocalName("ECG Receiver");
  BLE.setAdvertisedService(ecgService);
  ecgService.addCharacteristic(ecgCharacteristic);
  BLE.addService(ecgService);
  ecgCharacteristic.writeValue(-1);
  BLE.advertise();
  #pragma endregion

  #pragma region FIR Setup
  fir.setFilterCoeffs(firCoefficients);
  #pragma endregion

  #pragma region Pin Output Setup
  pinMode(leftArmPin, OUTPUT);
  pinMode(rightArmPin, OUTPUT);
  pinMode(rightLegPin, OUTPUT);
  analogWriteResolution(12);
  #pragma endregion
}

///<summary>The main program loop containing BLE communication, filtering, and printing values</summary>
void loop() {
  BLEDevice central = BLE.central();
  Serial.println("Looking for transmitter");
  delay(1000);
  //loops here until a master device is found

  if (central) {
    //master device is found
    Serial.println("Found transmitter");

    previousValue = 0;
    ecgValue = 0;
    leftArm = 0;
    rightArm = 0;
    rightLeg = 0;
    while (central.connected()) {
      if (ecgCharacteristic.written()) {
         ecgValue = (uint16_t) ecgCharacteristic.value();
         //reads written BLE value
         VoltageOutput(ecgValue, &leftArm, &rightArm, &rightLeg, maxOutput, 0.3);
         analogWrite(leftArmPin, leftArm);
         analogWrite(rightArmPin, rightArm);
         analogWrite(rightLegPin, rightLeg);
       }
    }
    Serial.println("Disconnected to central device!");
    //connection lost, device will search for the transmitter to reconnect
  }
}
//#pragma endregion