# Wireless ECG Source Code 

The source code for this project is in two parts: [transmitter](https://github.com/KristerJLawlor/WirelessECG/blob/main/Source%20Code/Transmitter.ino), and [receiver](https://github.com/KristerJLawlor/WirelessECG/blob/main/Source%20Code/Receiver.ino) 

The transmitter program was responsible for communication with the MAX86150 using I2C to read a patient's ECG signal. The ECG signal data is then sent to the receiver for processing and displaying the data

The receiver program was responsible for taking the ECG value given by the transmitter and recreating the signal into three leads that can be plugged into an ECG monitor

The untested source code has been reorganized, optimized, and commented more than the official code, however, it has not been tested like the official code