/*
Advanced_I2C.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software
and associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "MPU9250.h"

#include "BluetoothSerial.h" //Header File for Serial Bluetooth, will be added by default into Arduino

BluetoothSerial ESP_BT; // Object for Bluetooth

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;
String inString = "";

int cycles   = 10; // number of cycles of measurements
float period = 1;
float freq   = 1;  // frequency of measurements, Hz
float input  = -1; // a variable to handle the inputs
float input2 = -2;
int checky   = 0; // just to check things

void setup()
{
    // Our programs waits for serial
    // Serial.begin(115200);
    Serial.begin(9600);                // Start Serial monitor in 9600
    ESP_BT.begin("ESP32_LED_Control"); // Name of your Bluetooth Signal
    Serial.println("Bluetooth Device is Ready to Pair");
    while (!Serial) {
    }

    // start communication with IMU
    status = IMU.begin();
    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
        while (1) {
        }
    }
    // setting the accelerometer full scale range to +/-8G
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
    // setting the gyroscope full scale range to +/-500 deg/s
    IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    // setting DLPF bandwidth to 20 Hz
    IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    // setting SRD to 19 for a 50 Hz update rate
    IMU.setSrd(19);

    // Calibrate Magnetometer
    Serial.print("Start Magnetometer Calibration. Please make oxtarakia in: 3,");
    delay(1000);
    Serial.print(" 2,");
    delay(1000);
    Serial.print(" 1,");
    delay(1000);
    Serial.print(" Go!");
    IMU.calibrateMag();
    Serial.println("Complete Magnetometer Calibration");

    // Calibrate Gyroscope
    Serial.println("Start Gyroscope Calibration...");
    int status_gyro = IMU.calibrateGyro();
    Serial.println("Complete Gyroscope Calibration");
    if (status_gyro > 0) {
        Serial.println("Gyro Calibration Succesful!");
    }

    // Calibrate Accelorometer
    Serial.println("Start Accelerometer Calibration...");
    int status_accel = IMU.calibrateAccel();
    Serial.println("Complete Accelerometer Calibration");
    if (status_gyro > 0) {
        Serial.println("Accelerometer Calibration Succesful!");
    }

    Serial.println("D\t");

    // clear buffer?
    // while(Serial.available()){Serial.read();}
    // delay(1000);

    // Receive the number of cycles we want it to run
    // put your main code here, to run repeatedly:
    // while (Serial.available() > 0) {

    // Here we take the number of cycles and frequency of measurements

    // Number of cycles
    // Serial.println("Gimme number of cycles ");
    delay(10000);
    if ((Serial.available() > 0)) {
        input = Serial.parseFloat();
    }
    else {
        input = 69;
    }
    Serial.println(input);
    cycles = input;

    // Frequency of measurements
    // Serial.println("Gimme freq of measurements ");
    delay(10000);
    if ((Serial.available() > 0)) {
        input2 = Serial.parseFloat();
    }
    else {
        input2 = 2;
    }
    Serial.println(input2);
    freq = input2;
}

void loop()
{

    delay(10000);

    // Number of cycles equals the number we entered before
    cycles = input;

    // Period = 1/freq. That's the time of delay betweem two measurements
    float period  = 1 / freq;
    float delayyy = period * 1000; // Actually we want ms motherfucker

    // There's one final delay before we start taking measurements. We
    // once again check with the help of Serial

    checky = Serial.parseFloat();
    while (1) {
        // Serial.println("Gimme 69!!!!!!");
        if (checky == 69) {
            break;
        }
        delay(500);
        checky = Serial.parseInt();
        // Serial.println(checky);
        // Serial.println("Still into loop");
    }

    delay(1000);

    for (int i = 0; i < cycles; i++) {

        // read the sensor
        IMU.readSensor();

        // Send the data through the serial monitor
        Serial.print(IMU.getAccelX_mss(), 6);
        Serial.print("\t");
        Serial.print(IMU.getAccelY_mss(), 6);
        Serial.print("\t");
        Serial.print(IMU.getAccelZ_mss(), 6);
        Serial.print("\t");
        Serial.print(IMU.getGyroX_rads(), 6);
        Serial.print("\t");
        Serial.print(IMU.getGyroY_rads(), 6);
        Serial.print("\t");
        Serial.print(IMU.getGyroZ_rads(), 6);
        Serial.print("\t");
        Serial.print(IMU.getMagX_uT(), 6);
        Serial.print("\t");
        Serial.print(IMU.getMagY_uT(), 6);
        Serial.print("\t");
        Serial.print(IMU.getMagZ_uT(), 6);
        Serial.print("\t");
        Serial.print(IMU.getMagBiasX_uT(), 6);
        Serial.print("\t");
        Serial.print(IMU.getMagBiasY_uT(), 6);
        Serial.print("\t");
        Serial.print(IMU.getMagBiasZ_uT(), 6);
        Serial.print("\t");
        Serial.print(IMU.getTemperature_C(), 6);
        Serial.println("");

        delay(delayyy);
    }

    Serial.println("E\t");

    Serial.end();
}