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

BluetoothSerial SerialBT;

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;

bool isConnected = false;
float freq       = -1; // frequency of measurements, Hz
int cycles       = -1; // Number of measurement cycles (loops)

void getMeasurements(int cycles, int freq)
{
    // Period = 1/freq. That's the time of delay betweem two measurements
    int period = 1000 / freq;

    for (int i = 0; i < cycles; i++) {

        // read the sensor
        IMU.readSensor();

        // Send the data through the serial monitor
        SerialBT.print(IMU.getAccelX_mss(), 6);
        SerialBT.print("\t");
        SerialBT.print(IMU.getAccelY_mss(), 6);
        SerialBT.print("\t");
        SerialBT.print(IMU.getAccelZ_mss(), 6);
        SerialBT.print("\t");
        SerialBT.print(IMU.getGyroX_rads(), 6);
        SerialBT.print("\t");
        SerialBT.print(IMU.getGyroY_rads(), 6);
        SerialBT.print("\t");
        SerialBT.print(IMU.getGyroZ_rads(), 6);
        SerialBT.print("\t");
        SerialBT.print(IMU.getMagX_uT(), 6);
        SerialBT.print("\t");
        SerialBT.print(IMU.getMagY_uT(), 6);
        SerialBT.print("\t");
        SerialBT.print(IMU.getMagZ_uT(), 6);
        SerialBT.print("\t");
        SerialBT.print(IMU.getMagBiasX_uT(), 6);
        SerialBT.print("\t");
        SerialBT.print(IMU.getMagBiasY_uT(), 6);
        SerialBT.print("\t");
        SerialBT.print(IMU.getMagBiasZ_uT(), 6);
        SerialBT.print("\t");
        SerialBT.print(IMU.getTemperature_C(), 6);
        SerialBT.println("");

        delay(period);
    }
    SerialBT.println("E\t");
}

void callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    if (event == ESP_SPP_SRV_OPEN_EVT) {
        Serial.println("Client Connected");
        isConnected = true;
    }
}

float parseFloat()
{
    char inputArr[ 80 ] = "";
    int i               = 0;
    int carriage        = 0;

    while (SerialBT.available()) {

        // Handling code here
        inputArr[ i ] = char(SerialBT.read());

        // Serial.println(inputArr[ i ]);
        if (inputArr[ i ] == '\n') {
            carriage++;
            break;
        }

        // if (carriage == 3 || i > 7)
        // break;

        i++;
    }

    return (atof(inputArr));
}

void setup()
{
    Serial.begin(115200);
    // Serial.begin(115200);
    SerialBT.register_callback(callback);

    if (!SerialBT.begin("ESP32")) {
        Serial.println("An error occurred initializing Bluetooth");
    }
    else {
        Serial.println("Bluetooth initialized");
    }
    
    // After we establish bluetooth communication, we have to wait until we have a serial input from MATLAB
    // Then, the IMU initilization and calibration can start
    while (!SerialBT.available()) {}
    Serial.println("OK. I'm ESP32 and I'm listening to you... Wait for calibration commands.");
    while(SerialBT.available()) {SerialBT.read();}
    Serial.println("Received serial...");

    // Start Communication with IMU
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
    Serial.print("Start Magnetometer Calibration. Please make oxtarakia...");
    SerialBT.print("Start Magnetometer Calibration. Please make oxtarakia... ");
    delay(1000);
    // Serial.print(" 2,");
    // delay(1000);
    // Serial.print(" 1,");
    // delay(1000);
    // Serial.print(" Go!");
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
    SerialBT.println("Start Accelerometer Calibration...");
    int status_accel = IMU.calibrateAccel();
    Serial.println("Complete Accelerometer Calibration");
    SerialBT.println("Complete Accelerometer Calibration");
    if (status_accel > 0) {
        Serial.println("Accelerometer Calibration Succesful!");
        SerialBT.println("Accelerometer Calibration Succesful!");
    }

    Serial.println("D\t");
    SerialBT.println("D\t");

    // clear buffer?

    // Receive the number of cycles we want it to run

    
}

void loop()
{
    Serial.println("New loop and waiting for input and it's right..."); // Just for test 

    // Here we take the number of cycles and frequency of measurements
    while (!SerialBT.available()) {}
    cycles = parseFloat();
    Serial.println(cycles);
    while (!SerialBT.available()) {}
    freq = parseFloat();
    if (freq == 0) {
        Serial.println("Frequency can't be 0, setting it to 10Hz...");
        freq = 10;
        }
    Serial.println(freq);
    

    //delay(500);

    if (isConnected) {
        while (!SerialBT.available()) {}
        while (1) {   
            int checky = (int)parseFloat();
            if (checky == 69) {
                Serial.println(checky);
                getMeasurements(cycles, freq);
                break;
            }
            delay(500);
        }
        
        while (SerialBT.available())
        {
             SerialBT.read(); 
        }
          
        Serial.println("Cleaned shit, let's go again!");
    }
}