//Uses https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library
/*
    @Purpose: Simple Accelerometer Reading
    @Author: Kevin Hu
*/

#include <Wire.h>

#include "MPU9250.h"
#include "quaternionFilters.h"

elapsedMillis accelReadTimer;

#define LED_PIN 13
#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    while (!Serial) {
    ; // wait for serial port to connect.
    } 

    pinMode(LED_PIN, OUTPUT);

    Serial.println(F("MPU9250 is online..."));

    myIMU.MPU9250SelfTest(myIMU.selfTest);

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    myIMU.initMPU9250();

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // Device calibration, wave device in a figure of 8 for 15 seconds to get
    // more realistic biases
    //myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);

    // Single line print for csv file
    //Serial.println("AccelX, AccelY, AccelZ, GyroRateX, GyroRateY, GyroRateZ, MagX, MagY, MagZ")
}



void loop() {
    if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) {
        myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
        // Convert accelerometer readings into g's. This depends on scale being set
        myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
        myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
        myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

        myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
        // Convert gyro readings into deg/s. This depends on scale being set
        myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
        myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
        myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

        myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
        // Convert magnetometer readings into milliGauss
        // Include factory calibration per data sheet and user environmental
        // corrections
        // Get actual magnetometer value, this depends on scale being set
        myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
        myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
        myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
    }

        // Must be called before updating quaternions!
        myIMU.updateTime();

        // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
        // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
        // (+ up) of accelerometer and gyro! We have to make some allowance for this
        // orientationmismatch in feeding the output to the quaternion filter. For the
        // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
        // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
        // modified to allow any convenient orientation convention. This is ok by
        // aircraft orientation standards! Pass gyro rate as rad/s
        MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                                myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                                myIMU.mx, myIMU.mz, myIMU.deltat);

    if (accelReadTimer >= 250) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // toggle led
        accelReadTimer = 0;
        Serial.println("AccelX, AccelY, AccelZ, GyroRateX, GyroRateY, GyroRateZ, MagX, MagY, MagZ");
/*
        Serial.print((int)1000 * myIMU.ax );
        Serial.print(",");
        Serial.print((int)1000 * myIMU.ay );
        Serial.print(",");
        Serial.print((int)1000 * myIMU.az );
        Serial.print(",");
        Serial.print(myIMU.gx, 3);
        Serial.print(",");
        Serial.print(myIMU.gy, 3);
        Serial.print(",");
        Serial.print(myIMU.gz, 3);
        Serial.print(",");
        */
        Serial.print(myIMU.mx);
        Serial.print(",");
        Serial.print(myIMU.my);
        Serial.print(",");
        Serial.print(myIMU.mz);

        Serial.println();
    }
}
