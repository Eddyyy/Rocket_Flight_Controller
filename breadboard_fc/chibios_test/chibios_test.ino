#include <Wire.h>

#include "MPU9250.h"
#include "quaternionFilters.h"
#include "ChRt.h"

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0

typedef struct imuReadings {
    float ax,ay,az,gx,gy,gz,mx,my,mz;
} IMU_t;

volatile IMU_t IMU;

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

MUTEX_DECL(imuMutex);

static THD_WORKING_AREA(waThd1, 64);     // Printing thread
static THD_WORKING_AREA(waThd2, 128);    // IMU readings thread

static THD_FUNCTION(printThd, arg) {
    (void)arg;
    float ax,ay,az,gx,gy,gz,mx,my,mz;
    while(true) {
        Serial.println("AccelX,AccelY,AccelZ,GyroRateX,GyroRateY,GyroRateZ,MagX,MagY,MagZ");
        chMtxLock(&imuMutex);
        ax = IMU.ax; ay = IMU.ay; az = IMU.az;
        gx = IMU.gx; gy = IMU.gy; gz = IMU.gz;
        mx = IMU.mx; my = IMU.my; mz = IMU.mz;
        chMtxUnlock(&imuMutex);
        Serial.printf("%f,%f,%f,%f,%f,%f,%f,%f,%f",ax,ay,az,gx,gy,gz,mx,my,mz);
        Serial.println();
        chThdSleepMilliseconds(1000);
    }
}

static THD_FUNCTION(readIMU, arg) {
    (void)arg;
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    myIMU.initMPU9250();
    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    while(true) {
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

            // Store volatile IMU
            chMtxLock(&imuMutex);
            IMU.ax = myIMU.ax; IMU.ay = myIMU.ay; IMU.az = myIMU.az;
            IMU.gx = myIMU.gx; IMU.gy = myIMU.gy; IMU.gz = myIMU.gz;
            IMU.mx = myIMU.mx; IMU.my = myIMU.my; IMU.mz = myIMU.mz;
            chMtxUnlock(&imuMutex);
            // Exit protected region

            chThdSleepMilliseconds(50);
    }
}

void chSetup() {
    // Schedule thread 1.
    chThdCreateStatic(waThd1, sizeof(waThd1), NORMALPRIO, printThd, NULL);

    // Schedule thread 2.
    chThdCreateStatic(waThd2, sizeof(waThd2), NORMALPRIO+1, readIMU, NULL);
}

void setup() {
    Serial.begin(115200);
    // Wait for USB Serial.
    while (!Serial) {
    ;
    }

    chBegin(chSetup);
    // chBegin() resets stacks and should never return.
    while (true) {}    
}

void loop() {}