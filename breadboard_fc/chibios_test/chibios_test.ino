#include <Wire.h>

#include "MPU9250.h"
#include "quaternionFilters.h"
#include "TinyGPS++.h"
#include "read_barometer.h"
#include "ChRt.h"

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0

typedef struct imuReadings {
    float ax,ay,az,gx,gy,gz,mx,my,mz;
    bool isAvailable;
} IMU_t;

typedef struct gpsReadings {
    float lat,lng,hdop,alt,speed,course;
    uint32_t time;
    bool isSpeedUpd, isCourseUpd, isTimeUpd, isAltUpd, isHdopUpd;
    bool isAvailable;
} GPS_t;

typedef struct baroReadins {
    float temp, pressure;
    bool isAvailable;
} BARO_t;

volatile IMU_t IMU;
volatile GPS_t GPS;
volatile BARO_t baro;

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);
TinyGPSPlus myGPS;


MUTEX_DECL(imuMutex);
MUTEX_DECL(gpsMutex);
MUTEX_DECL(baroMutex);

static THD_WORKING_AREA(waThd1, 64);     // Printing thread

static THD_FUNCTION(printThd, arg) {
    (void)arg;
    float ax,ay,az,gx,gy,gz,mx,my,mz;
    float lat,lng,hdop,alt,speed,course;
    float temp, pressure;
    uint32_t gpsTime;
    bool isSpeedUpd, isCourseUpd, isTimeUpd, isAltUpd, isHdopUpd;
    while(true) {
        if (IMU.isAvailable) {
            chMtxLock(&imuMutex);
            ax = (int)1000 * IMU.ax; ay = (int)1000 * IMU.ay; az = (int)1000 * IMU.az;
            gx = IMU.gx; gy = IMU.gy; gz = IMU.gz;
            mx = IMU.mx; my = IMU.my; mz = IMU.mz;
            IMU.isAvailable = false;
            chMtxUnlock(&imuMutex);
            Serial.println("AccelX,AccelY,AccelZ,GyroRateX,GyroRateY,GyroRateZ,MagX,MagY,MagZ");
            Serial.print(ax); Serial.print(",");
            Serial.print(ay); Serial.print(",");
            Serial.print(az); Serial.print(",");
            Serial.print(gx, 3); Serial.print(",");
            Serial.print(gy, 3); Serial.print(",");
            Serial.print(gz, 3); Serial.print(",");
            Serial.print(mx); Serial.print(",");
            Serial.print(my); Serial.print(",");
            Serial.print(mz);
            Serial.println();
            Serial.println(GPS.isAvailable);
        }
        if (GPS.isAvailable) {
            chMtxLock(&gpsMutex);
            lat = GPS.lat; lng = GPS.lng; hdop = GPS.hdop;
            alt = GPS.alt; speed = GPS.speed; course = GPS.course;
            gpsTime = GPS.time; isSpeedUpd = GPS.isSpeedUpd;
            isCourseUpd = GPS.isCourseUpd; isTimeUpd = GPS.isTimeUpd;
            isAltUpd = GPS.isAltUpd; isHdopUpd = GPS.isHdopUpd;
            GPS.isAvailable = false;
            chMtxUnlock(&gpsMutex);
            Serial.println("Time,Lat,Long,HDOP,Alt,Speed,Course");
            if (isTimeUpd) {
                Serial.print(gpsTime); Serial.print(",");
            } else {
                Serial.print("NaN"); Serial.print(",");
            }
            Serial.print(lat); Serial.print(",");
            Serial.print(lng); Serial.print(",");
            if (isHdopUpd) {
                Serial.print(hdop); Serial.print(",");
            } else {
                Serial.print("NaN"); Serial.print(",");
            }
            if (isAltUpd) {
                Serial.print(alt); Serial.print(",");
            } else {
                Serial.print("NaN"); Serial.print(",");
            }
            if (isSpeedUpd) {
                Serial.print(speed); Serial.print(",");
            } else {
                Serial.print("NaN"); Serial.print(",");
            }
            if (isCourseUpd) {
                Serial.print(course); Serial.print(",");
            } else {
                Serial.print("NaN"); Serial.print(",");
            }
            Serial.println();
        }

        if (baro.isAvailable) {
            chMtxLock(&baroMutex);
            temp = baro.temp ; pressure = baro.pressure;
            baro.isAvailable = false;
            chMtxUnlock(&baroMutex);
            Serial.println("Temp,Pressure");
            Serial.print(temp); Serial.print(",");
            Serial.print(pressure); Serial.println();
        }
        chThdSleepMilliseconds(700);
    }
}

static THD_WORKING_AREA(waThd2, 128);    // IMU readings thread

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
            IMU.isAvailable = true;
            chMtxUnlock(&imuMutex);
            // Exit protected region

            chThdSleepMilliseconds(127);
    }
}

static THD_WORKING_AREA(waThd3, 128);

static THD_FUNCTION(readGPS, arg) {
    (void)arg;
    while (true) {
        while (Serial2.available()) {
            myGPS.encode(Serial2.read());
        }
        GPS_t gpsLocal;
        gpsLocal.lat = 0; gpsLocal.lng = 0; gpsLocal.alt = 0; gpsLocal.hdop = 0;
        gpsLocal.speed = 0; gpsLocal.time = 0; gpsLocal.course = 0;
        gpsLocal.isAltUpd = false; gpsLocal.isHdopUpd = false; gpsLocal.isTimeUpd = true;
        gpsLocal.isSpeedUpd = false; gpsLocal.isCourseUpd = false; gpsLocal.isAvailable = false;
        if (myGPS.location.isUpdated()) {
            gpsLocal.lat = myGPS.location.lat();
            gpsLocal.lng = myGPS.location.lng();
            if (myGPS.altitude.isUpdated()) {
                gpsLocal.isAltUpd = true;
                gpsLocal.alt = myGPS.altitude.meters();
            }
            if (myGPS.hdop.isUpdated()) {
                gpsLocal.isHdopUpd = true;
                gpsLocal.hdop = myGPS.hdop.hdop();
            }
            if (myGPS.time.isUpdated()) {
                gpsLocal.isTimeUpd = true;
                gpsLocal.time = myGPS.time.value();
            }
            if (myGPS.speed.isUpdated()) {
                gpsLocal.isSpeedUpd = true;
                gpsLocal.speed = myGPS.speed.mps();
            }
            if (myGPS.course.isUpdated()) {
                gpsLocal.isCourseUpd = true;
                gpsLocal.course = myGPS.course.deg();
            }
            chMtxLock(&gpsMutex);
            GPS.lat = gpsLocal.lat; GPS.lng = gpsLocal.lng; GPS.hdop = gpsLocal.hdop;
            GPS.alt = gpsLocal.alt; GPS.time = gpsLocal.time;
            GPS.speed = gpsLocal.speed; GPS.course = gpsLocal.course; 
            GPS.isSpeedUpd = gpsLocal.isSpeedUpd; GPS.isCourseUpd = gpsLocal.isCourseUpd;
            GPS.isAvailable = true;
            chMtxUnlock(&gpsMutex);
        }
        chThdSleepMilliseconds(203);

    }
}

static THD_WORKING_AREA(waThd4, 128);     // Baro thread

static THD_FUNCTION(readBaro, arg) {
    (void)arg;
    while(true) {
        BARO_t baroLocal;
        baroLocal.temp = 0; baroLocal.pressure = 0; baroLocal.isAvailable = false;
        Wire.beginTransmission(BMP280_I2C_ADDR);
        Wire.write(BMP280_REG_CTRL_MEAS);
        Wire.write(0x00 | 0b01 | 0b001<<2 | 0b001<<5);
        Wire.endTransmission();

        // Burst read temp and preasure
        Wire.beginTransmission(BMP280_I2C_ADDR);
        Wire.write(0xF7);
        Wire.endTransmission();
        Wire.requestFrom(BMP280_I2C_ADDR, 6);
        
        // Preasure val
        int32_t preasureRaw = Wire.read();
        preasureRaw <<= 8;
        preasureRaw |= Wire.read();
        preasureRaw <<= 8;
        preasureRaw |= Wire.read();

        // Temperature
        int32_t tempRaw = Wire.read();
        tempRaw <<= 8;
        tempRaw |= Wire.read();
        tempRaw <<= 8;
        tempRaw |= Wire.read();
        baroLocal.temp = convert_temp(tempRaw);
        baroLocal.pressure = convert_preasure(preasureRaw);
        chMtxLock(&baroMutex);
        baro.temp = baroLocal.temp ; baro.pressure = baroLocal.pressure;
        baro.isAvailable = true;
        chMtxUnlock(&baroMutex);
        chThdSleepMilliseconds(279);
    }
}

void chSetup() {
    // Schedule IMU read thread.
    chThdCreateStatic(waThd2, sizeof(waThd2), NORMALPRIO+4, readIMU, NULL);

    // Schedule GPS read thread.
    chThdCreateStatic(waThd3, sizeof(waThd3), NORMALPRIO+3, readGPS, NULL);

    // Schedule Baro read thread.
    chThdCreateStatic(waThd4, sizeof(waThd4), NORMALPRIO+2, readBaro, NULL);

    // Schedule print thread.
    chThdCreateStatic(waThd1, sizeof(waThd1), NORMALPRIO+1, printThd, NULL);
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600);
    Wire.begin();
    // Wait for USB Serial.
    while (!Serial) {
    ;
    }
    collect_calib_params();
    chBegin(chSetup);
    // chBegin() resets stacks and should never return.
    while (true) {}  
}

void loop() {}