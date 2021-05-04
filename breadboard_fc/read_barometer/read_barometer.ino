/*
    @Purpose: Simple barometer reading
    @Author: Edward Thomson
*/

#include <inttypes.h>
#include <Wire.h>


elapsedMillis baroReadTimer;

#define LED_PIN 13

#define BMP280_I2C_ADDR 0x76

#define BMP280_MODE_SLEEP  0
#define BMP280_MODE_FORCED 1
#define BMP280_MODE_NORMAL 3
#define BMP280_MODE BMP280_MODE_NORMAL

#define BMP280_OVERSAMPLING_1  1
#define BMP280_OVERSAMPLING_2  2
#define BMP280_OVERSAMPLING_4  3
#define BMP280_OVERSAMPLING_8  4
#define BMP280_OVERSAMPLING_16 5
#define BMP280_OVERSAMPLING_P BMP280_OVERSAMPLING_16
#define BMP280_OVERSAMPLING_T BMP280_OVERSAMPLING_2

#define BMP280_FILTER_COEFFICIENT 2

#define BMP280_ID            0x58
#define BME280_ID            0x60

#define BMP280_REG_CALIB     0x88
#define BMP280_REG_ID        0xD0
#define BMP280_REG_RESET     0xE0
#define BMP280_REG_STATUS    0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG    0xF5
#define BMP280_REG_DATA      0xF7

// Internal calibration registers
int16_t _t2, _t3, _p2, _p3, _p4, _p5, _p6, _p7, _p8, _p9;
uint16_t _t1, _p1;

// Temperature values
int32_t t_fine;

/*
    Collects calibration params
*/
void collect_calib_params() {

    // read the calibration data
    uint8_t buf[24];
    // _dev->read_registers(BMP280_REG_CALIB, buf, sizeof(buf));
    Wire.beginTransmission(BMP280_I2C_ADDR);
    Wire.write(BMP280_REG_CALIB);
    Wire.endTransmission();
    for (i=0; i<=sizeof(buf); i++) {
        buf[i] = Wire.read();
    }

    _t1 = ((int16_t)buf[1] << 8) | buf[0];
    _t2 = ((int16_t)buf[3] << 8) | buf[2];
    _t3 = ((int16_t)buf[5] << 8) | buf[4];
    _p1 = ((int16_t)buf[7] << 8) | buf[6];
    _p2 = ((int16_t)buf[9] << 8) | buf[8];
    _p3 = ((int16_t)buf[11] << 8) | buf[10];
    _p4 = ((int16_t)buf[13] << 8) | buf[12];
    _p5 = ((int16_t)buf[15] << 8) | buf[14];
    _p6 = ((int16_t)buf[17] << 8) | buf[16];
    _p7 = ((int16_t)buf[19] << 8) | buf[18];
    _p8 = ((int16_t)buf[21] << 8) | buf[20];
    _p9 = ((int16_t)buf[23] << 8) | buf[22];


}

/*!
   Reads the temperature from the device.
   @return The temperature in degress celcius.
 */
float convert_temp(int32_t adc_T) {
  int32_t var1, var2;

  adc_T >>= 4;

  var1 = ((((adc_T >> 3) - ((int32_t)_t1 << 1))) *
          ((int32_t)_t2)) >>
         11;

  var2 = (((((adc_T >> 4) - ((int32_t)t1)) *
            ((adc_T >> 4) - ((int32_t)t1))) >>
           12) *
          ((int32_t)t3)) >>
         14;

  t_fine = var1 + var2;

  float T = (t_fine * 5 + 128) >> 8;
  return T / 100;
}

/*!
 * Reads the barometric pressure from the device.
 * @return Barometric pressure in Pa.
 */
float convert_preasure(int32_t adc_P) {
  int64_t var1, var2, p;

  // Must be done first to get the t_fine variable set up
  readTemperature();

  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)_bmp280_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3) >> 8) +
         ((var1 * (int64_t)_bmp280_calib.dig_P2) << 12);
  var1 =
      (((((int64_t)1) << 47) + var1)) * ((int64_t)_bmp280_calib.dig_P1) >> 33;

  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7) << 4);
  return (float)p / 256;
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    while (!Serial) {
    ; // wait for serial port to connect.
    } 

    pinMode(LED_PIN, OUTPUT);

    collect_calib_params();
}

void loop() {
    
    if (baroReadTimer >= 1000) {
        baroReadTimer = 0;
        Serial.println("Presure, Temperature");

        // Set mode to forced
        // Set oversampling to 1x for temp and preasure
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
        Wire.read();

        // Temperature
        int32_t tempRaw = Wire.read();
        tempRaw <<= 8;
        tempRaw |= Wire.read();
        Wire.read();
        float temp = convert_temp(tempRaw);

        Serial.print(preasureRaw);
        Serial.print("\t");
        Serial.print(tempRaw);
        Serial.println();

    }

    if (baroReadTimer <= 500) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, LOW);
    }
}
