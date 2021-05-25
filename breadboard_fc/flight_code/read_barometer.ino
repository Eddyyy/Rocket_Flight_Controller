/*
    @Purpose: Simple barometer reading
    @Author: Edward Thomson
*/

#include "read_barometer.h"
#include <inttypes.h>
#include <Wire.h>

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
    Wire.requestFrom(BMP280_I2C_ADDR, sizeof(buf));
    for (int i=0; i<int(sizeof(buf)); i++) {
        buf[i] = Wire.read();
    }

    t1 = ((int16_t)buf[1] << 8) | buf[0];
    t2 = ((int16_t)buf[3] << 8) | buf[2];
    t3 = ((int16_t)buf[5] << 8) | buf[4];

    p1 = ((int16_t)buf[7] << 8) | buf[6];
    p2 = ((int16_t)buf[9] << 8) | buf[8];
    p3 = ((int16_t)buf[11] << 8) | buf[10];
    p4 = ((int16_t)buf[13] << 8) | buf[12];
    p5 = ((int16_t)buf[15] << 8) | buf[14];
    p6 = ((int16_t)buf[17] << 8) | buf[16];
    p7 = ((int16_t)buf[19] << 8) | buf[18];
    p8 = ((int16_t)buf[21] << 8) | buf[20];
    p9 = ((int16_t)buf[23] << 8) | buf[22];


}

/*!
   Reads the temperature from the device.
   @return The temperature in degress celcius.
 */
float convert_temp(int32_t adc_T) {
  int32_t var1, var2;

  adc_T >>= 4;

  var1 = ((((adc_T >> 3) - ((int32_t)t1 << 1))) *
          ((int32_t)t2)) >>
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
 * Temperature must be read  and calculated before reading preasure
 * @return Barometric pressure in Pa.
 */
float convert_preasure(int32_t adc_P) {
  int64_t var1, var2, p;

  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)p6;
  var2 = var2 + ((var1 * (int64_t)p5) << 17);
  var2 = var2 + (((int64_t)p4) << 35);
  var1 = ((var1 * var1 * (int64_t)p3) >> 8) +
         ((var1 * (int64_t)p2) << 12);
  var1 =
      (((((int64_t)1) << 47) + var1)) * ((int64_t)p1) >> 33;

  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)p9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)p8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)p7) << 4);
  return (float)p / 256;
}