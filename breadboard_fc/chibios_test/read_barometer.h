#ifndef __ReadBarometer_h
#define __ReadBarometer_h

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
int16_t t2, t3, p2, p3, p4, p5, p6, p7, p8, p9;
uint16_t t1, p1;

// Temperature values
int32_t t_fine;

void collect_calib_params();
float convert_temp(int32_t adc_T);
float convert_preasure(int32_t adc_P);
#endif