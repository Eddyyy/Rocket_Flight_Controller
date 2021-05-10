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
int16_t t2, t3, p2, p3, p4, p5, p6, p7, p8, p9;
uint16_t t1, p1;

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
        preasureRaw <<= 8;
        preasureRaw |= Wire.read();

        // Temperature
        int32_t tempRaw = Wire.read();
        tempRaw <<= 8;
        tempRaw |= Wire.read();
        tempRaw <<= 8;
        tempRaw |= Wire.read();
        float temp = convert_temp(tempRaw);
        float preasure = convert_preasure(preasureRaw);

        Serial.print(preasure);
        Serial.print("\t");
        Serial.print(temp);
        Serial.println();

    }

    if (baroReadTimer <= 500) {
        digitalWrite(LED_PIN, HIGH);
    } else {
        digitalWrite(LED_PIN, LOW);
    }
}
