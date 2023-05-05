/**
 * Adafruit_BME280.c
 *  This file is based off the Adafruit_BME280 arduino repository. It has methods to read from the sensor using I2C.
 *  At the time of this version, it does not work.
 *      author: Christian Cipolletta
 *      version: 5/5/2023
 */

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "I2C.h"
#include "Adafruit_BME280.h"

/**
 *  Reads the temperature from the BME280
 */
float readTemperature(void)
{

    float temperature = 0;

    int32_t var1, var2;

    int32_t adc_T = I2CB1_Master_ReadReg(BME280_ADDRESS, BME280_REGISTER_TEMPDATA, 8);

    adc_T >>= 4;

    var1  = ((((adc_T>>3) - ((int32_t)cal_data.dig_T1 <<1))) *

             ((int32_t)cal_data.dig_T2)) >> 11;

    var2  = (((((adc_T>>4) - ((int32_t)cal_data.dig_T1)) *

               ((adc_T>>4) - ((int32_t)cal_data.dig_T1))) >> 12) *

             ((int32_t)cal_data.dig_T3)) >> 14;

    t_fine = var1 + var2;


    temperature  = (t_fine * 5 + 128) >> 8;

    temperature = temperature / 100;

    return temperature;
}

/**
 *  Reads the pressure from the BME280
 */
float readPressure(void) {

    float pressure = 0;

    int64_t var1, var2, p;

    int32_t adc_P = I2CB1_Master_ReadReg(BME280_ADDRESS, BME280_REGISTER_PRESSUREDATA, 24);
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;

    var2 = var1 * var1 * (int64_t)cal_data.dig_P6;

    var2 = var2 + ((var1*(int64_t)cal_data.dig_P5)<<17);

    var2 = var2 + (((int64_t)cal_data.dig_P4)<<35);

    var1 = ((var1 * var1 * (int64_t)cal_data.dig_P3)>>8) +

    ((var1 * (int64_t)cal_data.dig_P2)<<12);

    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)cal_data.dig_P1)>>33;


    if (var1 == 0) {

        // return 0;  // avoid exception caused by division by zero
        pressure = 0.0;
    }

    p = 1048576 - adc_P;

    p = (((p<<31) - var2)*3125) / var1;

    var1 = (((int64_t)cal_data.dig_P9) * (p>>13) * (p>>13)) >> 25;

    var2 = (((int64_t)cal_data.dig_P8) * p) >> 19;


    p = ((p + var1 + var2) >> 8) + (((int64_t)cal_data.dig_P7)<<4);

    pressure = (float)p/256;

    return pressure;
}

/**
 *  Reads the humidity from the BME280
 */
float readHumidity(void) {

    float humidity = 0;

    int32_t adc_H = I2CB1_Master_ReadReg(BME280_ADDRESS, BME280_REGISTER_HUMIDDATA, 16);

    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)cal_data.dig_H4) << 20) -

                    (((int32_t)cal_data.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *

                 (((((((v_x1_u32r * ((int32_t)cal_data.dig_H6)) >> 10) *

                      (((v_x1_u32r * ((int32_t)cal_data.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +

                    ((int32_t)2097152)) * ((int32_t)cal_data.dig_H2) + 8192) >> 14));


    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *

                               ((int32_t)cal_data.dig_H1)) >> 4));


    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;

    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;

    float h = (v_x1_u32r>>12);

    // return  h / 1024.0;
    humidity = h / 1024.0;

    return humidity;
}
