#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "I2C.h"
#include "Adafruit_BME280.h"


/* MasterTypeX are example buffers initialized in the master, they will be
 * sent by the master to the slave.
 * SlaveTypeX are example buffers initialized in the slave, they will be
 * sent by the slave to the master.
 * */

uint8_t MasterType2 [TYPE_2_LENGTH] = {'F', '4', '1', '9', '2', 'B'};
uint8_t MasterType1 [TYPE_1_LENGTH] = { 8, 9};
uint8_t MasterType0 [TYPE_0_LENGTH] = { 11};


uint8_t SlaveType2 [TYPE_2_LENGTH] = {0};
uint8_t SlaveType1 [TYPE_1_LENGTH] = {0};
uint8_t SlaveType0 [TYPE_0_LENGTH] = {0};

/* Used to track the state of the software state machine*/
I2C_Mode MasterMode = IDLE_MODE;

/* The Register Address/Command to use*/
uint8_t TransmitRegAddr = 0;

/* ReceiveBuffer: Buffer used to receive data in the ISR
 * RXByteCtr: Number of bytes left to receive
 * ReceiveIndex: The index of the next byte to be received in ReceiveBuffer
 * TransmitBuffer: Buffer used to transmit data in the ISR
 * TXByteCtr: Number of bytes left to transfer
 * TransmitIndex: The index of the next byte to be transmitted in TransmitBuffer
 * */
uint8_t ReceiveBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t RXByteCtr = 0;
uint8_t ReceiveIndex = 0;
uint8_t TransmitBuffer[MAX_BUFFER_SIZE] = {0};
uint8_t TXByteCtr = 0;
uint8_t TransmitIndex = 0;

I2C_Mode I2CB1_Master_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;
    RXByteCtr = count;
    TXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB1I2CSA = dev_addr;
    UCB1IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB1IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB1IE |= UCTXIE;                        // Enable TX interrupt

    UCB1CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(LPM0_bits + GIE);              // Enter LPM0 w/ interrupts

    return MasterMode;

}


I2C_Mode I2CB1_Master_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t count)
{
    /* Initialize state machine */
    MasterMode = TX_REG_ADDRESS_MODE;
    TransmitRegAddr = reg_addr;

    //Copy register data to TransmitBuffer
    CopyArray(reg_data, TransmitBuffer, count);

    TXByteCtr = count;
    RXByteCtr = 0;
    ReceiveIndex = 0;
    TransmitIndex = 0;

    /* Initialize slave address and interrupts */
    UCB1I2CSA = dev_addr;
    UCB1IFG &= ~(UCTXIFG + UCRXIFG);       // Clear any pending interrupts
    UCB1IE &= ~UCRXIE;                       // Disable RX interrupt
    UCB1IE |= UCTXIE;                        // Enable TX interrupt

    UCB1CTLW0 |= UCTR + UCTXSTT;             // I2C TX, start condition
    __bis_SR_register(LPM0_bits + GIE);              // Enter LPM0 w/ interrupts

    return MasterMode;
}

void CopyArray(uint8_t *source, uint8_t *dest, uint8_t count)
{
    uint8_t copyIndex = 0;
    for (copyIndex = 0; copyIndex < count; copyIndex++)
    {
        dest[copyIndex] = source[copyIndex];
    }
}

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
