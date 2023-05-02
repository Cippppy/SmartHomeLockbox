#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "Drivers/GPIO_Driver.h"
#include "Drivers/FR2355_UART_Driver.h"
#include <Drivers/Adafruit_BME280.h>

/**
 * Final2.c
 */

// Variable Instantiation
char Vibration_Data_In;
float Temperature_Data_In;
float Humidity_Data_In;
float Pressure_Data_In;
volatile float CO2_Reading;
unsigned int position;

//on board temperature definitions
volatile float IntDegF;
#define CALADC_15V_30C  *((unsigned int *)0x1A1A)
#define CALADC_15V_85C  *((unsigned int *)0x1A1C)

//ADC mode definitions
#define CO2 1
#define OBTEMP 0
volatile unsigned char modeADC = 0x00;

// Text that prints to terminal if distance < 15
char alertText[] = {'D','A','N','G','E','R', ' ','N','E','A','R'};

// Text that prints to terminal when uart connects
char connectedText[] = {'S','u','c','c','e','s','s','f','u','l','l','y',' ','C','o','n','n','e','c','t','e','d'};

volatile unsigned long distance = 20;    // Holds distance calculated by ultrasonic sensor
volatile unsigned int pulseCount = 0;   // Holds count used to calculate distance

// Peripheral Instantiation
void Init_HCSR04();
void Init_Buzzer();
void Init_Servo();
void Init_LED();
void Init_WIFI();
void Init_TEMP();
void Init_CO2();
void Init_UART_Pins();
void Run_Ultrasonic();
// Method Instantiation
void readOBTemp();
void Init_ADC();
void readCO2();
void updateThingSpeak();
//void Run_Temp();
void Ultrasonic_Wait(unsigned int timeToWait);
void setServo(unsigned int angle);


void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;                 // Stop WDT
    distance = 20;
    Init_HCSR04();
    Init_Buzzer();
    Init_Servo();
    Init_LED();
    Init_WIFI();
    Init_TEMP();
    Init_CO2();
    Init_ADC();
    Init_UART_Pins();

    // Disable the GPIO power-on default high-impedance mode to activate
    // previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    __bis_SR_register(LPM0_bits + LPM3_bits + GIE); // Enter LPM0 and LPM3
    __enable_interrupt();   // Enables interrupts
    __no_operation();       // For debugger

    while(1)
    {
        Run_Ultrasonic();
        readOBTemp();
        while((ADCIFG & ADCIFG)==0);
        readCO2();
//        Run_Temp();
//        Temperature_Data_In = readTemperature();
//        Humidity_Data_In = readHumidity();
//        Pressure_Data_In = readPressure();
        if(distance < 15) { // If distance is less than 15 (maybe cm?)
            setServo(180);
            gpioWrite(3,2,1);   // Turn on Red LED
            gpioWrite(3,3,1);   // Turn on Blue LED
            gpioWrite(3,7,1);   // Turn on Buzzer
        }
        else {  // If distance is further than 15
            setServo(0);
            gpioWrite(3,2,0);   // Turn off Red LED
            gpioWrite(3,3,0);   // Turn off Blue LED
            gpioWrite(3,7,0);   // Turn off buzzer
        }
        updateThingSpeak();
        Ultrasonic_Wait(65000); // Wait some time (Maybe 65000 ms)
    }
}


/**
 * Peripheral Setups
 */

// Sets up ADC for On-Board Temp sensor and CO2 sensor
void Init_ADC()
{
    ADCCTL0 |= ADCSHT_8 | ADCON;                                  // ADC ON,temperature sample period>30us
    ADCCTL1 |= ADCSHP;                                            // s/w trig, single ch/conv, MODOSC
    ADCCTL2 &= ~ADCRES;                                           // clear ADCRES in ADCCTL
    ADCCTL2 |= ADCRES_2;                                          // 12-bit conversion results
    ADCMCTL0 |= ADCSREF_1 | ADCINCH_12;                           // ADC input ch A12 => temp sense
    ADCIE |=ADCIE0;                                               // Enable the Interrupt request for a completed ADC_B conversion
    modeADC = OBTEMP;

    PMMCTL0_H = PMMPW_H;                                          // Unlock the PMM registers
    PMMCTL2 |= INTREFEN | TSENSOREN;                              // Enable internal reference and temperature sensor
    __delay_cycles(400);                                          // Delay for reference settling
}

// Setting up pins for Computer terminal connection
void Init_UART_Pins()
{
    P4DIR = 0xFF; P2DIR = 0xFF;
    P4REN = 0xFF; P2REN = 0xFF;
    P4OUT = 0x00; P2OUT = 0x00;
    // Configure UART pins
    P4SEL0 |= BIT2 | BIT3;                    // set 2-UART pin as second function
}


// Setup Ultrasonic sensor to Pins 5.1 and 5.0 and Timer B2
void Init_HCSR04()
{
    gpioInit(5,1,1); // Set Pin 5.1 to OUTPUT (TRIG)
    gpioWrite(5,1,0); // Set Pin 5.1 to LOW

    gpioInit(5,0,0); // Set Pin 5.0 to INPUT (ECHO)
    P5SEL0 |= BIT0;
    P5SEL1 &= ~BIT0;

    // Configure Timer_B2
    TB2CCR0 = 50;   // Set to 50 because anything lower is too fast
    TB2CCTL0 = CCIE;    // TB2CCR0 interrupt enabled
    TB2CTL = TBSSEL__SMCLK | MC_2 | TBCLR;    // SMCLK, continuous mode, clear TBR
}

// Setup Buzzer to Pin 3.7
void Init_Buzzer()
{
    gpioInit(3,7,1); // Set Pin 3.7 to OUTPUT (Transistor Base)
    gpioWrite(3,7,0); // Set Pin 3.7 to LOW
}

// Setup Servo to Pin 6.0 and Timer B3
void Init_Servo()
{
    gpioInit(6,0,1); // Set Pin 6.0 to OUTPUT (Servo PWM)
    P6SEL0 |= BIT0;
    P6SEL1 &= ~BIT0; // P6.0 options select

    // Configure Timer_B3 for PWM
    TB3CCR0 = 39000;                         // PWM Period of 50Hz
    TB3CCTL1 = OUTMOD_7;                      // CCR1 reset/set
    TB3CCR1 = 39000*0.075;                   // CCR1 PWM duty cycle
    TB3CTL = TBSSEL__SMCLK | MC__UP | TBCLR | ID_2;  // SMCLK, up mode, clear TBR, Divide clock by 4
}

// Setup LEDs to Pin 3.2 and 3.3
void Init_LED()
{
    gpioInit(3,2,1); // Set Pin 3.2 to OUTPUT (RED LED)
    gpioWrite(3,2,0); // Set Pin 3.2 to LOW

    gpioInit(3,3,1); // Set Pin 3.3 to OUTPUT (BLUE LED)
    gpioWrite(3,3,0); // Set Pin 3.3 to LOW
}

// Setup Wifi Module
void Init_WIFI()
{
    UCA0CTLW0 |= UCSWRST; // Set USCI_A0 into software reset

    UCA0CTLW0 |= UCSSEL__ACLK; // Choose BRCLK=ACLK=32.768kHz
    UCA0BRW = 3; // Prescaler
    UCA0MCTLW |= 0X9200;    // Modulation

    // Tx Pin Setup
    P1SEL0 |= BIT7;
    P1SEL1 &= ~BIT7;

    UCA0CTLW0 &= ~UCSWRST; // Take USCI_A0 out of software reset
    UCA0IE |= UCRXIE;
}

// Setup Temperature Sensor to I2C Pins 1.6 and 1.7
void Init_TEMP()
{
    UCB1CTLW0 |= UCSWRST; // Set USCI_B1 into software reset

    UCB1CTLW0 |= UCSSEL_3; // Choose BRCLK=SMCLK=1MHz
    UCB1BRW = 10; // Divide BRCLK by 10 for SCL = 100kHz
    // Put into I2C mode, Put into Master mode, Synchronous mode
    UCB1CTLW0 |= UCMODE_3 | UCMST | UCSYNC | UCTR;
    // Slave addr is 7 bits, single master
    UCB1CTLW0 &= ~UCMM & ~UCSLA10;
    UCB1I2CSA = BME280_ADDRESS; // Slave address = 0x77 or 0x76
    UCB1CTLW1 |= UCASTP_2; // Auto STOP when UCB0TBCNT reached
    UCB1TBCNT = 20; // Send 1 byte of data

    // SCL Pin Setup
    P4SEL0 |= BIT7;
    P4SEL1 &= ~BIT7;

    // SDA Pin Setup
    P4SEL0 |= BIT6;
    P4SEL1 &= ~BIT6;

    UCB1CTLW0 &= ~UCSWRST; // Take USCI_B1 out of software reset

    UCB1IE |= UCTXIE; // Enable I2C Tx0 IRQ (writing interrupt)
    UCB1IE |= UCRXIE | UCNACKIE | UCBCNTIE; // Enable I2C Rx0 IRQ (receiving interrupt)
}

// Setup Vibration sensor to I2C Pins 1.2 and 1.3
void Init_Vibration()
{
    UCB0CTLW0 |= UCSWRST; // Set USCI_B0 into software reset

    UCB0CTLW0 |= UCSSEL_3; // Choose BRCLK=SMCLK=1MHz
    UCB0BRW = 10; // Divide BRCLK by 10 for SCL = 100kHz
    UCB0CTLW0 |= UCMODE_3 | UCMST; // Put into I2C mode, Put into Master mode
    UCB0I2CSA = 0x0068; // Slave address = 0x68
    UCB0CTLW1 |= UCASTP_2; //Auto STOP when UCB0TBCNT reached
    UCB0TBCNT = 0x01; // Send 1 byte of data

    // SCL Pin Setup
    P1SEL0 |= BIT3;
    P1SEL1 &= ~BIT3;

    // SDA Pin Setup
    P1SEL0 |= BIT2;
    P1SEL1 &= ~BIT2;

    UCB0CTLW0 &= ~UCSWRST; // Take USCI_B0 out of software reset

    UCB0IE |= UCTXIE0; // Enable I2C Tx0 IRQ (writing interrupt)
    UCB0IE |= UCRXIE0; // Enable I2C Rx0 IRQ (receiving interrupt)
}

// Setup CO2 Sensor to Pin 1.1 and ADC
void Init_CO2()
{
    gpioInit(1,1,0); // Set Pin 1.1 to INPUT (C02 Sensor)
}


/**
 * Peripheral Functions
 */

void Run_Ultrasonic()
{
    TB2CCR0 = 50;
    pulseCount = 0;
    unsigned int countLow = 0x00;   // Set count of LOW part of echo pulse to 0
    unsigned int countHigh = 0x00;  // Set count of HIGH part of echo pulse to 0
    gpioWrite(5,1,0);   // Set Trig pin LOW
    __delay_cycles(2);  // Delay 5 microseconds
    gpioWrite(5,1,1);   // Set Trig pin HIGH
    __delay_cycles(10); // Delay 10 microseconds
    gpioWrite(5,1,0);   // Set Trig Pin LOW
    while(gpioRead(5,0) == 0);
    countLow = pulseCount * 5;   // Set count of LOW part of echo pulse to pulseCount times 5 because of TB2CCR0 offset
    while(gpioRead(5,0) == 1);
    countHigh = pulseCount * 5;  // Set count of HIGH part of echo pulse to pulseCount times 5 because of TB2CCR0 offset
    unsigned int pulseWidth = countHigh - countLow; // Pulse width = total count - count of LOW part
    distance = (unsigned long) pulseWidth / 58.0;   // Conversion to cm according to datasheet
}

// Will be used to set servo angle
void setServo(unsigned int angle)
{
    if(angle >= 180) {
        TB3CCR1 = 39000 * 0.125;
    }
    else if(angle <= 0) {
        TB3CCR1 = 39000 * 0.03;
    }
    else {
        TB3CCR1 = 20.6 * angle + 1170;
    }
}

// Used to make the ultrasonic wait without using delay
void Ultrasonic_Wait(unsigned int timeToWait)
{
    unsigned int timeWaited = pulseCount;
    while((pulseCount - timeWaited) <= timeToWait);
}

//void Run_Temp() {
////    __delay_cycles(2000);
////    while (UCB1CTLW1 & UCTXSTP);         // Ensure stop condition got sent
////    UCB1CTL1 |= UCTXSTT;
//    UCB1CTLW0 |= UCTR;
//    UCB1CTLW0 |= UCTXSTT;
//    while ((UCB1IFG & UCSTPIFG) == 0);
//    UCB1CTLW0 &= ~UCTR;
//    UCB1CTLW0 |= UCTXSTT;
//    while ((UCB1IFG & UCSTPIFG) == 0);
//    UCB0IFG &= ~UCSTPIFG;
//}

// Read the CO2 sensor (A0)
void readCO2()
{
    modeADC = CO2;
    ADCMCTL0 |= ADCINCH_1;
    ADCCTL0 |= ADCENC | ADCSC;                                    // Sampling and conversion start
}

// Read the onboard Temperature sensor
void readOBTemp()
{
    modeADC = OBTEMP;
    ADCMCTL0 |= ADCINCH_12;
    ADCCTL0 |= ADCENC | ADCSC;                                    // Sampling and conversion start
}

void updateThingSpeak()
{
    int intOBT = (int) IntDegF;
    int intCO2 = (int) CO2_Reading;
    int intBME280 = (int) Temperature_Data_In;
    char message[13];
    message[0] = (intOBT / 100) + '0';
    message[1] = ((intOBT / 10) % 10) + '0';
    message[2] = (intOBT % 10) + '0';
    message[3] = ';';
    message[4] = (intCO2 / 1000) + '0';
    message[5] = ((intCO2 / 100) % 10) + '0';
    message[6] = ((intCO2 / 10) % 10) + '0';
    message[7] = (intCO2 % 10) + '0';
    message[8] = ':';
    message[9] = (intBME280 / 100) + '0';
    message[10] = ((intBME280 / 10) % 10) + '0';
    message[11] = (intBME280 % 10) + '0';
    message[12] = '.';
    //uart_Print(message);
    while((UCA0IFG & UCTXIFG)==0);
    UCA0TXBUF = message[0];
    while((UCA0IFG & UCTXIFG)==0);
    UCA0TXBUF = message[1];
    while((UCA0IFG & UCTXIFG)==0);
    UCA0TXBUF = message[2];
    while((UCA0IFG & UCTXIFG)==0);
    UCA0TXBUF = message[3];
    while((UCA0IFG & UCTXIFG)==0);
    UCA0TXBUF = message[4];
    while((UCA0IFG & UCTXIFG)==0);
    UCA0TXBUF = message[5];
    while((UCA0IFG & UCTXIFG)==0);
    UCA0TXBUF = message[6];
    while((UCA0IFG & UCTXIFG)==0);
    UCA0TXBUF = message[7];
    while((UCA0IFG & UCTXIFG)==0);
    UCA0TXBUF = message[8];
    while((UCA0IFG & UCTXIFG)==0);
    UCA0TXBUF = message[9];
    while((UCA0IFG & UCTXIFG)==0);
    UCA0TXBUF = message[10];
    while((UCA0IFG & UCTXIFG)==0);
    UCA0TXBUF = message[11];
    while((UCA0IFG & UCTXIFG)==0);
    UCA0TXBUF = message[12];
}

/**
 * Interrupt Routines
 */

// Interrupt routine for Vibration sensor
#pragma vector=EUSCI_B0_VECTOR
__interrupt void EUSCI_B0_I2C_ISR(void)
{
    switch(UCB0IV) // Which IRQ flag triggered
    {
    case 0x16 : // RXIFG0
        Vibration_Data_In = UCB0RXBUF; // Runs when Rx buffer has received data from the slave.
        break;
    case 0x18 : // TXIFG0
        UCB0TXBUF = 0xBB; // Runs when Tx buffer is ready to receive data from master.
        break;
    default :
        break;
    }
}

// Interrupt routine for Temperature sensor
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = USCI_B1_VECTOR
__interrupt void USCIB1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_B1_VECTOR))) USCIB1_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(UCB1IV) {
    case USCI_I2C_UCNACKIFG :
        UCB1CTL1 |= UCTXSTT;
        break;
    case 0x16 :
        Temperature_Data_In = UCB1RXBUF;
        break;
    case 0x18 :
        UCB1TXBUF = BME280_REGISTER_TEMPDATA;
        break;
    default :
        break;
    }
}

// Timer B2 interrupt service routine (For counting how long echo is high)
#pragma vector = TIMER2_B0_VECTOR
__interrupt void Timer2_B0_ISR(void)
{
    pulseCount++;
    TB2CCR0 += 50;               // Add Offset to TB0CCR0 (anything smaller than 50 is too fast)
}

// ADC interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC_VECTOR
__interrupt void ADC_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC_VECTOR))) ADC_ISR (void)
#else
#error Compiler not supported!
#endif
{
    volatile float temp;

    volatile float IntDegC;

    switch(__even_in_range(ADCIV,ADCIV_ADCIFG))
    {
        case ADCIV_NONE:
            break;
        case ADCIV_ADCOVIFG:
            break;
        case ADCIV_ADCTOVIFG:
            break;
        case ADCIV_ADCHIIFG:
            break;
        case ADCIV_ADCLOIFG:
            break;
        case ADCIV_ADCINIFG:
            break;
        case ADCIV_ADCIFG:
            if(modeADC) {
                CO2_Reading = ADCMEM0;
            }
            else {
                temp = ADCMEM0;
                // Temperature in Celsius
                // The temperature (Temp, C)=
                IntDegC = (temp-CALADC_15V_30C)*(85-30)/(CALADC_15V_85C-CALADC_15V_30C)+30;

                // Temperature in Fahrenheit
                // Tf = (9/5)*Tc | 32
                IntDegF = 9*IntDegC/5+32;
                //__bic_SR_register_on_exit(LPM3_bits);               // Exit LPM3
            }
            break;
        default:
            break;
    }
}
