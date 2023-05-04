/*
 * FR2355_UART.c
 *
 *  Created on: Dec 7, 2022
 *      Author: traffo17
 */

#include <msp430.h>
#include "FR2355_UART_Driver.h"


/**
 * UART Init for 9600 Baud
 */

void uart_Init_9600(){
    PM5CTL0 &= ~LOCKLPM5;                    // Disable the GPIO power-on default high-impedance mode
                                              // to activate 1previously configured port settings

     __bis_SR_register(SCG0);                 // disable FLL
     CSCTL3 |= SELREF__REFOCLK;               // Set REFO as FLL reference source
     CSCTL1 = DCOFTRIMEN_1 | DCOFTRIM0 | DCOFTRIM1 | DCORSEL_3;// DCOFTRIM=3, DCO Range = 8MHz
     CSCTL2 = FLLD_0 + 243;                  // DCODIV = 8MHz
     __delay_cycles(3);
     __bic_SR_register(SCG0);                // enable FLL
     Software_Trim();                        // Software Trim to get the best DCOFTRIM value

     CSCTL4 = SELMS__DCOCLKDIV | SELA__REFOCLK; // set default REFO(~32768Hz) as ACLK source, ACLK = 32768Hz
                                              // default DCODIV as MCLK and SMCLK source



     // Configure UART
     UCA1CTLW0 |= UCSWRST;
     UCA1CTLW0 |= UCSSEL__SMCLK;

     // Baud Rate calculation
     // 8000000/(16*9600) = 52.083
     // Fractional portion = 0.083
     // User's Guide Table 17-4: UCBRSx = 0x49
     // UCBRFx = int ( (52.083-52)*16) = 1
     UCA1BR0 = 52;                             // 8000000/16/9600
     UCA1BR1 = 0x00;
     UCA1MCTLW = 0x4900 | UCOS16 | UCBRF_1;

     UCA1CTLW0 &= ~UCSWRST;                    // Initialize eUSCI
     UCA1IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt

}

/**
 * UART Software Trim
 * Used to trim the oscillator within the USCI peripheral to match the exact clock.
 */

void Software_Trim()
{
    unsigned int oldDcoTap = 0xffff;
    unsigned int newDcoTap = 0xffff;
    unsigned int newDcoDelta = 0xffff;
    unsigned int bestDcoDelta = 0xffff;
    unsigned int csCtl0Copy = 0;
    unsigned int csCtl1Copy = 0;
    unsigned int csCtl0Read = 0;
    unsigned int csCtl1Read = 0;
    unsigned int dcoFreqTrim = 3;
    unsigned char endLoop = 0;

    do
    {
        CSCTL0 = 0x100;                         // DCO Tap = 256
        do
        {
            CSCTL7 &= ~DCOFFG;                  // Clear DCO fault flag
        }while (CSCTL7 & DCOFFG);               // Test DCO fault flag

        __delay_cycles((unsigned int)3000 * MCLK_FREQ_MHZ);// Wait FLL lock status (FLLUNLOCK) to be stable
                                                           // Suggest to wait 24 cycles of divided FLL reference clock
        while((CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)) && ((CSCTL7 & DCOFFG) == 0));

        csCtl0Read = CSCTL0;                   // Read CSCTL0
        csCtl1Read = CSCTL1;                   // Read CSCTL1

        oldDcoTap = newDcoTap;                 // Record DCOTAP value of last time
        newDcoTap = csCtl0Read & 0x01ff;       // Get DCOTAP value of this time
        dcoFreqTrim = (csCtl1Read & 0x0070)>>4;// Get DCOFTRIM value

        if(newDcoTap < 256)                    // DCOTAP < 256
        {
            newDcoDelta = 256 - newDcoTap;     // Delta value between DCPTAP and 256
            if((oldDcoTap != 0xffff) && (oldDcoTap >= 256)) // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim--;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
            }
        }
        else                                   // DCOTAP >= 256
        {
            newDcoDelta = newDcoTap - 256;     // Delta value between DCPTAP and 256
            if(oldDcoTap < 256)                // DCOTAP cross 256
                endLoop = 1;                   // Stop while loop
            else
            {
                dcoFreqTrim++;
                CSCTL1 = (csCtl1Read & (~DCOFTRIM)) | (dcoFreqTrim<<4);
            }
        }

        if(newDcoDelta < bestDcoDelta)         // Record DCOTAP closest to 256
        {
            csCtl0Copy = csCtl0Read;
            csCtl1Copy = csCtl1Read;
            bestDcoDelta = newDcoDelta;
        }

    } while(endLoop == 0);                      // Poll until endLoop == 1

    CSCTL0 = csCtl0Copy;                       // Reload locked DCOTAP
    CSCTL1 = csCtl1Copy;                       // Reload locked DCOFTRIM
    while(CSCTL7 & (FLLUNLOCK0 | FLLUNLOCK1)); // Poll until FLL is locked
}




/** uart_Print(char *InputString)
 * Sends a STRING over UART using polling. Note that the input is a pointer to a character array which is NULL terminated
 * Input: pointer to char array with NULL termination.
 *
 * Requires uart_Init_9600 to be run prior to using this function.
 */
void uart_Print(char *InputString){
    while(*InputString != 0) {
        while (!(UCA1IFG&UCTXIFG));
        UCA1TXBUF = *InputString++;
    }
    while (!(UCA1IFG&UCTXIFG));
    UCA1TXBUF = 0x0A;
    while (!(UCA1IFG&UCTXIFG));
    UCA1TXBUF = 0x0D;
}
