/*
 * GPIO_Driver.c
 *
 *  Created on: Jan 28, 2023
 *      Author: Christian Cipolletta
 */



#include "GPIO_Driver.h"
#include <msp430.h>


/**
 * gpioInit(char Port, char Pin, char Direction)
 * Used to configure a single GPIO pin as an input or output.
 *
 * char Port: takes values from 1-6
 * char Pin: takes values from 0-7
 * char Direction: 0 for Input, 1 for Output
 *
 * Example: Setting pin 1.0 to an output
 * gpioInit(1, 0, 1);
 */

void gpioInit(char Port, char Pin, char Direction){

   unsigned char PinNumber = 0x01 << Pin;       // Creates PinNumber which represents the
                                                // BIT that needs to be flipped based on the selected Pin

   if(Direction) {                              // If the Direction is Output (1)

       switch(Port) {                           // Case statement that sets the correct PxOUT to 0
                                                // and PxDIR to 1 based on which Port is choosen

       case 1 :
           P1OUT &= ~PinNumber;
           P1DIR |= PinNumber;
           break;
       case 2 :
           P2OUT &= ~PinNumber;
           P2DIR |= PinNumber;
           break;
       case 3 :
           P3OUT &= ~PinNumber;
           P3DIR |= PinNumber;
           break;
       case 4 :
           P4OUT &= ~PinNumber;
           P4DIR |= PinNumber;
           break;
       case 5 :
           P5OUT &= ~PinNumber;
           P5DIR |= PinNumber;
           break;
       case 6 :
           P6OUT &= ~PinNumber;
           P6DIR |= PinNumber;
           break;

       }
   }

   else {                                    // If Direction is Input (0)

       switch(Port) {                       // Case statement that sets the correct
                                            // PxREN to high, PxOUT to high, and PxDIR to low

       case 1 :
           P1REN |= PinNumber;
           P1DIR &= ~PinNumber;
           P1OUT |= PinNumber;
           break;
       case 2 :
           P2REN |= PinNumber;
           P2DIR &= ~PinNumber;
           P2OUT |= PinNumber;
           break;
       case 3 :
           P3REN |= PinNumber;
           P3DIR &= ~PinNumber;
           P3OUT |= PinNumber;
           break;
       case 4 :
           P4REN |= PinNumber;
           P4DIR &= ~PinNumber;
           P4OUT |= PinNumber;
           break;
       case 5 :
           P5REN |= PinNumber;
           P5DIR &= ~PinNumber;
           P5OUT |= PinNumber;
           break;
       case 6 :
           P6REN |= PinNumber;
           P6DIR &= ~PinNumber;
           P6OUT |= PinNumber;
           break;

       }
   }
}



/**
 * gpioWrite(char Port, char Pin, char Value)
 * Used to write a 1 or 0 to a Pin which has already been declared as an output.
 *
 * char Port: takes values from 1-6
 * char Pin: takes values from 0-7
 * char Value: 0 or 1
 *
 * Example: Setting pin 1.0 to a 1
 * gpioWrite(1, 0, 1);
 */

void gpioWrite(char Port, char Pin, char Value){

    char PinNumber = 0x01 << Pin;

    if(Value) {                         // If the Value is 1

        switch(Port) {                  // Case statement that sets PxOUT to high
                                        // depending on the Port

        case 1 :
            P1OUT |= PinNumber;
            break;
        case 2 :
            P2OUT |= PinNumber;
            break;
        case 3 :
            P3OUT |= PinNumber;
            break;
        case 4 :
            P4OUT |= PinNumber;
            break;
        case 5 :
            P5OUT |= PinNumber;
            break;
        case 6 :
            P6OUT |= PinNumber;
            break;

        }
    }

    else {                              // If the Value is 0

        switch(Port) {                  // Case statement that sets PxOUT to low

        case 1 :
            P1OUT &= ~PinNumber;
            break;
        case 2 :
            P2OUT &= ~PinNumber;
            break;
        case 3 :
            P3OUT &= ~PinNumber;
            break;
        case 4 :
            P4OUT &= ~PinNumber;
            break;
        case 5 :
            P5OUT &= ~PinNumber;
            break;
        case 6 :
            P6OUT &= ~PinNumber;
            break;

        }
    }
}


/**
 * gpioRead(char Port, char Pin)
 * Used to Read from a pin which has been declared as an input.
 * Returns a 1 or 0 based on pin value.
 *
 * char Port: takes values from 1-6
 * char Pin: takes values from 0-7
 *
 * Example: Reading Pin 1.1
 * unsigned char PinState = gpioRead(1, 1);
 */

char gpioRead(char Port, char Pin){

    char Value;                         // Creates a char that will be returned

    char PinNumber = 0x01 << Pin;

    switch(Port) {                      // Case statement that sets Value to whatever
                                        // PxIN is depending on the Port

    case 1 :
        Value = P1IN & PinNumber;
        break;
    case 2 :
        Value = P2IN & PinNumber;
        break;
    case 3 :
        Value = P3IN & PinNumber;
        break;
    case 4 :
        Value = P4IN & PinNumber;
        break;
    case 5 :
        Value = P5IN & PinNumber;
        break;
    case 6 :
        Value = P6IN & PinNumber;
        break;

    }

    return Value;

}

