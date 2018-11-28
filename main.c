#include "msp.h"
#include "stdint.h"


/**
 * Jordyn Hatcher
 * Jacob Hatcher
 * Professor Zuidema
 * EGR 226-902
 * Lab 6 Part 1
 * Description: This code sets up the LCD and has
 * a cursor blinking in the top left corner.
 *
 * Cited from Zuidema's and Brakora's  lecture
 */
void LCD_init(void); // makes the cursor blink

void commandWrite(uint8_t command); //writing one byte of command by calling the pushByte() function with the command parameter
void dataWrite(uint8_t data); //writing one byte of data by calling the pushByte() function with the data parameter

void pushByte(uint8_t byte);    // First pushes the most significant 4 bits of the byte into the data pins by calling the pushNibble() function
//next, it pushes the least significant 4 bits onto the data pins by calling the pushNibble() function

void pushNibble(uint8_t nibble);    //pushes 1 nibble onto the data pins and pulses the enable pin


void PulseEnablePin (void);     //This function will sequence the enable (E) pin


void delay_micro(uint32_t microsec);    //SysTick timer generates delay in microseconds
void delay_milli(uint32_t milli);     //SysTick timer generates delay in milliseconds





void SysTick_Init(void); // function that initializes the SysTick Timer


void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;  // stop watchdog timer
    SysTick_Init(); //calls SysTick initialization function
    P4->SEL0 &= ~(BIT0 | BIT2 | BIT4 | BIT5 | BIT6 | BIT7);  //sets port 4 pins for GPIO
    P4->SEL1 &= ~(BIT0 | BIT2 | BIT4 | BIT5 | BIT6 | BIT7);  //sets port 4 pins for GPIO
    P4->DIR |= (BIT0 | BIT2 | BIT4 | BIT5 | BIT6 |BIT7); //makes these pins outputs (port pins P4.4 - P4.7 wired to DB4-DB7)

    LCD_init(); //calls this function

    while(1) //keeps going through the function
    {

    }
}



void  LCD_init() //this function makes the cursor blink
    {
    commandWrite(3); //resets
    delay_milli(100); //delays 100 milliseconds
    commandWrite(3); //resets
    delay_micro(200); //delays 200 microseconds
    commandWrite(3); // resets
    delay_milli(100); //delays 100 milliseconds

    commandWrite(2); //setting the 4-bit mode
    delay_micro(100); //delays 100 microseconds
    commandWrite(2); //setting the 4 bit mode
    delay_micro(100); //delays 100 microseconds

    commandWrite(8); //2-lines 5x7 format
    delay_micro(100); //delays 100 microseconds
    commandWrite(0x0F); //display ON, cursor ON and blinking
    delay_micro(100); //delays 100 microseconds
    commandWrite(1); //clears everything and sets the cursor to home address
    delay_micro(100); //delays 100 microseconds
    commandWrite(6); //increment cursor
    delay_milli(10); //delays 10 milliseconds
    }


    void commandWrite(uint8_t command) //writes one byte of command
    {
    P4OUT &= ~(BIT0); //sets RS = 0
    pushByte(command); //runs pushByte() function
    }

    void dataWrite(uint8_t data) //writes one byte of data from user input
    {
    P4OUT |= (BIT0); //sets RS = 1
    pushByte(data);    //runs pushByte() function
    }


    void pushByte(uint8_t byte)
        {
        uint8_t nibble; //4-bits

        nibble = (byte & 0xF0) >> 4; //pushes the most significant 4 bits of the byte onto data pins
        pushNibble(nibble); //runs this function

        nibble = byte & 0x0F;//pushes the least significant 4 bits of the byte onto data pins
        pushNibble(nibble); //runs this function
        delay_micro(100); //delay 100 us
        }

    void pushNibble(uint8_t nibble)
    {
    P4OUT &= ~(BIT4|BIT5|BIT6|BIT7);  //clear P4.4-P4.7
    //P4OUT |= (nibble & (BIT4|BIT5|BIT6|BIT7)) << 4; //shifts bits left 4
    P4OUT |= (nibble & 0x0F) << 4; //pins P4.4-P4.7 wired to DB4-DB7

    PulseEnablePin(); //calls pulse enable pin function
    }


    void PulseEnablePin(void)
    {
        P4OUT &= ~(BIT2); //makes E LOW
        delay_micro(10); //delay 10 microsec
        P4OUT |= (BIT2); //makes E HIGH
        delay_micro(10); //delay 10 microsec
        P4OUT &= ~(BIT2); //makes E LOW
        delay_micro(10); //delay 10 microsec
    }


void SysTick_Init( )
{
    SysTick->CTRL = 0;              //Clear the timer.
    SysTick->LOAD = 0x00FFFFFF;     //Set the reload value to its max value
    SysTick->VAL  = 0;              //Clear the current value of the timer
    SysTick->CTRL = 0x00000005;     //Reset the timer
}


void delay_micro(uint32_t microsec)
{

    SysTick->LOAD = ((microsec * 3) - 1);  //Set the reload value to the desired delay.
    SysTick->VAL  = 0;   //Clear the current value in the timer.
    while((SysTick->CTRL & 0x00010000) == 0); //Set the counter flag.
}
void delay_milli(uint32_t milli)
{

    SysTick->LOAD = ((milli * 3000) - 1);  //Set the reload value to the desired delay.
    SysTick->VAL  = 0;   //Clear the current value in the timer.
    while((SysTick->CTRL & 0x00010000) == 0); //Set the counter flag.
}

