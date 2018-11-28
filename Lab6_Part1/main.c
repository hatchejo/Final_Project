#include "msp.h"
#include "stdint.h"

/*
 * Jacob Arenz
 * Professor Zuidema
 * EGR 226-902
 * Lab 6 Part 1
 * Description: Code that displays the cursor on the screen in the top
 * right corner and blinks it
 *
 * code Referenced from Brakora's Lecture Slides
*/
void LCD_Init(void);
void SysTick_Init(void); // function that Initializes the SysTick Timer.
void delay_micro(uint32_t microsec); //Systick timer delay in micro-seconds
void delay_ms(uint32_t ms); //Systick timer delay in milli-seconds
void PulseEnablePin(void); //will sequence the Enable (E) pin
void pushNibble(uint8_t nibble); //this function pushes 1 nibble(4 bits) onto the data pins and pulses E pin
void pushByte(uint8_t byte); //pushes most significant 4 bits onto the data pins by calling pushNibble()
                            //then pushes least significant 4 bits onto the data pins  by calling pushNibble()
void commandWrite(uint8_t command); //writing 1 byte of command by calling pushByte() with data parameter
void dataWrite(uint8_t data); //writing 1 byte of DATA by calling pushByte() with data parameter


void main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

    SysTick_Init( );  //initialize the SysTick Timer.

    P4->SEL0 &= ~(BIT0 | BIT2 | BIT4 | BIT5 | BIT6 | BIT7);  //sets P4.1-P4.7 for GPIO.
    P4->SEL1 &= ~(BIT0 | BIT2 | BIT4 | BIT5 | BIT6 | BIT7);  //sets P4.1-P4.7  for GPIO
    P4->DIR  |=  (BIT0 | BIT2 | BIT4 | BIT5 | BIT6 | BIT7);  //makes P4.1-P4.7 inputs.

    LCD_Init(); //calling function that displays cursor on the screen and blinks it

    while(1)
    {


    }
}


/*
 * this function LCD_Init() does the Initialization steps for the
 * LCD
 */

void LCD_Init()
{
    commandWrite(3); //resets
    delay_ms(10); //delay 10 ms
    commandWrite(3); //resets
    delay_micro(200); //delay 200 microseconds
    commandWrite(3); //resets
    delay_ms(10); //delay 10 ms

    commandWrite(2); //setting the 4-bit mode
    delay_micro(100); //delaying 100 microseconds
    commandWrite(2); //setting the 4 bit mode
    delay_micro(100); //delaying 100 microseconds

    commandWrite(8); // 2-lines, 5x7 format
    delay_micro(100); //delay 100 microseconds
    commandWrite(0x0F); //Display ON, Cursor ON and blinking
    delay_micro(100); //delay 100 microseconds
    commandWrite(1); //Clear Display, Move cursor to HOME position
    delay_micro(100); //delay 100 microseconds
    commandWrite(6); //increment cursor
    delay_ms(10); //delay 10 ms
}


/*
 * This function will sequence the Enable (E) pin between a 1 and a 0
 */
void PulseEnablePin()
{
    P4OUT &= ~BIT2; //pulse starts at E = 0
    delay_micro(10); //delay 10 microseconds
    P4OUT |= BIT2; //pulse set E = 1
    delay_micro(10); //delay 10 microseconds
    P4OUT &= ~BIT2; //pulse set E = 0
    delay_micro(10); //delay 10 microseconds
}

/*
 * this function pushByte() pushes most significant 4 bits onto
 * the data pins by calling pushNibble()
 *  then pushes least significant 4 bits onto the data pins  by calling pushNibble()
 */
void pushByte(uint8_t byte)
{
    uint8_t nibble;

    nibble = (byte & 0xF0) >> 4; //pushes the most significant 4 bits of the byte onto data pins
    pushNibble(nibble); //calls the function pushNibble
    nibble = byte & 0x0F; //pushes the least significant 4 bits of the byte onto data pins
    pushNibble(nibble); //calls the function push Nibble
    delay_micro(100); //delay 100 microseconds
}

/*
 * this function pushNibble() pushes 1 nibble(4 bits) onto the data pins and pulses E pin
 */

void pushNibble (uint8_t nibble)
{
    P4OUT &= ~ 0xF0; //clear P4.4-P4.7
    P4OUT |= (nibble & 0x0F) << 4; //pins P4.4-P4.7 wired to DB4-DB7

    PulseEnablePin(); //calls function PulseEnablePin()
}

/*
 * this function commandWrite() writes 1 byte to command
 * by calling pushByte() with data parameter
 */
void commandWrite(uint8_t command)
{
    P4OUT &= ~BIT0; //sets RS = 0
    pushByte(command); //calls the function pushByte()
}

/*
 * This function dataWrite() writes 1 byte of DATA
 * by calling pushByte() with data parameter
 */
void dataWrite(uint8_t data)
{
    P4OUT |= BIT0; //sets RS = 1
    pushByte(data); //calls the function pushByte()
}

/*
 * SysTick_Init( ) initializes the SysTick Timer for use throughout
 * the program.
 */
    void SysTick_Init()
    {
        SysTick->CTRL = 0;              //Clear the timer.
        SysTick->LOAD = 0x00FFFFFF;     //Set the reload value to its max value
        SysTick->VAL  = 0;              //Clear the current value of the timer
        SysTick->CTRL = 0x00000005;     //Reset the timer.
    }

 /*
  * delay_ms( ) controls the delay of the program. Whatever delay
  * is sent to it via the ms parameter and will delay in milliseconds.
  */
void delay_ms( uint32_t ms )
{
    SysTick->LOAD = ( (ms * 3000) - 1 );  //Set the reload value to the desired delay.
    SysTick->VAL  = 0;   //Clear the current value in the timer.
    while((SysTick->CTRL & 0x00010000) == 0); //Set the counter flag.
}

/*
 * delay_micro( ) controls the delay of the program. Whatever delay
 * is sent to it via the mircosec parameter and will delay in microseconds.
 */
void delay_micro( uint32_t microsec )
{

    SysTick->LOAD = ( (microsec * 3) - 1 );  //Set the reload value to the desired delay.
    SysTick->VAL  = 0;   //Clear the current value in the timer.
    while(( SysTick->CTRL & 0x00010000) == 0); //Set the counter flag.
}
