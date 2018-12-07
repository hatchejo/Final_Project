#include "msp.h"
#include "stdint.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/**
 * Jacob Arenz
 * Jordyn Hatcher
 * Professor Zuidema
 * Final Project
 * Description: This is the code for an alarm clock, that allows you to set the current time using button
 * interrupts for increasing or decreasing the hour or minutes. It also allows for the setting of a timer
 * using the same button used to set the time. Buttons are also used to set turn on and off the alarm as well as
 * snooze it. ALong with changing time the code also has functions that allow for you to read and display the temperature
 * in degrees Fahrenheit to the LCD screen. All these function are displayed on the four rows of the LCD screen.
 * Cited from Zuidema, Brakora, and Kandalaft
 */

/* TIMERS */
/************************************************************************************************************************************************/
void SysTick_Init(void);    //function that initializes the SysTick Timer
void delay_micro(uint32_t microsec);    //SysTick timer generates delay in microseconds
void delay_milli(uint32_t milli);     //SysTick timer generates delay in milliseconds


/* LCD */
/************************************************************************************************************************************************/
void LCD_pins_init(void);
void LCD_init(void); // makes its blink
void commandWrite(uint8_t command); //writing one byte of command by calling the pushByte() function with the command parameter
void dataWrite(uint8_t data); //writing one byte of data by calling the pushByte() function with the data parameter
void pushByte(uint8_t byte);    // First pushes the most significant 4 bits of the byte into the data pins by calling the pushNibble() function
//next, it pushes the least significant 4 bits onto the data pins by calling the pushNibble() function
void pushNibble(uint8_t nibble);    //pushes 1 nibble onto the data pins and pulses the enable pin
void PulseEnablePin (void);     //This function will sequence the enable (E) pin


/* TEMPERATURE SENSOR */
/************************************************************************************************************************************************/
void ADC14_init(void);  //ADC initialization
void tempconversion(void); //converts the temperature from sensor
void printtemp(void); //prints the temperature to the LCD
void reverse(char *str, int len);  //reverses the string
int intToStr(int x, char str[], int d); //changes the integer into a string
void ftoa(float n, char *res, int afterpoint); //changes a float to an array


float nADC, result, tempC, tempF; //nADC is the raw value from potentiometer, result is the converted voltage read
char temparray[17]; //array for Celsius
char temparray2[17]; //array for Fahrenheit
char currenttemp[6] ="Temp: "; //array to spell out this statement on the LCD


/* REAL TIME CLOCK*/
/************************************************************************************************************************************************/
void configRTC(void);  //real time clock configuration
void printRTC(void);    // prints out the real time to the LCD

/* global structure variable called now for setting the RTC*/
struct
{
    uint8_t sec; //variables used in this structure for hours, minutes and seconds
    uint8_t min;
    uint8_t hour;
} now;

uint8_t RTC_flag = 0, RTC_alarm; //variables used for flags


/*BUTTON CONFIGURATIONS */
/************************************************************************************************************************************************/

void button_speed_config(void); //on-board buttons configuration
void button_config(void); //off board switch buttons configuration


/* ALARM */
/************************************************************************************************************************************************/
void alarm_statement(void); //"Alarm:"
void printalarm(void); //prints the alarm time
void set_time(void); //blue button, set time function
void alarm_on_func(void); //alarm on function
void alarm_off_func(void); //alarm off function
void alarm_snooze_func(void);//alarm snooze function


char AM[2] ="AM"; //array for AM
char PM[2] ="PM"; //array for PM
//arrays for states of the alarm status
char alarm[6] = "Alarm:";
char alarm_on[6] = "ON    ";
char alarm_off[6] = "OFF   ";
char alarm_snooze[6] = "SNOOZE";
char alarm_time[9] = "Alarm at:";

int alarm_update = 0; //alarm flag
int alarm_enable =0;


/* states for the clock state*/
enum clock_states{
SET_TIME, //state for setting the time
SET_ALARM, //state for setting the alarm
MAIN //main state of the clock
 };

enum clock_states current_clock_state = MAIN; //sets the default state of the clock to be MAIN

/* states for the alarm state */
enum alarm_states{
ALARM_ON, //state for turning the alarm on
ALARM_OFF, //state for turning the alarm off
ALARM_SNOOZE //state for snoozing the alarm
 };

 enum alarm_states current_alarm_state = ALARM_OFF; //sets the default state of the alarm to be off



/* GLOABL VARIABLES */
/************************************************************************************************************************************************/
//these are all place-holders/counters for interrupts
 uint8_t x, value = 0;
volatile int ctrA = 0; //counter for set time button (blue)
volatile int ctrB = 0; //counter for set alarm button (black)
volatile int ctrC = 0; //counter for on/off/up button (green)
volatile int ctrD = 0; //counter for snooze/down button (red)

//variables used to increment and decrement when setting the time and alarm
uint8_t hours = 12;
uint8_t minutes = 30;
uint8_t seconds = 55;
uint8_t alarm_hours = 06;
uint8_t alarm_minutes = 00;



 /************************************************************************************************************************************************
 *************************************************************************************************************************************************
 ************************************************************************************************************************************************/

/*
 * main function where the initialization functions are called, interrupts are enabled and
 * the states are changed.
 */
/************************************************************************************************************************************************/
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;  // stop watchdog timer
    SysTick_Init(); //SysTick timer
    LCD_pins_init(); //LCD pin initialization
    LCD_init(); //LCD initialization
    ADC14_init(); //ADC conversion
    tempconversion(); //temperature conversion
    printtemp(); //prints the temperature
    printalarm(); //prints the alarm RTC
    alarm_statement(); //prints the statement "Alarm:"

//initialization of all interrupts
    __disable_irq(); //globally disables all interrupts
    button_speed_config(); //configures the MSP on-board buttons to control the speed
    NVIC_EnableIRQ(PORT1_IRQn); //PORT 1 interrupt handler

    button_config(); //configures all the buttons as interrupts
    NVIC_EnableIRQ(PORT2_IRQn); // PORT 2 interrupt handler for all buttons

    configRTC(); //RTC clock configuration
    NVIC_EnableIRQ(RTC_C_IRQn); //RTC interrupt handler for when the alarm goes off
    __enable_irq(); //globally enables all interrupt



    while(1)
        {
 /************************************* SWITCHES CURRENT CLOCK STATE ************************************ */
        switch (current_clock_state) //sets up switch case for current_clock_state
        {
        case MAIN:
            printRTC(); //runs this function

            if(ctrA == 1) //if set time button (blue) is pushed
            {
                current_clock_state = SET_TIME; //go to this state
            }

            if(ctrB == 1) //if set alarm button (black) is pushed
            {
                current_clock_state = SET_ALARM; //go to this state
            }
            break;

        case SET_TIME:
           printRTC(); //runs this function

            if(hours > 23) //if the increment goes above 23 hours
            {
              hours = 0; //set hours back to 0
            }

            if(ctrA == 3) //if set time button (blue) is pressed for the third time
            {
                current_clock_state = MAIN; //switch back to MAIN
            }
            break;


        case SET_ALARM:
            printalarm(); //runs this function

            if(alarm_hours > 23) //if the increment goes above 23 hours
            {
                alarm_hours = 0; //set alarm hour back to
            }


            if(ctrB == 3) //if set alarm button (black) is pressed for the third time
            {
                current_clock_state = MAIN; //switch to MAIN
            }
            break;
        }


/************************************* SWITCHES CURRENT ALARM STATE ************************************ */
        switch (current_alarm_state)//sets up switch case for current_alarm_state
        {
        case ALARM_ON:

            if(ctrC == 2)
            {
                current_alarm_state = ALARM_OFF; //switch alarm state to off
                ctrC = 0; //resets counter to 0
            }

            if(ctrD == 1) //if the button was pushed once
             {
                current_alarm_state = ALARM_SNOOZE; //switch the alarm state to on
                ctrD = 0; //resets counter to 0
             }
            alarm_enable = 1;
            alarm_on_func(); //runs this function
            break;


        case ALARM_OFF:

            if(ctrC == 1) //if the button was pushed once
            {
            current_alarm_state = ALARM_ON; //switch the alarm state to on
            }

            alarm_off_func(); //runs this function
            break;


        case ALARM_SNOOZE:

            alarm_snooze_func(); //runs this function
            break;

        }

     }
}
/************************************************************************************************************************************************/


/************************************************************************************************************************************************
*************************************************************************************************************************************************
************************************************************************************************************************************************/


/*
 * this function is used to print the alarm TIME to the
 * LCD screen on the third row in minutes and hours
 */
/************************************************************************************************************************************************/
void printalarm()
{
    //arrays to store the RTC values
    char alarm_time[6];
    char change_hour[6];
    char change_min[6];

    //arrays for AM or PM
    char AM_alarm[]="AM";
    char PM_alarm[]="PM";

    sprintf(alarm_time,"%02d:%02d\n",alarm_hours,alarm_minutes);  //converts to string

    sprintf(change_hour,"  :%02d\n",alarm_minutes); //converts to string

    sprintf(change_min, "%02d:  \n",alarm_hours);   //converts to string

    commandWrite(0x90); //third line of LCD
    delay_milli(10);//delay 10 ms

    for(x = 0; x < 5; x++)//prints alarm time
    {
        dataWrite(alarm_time[x]);//prints alarm time

        delay_milli(50);//delay 100 milliseconds between each letter
    }

    if(ctrB == 1)//if black button has been pressed
    {
        commandWrite(0x90); //third line of LCd
        delay_milli(10);//delay 10 ms

        for(x = 0; x < 5; x++)//loop to iterate through array change_hour
        {
           dataWrite(change_hour[x]);//prints this to LCD

           delay_milli(50);//delay 10 milliseconds between each letter
        }
    }

    if(ctrB == 2)//if black has been pressed again
    {
        commandWrite(0x90); //third line
        delay_milli(10);//delay 10 ms

        for(x = 0; x < 5; x++)//loop to iterate through array change_min
        {
            dataWrite(change_min[x]);   //print this to LCD
            delay_milli(50);//delay 10 milliseconds between each letter
        }
    }

    if(alarm_hours < 11)
        {
            commandWrite(0x95); //moves cursor to the fifth spot in line three
            delay_milli(10);    //delay 10 ms

            for(x = 0; x < 2; x++)//for loop to print AM
            {
                dataWrite(AM_alarm[x]);//print each letter to LCD

                delay_milli(50);//delay 100 milliseconds between letters
            }
        }

        if(alarm_hours > 11 && alarm_hours < 24)
        {
            commandWrite(0x95); //moves cursor to the fifth spot in line three
            delay_milli(10);//delay 10 ms

            for(x = 0; x < 2; x++)//for loop to print AM
            {
                dataWrite(PM_alarm[x]);//print each letter to LCD

                delay_milli(50);//delay 100 milliseconds between letters
            }
        }

}
/************************************************************************************************************************************************/


/*
 * this function is used to print the alarm status 'ALARM:' to the
 * LCD screen on the second row
 */
/************************************************************************************************************************************************/
void alarm_statement()
{
    commandWrite(0xC0);//writes to the second row of the LCD screen
    for(x=0; x<6; x++)//loop to iterate through array alarm
           {
               dataWrite(alarm[x]);//displays array for ALARM:
           }
    delay_micro(10);//10 millisecond delay
}
/************************************************************************************************************************************************/


/*
 * this function is used to print the alarm status 'ON' to the
 * LCD screen on the second row
 */
/************************************************************************************************************************************************/
void alarm_on_func(void)
{

    commandWrite(0xC6);//writes to the second row of the LCD screen
        for(x=0; x<6; x++) //loop to iterate through array alarm_on
        {
            dataWrite(alarm_on[x]); //displays array for ON
        }
        delay_milli(10); //10 millisecond delay

}
/************************************************************************************************************************************************/


/*
 * this function is used to print the alarm status 'OFF' to the
 * LCD screen on the second row
 */
/************************************************************************************************************************************************/
void alarm_off_func(void)
{

    commandWrite(0xC6); //writes to the second row of the LCD screen
        for(x=0; x<6; x++) //loop to iterate through array alarm_off
        {
            dataWrite(alarm_off[x]); //displays array for OFF
        }
        delay_milli(10); //10 millisecond delay

}
/************************************************************************************************************************************************/


/*
 * this function is used to print the alarm status 'SNOOZE' to the
 * LCD screen on the second row
 */
/************************************************************************************************************************************************/
void alarm_snooze_func(void)
{
    commandWrite(0xC6); //writes to the second row of the LCD screen
        for(x=0; x<6; x++) //loop to iterate through array alarm_snooze
        {
            dataWrite(alarm_snooze[x]); //display array for SNOOZE
        }
        delay_milli(10); //10 millisecond delay

}
/************************************************************************************************************************************************/

/*
 * This Function does all the configuration steps for the interrupts to
 * be used on the on-board buttons
 */
/************************************************************************************************************************************************/
    void button_speed_config(void)
    {
       P1->SEL0 &= ~(BIT1 | BIT4); //sets for GPIO
       P1->SEL1 &= ~(BIT1 | BIT4); //sets for GPIO
       P1->DIR |= (BIT1 | BIT4); //set P1.1 as an Input
       P1->REN |= (BIT1 | BIT4); //enable pull-up resistor (P1.1 output high)
       P1->OUT |= (BIT1 | BIT4); //make P1.1 default to a '1'
       P1->IES |= (BIT1 | BIT4); //set P1.1's Interrupt to trigger when it goes from high to low
       P1->IE |= (BIT1 | BIT4); //set interrupt on for P1.1
       P1->IFG &= ~(BIT1 | BIT4); //clear flag before exiting the interrupt
    }
/************************************************************************************************************************************************/


    /*
     * This Function does all the configuration steps for the interrupts to
     * be used on the off-board buttons
     */
/************************************************************************************************************************************************/
void button_config(void) //SET Time buttons
{
    P2->SEL0 &= ~(BIT3|BIT4|BIT5|BIT6); //sets for GPIO
    P2->SEL1 &= ~(BIT3|BIT4|BIT5|BIT6); //sets for GPIO
    P2->DIR &= ~(BIT3|BIT4|BIT5|BIT6); //set P2.5 as an Input
    P2->REN |= (BIT3|BIT4|BIT5|BIT6); //enable pull-up resistor
    P2->OUT |= (BIT3|BIT4|BIT5|BIT6); //make P2.5 equal to 1
    P2->IES |= (BIT3|BIT4|BIT5|BIT6); //set P2.5's Interrupt to trigger when it goes from high to low //right now dsabled
    P2->IE |= (BIT3|BIT4|BIT5|BIT6); //set interrupt on for P2.5 rn its disabled
    P2->IFG &= ~(BIT3|BIT4|BIT5|BIT6); //clear flag before exiting the interrupt
}
/************************************************************************************************************************************************/


/*
 * This function does the RTC configuration steps
 */
/************************************************************************************************************************************************/
void configRTC(void)
{
    RTC_C->CTL0     =   0xA500;     //Write Code, IE on RTC Ready
    RTC_C->CTL13    =   0x0000;

    RTC_C->TIM0     = 30 <<8 | 55; //sets minutes to 30 and seconds to 55
    RTC_C->TIM1     = 12; //sets hours to 12
    RTC_C->PS1CTL   = 0b11010;

    RTC_C->AMINHR   = (alarm_hours)<<8 | (alarm_minutes) | BIT(15) | BIT(7);
    RTC_C->CTL0     = ((0xA500) | BIT5);

    NVIC_EnableIRQ(RTC_C_IRQn);
}
/************************************************************************************************************************************************/



/*
 * This function prints the RTC to the LCD screen
 */
/************************************************************************************************************************************************/
void printRTC(void)
{

       char current_time[9];//current time array
       char set_hour[9];//used to set hour
       char set_min[9];//used to set minute


       sprintf(current_time, "%02d:%02d:%02d\n",now.hour, now.min, now.sec); //convert current time to string

       sprintf(set_hour, "  :%02d:%02d\n",now.min,now.sec);//convert current time to flash hour string

       sprintf(set_min, "%02d:  :%02d\n",now.hour,now.sec);//convert current time to flash min string

       commandWrite(0x83);//command cursor to go to first line
       delay_milli(10);//delay 100 milliseconds

       for(x = 0; x < 8; x++)//for loop to print time
       {
           dataWrite(current_time[x]);//print each digit of current time to LCD
           delay_milli(50);//delay 100 milliseconds between digits
       }


       if(ctrA == 1)//conditional to check if set time button has been pressed once
       {
           commandWrite(0x83);//command cursor to go to first line of LCD
           delay_milli(10);//delay 10 milliseconds

           for(x = 0; x < 8; x++)//print minutes and seconds of time to flash hour
           {
               dataWrite(set_hour[x]);//print each digit of current time to LCD
               delay_milli(50);//delay 100 milliseconds between digits
           }
       }
       else if(ctrA == 2)
       {
           commandWrite(0x83);//command cursor to go to first line of LCD
           delay_milli(10);//delay 10 milliseconds

           for(x = 0; x < 8; x++)
           {
               dataWrite(set_min[x]);//print each digit of current time to LCD
               delay_milli(10);//delay 100 milliseconds between digits
           }
       }

       if(hours < 11)
       {
           commandWrite(0x8B);//command cursor to go to space beyond time
           delay_milli(10);//delay 100 milliseconds
           for(x = 0; x < 2; x++)//for loop to print AM
           {
               dataWrite(AM[x]);//print each letter to LCD
               delay_milli(50);//delay 100 milliseconds between letters
           }
       }

       else if(hours > 11 && hours < 24)
       {
           commandWrite(0x8B);//command cursor to go to space beyond time
           delay_milli(10);//delay 100 milliseconds
           for(x = 0; x < 2; x++)//for loop to print AM
           {
               dataWrite(PM[x]);//print each letter to LCD
               delay_milli(50);//delay 100 milliseconds between letters
           }
       }
}
/************************************************************************************************************************************************/


/*
 * This function handles all the initialization steps to
 * read values from the temperature sensor
 */
/************************************************************************************************************************************************/
void ADC14_init(void)
{
   P5SEL0|=BIT4;//configure  P5.4 for A0 input
   P5SEL1|=BIT4; //configures P5.4 for A0 input
   ADC14->CTL0&=~0x00000002;// this disables ADC14ENC during configuration
   ADC14->CTL0|=0x04400110;// sets ADC to S/H pulse mode, SMCLK, 16 sample clocks
   ADC14->CTL1=0x00000030;//sets to 14 bit resolution
   ADC14->CTL1|=0x00000000;//Selecting ADC14CSTARTADDx mem0 register
   ADC14->MCTL[0]=0x00000001;//ADC14INCHx=0 for first spot in array
   ADC14->CTL0|=0x00000002;//enables ADC14ENC along with starting the ADC after configuration
}
/************************************************************************************************************************************************/


/*
 * This function handles all of the conversion steps to
 * get from a voltage to a temperature value in degrees Fahrenheit
 */
/************************************************************************************************************************************************/
void tempconversion(void)
{
    ADC14->CTL0|=1; //starts the conversion
    while(!ADC14->IFGR0); //waits for the conversion to be complete
    result = ADC14->MEM[0];  //gets the value from the ADC
    nADC = result*(3.3/16384); //voltage conversion equations
    nADC = nADC * 1000;
    tempC = (nADC-500)/10;  //converts voltage reading to degrees Celsius
    tempF = ((tempC*9)/5)+32; //conversion form Celsius to Fahrenheit
    ftoa(tempF, temparray, 1); //stores value to floating point
}
/************************************************************************************************************************************************/


/*
 * this function is used to print the values found using the temperature sensor
 * and ADC conversion the the fourth row of the LCD screen
 */
/************************************************************************************************************************************************/
void printtemp()
{
    commandWrite(0xD0); //writes to the fourth row, first spot
    for(x=0; x<6; x++) //loop to read through array
    {
        dataWrite(currenttemp[x]); //displays array to the LCD screen
    }
    commandWrite(0xD5); //writes to the fourth row, spot five
    delay_micro(10); //10 microsecond delay
    for(x=0; x<4; x++) //loop to read through the array
    {
        dataWrite(temparray[x]); //displays the temperature to the LCD screen in degrees Fahrenheit
        delay_milli(10); //10 millisecond delay
    }
    dataWrite(0xDF); //displays degree symbol to LCD screen
    dataWrite('F'); //displays degree symbol
}
/************************************************************************************************************************************************/


/*
 * this function reverses a string of a certain length
 */
/************************************************************************************************************************************************/

void reverse(char *str, int len)
{
    int i=0, j=len-1, temp; //initializing local variables
    while (i<j) //loop to run as long as i is less than j
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; //increments i
        j--; //decrements j
    }
}
/************************************************************************************************************************************************/


/*
 * this function is used to convert integer numbers into
 * strings to be able to print the values to the LCD screen
 */
/************************************************************************************************************************************************/
int intToStr(int x, char str[], int d) //integer x into string str[]|d is the number of digits needed for output
{
   int i = 0; //local variable
   while (x)
   {
       str[i++] = (x%10) + '0';  //stores value of x into this string
       x = x/10;
   }

   // If number of digits required is more, then
   // add 0s at the beginning
   while (i < d)
       str[i++] = '0';

   reverse(str, i);
   str[i] = '\0';
   return i;
}
/************************************************************************************************************************************************/


/*
 * this function is used to convert floating point numbers into
 * strings to be able to print the values to the LCD screen
 */
/************************************************************************************************************************************************/
void ftoa(float n, char *res, int afterpoint)
{

   int ipart = (int)n;  // Extract integer part
   float fpart = n - (float)ipart;// Extract floating part
   int i = intToStr(ipart, res, 0); // convert integer part to string

   if (afterpoint != 0)  // check for display option after point
   {
       res[i] = '.';  // add dot

       fpart = fpart * pow(10, afterpoint);

       intToStr((int)fpart, res + i + 1, afterpoint);
   }
}
/************************************************************************************************************************************************/



/*
 * This function initializes all the pins used for to set up the LCD screen
 */
/************************************************************************************************************************************************/
    void LCD_pins_init()
    {
        P4->SEL0 &= ~(BIT0 | BIT2 | BIT4 | BIT5 | BIT6 | BIT7);  //sets port 4 pins for GPIO
        P4->SEL1 &= ~(BIT0 | BIT2 | BIT4 | BIT5 | BIT6 | BIT7);  //sets port 4 pins for GPIO
        P4->DIR |= (BIT0 | BIT2 | BIT4 | BIT5 | BIT6 |BIT7); //makes these pins outputs (port pins P4.4 - P4.7 wired to DB4-DB7)
    }
/************************************************************************************************************************************************/


    /*
      * This function LCD_Init() does the Initialization steps for the
      * 16 x 4 LCD screen
      */
/************************************************************************************************************************************************/
    void  LCD_init()
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
/************************************************************************************************************************************************/


    /*
     * this function commandWrite() writes 1 byte to command
     * by calling pushByte() with data parameter
     */
/************************************************************************************************************************************************/
    void commandWrite(uint8_t command)
    {
    P4->OUT &= ~(BIT0); //sets RS = 0
    pushByte(command); //runs pushByte() function
    }
/************************************************************************************************************************************************/


    /*
     * This function dataWrite() writes 1 byte of DATA
     * by calling pushByte() with data parameter
     */
 /************************************************************************************************************************************************/
    void dataWrite(uint8_t data)
    {
    P4->OUT |= (BIT0); //sets RS = 1
    pushByte(data);    //runs pushByte() function
    }
/************************************************************************************************************************************************/


    /*
     * this function pushByte() pushes most significant 4 bits onto
     * the data pins by calling pushNibble()
     *  then pushes least significant 4 bits onto the data pins  by calling pushNibble()
     */
/************************************************************************************************************************************************/
    void pushByte(uint8_t byte)
        {
        uint8_t nibble; //4-bits

        nibble = (byte & 0xF0) >> 4; //pushes the most significant 4 bits of the byte onto data pins
        pushNibble(nibble); //runs this function

        nibble = byte & 0x0F;//pushes the least significant 4 bits of the byte onto data pins
        pushNibble(nibble); //runs this function
        delay_micro(100); //delay 100 us
        }
/************************************************************************************************************************************************/


    /*
     * this function pushNibble() pushes 1 nibble(4 bits) onto the data pins and pulses E pin
     */
/************************************************************************************************************************************************/
    void pushNibble(uint8_t nibble)
    {
    P4->OUT &= ~(BIT4|BIT5|BIT6|BIT7);  //clear P4.4-P4.7
    //P4->OUT |= (BIT4|BIT5|BIT6|BIT7);
    //P4->OUT |= (nibble & (BIT4|BIT5|BIT6|BIT7)) << 4; //shifts bits left 4
    P4->OUT |= (nibble & 0x0F) << 4; //shifts bits left

    PulseEnablePin(); //calls pulse enable pin function
    }
/************************************************************************************************************************************************/


    /*
     * This function will sequence the Enable (E) pin between a 1 and a 0
     */
/************************************************************************************************************************************************/
    void PulseEnablePin(void)
    {
        P4->OUT &= ~(BIT2); //makes E LOW
        delay_micro(100); //delay 10 microsec
        P4->OUT |= (BIT2); //makes E HIGH
        delay_micro(100); //delay 10 microsec
        P4->OUT &= ~(BIT2); //makes E LOW
        delay_micro(100); //delay 10 microsec
    }
/************************************************************************************************************************************************/


    /*
     * SysTick_Init( ) initializes the SysTick Timer for use throughout
     * the program.
     */
/************************************************************************************************************************************************/
void SysTick_Init( )
{
    SysTick->CTRL = 0;              //Clear the timer.
    SysTick->LOAD = 0x00FFFFFF;     //Set the reload value to its max value
    SysTick->VAL  = 0;              //Clear the current value of the timer
    SysTick->CTRL = 0x00000005;     //Reset the timer
}
/************************************************************************************************************************************************/


/*
 * delay_micro( ) controls the delay of the program. Whatever delay
 * is sent to it via the mircosec parameter and will delay in microseconds.
 */
/************************************************************************************************************************************************/
void delay_micro(uint32_t microsec)
{

    SysTick->LOAD = ((microsec * 3) - 1);  //Set the reload value to the desired delay.
    SysTick->VAL  = 0;   //Clear the current value in the timer.
    while((SysTick->CTRL & 0x00010000) == 0); //Set the counter flag.
}
/************************************************************************************************************************************************/


/*
 * delay_milli( ) controls the delay of the program. Whatever delay
 * is sent to it via the ms parameter and will delay in milliseconds.
 */
/************************************************************************************************************************************************/
void delay_milli(uint32_t milli)
{

    SysTick->LOAD = ((milli * 3000) - 1);  //Set the reload value to the desired delay.
    SysTick->VAL  = 0;   //Clear the current value in the timer.
    while((SysTick->CTRL & 0x00010000) == 0); //Set the counter flag.
}
/************************************************************************************************************************************************/


/*
 * this is the interrupt handler for port 2, it contains the comands for all
 * off board buttons used for the project. For setting time, setting alarm and
 * switching between alarm states(ON, OFF and SNOOZE)
 */
/************************************************************************************************************************************************/
void PORT2_IRQHandler(void)
{
        /*******************************/
        if(P2 -> IFG & BIT6)//conditional to see if set alarm button (black) has been pressed
            {
            delay_milli(10); //10 millisecond delay
            if(P2 -> IFG & BIT6)//conditional to see if set alarm button (black) has been pressed
                       {

            delay_micro(100);//100 microsecond delay

            ctrB++;//increments this global counter variable

                if(ctrB > 3)//if button has been pressed more than 3 times
                {
                    ctrB = 0;//reset number of presses to zero
                }
                P2 -> IFG &= ~BIT6;//clears interrupt flag
                       }
            }
        /*******************************/

            if(P2 -> IFG & BIT5)//conditional to see if set time button (blue) has been pressed
            {
                delay_milli(10);//10 millisecond delay

                if(P2 -> IFG & BIT5)//conditional to see if set time button (blue) has been pressed
                {

                ctrA++;//increments this global counter variable

                if(ctrA > 3)//if button has been pressed more than 3 times
                {
                    ctrA = 0;//reset number of presses to zero
                }

                P2 -> IFG &= ~BIT5;//clears interrupt flag
                }
            }
         /*******************************/

            if(P2 -> IFG & BIT3)//conditional to check if on/off/up button (green) has been pressed
            {
               delay_milli(10); //10 millisecond delay
               if(P2 -> IFG & BIT3)//conditional to check if on/off/up button (green) has been pressed
               {
                if(current_clock_state == MAIN) //checks current clock state
                {
                    ctrC++;//increments this global counter variable

                    if(ctrC > 3)//if button has been pressed more than 3 times

                        ctrC = 0;//reset number of presses to zero

                    P2 -> IFG &= ~BIT3;//clears interrupt flag
                }
               }



                if(ctrA == 1)//if the set time button (blue) has been pressed once
                {
                    hours++;//increment the hours

                    RTC_C -> TIM1++;//increments the value stored in hours register

                    if((RTC_C -> TIM1 ) > 12)//if hours register is greater than 12
                    {
                        RTC_C -> TIM1 = 1;//reset to 1
                    }

                    if(hours > 24)//if hours is greater than 24

                        hours = hours - 24;//subtract 24 to start again

                    P2 -> IFG &= ~BIT3;//clears interrupt flag
                }



                if(ctrA == 2)//checks to see if set time button (blue) has been pressed again
                {
                    minutes++;//increment the minutes

                    if((RTC_C -> TIM0 & 0xFF00) > 0 << 8)
                    {
                        RTC_C-> TIM0 = (((RTC_C->TIM0 & 0xFF00) >> 8) +1) << 8;//increment values stored in minutes register
                    }

                    if((RTC_C -> TIM0 & 0xFF00)== 0 <<8)
                    {
                        RTC_C -> TIM0 =((RTC_C -> TIM0<<8 & 0xFF00) + 59);
                    }

                    if(minutes > 59)//if minutes is greater than 59

                        minutes = minutes - 59;//subtract 59 to start again
                    P2 -> IFG &= ~BIT3;//clears interrupt flag
                }


               if(ctrB == 1) //checks to see if the set alarm button (black) was pressed
                {
                    alarm_hours++;//increment alarm hours

                    RTC_C -> TIM1++;    //increments alarm hours in register

                    if((RTC_C -> TIM1 ) > 12)   //if alarm hours register is greater than 12
                    {
                        RTC_C -> TIM1 = 1;  //reset value to 1
                    }

                    if(alarm_hours > 24)//if alarm hours is greater than 24
                    {
                        alarm_hours = alarm_hours - 24;//subtract 24 to start over
                    }
                    P2 -> IFG &= ~BIT3;//clears interrupt flag
                }



                if(ctrB == 2) //checks to see if the set alarm button(black) was pressed again
                {
                    alarm_minutes++; //increments the alarm minutes

                    if((RTC_C -> TIM0 & 0xFF00) > 0<<8)
                    {
                        RTC_C-> TIM0 = (((RTC_C->TIM0 & 0xFF00) >> 8)+1)<<8;//increment values stored in minutes register
                    }

                    if((RTC_C -> TIM0 & 0xFF00)==0<<8)
                    {
                        RTC_C -> TIM0 =((RTC_C -> TIM0<<8 & 0xFF00)+59);
                    }

                    if(alarm_minutes>59)//if minutes count variable is greater than 59
                    {
                        alarm_minutes=alarm_minutes-59;//subtract 58 to loop again
                    }
                    P2 -> IFG &= ~BIT3;//clears interrupt flag
                }

               }



            /*******************************/

            if(P2 -> IFG & BIT4)    //checks to see if snooze/down button has been pressed
            {

                delay_milli(10); //delay 100 microseconds

                if(P2 -> IFG & BIT4)    //checks to see if snooze/down button has been pressed

                {


                if(ctrA == 1)//conditional to check if set time button (blue) has been pressed
                {
                    hours--; //decrement hours

                    RTC_C -> TIM1--;//decrement hours in the register

                    if((RTC_C -> TIM1 ) < 1)//if hours in the register is less than 1
                    {
                        RTC_C -> TIM1 = 12;//resets to 12
                    }

                    if(hours << 0)//if hour count variable is less than 0

                        hours = 23;//reset

                    P2 -> IFG &= ~BIT4;//clears interrupt flag
                }


                if(ctrA == 2)//checks to see if set time button (blue) has been pressed again
                {
                    minutes--; //decrements the minutes

                    if((RTC_C -> TIM0 & 0xFF00) > 0 << 8)
                    {
                        RTC_C-> TIM0 = ((RTC_C -> TIM0 & 0xFF00) - 1);//decrements the minutes in the register
                    }
                    P2 -> IFG &= ~BIT4;//clear flag


                    if(ctrB == 1)//conditional to check if set Alarm button (black) has been pressed
                                   {
                                       alarm_hours--; //decrement hours

                                       RTC_C -> TIM1--;//decrement hours in the register

                                       if((RTC_C -> TIM1 ) < 1)//if hours in the register is less than 1
                                       {
                                           RTC_C -> TIM1 = 12;//resets to 12
                                       }

                                       if(alarm_hours << 0)//if hour count variable is less than 0

                                           alarm_hours = 23;//reset

                                       P2 -> IFG &= ~BIT4;//clears interrupt flag
                                   }


                                   if(ctrB == 2)//checks to see if set time button (blue) has been pressed again
                                   {
                                       alarm_minutes--; //decrements the minutes

                                       if((RTC_C -> TIM0 & 0xFF00) > 0 << 8)
                                       {
                                           RTC_C-> TIM0 = ((RTC_C -> TIM0 & 0xFF00) - 1);//decrements the minutes in the register
                                       }
                                       P2 -> IFG &= ~BIT4;//clear flag
                                   }
                    }
                }
            }


}
/************************************************************************************************************************************************/



/*
 * This is the RTC interrupt handler that is used set the
 * time for the alarm
 */
/************************************************************************************************************************************************/
void RTC_C_IRQHandler(void)
{


    if(RTC_C->PS1CTL & BIT0) //checks to see if interrupt is enabled
    {
        now.sec  =  RTC_C->TIM0>>0 & 0x00FF; //Records seconds (from bottom 8 bits of TIM0)
        now.min  =  RTC_C->TIM0>>8 & 0x00FF; //Records minutes (from top 8 bits of TIM0)
        now.hour =  RTC_C->TIM1>>0 & 0x00FF; //Records hours (from bottom 8 bits of TIM0)

        RTC_flag = 1;
        RTC_C->PS1CTL &= ~BIT0; //resets interrupt flag
    }

    if(RTC_C->CTL0 & BIT1)                              // Alarm happened!
    {
        alarm_update = 1;                               // Send flag to main program to notify a time update occurred.
        RTC_C->CTL0 = (0xA500) | BIT5;                  // Resetting the alarm flag.  Need to also write the secret code
                                                        // and rewrite the entire register.
                                                        // TODO: It seems like there is a better way to preserve what was already
                                                        // there in case the setup of this register needs to change and this line
                                                        // is forgotten to be updated.
    }

}
/************************************************************************************************************************************************/
