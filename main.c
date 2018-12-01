#include "msp.h"
#include "stdint.h"
#include <stdio.h>
#include <string.h>


/**
 * Jordyn Hatcher
 * Jacob Arenz
 * Professor Zuidema
 * Final Project
 * Description:
 */

/* TIMERS */
/************************************************************************************************************************************************/
void SysTick_Init(void); // function that initializes the SysTick Timer
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
void tempconversion(void); //converts the temp from sensor
void printtemp(void); //prints the temp to the LCD
void reverse(char *str, int len);  //reverses the string
int intToStr(int x, char str[], int d); //changes the integer into a string
void ftoa(float n, char *res, int afterpoint); //changes a float to an array


float nADC, result, tempC, tempF; //nADC is the raw value from potentiometer, result is the converted voltage read
char temparray[17];
char temparray2[17];
char currenttemp[6] ="Temp: ";


/* REAL TIME CLOCK*/
/************************************************************************************************************************************************/
void configRTC(void);  //real time clock configuration
void printRTC(void);    // prints out the real time to the LCD

/* global struct variable called now */
struct
{
    uint8_t sec;
    uint8_t min;
    uint8_t hour;
} now;

uint8_t RTC_flag = 0, RTC_alarm;


/*BUTTON CONFIGURATIONS */
/************************************************************************************************************************************************/
void button_green_config(void);
void button_red_config(void);
void button_speed_config(void);

void button_white_init(void);
void button_blue_init(void);
void button_black_init(void); //temperature sensor configuration


/* LED INITIALIZATION */
/************************************************************************************************************************************************/
void LED_init(void);


/* ALARM */
/************************************************************************************************************************************************/
void alarm_time_statement(void);
void main_state(void);
void set_time(void); //white button
void set_alarm(void); // blue button
void alarm_on_func(void);
void alarm_off_func(void);
void alarm_snooze_func(void);


char alarm_on[16] = "Alarm: ON       ";
char alarm_off[16] = "Alarm: OFF      ";
char alarm_snooze[16] = "Alarm: SNOOZE   ";
char alarm_time[11] = "Alarm Time:";

//testers
char juicy[16] = "JUICY      BOOTY";
char gucci[16] = "GUCCI BOOTY";
enum clock_states{
SET_TIME,
SET_ALARM,
MAIN
 };

enum clock_states current_clock_state = MAIN;

enum alarm_states{
ALARM_ON,
ALARM_OFF,
ALARM_SNOOZE
 };
 enum alarm_states current_alarm_state = ALARM_OFF;

/* GLOABL VARIABLES */
/************************************************************************************************************************************************/
uint8_t x, value = 0;
volatile int pctr = 0;
//volatile int ctr = 0;




 /************************************************************************************************************************************************
 *************************************************************************************************************************************************
 ************************************************************************************************************************************************/




/* MAIN */
/************************************************************************************************************************************************/
void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;  // stop watchdog timer

    SysTick_Init(); //systick timer

    LCD_pins_init(); //LCD pin initialization
    LCD_init(); //LCD intialization

    ADC14_init(); //ADC conversion
    tempconversion(); //temperature conversion
    printtemp(); //prints the temperature

    button_white_init();
    button_blue_init();
    button_black_init();

    LED_init();

    alarm_time_statement();

/*INTERRUPTS*/
    __disable_irq();
    button_speed_config();
    NVIC_EnableIRQ(PORT1_IRQn);

    button_green_config();
    button_red_config();
    //button_black_config();
    //NVIC_EnableIRQ(PORT2_IRQn);

    configRTC();
    NVIC_EnableIRQ(RTC_C_IRQn);
    __enable_irq();


    while(1)
        {

        switch (current_clock_state)
        {
        case MAIN:
            main_state();
            break;

        case SET_TIME:
            set_time();
            break;

        case SET_ALARM:
            set_alarm();
            break;
        }


        switch (current_alarm_state)
        {
        case ALARM_ON:
            alarm_on_func();
            break;

        case ALARM_OFF:
            alarm_off_func();
            break;

        case ALARM_SNOOZE:
            alarm_snooze_func();
            break;
        }




        if(RTC_flag)
        {
            printRTC();
            RTC_flag = 0;
        }
        if(RTC_alarm)
        {
            printf("ALARM\n");
            RTC_alarm = 0;

        }
        }
}
/************************************************************************************************************************************************/




/************************************************************************************************************************************************
*************************************************************************************************************************************************
************************************************************************************************************************************************/




/************************************************************************************************************************************************/
    void PORT1_IRQHandler(void)
    {
       if(P1->IFG & BIT1) //if statement for the light button
       {

           commandWrite(0xC0);
           delay_micro(10);
           dataWrite('z');
           P1->IFG &= ~BIT1; //clears the flag before exiting interrupt
       }

       if(P1->IFG & BIT4) //if statement for the light button
       {

               commandWrite(0xC0);
               delay_micro(10);
               dataWrite(' ');
           P1->IFG &= ~BIT4; //clears the flag before exiting interrupt
       }

    }

/************************************************************************************************************************************************/



/************************************************************************************************************************************************/
void PORT2_IRQHandler(void)
{
//   if(P2->IFG & BIT3) //if statement for the light button
//   {
//       pctr++;
//
//       if(pctr % 2 == 1)
//       {
//           commandWrite(0xC0);
//           delay_micro(10);
//           for(x=0; x<10; x++)
//               {
//                 dataWrite(alarm_on[x]);
//                 //delay_milli(100);
//               }
//       }
//
//       if(pctr % 2 == 0)
//       {
//           commandWrite(0xC0);
//           delay_micro(10);
//               for(x=0; x<10; x++)
//               {
//                   dataWrite(alarm_off[x]);
//                   //delay_milli(100);
//               }
//       }
//       P2->IFG &= ~BIT3; //clears the flag before exiting interrupt
//   }
//
//   if(P2->IFG & BIT4)
//   {
//
//              commandWrite(0xC0);
//              delay_micro(10);
//              for(x=0; x<11; x++)
//                  {
//                    dataWrite(alarm_snooze[x]);
//                    //delay_milli(100);
//                  }
//          P2->IFG &= ~BIT4; //clears the flag before exiting interrupt
//      }

}
/************************************************************************************************************************************************/



/************************************************************************************************************************************************/
void RTC_C_IRQHandler(void)
{
    if(RTC_C->CTL0 & BIT1) //alarm happened
    {
        RTC_alarm = 1;
        RTC_C->CTL0 = 0xA500;
    }

    if(RTC_C->PS1CTL & BIT0) //checks to see if interrupt is enabled
    {
        now.sec  =  RTC_C->TIM0>>0 & 0x00FF; //Records seconds (from bottom 8 bits of TIM0)
        now.min  =  RTC_C->TIM0>>8 & 0x00FF; //Records minutes (from top 8 bits of TIM0)
        now.hour =  RTC_C->TIM1>>0 & 0x00FF; //Records hours (from bottom 8 bits of TIM0)

        RTC_flag = 1;
        RTC_C->PS1CTL &= ~BIT0; //resets interrupt flag
    }
}
/************************************************************************************************************************************************/



/* ALARM STATUS */
/************************************************************************************************************************************************/
void alarm_time_statement(void)
{
        commandWrite(0x90);
        for(x=0; x<11; x++)
               {
                   dataWrite(alarm_time[x]);
               }
        delay_micro(10);

}
/************************************************************************************************************************************************/



/* SET ALARM FUNCTION */
/************************************************************************************************************************************************/
void set_alarm(void) //blue button
{
    commandWrite(0xC0);
    for(x=0; x<16; x++)
           {
               dataWrite(gucci[x]);
           }
    delay_micro(10);

    if(!(P2->IN & BIT5)) current_clock_state = SET_TIME; //if white button pressed go to that state
    if(!(P2->IN & BIT7)) current_clock_state = MAIN; //if white button pressed go to that state
}
/************************************************************************************************************************************************/



/* SET TIME */
/************************************************************************************************************************************************/
void set_time(void) //white button
{
    commandWrite(0xC0);
    for(x=0; x<16; x++)
           {
               dataWrite(juicy[x]);
           }
    delay_micro(10);
}
/************************************************************************************************************************************************/



/* MAIN STATE OF CLOCK */
/************************************************************************************************************************************************/
void main_state(void) //black button if necessary
{

    if(!(P2->IN & BIT3)) //if statement for the light button
    {
        current_alarm_state = ALARM_ON;
    }

    if(!(P2->IN & BIT5)) //if statement for the light button
    {
        current_clock_state = SET_TIME;
    }

    if(!(P2->IN & BIT6)) //if statement for the light button
    {
        current_clock_state = SET_ALARM;
    }

}
/************************************************************************************************************************************************/



/************************************************************************************************************************************************/
void alarm_on_func(void)
{

    commandWrite(0xC0);
        for(x=0; x<16; x++)
        {
            dataWrite(alarm_on[x]);
        }
        delay_micro(10);

        if(!(P2->IN & BIT4))
            {
            current_alarm_state = ALARM_SNOOZE;
            }
        if(!(P2->IN & BIT3))
            {
            current_alarm_state = ALARM_OFF;
            }


}
/************************************************************************************************************************************************/



/************************************************************************************************************************************************/
void alarm_off_func(void)
{

    commandWrite(0xC0);
        for(x=0; x<16; x++)
        {
            dataWrite(alarm_off[x]);
        }
        delay_micro(10);

        if(!(P2->IN & BIT3))
            {
            current_alarm_state = ALARM_ON;
            }

}
/************************************************************************************************************************************************/



/************************************************************************************************************************************************/
void alarm_snooze_func(void)
{
    commandWrite(0xC0);
        for(x=0; x<16; x++)
        {
            dataWrite(alarm_snooze[x]);
        }
        delay_micro(10);

        if(!(P2->IN & BIT3))
            {
            current_alarm_state = ALARM_OFF;
            }


}


/* LED AND THEIR PWM CONFIGURATION */
/************************************************************************************************************************************************/
    void LED_init(void)
    {
      P5->SEL0 &= ~(BIT6);
      P5->SEL1 &= ~(BIT6);
      P5->DIR |= (BIT6);
      P5->OUT &= ~(BIT6);

//      TIMER_A2->CCR[0]= led_period; //PWM Period (# of cycles)
//      TIMER_A2->CCR[1]= 1; //initial duty cycle
//      TIMER_A2->CCTL[1]= TIMER_A_CCTLN_OUTMOD_7; //CCR1 reset/set mode 7
//
//      TIMER_A2->CTL |= TASSEL_2 | MC_1 | TACLR; //SMCLK, UP mode, Clear
    }
/************************************************************************************************************************************************/



/* ON BOARD BUTTON INTERRUPT CONFIGURATION */
/************************************************************************************************************************************************/
    void button_speed_config(void)
    {
       P1->SEL0 &= ~(BIT1 | BIT4); //sets for GPIO
       P1->SEL1 &= ~(BIT1 | BIT4); //sets for GPIO
       P1->DIR &= ~(BIT1 | BIT4); //set P1.1 as an Input
       P1->REN |= (BIT1 | BIT4); //enable pull-up resistor (P1.1 output high)
       P1->OUT |= (BIT1 | BIT4); //make P1.1 default to a '1'
       P1->IES |= (BIT1 | BIT4); //set P1.1's Interrupt to trigger when it goes from high to low
       P1->IE |= (BIT1 | BIT4); //set interrupt on for P1.1
       P1->IFG &= ~(BIT1 | BIT4); //clear flag before exiting the interrupt
    }
/************************************************************************************************************************************************/



/* GREEN BUTTON CONFIGURATION */
/************************************************************************************************************************************************/
    void button_green_config(void)
    {
        P2->SEL0 &= ~(BIT3); //sets for GPIO
        P2->SEL1 &= ~(BIT3); //sets for GPIO
        P2->DIR &= ~(BIT3); //set P2.3 as an Input
        P2->REN |= (BIT3); //enable pull-up resistor (P2.3 output high)
        P2->OUT |= (BIT3); //make P2.3 default to a '1'
        //P2->IES |= (BIT3); //set P2.3's Interrupt to trigger when it goes from high to low
        P2->IE &= ~(BIT3); //set interrupt on for P2.3
        //P2->IFG &= ~(BIT3); //clear flag before exiting the interrupt
    }
/************************************************************************************************************************************************/



/* RED BUTTON CONFIGURATION */
/************************************************************************************************************************************************/
    void button_red_config(void)
     {
         P2->SEL0 &= ~(BIT4); //sets for GPIO
         P2->SEL1 &= ~(BIT4); //sets for GPIO
         P2->DIR &= ~(BIT4); //set P2.4 as an Input
         P2->REN |= (BIT4); //enable pull-up resistor (P2.4 output high)
         P2->OUT |= (BIT4); //make P2.4 default to a '1'
        // P2->IES |= (BIT4); //set P2.4's Interrupt to trigger when it goes from high to low
         P2->IE &= ~(BIT4); //set interrupt on for P2.4
         //P2->IFG &= ~(BIT4); //clear flag before exiting the interrupt
      }
/************************************************************************************************************************************************/



/* WHITE BUTTON INITIALIZATION */
/************************************************************************************************************************************************/
void button_white_init(void)
{
    P2->SEL0 &= ~(BIT5); //sets for GPIO
    P2->SEL1 &= ~(BIT5); //sets for GPIO
    P2->DIR &= ~(BIT5); //set P2.5 as an Input
    P2->REN |= (BIT5); //enable pull-up resistor
    P2->OUT |= (BIT5); //make P2.5 equal to 1
}
/************************************************************************************************************************************************/



/* BLUE BUTTON INITIALIZATION */
/************************************************************************************************************************************************/
void button_blue_init(void)
{
    P2->SEL0 &= ~(BIT6); //sets for GPIO
    P2->SEL1 &= ~(BIT6); //sets for GPIO
    P2->DIR &= ~(BIT6); //set P2.6 as an Input
    P2->REN |= (BIT6); //enable pull-up resistor
    P2->OUT |= (BIT6); //make P2.6 equal to 1
}
/************************************************************************************************************************************************/



/* BLACK BUTTON INITIALIZATION */
/************************************************************************************************************************************************/
void button_black_init(void)
{
   P2->SEL0 &= ~(BIT7); //sets for GPIO
   P2->SEL1 &= ~(BIT7); //sets for GPIO
   P2->DIR &= ~(BIT7); //set P2.7 as an Input
   P2->REN |= (BIT7); //enable pull-up resistor
   P2->OUT |= (BIT7); //make P2.7 equal to 1
}
/************************************************************************************************************************************************/



/* RTC CONFIGURATION */
/************************************************************************************************************************************************/
void configRTC(void)
{
    RTC_C->CTL0     =   0xA500;     //Write Code, IE on RTC Ready
    RTC_C->CTL13    =   0x0000;

    RTC_C->TIM0     = 30<<8 | 00;
    RTC_C->TIM1     = 2<<8 | 12;
    RTC_C->PS1CTL   = 0b11010;

    RTC_C->AMINHR   = 12<<8 | 31 | BIT(15) | BIT(7);
    RTC_C->CTL0     = ((0xA500) | BIT5);

}
/************************************************************************************************************************************************/



/* PRINTS RTC TO LCD */
/************************************************************************************************************************************************/
void printRTC(void)
{
    char realtime[16]; //array for real time
    //char XM[5] = "PM  ";
    int i; //placeholder

    commandWrite(0x80); //setting cursor to the second row
    delay_milli(10); //10 millisecond delay
    sprintf(realtime, "   %02d:%02d:%02d \n",now.hour, now.min, now.sec); //takes the realtime and stores it in array realtime[]

       for(i = 0; i < 11; i++)
       {
           dataWrite(realtime[i]); //prints realtime[] to LCD screen
       }
}
/************************************************************************************************************************************************/



/* ADC INITIALIZATION */
/************************************************************************************************************************************************/
void ADC14_init(void)
{
   P5SEL0|=BIT4;//configure pin 5.4 for A0 input
   P5SEL1|=BIT4;
   ADC14->CTL0&=~0x00000002;//disable ADC14ENC during configuration
   ADC14->CTL0|=0x04400110;//S/H pulse mode, SMCLK, 16 sample clocks
   ADC14->CTL1=0x00000030;//14 bit resolution
   ADC14->CTL1|=0x00000000;//Selecting ADC14CSTARTADDx mem0 REGISTER
   ADC14->MCTL[0]=0x00000001;//ADC14INCHx=0 for mem[0]
   ADC14->CTL0|=0x00000002;//enable ADC14ENC, starts the ADC after configuration
}
/************************************************************************************************************************************************/



/* TEMPERATURE CONVERSION */
/************************************************************************************************************************************************/
void tempconversion(void)
{
    ADC14->CTL0|=1;                         //start conversion
    while(!ADC14->IFGR0);                   //wait for conversion to complete
    result = ADC14->MEM[0];                   //get the value from the ADC
    nADC = result*(3.3/16384);
    nADC=nADC*1000;
    tempC = (nADC-500)/10;
    tempF = ((tempC*9)/5)+32;
    ftoa(tempF, temparray, 1); //these were 2s
    //ftoa(tempC, temparray2, 1);
}
/************************************************************************************************************************************************/



/* PRINTS TEMP */
/************************************************************************************************************************************************/
void printtemp()
{
    commandWrite(0xD0);
    for(x=0; x<6; x++)
    {
        dataWrite(currenttemp[x]);
    }
    commandWrite(0xD5);
    delay_micro(10);
    for(x=0; x<4; x++) //x was set to 5
    {
        dataWrite(temparray[x]);
        delay_milli(100);
    }
    dataWrite(0xDF); //degree symbol
    dataWrite('F'); //degree symbol
}
/************************************************************************************************************************************************/



/* This function reverses a string 'str' of length 'len'*/
/************************************************************************************************************************************************/

void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}
/************************************************************************************************************************************************/



// Converts a given integer x to string str[]. d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
/************************************************************************************************************************************************/
int intToStr(int x, char str[], int d)
{
   int i = 0;
   while (x)
   {
       str[i++] = (x%10) + '0';
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



// Converts a floating point number to string.
/************************************************************************************************************************************************/
void ftoa(float n, char *res, int afterpoint)
{
   // Extract integer part
   int ipart = (int)n;

   // Extract floating part
   float fpart = n - (float)ipart;

   // convert integer part to string
   int i = intToStr(ipart, res, 0);

   // check for display option after point
   if (afterpoint != 0)
   {
       res[i] = '.';  // add dot

       // Get the value of fraction part upto given no.
       // of points after dot. The third parameter is needed
       // to handle cases like 233.007
       fpart = fpart * pow(10, afterpoint);

       intToStr((int)fpart, res + i + 1, afterpoint);
   }
}
/************************************************************************************************************************************************/



 /* LCD PIN INITIALIZATION */
/************************************************************************************************************************************************/
    void LCD_pins_init()
    {
        P4->SEL0 &= ~(BIT0 | BIT2 | BIT4 | BIT5 | BIT6 | BIT7);  //sets port 4 pins for GPIO
        P4->SEL1 &= ~(BIT0 | BIT2 | BIT4 | BIT5 | BIT6 | BIT7);  //sets port 4 pins for GPIO
        P4->DIR |= (BIT0 | BIT2 | BIT4 | BIT5 | BIT6 |BIT7); //makes these pins outputs (port pins P4.4 - P4.7 wired to DB4-DB7)
    }
/************************************************************************************************************************************************/


 /* LCD INITIALIZATION */
/************************************************************************************************************************************************/
    void  LCD_init()
    {
        commandWrite(3); //resets
        delay_milli(100); //delays 100 milliseconds
        commandWrite(3); //resets
        delay_micro(200); //delays 200 microseconds
        commandWrite(3); // resests
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


/* COMMAND WRITE */
/************************************************************************************************************************************************/
    void commandWrite(uint8_t command)
    {
    P4->OUT &= ~(BIT0); //sets RS = 0
    pushByte(command); //runs pushByte() function
    }
/************************************************************************************************************************************************/


 /* DATA WRITE */
 /************************************************************************************************************************************************/
    void dataWrite(uint8_t data)
    {
    P4->OUT |= (BIT0); //sets RS = 1
    pushByte(data);    //runs pushByte() function
    }
/************************************************************************************************************************************************/


/* PUSH BYTE */
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


/* COMMAND WRITE */
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


/************************************************************************************************************************************************/
void SysTick_Init( )
{
    SysTick->CTRL = 0;              //Clear the timer.
    SysTick->LOAD = 0x00FFFFFF;     //Set the reload value to its max value
    SysTick->VAL  = 0;              //Clear the current value of the timer
    SysTick->CTRL = 0x00000005;     //Reset the timer
}
/************************************************************************************************************************************************/


/************************************************************************************************************************************************/
void delay_micro(uint32_t microsec)
{

    SysTick->LOAD = ((microsec * 3) - 1);  //Set the reload value to the desired delay.
    SysTick->VAL  = 0;   //Clear the current value in the timer.
    while((SysTick->CTRL & 0x00010000) == 0); //Set the counter flag.
}
/************************************************************************************************************************************************/


/************************************************************************************************************************************************/
void delay_milli(uint32_t milli)
{

    SysTick->LOAD = ((milli * 3000) - 1);  //Set the reload value to the desired delay.
    SysTick->VAL  = 0;   //Clear the current value in the timer.
    while((SysTick->CTRL & 0x00010000) == 0); //Set the counter flag.
}
/************************************************************************************************************************************************/
