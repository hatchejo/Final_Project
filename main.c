


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

void button_speed_config(void);
void button_config(void);
//void button_black_config(void); //temperature sensor configuration


/* LED INITIALIZATION */
/************************************************************************************************************************************************/
void LED_init(void);


/* ALARM */
/************************************************************************************************************************************************/
void alarm_statement(void);
void alarm_time_statement(void);
void main_state(void);
void set_time(void); //white button
void set_alarm(void); // blue button
void alarm_on_func(void);
void alarm_off_func(void);
void alarm_snooze_func(void);

char AM[2] ="AM";
char PM[2] ="PM";

char alarm[6] = "Alarm:";
char alarm_on[6] = "ON    ";
char alarm_off[6] = "OFF   ";
char alarm_snooze[6] = "SNOOZE";
char alarm_time[9] = "Alarm at:";



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
volatile int ctrA = 0;
volatile int ctrB = 0;
volatile int ctrC = 0;
volatile int ctrD = 0;

volatile int hours = 0;
volatile int minutes = 0;
volatile int alarm_hours = 01;
volatile int alarm_minutes = 00;




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



   // button_black_config();

    LED_init();

    alarm_time_statement();
    alarm_statement();

/*INTERRUPTS*/
    __disable_irq();
    button_speed_config();
    NVIC_EnableIRQ(PORT1_IRQn);

    button_config();
    //button_black_config();
    NVIC_EnableIRQ(PORT2_IRQn);

    configRTC();
    NVIC_EnableIRQ(RTC_C_IRQn);
    __enable_irq();


    while(1)
        {

        if(alarm_hours > 12)
             alarm_hours = alarm_hours - 12;

         if(hours > 23)
             hours = hours - 24;

         if(alarm_minutes > 59)//check if alarm minutes is greater than 59
             alarm_minutes = alarm_minutes - 60;//reset to loop again

         if(minutes > 59)//check if minutes value is greater than 59
            minutes = minutes - 60;//reset to loop again


        switch (current_clock_state)
        {
        case MAIN:
            printRTC();
            //delay_milli(1000);

            if(ctrA == 1) //if statement for the light button
            {
                current_clock_state = SET_TIME;
            }

            if(!(P2->IN & BIT6)) //if statement for the light button
            {
                delay_milli(300);
                current_clock_state = SET_ALARM;
            }
            main_state();
            break;

        case SET_TIME:
            //set_time();
            printRTC();

            if(hours > 23)
            {
              hours = 0;
            }

            if(ctrA ==3)
            {
                current_clock_state = MAIN;
            }


        case SET_ALARM:
            if(!(P2->IN & BIT6)) //if statement for the light button
            {
                delay_milli(100);
                current_clock_state = MAIN;
            }
            set_alarm();
            break;
        }





        switch (current_alarm_state)
        {
        case ALARM_ON:
            if(!(P2->IN & BIT3))
            {
                delay_milli(300);
                current_alarm_state = ALARM_OFF;
            }

            if(!(P2->IN & BIT4))
            {
                delay_milli(300);
                current_alarm_state = ALARM_SNOOZE;
            }

            alarm_on_func();
            break;


        case ALARM_OFF:
            if(!(P2->IN & BIT3))
                {
                  delay_milli(300);
                  current_alarm_state = ALARM_ON;
                }

            alarm_off_func();
            break;

        case ALARM_SNOOZE:

            if(!(P2->IN & BIT3))
               {
                  delay_milli(300);
                  current_alarm_state = ALARM_OFF;
                }

            alarm_snooze_func();
            break;
        }




//        if(RTC_flag)
//        {
////            printRTC();
//            RTC_flag = 0;
//        }
//        if(RTC_alarm)
//        {
//            printf("ALARM\n");
//            RTC_alarm = 0;
//
//        }
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

        if(P2 -> IFG & BIT6)//conditional to see if set alarm button has been pressed
            {
                ctrB++;//increment set alarm global variable
                if(ctrB>3)//if button has been pressed more than 3 times
                    ctrB=0;//reset number of presses to zero
                P2 -> IFG &= ~BIT6;//clear interrupt flag
            }
            if(P2 -> IFG & BIT5)//conditional to see if set time button has been pressed
            {
                ctrA++;//increment set time global variable
                if(ctrA>3)//if button has been pressed more than 3 times
                    ctrA=0;//reset number of presses to zero
                P2 -> IFG &= ~BIT5;//clear interrupt flag
            }
            if(P2 -> IFG & BIT3)//conditional to check if on/off/up button has been pressed
            {
                if(ctrA==1)//if the set time button has been pressed once
                {
                    hours++;//increment hour count variable
                    RTC_C->TIM1++;//increment value stored in hours register
                    if((RTC_C -> TIM1 )>12)
                    {//if hours register is greater than 12
                        RTC_C -> TIM1=1;
                    }//reset to 1
                    if(hours>24)//if hour count variable is greater than 24
                        hours=hours-24;//subtract 24 to loop again
                }
                else if(ctrA==2)//conditional to check if set time button has been pressed twice
                {
                    minutes++;//increment minute count variable
                    if((RTC_C -> TIM0 & 0xFF00) > 0<<8)
                        RTC_C-> TIM0 = (((RTC_C->TIM0 & 0xFF00) >> 8)+1)<<8;//increment values stored in minutes register
                                //((RTC_C->TIM0 & 0xFF00)-1);//increment value stored in minutes register
                    if((RTC_C -> TIM0 & 0xFF00)==0<<8)
                        RTC_C -> TIM0 =((RTC_C -> TIM0<<8 & 0xFF00)+59);
        //            if((RTC_C -> TIM0)>59)//if minutes register is greater than 59
        //            {
        //                RTC_C->TIM0=0;//reset to 1
        //            }
                    if(minutes>59)//if minutes count variable is greater than 59
                        minutes=minutes-59;//subtract 58 to loop again
                }
                else if(ctrB==1)
                {
                    alarm_hours++;//increment alarm hour count variable
                    hours++;//increment hour count variable
        //            if(alarmHour>12);//check if alarm hour is greater than 12
        //                alarmHour=alarmHour-11;//loop back to 1 once it goes over
        //            if(hours>23)//check if hour value is greater than 24
        //                hours=hours-23;//subtract 24 to loop again
                }
                else if(ctrB==2)
                {
                    alarm_minutes++;//increment alarm minute count variable
                    minutes++;//increment minutes count variable
        //            if(alarmMin>59)//check if alarm minutes is greater than 59
        //                alarmMin=alarmMin-60;//reset to loop again
        //            if(minutes>59)//check if minutes value is greater than 59
        //                minutes=minutes-60;//reset to loop again
                }
                P2 -> IFG &= ~BIT3;//clear flag
            }
            if(P2 -> IFG & BIT4)//conditional to check if snooze/down button has been pressed
            {
                if(ctrA==1)//conditional to check if set time button has been pressed once
                {
                    //hours=23;//set hour count variable to 22
                    hours--;//decrement hour count variable
                    RTC_C->TIM1--;//decrement value stored in hours register
                    if((RTC_C -> TIM1 )<1)//if hours register is less than 1
                    {
                        RTC_C -> TIM1=12;//reset to 12
                    }
                    if(hours<0)//if hour count variable is less than 0
                        hours=23;//reset to loop again
                }
                else if(ctrA==2)//conditional to check if set time button has been pressed twice
                {
                    minutes--;
                    if((RTC_C -> TIM0 & 0xFF00) > 0<<8)
                        RTC_C-> TIM0 = ((RTC_C->TIM0 & 0xFF00)-1);//decrement value stored in minutes register
                }

                P2 -> IFG &= ~BIT4;//clear flag
            }



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



/* ALARM TIME STATEMENT */
/************************************************************************************************************************************************/
void alarm_time_statement()
{
    char alarmTime[6];//declare empty string to hold alarm time
    char flashHour[6];//declare empty string to flash hour time
    char flashMin[6];//declare empty string to flash min time
    char morning[]="AM";//declare and initialize string to display AM
    char evening[]="PM";//declare and initialize string to display PM
    sprintf(alarmTime,"%02d:%02d\n",alarm_hours,alarm_minutes);//convert alarm hour and minute global variables to string
    sprintf(flashHour,"  :%02d\n",alarm_minutes);//convert alarm minute to string to be flashed when hours update
    sprintf(flashMin, "%02d:  \n",alarm_hours);//convert alarm hour to string to be flashed when minutes update
    commandWrite(0x90);//command cursor to go to start of third line
    delay_milli(100);//delay 100 milliseconds
    for(x=0;x<5;x++)//for loop to print alarm time
    {
        dataWrite(alarmTime[x]);//print each number of time
        delay_milli(100);//delay 100 milliseconds between each letter
    }
    if(ctrB==1)//if set alarm button has been pressed once
    {
        commandWrite(0x90);//command cursor to go to start of third line
        delay_milli(10);//delay 10 milliseconds
        for(x=0;x<5;x++)
        {
            dataWrite(flashHour[x]);//print each member of array to screen
            delay_milli(75);//delay 10 milliseconds between each letter
        }
    }
    else if(ctrB==2)//if set alarm button has been twice
    {
        commandWrite(0x90);//command cursor to go to start of third line
        delay_milli(10);//delay 10 milliseconds
        for(x=0;x<5;x++)
        {
            dataWrite(flashMin[x]);//print each member of array to screen
            delay_milli(75);//delay 10 milliseconds between each letter
        }
    }
    if(hours<11)
        {
            commandWrite(0x95);//command cursor to go to space beyond time
            delay_milli(10);//delay 100 milliseconds
            for(x=0;x<2;x++)//for loop to print AM
            {
                dataWrite(morning[x]);//print each letter to LCD
                delay_milli(75);//delay 100 milliseconds between letters
            }
        }
        else if(hours>11 && hours<24)
        {
            commandWrite(0x95);//command cursor to go to space beyond time
            delay_milli(10);//delay 100 milliseconds
            for(x=0;x<2;x++)//for loop to print AM
            {
                dataWrite(evening[x]);//print each letter to LCD
                delay_milli(75);//delay 100 milliseconds between letters
            }
        }

}
/************************************************************************************************************************************************/



/************************************************************************************************************************************************/
void alarm_statement()
{
    commandWrite(0xC0);
    for(x=0; x<6; x++)
           {
               dataWrite(alarm[x]);
           }
    delay_micro(10);
}
/************************************************************************************************************************************************/



/* SET ALARM FUNCTION */
/************************************************************************************************************************************************/
void set_alarm(void) //blue button
{
//    commandWrite(0xC6);
//    for(x=0; x<6; x++)
//           {
//               dataWrite(gucci[x]);
//           }
//    delay_micro(10);
}
/************************************************************************************************************************************************/



/* SET TIME */
/************************************************************************************************************************************************/
//void set_time(void)
//{
//    char current_time[9];//current time array
//    char set_hour[9];//used to set hour
//    char set_min[9];//used to set minute
//    char AM[2]="AM";
//    char PM[2]="PM";
//
//
//    commandWrite(0x83);//command cursor to go to first line
//    delay_milli(100);//delay 100 milliseconds
//
//    sprintf(current_time, "%02d:%02d:%02d\n",now.hour, now.min, now.sec); //convert current time to string
//    sprintf(set_hour, "  :%02d:%02d\n",now.min,now.sec);//convert current time to flash hour string
//    sprintf(set_min, "%02d:  :%02d\n",now.hour,now.sec);//convert current time to flash min string
//
//    for(x=0;x<8;x++)//for loop to print time
//    {
//        dataWrite(current_time[x]);//print each digit of current time to LCD
//        delay_milli(100);//delay 100 milliseconds between digits
//    }
//
//        commandWrite(0x83);//command cursor to go to first line of LCD
//        delay_milli(100);//delay 100 milliseconds
//        for(x=0;x<8;x++)//print minutes and seconds of time to flash hour
//        {
//            dataWrite(set_hour[x]);//print each digit of current time to LCD
//            delay_milli(100);//delay 100 milliseconds between digits
//        }
//
//    if(ctrA == 2)
//    {
//        commandWrite(0x83);//command cursor to go to first line of LCD
//        delay_milli(100);//delay 100 milliseconds
//        for(x=0;x<8;x++)
//        {
//            dataWrite(set_min[x]);//print each digit of current time to LCD
//            delay_milli(100);//delay 100 milliseconds between digits
//        }
//    }
//
//
//
//    if(hours < 11)
//    {
//        commandWrite(0x8B);//command cursor to go to space beyond time
//        delay_milli(100);//delay 100 milliseconds
//        for(x=0;x<2;x++)//for loop to print AM
//        {
//            dataWrite(AM[x]);//print each letter to LCD
//            delay_milli(100);//delay 100 milliseconds between letters
//        }
//    }
//
//    else if(hours > 11 && hours < 24)
//    {
//        commandWrite(0x8B);//command cursor to go to space beyond time
//        delay_milli(100);//delay 100 milliseconds
//        for(x=0;x<2;x++)//for loop to print AM
//        {
//            dataWrite(PM[x]);//print each letter to LCD
//            delay_milli(100);//delay 100 milliseconds between letters
//        }
//    }
//
//    if(ctrA == 3)
//                {
//                    current_clock_state= MAIN;
//                }//update state to current time
//
//}
/************************************************************************************************************************************************/



/* MAIN STATE OF CLOCK */
/************************************************************************************************************************************************/
void main_state(void) //black button if necessary
{

    if(hours < 11)
    {
        commandWrite(0x8B);//command cursor to go to space beyond time
        delay_milli(100);//delay 100 milliseconds
        for(x=0;x<2;x++)//for loop to print AM
        {
            dataWrite(AM[x]);//print each letter to LCD
            delay_milli(100);//delay 100 milliseconds between letters
        }
    }

    else if(hours > 11 && hours < 24)
    {
        commandWrite(0x8B);//command cursor to go to space beyond time
        delay_milli(100);//delay 100 milliseconds
        for(x=0;x<2;x++)//for loop to print AM
        {
            dataWrite(PM[x]);//print each letter to LCD
            delay_milli(100);//delay 100 milliseconds between letters
        }
    }

}
/************************************************************************************************************************************************/



/************************************************************************************************************************************************/
void alarm_on_func(void)
{

    commandWrite(0xC6);
        for(x=0; x<6; x++)
        {
            dataWrite(alarm_on[x]);
        }
        delay_micro(10);

}
/************************************************************************************************************************************************/



/************************************************************************************************************************************************/
void alarm_off_func(void)
{

    commandWrite(0xC6);
        for(x=0; x<6; x++)
        {
            dataWrite(alarm_off[x]);
        }
        delay_micro(10);

}
/************************************************************************************************************************************************/



/************************************************************************************************************************************************/
void alarm_snooze_func(void)
{
    commandWrite(0xC6);
        for(x=0; x<6; x++)
        {
            dataWrite(alarm_snooze[x]);
        }
        delay_micro(10);


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
       P1->DIR |= (BIT1 | BIT4); //set P1.1 as an Input
       P1->REN |= (BIT1 | BIT4); //enable pull-up resistor (P1.1 output high)
       P1->OUT |= (BIT1 | BIT4); //make P1.1 default to a '1'
       P1->IES |= (BIT1 | BIT4); //set P1.1's Interrupt to trigger when it goes from high to low
       P1->IE |= (BIT1 | BIT4); //set interrupt on for P1.1
       P1->IFG &= ~(BIT1 | BIT4); //clear flag before exiting the interrupt
    }
/************************************************************************************************************************************************/



/* WHITE, RED, GREEN AND BLUE BUTTON INITIALIZATION */
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


/* BLACK BUTTON INITIALIZATION */
/************************************************************************************************************************************************/
//void button_black_config(void)
//{
//   P2->SEL0 &= ~(BIT7); //sets for GPIO
//   P2->SEL1 &= ~(BIT7); //sets for GPIO
//   P2->DIR &= ~(BIT7); //set P2.7 as an Input
//   P2->REN |= (BIT7); //enable pull-up resistor
//   P2->OUT |= (BIT7); //make P2.7 equal to 1
//}
/************************************************************************************************************************************************/



/* RTC CONFIGURATION */
/************************************************************************************************************************************************/
void configRTC(void)
{
    RTC_C->CTL0     =   0xA500;     //Write Code, IE on RTC Ready
    RTC_C->CTL13    =   0x0000;

    RTC_C->TIM0     = 30<<8 | 55;
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
//    char realtime[16]; //array for real time
//    //char XM[5] = "PM  ";
//    int i; //placeholder
//
//    commandWrite(0x80); //setting cursor to the second row
//    delay_milli(10); //10 millisecond delay
//    sprintf(realtime, "   %02d:%02d:%02d \n",now.hour, now.min, now.sec); //takes the realtime and stores it in array realtime[]
//
//       for(i = 0; i < 11; i++)
//       {
//           dataWrite(realtime[i]); //prints realtime[] to LCD screen
//       }

       char current_time[9];//current time array
       char set_hour[9];//used to set hour
       char set_min[9];//used to set minute
       char AM[2]="AM";
       char PM[2]="PM";


       commandWrite(0x83);//command cursor to go to first line
       delay_milli(100);//delay 100 milliseconds


       sprintf(set_hour, "  :%02d:%02d\n",now.min,now.sec);//convert current time to flash hour string
       sprintf(set_min, "%02d:  :%02d\n",now.hour,now.sec);//convert current time to flash min string


       sprintf(current_time, "%02d:%02d:%02d\n",now.hour, now.min, now.sec); //convert current time to string
       for(x=0;x<8;x++)//for loop to print time
       {
           dataWrite(current_time[x]);//print each digit of current time to LCD
           delay_milli(100);//delay 100 milliseconds between digits
       }

       if(ctrA == 1)//conditional to check if set time button has been pressed once
       {
           commandWrite(0x83);//command cursor to go to first line of LCD
           delay_milli(100);//delay 100 milliseconds
           for(x=0;x<8;x++)//print minutes and seconds of time to flash hour
           {
               dataWrite(set_hour[x]);//print each digit of current time to LCD
               delay_milli(100);//delay 100 milliseconds between digits
           }
       }
       else if(ctrA == 2)
       {
           commandWrite(0x83);//command cursor to go to first line of LCD
           delay_milli(100);//delay 100 milliseconds
           for(x=0;x<8;x++)
           {
               dataWrite(set_min[x]);//print each digit of current time to LCD
               delay_milli(100);//delay 100 milliseconds between digits
           }
       }

       if(hours < 11)
       {
           commandWrite(0x8B);//command cursor to go to space beyond time
           delay_milli(100);//delay 100 milliseconds
           for(x=0;x<2;x++)//for loop to print AM
           {
               dataWrite(AM[x]);//print each letter to LCD
               delay_milli(100);//delay 100 milliseconds between letters
           }
       }

       else if(hours > 11 && hours < 24)
       {
           commandWrite(0x8B);//command cursor to go to space beyond time
           delay_milli(100);//delay 100 milliseconds
           for(x=0;x<2;x++)//for loop to print AM
           {
               dataWrite(PM[x]);//print each letter to LCD
               delay_milli(100);//delay 100 milliseconds between letters
           }
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
