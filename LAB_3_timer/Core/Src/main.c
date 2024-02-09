///////////////////////////   ECE433/533       Boise State University  ////////
// LAB3:  Counters/Timers     --
// Name:
//
// In this lab you will learn how to setup and use STM32L552 timers.
// There are 5 functions with different use cases of the timers that
// need to be implemented.
//
// General-purpose timer cookbook for STM32 microcontrollers:
//   https://www.st.com/resource/en/application_note/dm00236305-generalpurpose-timer-cookbook-for-stm32-microcontrollers-stmicroelectronics.pdf
// Training by ST:
//   https://st-onlinetraining.s3.amazonaws.com/STM32L5-WDG_TIMERS-General_Purpose_Timer_GPTIM/index.html
///////////////////////////////////////////////////////////////////////////////////
#include "stm32l552xx.h"
#include <stdio.h>
//#include "cortex_m33.h"

// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &  1) //Checks the bit number <idx> -- 0 means clear; !0 means set.


// Helping functions (Already coded)
void setClks();
void LPUART1init(void);
void LPUART1write(int c);
int  LPUART1read(void);
void myprint(char msg[]);
void RLEDinit();
void RLEDtoggle();

////////////////////////////////////
/*     Functions Declaration      */
/*    ** Need to be coded ***     */
////////////////////////////////////
void delay_ms(uint32_t val); // Use SysTick with 16Mhz source clock
void delay_us(uint32_t val); // Use Timer 5 with 16Mhz source clock
void freq_gen(uint16_t val); // Use Timer 3 and blink Green LED (PC7), 50% Duty Cycle, Freq  1k to 0.1Hz
void edge_counter();         // Use LPTIM2 routed to PD12
void pwm(uint32_t val);      // Use Timer 2 routed to PB11 (val should be 0 to 100)

uint32_t global_var; //testing var

//////////////////////////////////
/*         Main Function        */
//////////////////////////////////
int main(void) {
    char txt[256];
    setClks();     // Clocks are ready to use, 16Mhz for system
    LPUART1init(); // UART is ready to use
    RLEDinit();    // RED LED is ready to use


    // Hint: un-comment TEST CASEs one by one.
    // Code the the use function and test it.
    // Use debug mode

   // TEST CASE 1:

//
//   while(1){
//       RLEDtoggle();
//       delay_ms(1000);
//   }


//    // TEST CASE 2:
    bitset(RCC ->APB1ENR1,3); // enable TIM4 by Enable APB1 port for peripheral Clock (for this case TIM4)
    while(1){
        RLEDtoggle();
        delay_us(1000000);
    }


//    // TEST CASE 3:
//    freq_gen(5000); // LED is On for 5 second, OFF for 5 second


//    // TEST CASE 4: (should run in debug mode)
//    edge_counter();


//    // TEST CASE 5:  Using PWM for dimming an LED
//    int i=0;
//    while(1){
//        pwm(i);
//        delay_ms(1000);
//        i=(i+1)%101;   // so i is 0 to 100
//    }

}


//////////////////////////////////
/*   User Defined Function      */
//////////////////////////////////
// Use SysTick with 16Mhz source clock
void delay_ms(uint32_t val){
    // Using SysTick Timer:
    //        A delay function that can stall CPU 1msec to 100 sec, depending on val.
    //
    // useful link: https://www.youtube.com/watch?v=aLCUDv_fgoU
    //
    // The concept here is to make a delay block using SysTick timer for a delay of 1 msec.
    // The 1 msec delay will be inside a for loop that will loop for val times which will
    // result in a delay of as short as 1msec (for val=1) and as long as
    // 1msec*0xff_ffff (16,777.216 sec)

    // Here are the steps to set the SysTick to delay 1 msec
    //   1- Set the load register to achieve 1msec. Note that you have two options to source your
    //      timer clock. One is to use the HSI clock of 16MHz while the other to use 16MHz/8.
    //   2- Set the counter current value to 0 so that the counter starts
    //   3- Enable the counter and the bit to select which clock you want to use
    // Now the counter is counting down once it reaches 0, a flag in the control register
    // will be set -- use that flag.

	SysTick->CTRL = 0b101; // 2nd bit set the clock source 1st bit set the intetrupter of, and the
	//zero bit enable the system tick
	SysTick->VAL =0;
	for (uint32_t i=0; i<=val; i++){
		SysTick->LOAD=16000-1; // Set the Load to reach 15999 cycle
		while(bitcheck(SysTick->CTRL, 16) == 0); // Stay in the while loop until 1ms per the system tick
	}

}


// Use Timer 5 with 16Mhz source clock
void delay_us(uint32_t val){
    // Using TIM5: A delay function that can delay 1usec to 10 sec
    //
    // In this example will will use the 32bit TIM5 to generate the delay without a for loop.
    // If we prescale the clock to be 1MHz meaning 1usec, then we just need to delay by the val.
    // The range of the delays will be {1usec,  0xFFFF_FFFF*1usec= 4,297.967295sec}

    // Here are the steps to set the counter as a timer to delay x usec
    // 1- Enable the timer clock
    // 2- Set the prescaler to prescale the 16MHz to 1MHz (Note that you need to set it to N-1)
    // 3- Set the auto reload register to val (Note that you need to set it to N-1)
    // 4- Set the counter current value to 0
    // 5- Enable the timer
    // 6- Stall the CPU until the "Update Interrupt Flag" is raised.

	TIM5->PSC = 160-1;		// Perscaler setting, slow down the clk to 1MHz (The counter clock frequency CK_CNT is equal to fCK_PSC , Refer to 34.4.14
	TIM5->ARR = val -1 ;	// ARR is the value to be loaded in the actual auto-reload register. Referance 34.4.15
	TIM5->CNT = 0;			// counter need to be set to 0 to reset the cycle of operation
//	TIM5->CR1 = 1<<4;		//
	TIM5->CR1 |= 1;			//
	while(bitcheck(TIM5->SR, 0)==0){

	}


}


// Use Timer 3 and set to PC7, 50% Duty Cycle, Value 1 to 10,000 for freq 1Khz to 0.1Hz
void freq_gen(uint16_t val){
	// for CCER 34 4 13 bit 0?
    // Using Timer 3: A frequency generator function that generate a 50% duty cycle
    // with a period in millisecond.
    // The timer should be used to blink the LED on PC7
    //
    //    ____________              ____________              ____________
    // __|            |____________|            |____________|            |_
    //   |<--  VAL -->|
    //
    // Configure PC7 as output to drive the LED
    // Steps to setup Timer 3(not timer 8) As A frequency generator:
    //   1- Enable Timer Clock
    //   2- Set prescaler (choose a prescale value that could make your live easier?!) hint:16k
    //   3- Set auto reload register
    //   4- Reset the counter current value
    //   5- Enable the timer
    //   6- The LED will autmatically toggle.

    /* Timer and PC7 Config is here */

}


// Use LPTIM2 routed to PD12
void edge_counter(){
    // Use external input PD12 as the LPTIM2 clock source. Should be connected to the PIN RX of on the STLINK.
    // This way any character sent from the terminal will generate a waveform.
    // Each edge of the input signal increments the LPTIM2 counter by 1.
    // Timer need to count from positive edge to positive edge
    // Use external input Pin RX on the board (LPUART2 RX) as the LPTIM2 clock source.

    // LPUART1 is enabled so that you can can generated input for counters
    // For example if you send the letter 'U' (ASCII is 0x55=0101_0101) from realterm you
    // will see a waveform like this one:    (5 neg edges, 5 pos edges)
    //  _____   _   _   _   _   ______
    //       \_/ \_/ \_/ \_/ \_/
    // IDLE   S 0 1 2 3 4 5 6 7   STOP
    // The receiver channel of LPUART1 can be accessible from the STLINK RX pin
    // *** YOU MUST CONNECT STLINK RX PIN TO PD12 ***

    // Configure PD12 as input of LPTIM2_IN1

    // Configure LPTIM2 to use external input as counter clock source


    while (1){
       // In a debug mode monitor: LPTIM2->CNT while you are sending the letter 'U' from terminal
       // you are going to see 0 before you press, then 5 after the 1st 'U',
       // then 10 and so on.
   }
}



// Use Timer 4 routed to LED_BLUE (PB7)  (val should be 0 to 100)
void pwm(uint32_t val){
    // Use Timer 4:
    // This is a function to show you how to use the compare functionality of the timers.
    // The function will allow controlling the LED intensity using PWM.
    //
    // Useful linkL https://www.youtube.com/watch?v=BtAi6-7Lnlw
    //
    // Steps to setup the timer in an output compare mode:
    //  1- Enable clock
    //  2- Set prescaler ( don't not make any adjments)
    //  3- Set auto reload register (using the passed val)
    //  4- Set the Capture/Compare Mode Register to set output to PWM
    //  5- Set the match value to val (or something based on val?)
    //  6- Enable CHx compare mode which is connected to the PB7
    //  7- Reset the counter current value
    //  8- Enable the timer
    //  No need to do anything else! The PWM of the PB7 is done automatically by the TIM4, the CPU
    //  can do anything else.


    // configure PB7 to be driven by the clock

    // configure TIM3

}














/////////////////////
// Helping Functions
/////////////////////
void RLEDinit(){
// Enable clock going to GPIOA
RCC->AHB2ENR|=1;

// Set up the mode
GPIOA->MODER |= 1<<18; // setting bit 18
GPIOA->MODER &= ~(1<<19);
}
void RLEDtoggle(){
global_var = GPIOA->ODR ^= 1<<9;

}

void setClks(){
RCC->APB1ENR1 |=1<<28;   // Enable the power interface clock by setting the PWREN bits
RCC->APB1ENR2 |=0x1;     // Enable LPUART1EN clock
RCC->CCIPR1   |=0x800;   // 01 for HSI16 to be used for LPUART1
RCC->CCIPR1   &= ~(0x400);
RCC->CFGR     |=0x1;     // Use HSI16 as SYSCLK
RCC->CR       |=0x161;   // MSI clock enable; MSI=4 MHz; HSI16 clock enable
}
void LPUART1init(){
PWR->CR2      |=0x200;   // Enable VDDIO2 Independent I/Os supply
                        // Basically power up PORTG

// LPUART1 TX is connected to Port G pin 7, RX is connected to PG8
// GPIOG is connected to the AHB2 bus.
RCC->AHB2ENR |= (1<<6);   // Enable the clock of the GPIOG

// MCU LPUART1 TX is connected the MCU pin PG7
    // PG7 must be set to use AF (Alternate Function).
    // Note that you need to set AF to 8 to connect the LPUART1 TX to the GPIOG.7
GPIOG->MODER  &= ~(0x3<<(2*7)); // Clear the two bits for PG7
GPIOG->MODER  |=  (0x2<<(2*7)); // Set the mode to AF (10--> 0x2)
// Set the AF=8
GPIOG->AFR[0] &= ~(0xF<<(4*7)); // Clear the 4 bits for PG7
    GPIOG->AFR[0] |=  (0x8<<(4*7)); // Set the 4 bits to (8)

// MCU LPUART1 RX can be connected the MCU pin PG8
    // PA3 must be set to use AF (Alternate Function).
    // Note that you need to set AF to 8 to connect the LPUART1 RX to the GPIOG.8
GPIOG->MODER  &= ~(0x3<<(2*8)); // Clear the two bits for PG8
GPIOG->MODER  |=  (0x2<<(2*8)); // Set the mode to AF (10--> 0x2)

GPIOG->AFR[1] &= ~(0xF<<(4*0)); // Clear the 4 bits for PG8
    GPIOG->AFR[1] |=  (0x8<<(4*0)); // Set the 4 bits to (7)

// Enable the clock for the LPUART1
// LPUART1 is connected to the APB1 (Advanced Peripheral 1) bus.
    // LPUART1 enabled by setting bit 0

// LPUART1 CONFIGURATION //
    // We need to setup the baud rate to 115,200bps, 8 bit, 1 stop bit, no parity, and no hw flow control
LPUART1-> PRESC = 0;    //MSI16 going to the UART can be divided. 0000: input clock not divided

// Buadrate = (256 X LPUARTtck_pres)/LPUART_BRR
// LPUART_BRR = 256 * 16MHz / 115200=  35,555.5  ==> 0x8AE3
LPUART1->BRR = 0x8AE3;  //  (16000000/115200)<<8

// LPUART1 input clock is the HSI (high speed internal clock) which is 16MHz.
LPUART1->CR1  = 0x0;  // clear all settings
LPUART1->CR1 |= 1<<3; // Enable Transmitter
LPUART1->CR1 |= 1<<2; // Enable Receiver

// 00: 1 stop bit
LPUART1->CR2 = 0x0000;    // 1 stop bit and all other features left to default (0)
LPUART1->CR3 = 0x0000;    // no flow control and all other features left to default (0)

// Last thing is to enable the LPUART1 module (remember that we set the clock, configure GPIO, configure LPUART1)
LPUART1->CR1 |= 1; // Enable LPUART1

}


void myprint (char msg[]){
    uint8_t idx=0;
    while(msg[idx]!='\0' ){ LPUART1write(msg[idx++]);}
}



/* Write a character to LPUART1 */
void LPUART1write (int ch) {
    while (!(LPUART1->ISR & 0x0080)); // wait until Tx buffer empty
    LPUART1->TDR = (ch & 0xFF);
}

/* Read a character from LPUART1 */
int LPUART1read(void) {
    while (!(LPUART1->ISR & 0x0020)) {}   // wait until char arrives
    return LPUART1->RDR;
}
