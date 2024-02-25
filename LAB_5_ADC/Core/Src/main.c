#include "stm32l552xx.h"
#include "stdio.h"

// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bittoggle(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1    ) //Checks the bit number <idx> -- 0 means clear; !0 means set.


// Helping functions
void setClks();
void LPUART1init(void);
void LPUART1write(int c);
int  LPUART1read(void);
void myprint(char msg[]);
void RLEDinit();
void RLEDtoggle();
void TIMER_1_init();
void PC0_Pin_init();
void BTNinit();
void BTN_interrupt_int();
void ADC1_IN1_init_and_setup();
void ADC1_IN2_channel_change();
void delayMs(int n);
volatile int POT_data=0;


#define PERIOD 5 //ms

int main (void) {

    setClks();     // Clocks are ready to use, 16Mhz for system
    LPUART1init(); // UART is ready to use
    RLEDinit();    // RED LED is ready to use
    TIMER_1_init(); // timer 1 setup
    PC0_Pin_init();
    BTNinit();
    BTN_interrupt_int();
    ADC1_IN1_init_and_setup();


    // Enable TIM1
    TIM1->CR1 = 1;


    while (1){
    }

}

/********************
 * Interupts Section*
 ********************/
void EXTI13_IRQHandler(){
	RLEDtoggle();

	ADC1_IN2_channel_change();// Setup to channel 2

	delayMs(10);

	// Taking measurments from PC1 ADC1_IN2 JDATA.
	POT_data=ADC1->JDR2;

	ADC1_IN1_init_and_setup(); // reconfigure to Channel_1

    // 10- Clear pending Interrupt
	EXTI->FPR1 |= 1<<13; // Cleared by writing 1 to it!
	                     // Use RPR1 when trigger by Rising edge
}

void ADC1_2_IRQHandler(){
    uint16_t adc_val;
    char  txt [20];
    // Nested calls of this IRQ will be nasty! I will disable it.
	NVIC_DisableIRQ(ADC1_2_IRQn);
	// read conversion result  (Reading DR clear EOC flag)
	adc_val = (ADC1->DR)&0xfff;
	// VREF=3.3V, step size is 3.3V/(2^12)=3.3/4096
	float voltage = adc_val*(3.3/4096);

/*
* The formula to calculate the output voltage (Vout) at a given temperature (T°C) is:
*	Vout= Vnot + (TC * TempC)
*
*	Vout in this case is voltage =adc_val*(3.3/4096)
*	Vnot= 400 mV 9701 and 500mV for 9700
*	TC is the temperature coefficient (19.5 mV/°C for the MCP9701),
*	TempC is the ambient temperature in Celsius.
*	811mV is equal to the 70 Celsius
*
*	F = C * (9/5) + 32
*	(F-32)*(5/9)=C
*
 */
	sprintf(txt, "$%.02f;",  voltage); //enable float print (also did it for scanf)
	myprint(txt);
	NVIC_EnableIRQ(ADC1_2_IRQn);
}


/**************************************
 *  ADCs INITIALIZATION SETUP SECTION *
 * ************************************/

void ADC1_IN1_init_and_setup(){
// Enable ADC Clock
bitset(RCC->AHB2ENR, 13);  // Enable ADC clock
RCC->CCIPR1 |=0x3<<28;     // Route SYSCLK (HCLK) to ADC

// Turn on ADC Voltage Regulator
bitclear(ADC1->CR, 29);  // Get out of deep power down mode
bitset(ADC1->CR, 28);

// External Trigger Enable and Polarity; EXTEN= 0b01 for rising edge
bitset(ADC1->CFGR,   10);
bitclear(ADC1->CFGR, 11);

//External trigger (EXT0) is connected to TIM1_CH1; EXTSEL=0000
bitclear(ADC1->CFGR, 6); // 0b0000
bitclear(ADC1->CFGR, 7);
bitclear(ADC1->CFGR, 8);
bitclear(ADC1->CFGR, 9);
// Wait for the voltage regulator to stabilize
delayMs(10);

// Set up ADC1_IN1
ADC1->SQR1 = (1<<6)|(0); 	       // L=0 (one channel to read), SQ1=IN1 which is connected to PC0 (Called ADC1_IN1)

// Enable Interrupt so that the Timer trigger the ADC to do conversion
// Once the conversion is done, the EOC flag is raised and the IRQ handler for ADC is called.
NVIC_SetPriority(ADC1_2_IRQn, 1);
NVIC_EnableIRQ(ADC1_2_IRQn);
ADC1->IER  |= 1<<2;                // Enable EOC Interrupt
ADC1->CR   |= 1;            	   // Enable ADC

bitset(ADC1->CFGR, 12);  // OVRMOD: Disable overrun mode (ADC keeps going even if user does not read)

// Wait until ADC is Ready (ADRDY)
while(bitcheck(ADC1->ISR, 0)==0);

bitset(ADC1->CR, 2); // Start Conversion (won't actally do the conversion! It will wait for external trigger instead)
}

void ADC1_IN2_channel_change(){

// Set up ADC1_IN2
ADC1->JSQR = (1<<14)|(1); 	// L=1 (Two channel to read), SQ2=IN2 which is connected to PC1
bitset(ADC1->CR, 3);  // enable injection mode
}


/**********************
 *  PIN SETUP SECTION *
 * ********************/
void PC0_Pin_init(){
// PC0 is ADC1_IN1  (Check datasheet or slides)
RCC->AHB2ENR  |= 0b100;         // Enable GPIOC
bitset(GPIOC->MODER, 0);        // Setup PC0 to 0b11 (Analog input: 0b11)
bitset(GPIOC->MODER, 1);
}

void PC1_Pin_init(){
// PC0 is ADC1_IN1  (Check datasheet or slides)
RCC->AHB2ENR  |= 0b100;         // Enable GPIOC
bitset(GPIOC->MODER, 2);        // Setup PC1 (Analog input: 0b1100 in the modder)
bitset(GPIOC->MODER, 3);
}

void TIMER_1_init(){
// TIMER SETUP
bitset(RCC->APB2ENR, 11);    // enable TIM1 clock
TIM1->PSC = 16000 - 1;       // Divided 16MHz source clk by 16000, for 1ms tick


TIM1->ARR = PERIOD - 1;      // Count 1ms PERIOD times (This is where the Freq of the POT measurement)


TIM1->CCMR1 = 0x30;          // Set output to toggle on match (Output Compare Mode)
TIM1->CCR1 = 1;				 // Output will toggle when CNT==CCR1
bitset(TIM1->BDTR, 15);      // Main output enable
TIM1->CCER |= 1;             // Enable CH1 compare mode
TIM1->CNT = 0;               // Clear counter
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
GPIOA->ODR ^= 1<<9;
}

void setClks(){
RCC->APB1ENR1 |=1<<28;   // Enable the power interface clock by setting the PWREN bits
RCC->APB1ENR2 |=0x1;     // Enable LPUART1EN clock
RCC->CCIPR1   |=0x800;   // 01 for HSI16 to be used for LPUART1
RCC->CCIPR1   &= ~(0x400);
RCC->CFGR     |=0x1;     // Use HSI16 as SYSCLK
RCC->CR       |=0x161;   // MSI clock enable; MSI=4 MHz; HSI16 clock enable
}




/*********************************************
 * UART INIT and SETUP (Print is in here too)*
 *********************************************/

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
    while(msg[idx]!='\0')
    {
    	LPUART1write(msg[idx++]);
    }
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



/*******************************************
 * Button section (init and interupt setup)*
 *******************************************/
void BTNinit(){
	RCC->AHB2ENR |= (1<<2); // Enable GPIOC
	// Set up the mode for button at C13
	bitclear(GPIOC->MODER, 26); // Clear bit 26 and 27
	bitclear(GPIOC->MODER, 27);
}

void BTN_interrupt_int(){
	//	1- Enable GPIO as input
	BTNinit();

	//	2- Enable Clock to SYSCFG
	// To use EXTI you need to enable SYSCFG
	RCC->APB2ENR   |= 1;  // Enable Clock to SYSCFG & EXTI

	//	3- Select Input Using EXTI-->CRx
	EXTI->EXTICR[3] = (0x2)<<8;  // Select PC13


	//	4- Select Trigger type (Failing or Raising Edge)
	EXTI->FTSR1    |= 1<<13;     // Trigger on falling edge of PC13
	                             // Use RTSR1 register for rising edge

	//	5- Disable Interrupt Mask
	EXTI->IMR1     |= 1<<13;     // Interrupt mask disable for PC13

	//	6- Setup Interrupt Priority
	NVIC_SetPriority(EXTI13_IRQn, 0); // 0 is higher than 1 (3 bit priority)

	//	7- Enable IRQ in NVIC
	NVIC_EnableIRQ(EXTI13_IRQn);

	//	8- Enable Global Interrupt Flag
	__enable_irq();   // No need since it is enabled by default
	}


/***********************
*	Delay Timer        *
************************/
void delayMs(int n) {
    int i;

    /* Configure SysTick */
    SysTick->LOAD = 16000;  /* reload with number of clocks per millisecond */
    SysTick->VAL = 0;       /* clear current value register */
    SysTick->CTRL = 0x5;    /* Enable the timer */

    for(i = 0; i < n; i++) {
        while((SysTick->CTRL & 0x10000) == 0) /* wait until the COUNTFLAG is set */
            { }
    }
    SysTick->CTRL = 0;      /* Stop the timer (Enable = 0) */
}
