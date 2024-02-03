#include "stm32l552xx.h" // Has all definitions for the specific MCU
// You could include more libraries if needed


// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1   ) //Checks the bit number <idx> -- 0 means clear; !0 means set.
#define RED 0 // RED is used for setting the pins of LED funtion
#define BLUE 1 //BLUE is used for setting the pin of LED function
#define GREEN 2 //GREEN is used for setting the pin of the LED funciton.


void setClks();
void enGpiogPwr();
void initGpiog();
void initLPUART1();
void delay_ms(uint32_t);
void INT_USER_BUTTON();
void LED_ON(int);
void LED_OFF(int);
void LED_TOGGLE(int);
void LED_RBG_CLK_AND_PORT_INT();

//////////////////////////////////////////////
// Main function: Entry point of the program//
//////////////////////////////////////////////
int main(){
	//Enable clocks
	setClks();

	//Enable PWR going to PORT G
	// For pwr of gpio go to 8.6 in the refrence manual
	enGpiogPwr();

	//Configure GPIOG
	//Configuration will be call via the regitor section of TE=1 RE=1 UE =1
	initGpiog();

	//Configuring the LED Port for using the RBG Ports
	LED_RBG_CLK_AND_PORT_INT();

	//Configuring Push button PC13 (user Button)
	INT_USER_BUTTON();

	//Configure LPUART1
	initLPUART1();

	// Buffer for transmitting data via LPUART
	char buf [] = "Royce Coykendall ";// Sample output name for RealTerm

	char user_input; // Variable to store received character
	uint8_t i=0; // Index for iterating through `buf`

	// Infinite loop for continuous operation
	for(;;){
		// Check for initial signal from user button
	if(bitcheck(GPIOC->IDR, 13)== 1){

		delay_ms(20); // Debounce delay

		// Transmit data if TX FIFO is not full and user button is pressed
		while(bitcheck(LPUART1->ISR, 7) != 0 && bitcheck(GPIOC->IDR, 13)== 1){
			LPUART1->TDR = buf[i++];
			if (buf[i] == '\0'){ i=0; break;} // Reset index of if end of string is reached
			delay_ms(500); // Delay to allow data transmission
		}

	}


	// Receive data and control LEDs based on input
	while(bitcheck(LPUART1->ISR, 5) != 0 ){
		user_input = LPUART1->RDR; // Read received character

		//Toggle LEDs based on input character
		if(user_input == 'g'||user_input== 'G')LED_TOGGLE(GREEN);
		if(user_input == 'r'||user_input == 'R') LED_TOGGLE(RED);
		if(user_input == 'B'|| user_input == 'b') LED_TOGGLE(BLUE);

		//Turn off all LEDs if input is not a recongized character
		if(user_input != 'r' && user_input !='R' && user_input !='g' && user_input !='G'&& user_input !='b' && user_input !='B'){
			LED_OFF(RED);
			LED_OFF(BLUE);
			LED_OFF(GREEN);
		}
		//Exit loop if input is null character (unlikely scenario for safety)
		if (user_input == '\0') break;
	}
	}
}

void setClks(){
	bitset(RCC->APB1ENR1, 28);  // Enable Clock to PWR Interface
	bitset(RCC->AHB2ENR,   6);  // Enable Clock to GPIOG
	bitset(RCC->APB1ENR2,  0);  // Enable Clock to LPUART
	bitset(RCC->CCIPR1,   11);  // Select the high speed internal (HSI) oscillator as the clock to LPUART1 (16MHz)
	bitclear(RCC->CCIPR1, 10);  //
	bitset(RCC->CR, 8);         // HSI16 clock enable
}
void enGpiogPwr(){
	bitset(PWR->CR2, 9);        // Enable GPIOG power
}
void initGpiog(){
	//set GPIOG.7 to AF
	bitset(GPIOG->MODER,    15);  // Setting 0b10 in pin 7 two bit mode cfgs
	bitclear(GPIOG->MODER,  14);

	bitset(GPIOG->AFR[0],   31);  // Programming 0b1000
	bitclear(GPIOG->AFR[0], 30);
	bitclear(GPIOG->AFR[0], 29);
	bitclear(GPIOG->AFR[0], 28);

	//set GPIOG.8 to AF
	bitset(GPIOG->MODER,    17);  // Setting 0b10 in pin 8 two bit mode cfgs
	bitclear(GPIOG->MODER,  16);

	bitset(  GPIOG->AFR[1], 3);  // Programming 0b1000
	bitclear(GPIOG->AFR[1], 2);
	bitclear(GPIOG->AFR[1], 1);
	bitclear(GPIOG->AFR[1], 0);

}

/////////////////////////
// Supporting functions//
/////////////////////////
struct color_desired{
	GPIO_TypeDef *GPIOx;// GPIO_TypeDef is a structure that calls the bus that is needed
	int pin;// intager that calls the pin number
};

//Color houses the desired GPIO buses and the pins that will be used for this project.
static struct color_desired color[]={
	{GPIOA, 9},
	{GPIOB,7},
	{GPIOC,7}
};

//ON Fuction
//This function is using the output digit regitory(ODR) to add a bit value for gpio pin within the bit value.
// For referancing ODR see 11.6.6
void LED_ON(int LED_COLOR_ON){
	color[LED_COLOR_ON].GPIOx->ODR |= 1<<color[LED_COLOR_ON].pin;
}
//This is a old delay function from Lab_1 (refer to Lab_1 for more information)
void delay_ms (uint32_t msec){
	for(int i =0; i<254*msec;i++){
	}
}


//OFF Fuction
//This function is using the output digit regitory to remove the gpio pin bit value. (from Lab_1)
// For referancing ODR see 11.6.6
void LED_OFF(int LED_COLOR_OFF){

	color[LED_COLOR_OFF].GPIOx->ODR &= ~(1<<color[LED_COLOR_OFF].pin);
}

//New
void LED_TOGGLE(int LED_COLOR_toggle){
	bitflip(color[LED_COLOR_toggle].GPIOx->ODR,  color[LED_COLOR_toggle].pin);
}


INT_USER_BUTTON(){
//Enabling clock to Port C
RCC->AHB2ENR |=4;

//Setting pin 13 to be an input mode
bitclear(GPIOC->MODER,27);
bitclear(GPIOC->MODER,26);

}

void LED_RBG_CLK_AND_PORT_INT(){

RCC-> AHB2ENR |=7;

//MODER Reference can be found in 11.6.1
// GPIOA port MODER RED

bitset(GPIOA->MODER,   18);
bitclear(GPIOA->MODER, 19);


// GPIOB port MODER Blue

bitset(GPIOB->MODER,   14);
bitclear(GPIOB->MODER, 15);

// GPIOC port MODER GREEN
bitset(GPIOC->MODER,   14);
bitclear(GPIOC->MODER, 15);

}

void initLPUART1(){

	// BRR = 256*16000000/115200 = 355,555   = sec*cycle/sec/ buad rate
	// BRR = 256*16000000/57600= 71,111  <- Here was the calcualtion for using 56000 baud rate for are transsmition speed
	LPUART1->BRR = 71111;
	LPUART1->CR1 = 0xD; // 0x1101  --> TX, RX are enabled and UART is Enabled.
}

