#include "stm32l552xx.h"
#include "stdio.h"

#define RED 0
#define BLUE 1
#define GREEN 2

//STRUCTURE DECLEARTION
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


//OFF Fuction
//This function is using the output digit regitory to remove the gpio pin bit value.
// For referancing ODR see 11.6.6
void LED_OFF(int LED_COLOR_OFF){

	color[LED_COLOR_OFF].GPIOx->ODR &= ~(1<<color[LED_COLOR_OFF].pin);
}


// 1 msec is roughly 254 for loop i cycles.
// I used a one mintue timer to test the number of cycles
void delay_ms (uint32_t msec){
	for(int i =0; i<254*msec;i++){
	}
}

void LED_BSRR(int LED_COLOR_test){

	color[LED_COLOR_test].GPIOx->BSRR &= ~(1<<color[LED_COLOR_test].pin);
}




int main(){
// Turn on the clock going to GPIOA
// RCC: Reset and Clock Circuit
// RCC_AHB2ER -- RCC Chapter in Reference manual -- 9.8.42
RCC-> AHB2ENR |=7;

//MODER Reference can be found in 11.6.1
// GPIO port MODER RED
GPIOA->MODER |= 1<<18;
GPIOA->MODER &= ~(1<<19);

// GPIO port MODER Blue
GPIOB->MODER |= 1<<14;
GPIOB->MODER &= ~(1<<15);

// GPIO port MODER GREEN
GPIOC->MODER |= 1<<14;
GPIOC->MODER &= ~(1<<15);

//This is were the Higher level of abstraction happens.
while(1){

LED_ON(RED);
LED_ON(BLUE);
LED_ON(GREEN);

delay_ms(125);//125 msec

LED_OFF(RED);

delay_ms(125);//250 msec

LED_OFF(BLUE);
LED_ON(RED);

delay_ms(125);//375 msec

LED_OFF(RED);

delay_ms(125);//500 msec

LED_OFF(GREEN);
LED_ON(RED);
LED_ON(BLUE);

delay_ms(125);//625 msec

LED_OFF(RED);

delay_ms(125);//750 msec

LED_OFF(BLUE);
LED_ON(RED);

delay_ms(125);//875 msec

LED_OFF(RED);

delay_ms(125);//1000 msec





}
}
