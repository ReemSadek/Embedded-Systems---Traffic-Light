#ifndef Timer_H
#define Timer_H

//Initialization of Timer 0
extern void Timer0in(void);
//initialization of Timer 1
extern void Timer1in(void);
//Initialization of Port F and Push Buttons
extern void GPIOF_init(void);
//Initialization of Port  E used for Leds as stated in Word Document.
extern void GPIOE_init(void);
//Initialization of Port  E used for Leds as stated in Word Document.
extern void GPIOD_init(void);
//ISR For Buttons in case of Pedesterian Press
extern void GPIOPortF_Handler();
#endif
