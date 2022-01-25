#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc\tm4c123gh6pm.h"
#include "inc\hw_timer.h"
#include "inc\hw_gpio.h"
#include "driverlib\timer.h"
#include "driverlib\gpio.h"
#include "driverlib\sysctl.h"
#include "inc\tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/GPIO.h"
#include "DIO.h"
volatile int x=0; //Counter for the Traffic Light of Cars
volatile int flag=0; //Used for handling the delay of Pedesterian ISR
 // Variable t is used in the for loop to make pedesterian light a time of 2 seconds

void GPIOPortF_Handler(){
    if(flag==0){
    if(((GPIO_PORTF_DATA_R&0x01)==0)&&((GPIO_PORTE_DATA_R&0x01)==1)){ 
GPIO_PORTD_DATA_R|=0x01;  //N-S ped green light on
GPIO_PORTD_DATA_R&=~0x02; //N-S ped red light off
GPIO_PORTE_DATA_R|=0x04; //N-S red light on
GPIO_PORTE_DATA_R&=~0x03; //N-S green & yellow light off

volatile int t=0; //Variable t is used in the for loop to make pedesterian light a time of 2 seconds
for(t;t<2;t++){ 
    TimerEnable(TIMER1_BASE, TIMER_A); // enable the timer
     while((TIMER1_RIS_R&0x01)!=1){}
     TimerIntClear(TIMER1_BASE,TIMER_TIMA_TIMEOUT); //clear the timer interrupt
}
TimerDisable(TIMER1_BASE, TIMER_A); // disable the timer
    }
    else if(((GPIO_PORTF_DATA_R&0x10)==0)&&((GPIO_PORTE_DATA_R&0x08)==0x08)){
 GPIO_PORTD_DATA_R|=0x04; //E-W ped green light on
 GPIO_PORTD_DATA_R&=~0x08; //E-W ped red light off
 GPIO_PORTE_DATA_R|=0x20; //E-W red light on
 GPIO_PORTE_DATA_R&=~0x18;  //E-W green light off
 GPIO_PORTD_DATA_R&=~0x40; //E-W yellow light off

 volatile int t=0;
 for(t;t<2;t++){
     TimerEnable(TIMER1_BASE, TIMER_A); // enable the timer
      while((TIMER1_RIS_R&0x01)!=1){}
      TimerIntClear(TIMER1_BASE,TIMER_TIMA_TIMEOUT); // clear the timer interrupt
 }
 TimerDisable(TIMER1_BASE, TIMER_A); // disable the timer
     }
flag=1; //FLAG DOWN
TimerIntEnable(TIMER1_BASE,TIMER_TIMA_TIMEOUT);  // Setup the interrupt for the Timer1-TimerA timeouts
TimerEnable(TIMER1_BASE, TIMER_A); // enable the timer
    }
};
void Timer0A_Handler(){
    TimerIntClear(TIMER0_BASE,TIMER_TIMA_TIMEOUT); //clear the timer interrupt
    if(x<6){ // for 5 secs
        x++;
        GPIO_PORTE_DATA_R|=0x01; //N-S green light on
        GPIO_PORTE_DATA_R&=~0x06; //N-S red and yellow light off
        GPIO_PORTD_DATA_R|=0x02;  //N-S ped red light on
        GPIO_PORTD_DATA_R&=~0x01; //N-S ped green light off
        GPIO_PORTD_DATA_R|=0x04;  //E-W ped green light on
        GPIO_PORTD_DATA_R&=~0x08; //E-W ped red light off
        GPIO_PORTE_DATA_R|=0x20;  //E-W red light on
    }
    if(6<=x&&x<9){ // for 3 secs
        x++;
        GPIO_PORTE_DATA_R|=0x02; //N-S yellow light on
        GPIO_PORTE_DATA_R&=~0x05; //N-S red and green light off
    } if(x==9||x==10){ // for 1 sec
        x++;
        GPIO_PORTE_DATA_R|=0x04;  //N-S red light on
        GPIO_PORTE_DATA_R&=~0x03;  //N-S yellow and green light off
        GPIO_PORTD_DATA_R|=0x01;  //N-S ped green light on
        GPIO_PORTD_DATA_R&=~0x02; //N-S ped red light off
    } if(11<=x&&x<17){ // for 5 secs
        x++; 
        GPIO_PORTE_DATA_R|=0x08; //E-W green light on
        GPIO_PORTE_DATA_R&=~0x30; //N-S green and yellow light off
        GPIO_PORTD_DATA_R|=0x08; //E-W ped red light on
        GPIO_PORTD_DATA_R&=~0x04; // E-W ped green light off
        
    } if(17<=x&&x<20){ // for 3 secs
        x++;
        GPIO_PORTD_DATA_R|=0x40; //E-W yellow light on
        GPIO_PORTE_DATA_R&=~0x28; //E-W green and red light off
    } if(x==20){ // for 1 sec
        x=0;
        GPIO_PORTE_DATA_R|=0x20; //E-W red light on
        GPIO_PORTE_DATA_R&=~0x18;  //E-W green light off
        GPIO_PORTD_DATA_R&=~0x40; //E-W yellow light off
        GPIO_PORTD_DATA_R|=0x04;  //E-W ped green light on
        GPIO_PORTD_DATA_R&=~0x08; //E-W ped red light off
    }
}
void Timer1A_Handler(){
 flag=0;
    TimerIntClear(TIMER1_BASE,TIMER_TIMA_TIMEOUT); //clear the timer interrupt
    TimerIntDisable(TIMER1_BASE,TIMER_TIMA_TIMEOUT); // disable setup of interrupt for the Timer1-TimerA timeouts
    TimerDisable(TIMER1_BASE,TIMER_A); // disable the timer
}
void GPIOF_init(){
  SYSCTL_RCGCGPIO_R |= 0x00000020;      //Initialize clock to PORTF
  while((SYSCTL_PRGPIO_R&0x00000020) == 0){};   //safety for clock initialization
  GPIO_PORTF_LOCK_R = 0x4C4F434B; //unlock commit register
  GPIO_PORTF_CR_R = 0x1F;       //Enable change to PORTF
  GPIO_PORTF_DIR_R = 0x0E;      //Make led ports as output
  GPIO_PORTF_DEN_R = 0x1F;      // digital enable to pins
  GPIO_PORTF_PUR_R = 0x11;      //enable pull up
  GPIOIntRegister(GPIO_PORTF_BASE, GPIOPortF_Handler); //Registers an interrupt handler for a GPIO port
  GPIOIntTypeSet(GPIO_PORTF_BASE ,GPIO_PIN_4,GPIO_LOW_LEVEL); //Sets the interrupt type for pin 4
  GPIOIntTypeSet(GPIO_PORTF_BASE ,GPIO_PIN_0,GPIO_LOW_LEVEL); //Sets the interrupt type for pin 0
  GPIOIntEnable(GPIO_PORTF_BASE,GPIO_PIN_4); //Enables the GPIO interrupts
  GPIOIntEnable(GPIO_PORTF_BASE,GPIO_PIN_0); //Enables the GPIO interrupts
}
void GPIOD_init(){
  SYSCTL_RCGCGPIO_R |= 0x00000008;      //Initialize clock to PORTD
  while((SYSCTL_PRGPIO_R&0x00000008) == 0){};   //safety for clock initialization
  GPIO_PORTD_LOCK_R = 0x4C4F434B; //unlock commit register
  GPIO_PORTD_CR_R = 0xFF;       //Enable change to PORTD
  GPIO_PORTD_DIR_R = 0xFF;      //Make led ports as output 
  GPIO_PORTD_DEN_R = 0xFF;      // digital enable to pins
  GPIO_PORTD_DATA_R=0x0A;       // set data
}
void GPIOE_init(){
  SYSCTL_RCGCGPIO_R |= 0x00000010;      //Initialize clock to PORTE
  while((SYSCTL_PRGPIO_R&0x00000010) == 0){};   //safety for clock initialization
  GPIO_PORTE_LOCK_R = 0x4C4F434B;
  GPIO_PORTE_CR_R = 0x2F;       //Enable change to PORTE
  GPIO_PORTE_DIR_R = 0x2F;      //Make led ports as output
  GPIO_PORTE_DEN_R = 0x2F;      // digital enable to pins
}
void Timer0in(){
SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);  //enable clock for timer 0
 while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) //wait till clock is connected
   {}
  TimerConfigure(TIMER0_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC)); //set timer A for periodic
  TimerLoadSet(TIMER0_BASE, TIMER_A, 65535); // set Timer A for max time (4.096 ms)
  TimerPrescaleSet(TIMER0_BASE,TIMER_A,244);  //prescaling Timer A from 4.096 ms to 1s
  TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT); // enable timer interrupt
  TimerIntRegister(TIMER0_BASE,TIMER_A, Timer0A_Handler); //Registers an interrupt handler for the timer interrupt
  IntPrioritySet(INT_TIMER0A,0xE0); //Sets the priority of interrupt
  TimerEnable(TIMER0_BASE, TIMER_A); // enable the timer
   }
void Timer1in(){
SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);  //enable clock for timer 1
 while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER1)) //wait till clock is connected
   {}
  TimerConfigure(TIMER1_BASE, (TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PERIODIC)); //set timer A for periodic
  TimerLoadSet(TIMER1_BASE, TIMER_A, 65535); // set Timer A for max time (4.096 ms)
  TimerPrescaleSet(TIMER1_BASE,TIMER_A,244);  //prescaling Timer A from 4.096 ms to 1 sec
  TimerIntRegister(TIMER1_BASE,TIMER_A, Timer1A_Handler); //Registers an interrupt handler for the timer interrupt
  IntPrioritySet(INT_TIMER0A,0); //Sets the priority of interrupt
   }