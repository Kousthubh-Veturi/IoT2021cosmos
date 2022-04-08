//***************************************************************************************
//  COSMOS Lab 3, Part 1 - Systick Timer
//  Using interrupts to implement a stopwatch
//
//
//***************************************************************************************

#include <systick.h>
#include <gpio.h>
#include <interrupt.h>


volatile uint8_t seconds;
volatile uint8_t hundredths;
volatile uint8_t count;
uint8_t sw1;
uint8_t sw2;
uint16_t i;
uint8_t o = 0;
void RGB_output(uint8_t count) {
  if (count == 0) {
    // GPIO_toggleOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1 );
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
  } else if (count == 1) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0 );
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1 | GPIO_PIN2);
  } else if (count == 2) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1 );
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN2);
  } else if (count == 3) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1 | GPIO_PIN0 );
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);
  } else if (count == 4) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2 );
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1 | GPIO_PIN0);
  } else if (count == 5) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN2 );
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
  } else if (count == 6) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2 | GPIO_PIN1 );
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
  } else if (count == 7) {
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
  }
}

// Built-in Interrupt handler function name
void SysTick_Handler (void){

    /*
     * Implement stopwach 
     */
    
    
      
     sw2 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4);
     hundredths++;
     if(hundredths==100){
      seconds++;
      hundredths = 0;
   
      RGB_output(o);
      
      o++;
      
      if(o == 8){
        o = 0;
      }
     
      
     
     }

     
     return;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  // Push button 1 is an input (start button)
  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);

  // Push button 2 is an input (stop button)
  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN4);

  // Set RGB Led as output and set low
  GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
  GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);

  SysTick_registerInterrupt(SysTick_Handler);
  SysTick_setPeriod(480000);
  
  // Disable Interrupt for now
  SysTick_disableInterrupt();
  
}

void loop() {

  Serial.println("Press S1 to Start, Press S2 to Stop");
  
  
  /*
   * Poll SW1 Here
   * You must debounce this switch
   */
    sw1 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
    while(sw1!=GPIO_INPUT_PIN_LOW){
      sw1 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
    }
    
  /*
   * Counting has started - set LED low, reset all variables
   * You must properly reset the Systick Registers / Enable Interrupts
   */
   RGB_output(0);
   SysTick->VAL = 0;
   SysTick_enableModule();
   SysTick_enableInterrupt();


   
  /*
   * Poll SW2 Here
   * You must debounce this switch
   */
   sw2 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4);
   while(sw2!=GPIO_INPUT_PIN_LOW){
    sw2 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN4);
   }
  
  // Disable Interrupt
  SysTick_disableInterrupt();
  
  Serial.print(seconds);
  Serial.print(":");
  Serial.print(hundredths);
  Serial.print("\n");
  seconds = 0;
  hundredths = 0;
  o=0;
}
