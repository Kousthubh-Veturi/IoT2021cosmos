#include <driverlib.h>

uint8_t S1, S2;



void setup() {
  // put your setup code here, to run once:
  WDT_A_hold(WDT_A_BASE);

  GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
  GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2 | GPIO_PIN1 | GPIO_PIN0);
  GPIO_setAsInputPinWithPullupResistor(GPIO_PORT_P1, GPIO_PIN4 | GPIO_PIN1);
  GPIO_setOutputLowOnPin(GPIO_PORT_P2 , GPIO_PIN1);
  
}

void loop() {
  // put your main code here, to run repeatedly: 

      
  
}
