#include <driverlib.h>

volatile uint32_t i;
void setup() {
  // put your setup code here, to run once:
   WDT_A_hold(WDT_A_BASE);

   GPIO_setAsOutputPin(GPIO_PORT_P1,GPIO_PIN0);
}

void loop() {
  // put your main code here, to run repeatedly: 
  GPIO_toggleOutputOnPin(GPIO_PORT_P1,GPIO_PIN0);
  for( i = 100000;i>0;i--);
}
