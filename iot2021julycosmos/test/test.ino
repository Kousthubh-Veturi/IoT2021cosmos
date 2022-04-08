#include <driverlib.h>

volatile uint32_t i;
void setup() {
  // put your setup code here, to run once:
    WDT_A_hold(WDT_A_BASE);

    GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);
  
}

void loop() {
  // put your main code here, to run repeatedly: 
  
    GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN1 );
    GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN2 );   
    //GPIO_toggleOutputOnPin(GPIO_PORT_P2, GPIO_PIN3);
    //analogWrite( RED_LED , 255 );
    //analogWrite( BLUE_LED , 255);
    //digitalWrite(GPIO_PIN2,HIGH);
   //for(i=100000;i>0;i--);
    delay(500);
}
