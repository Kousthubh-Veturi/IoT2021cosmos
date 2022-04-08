#include <driverlib.h>

uint8_t count = 0;
uint8_t sw1;
uint8_t sw2;
void RGB_output(uint8_t count){
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
void setup() {
  // put your setup code here, to run once:
  WDT_A_hold(WDT_A_BASE);
  GPIO_setAsOutputPin(GPIO_PORT_P2 , GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
  GPIO_setOutputLowOnPin(GPIO_PORT_P2 , GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1 , GPIO_PIN1);
  pinMode(P4_2, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly: 
 
  sw1 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
  if(sw1 == GPIO_INPUT_PIN_LOW){
   
  
  count++;
  if(count>7){
    count = 0;
  }
  RGB_output(count);
  }
  sw1 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);

  int st = 0;
  if(sw1 == GPIO_INPUT_PIN_LOW){
    st = 1;
  }
  while(st != 0){
    sw1 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
    if(sw1 == GPIO_INPUT_PIN_LOW){
      delay(100);
       
    }else{
      st = 0;
      
    }
  }
  /*sw2 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);

  while(sw1 == sw2){
    delay(10);
  }*/
  digitalWrite(P4_2, HIGH); 
  delay(100);
  digitalWrite(P4_2, LOW);

 
  delay(100);
  

  
}
