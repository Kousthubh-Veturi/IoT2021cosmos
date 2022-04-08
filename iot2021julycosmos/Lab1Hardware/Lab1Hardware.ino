#include <driverlib.h>

uint8_t count = 0;
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
void setup() {
  WDT_A_hold(WDT_A_BASE);
  // put your setup code here, to run once:
  GPIO_setAsOutputPin(GPIO_PORT_P2 , GPIO_PIN0);
  GPIO_setAsOutputPin(GPIO_PORT_P2 , GPIO_PIN1);
  GPIO_setAsOutputPin(GPIO_PORT_P2 , GPIO_PIN2);

  pinMode(P4_2, OUTPUT);


}

void loop() {
  // put your main code here, to run repeatedly:
  RGB_output(count);
  count++;
  if (count > 7) {
    count = 0;
  }
   
   digitalWrite(P4_2, HIGH); 
   delay(100);
   digitalWrite(P4_2, LOW);

  delay(1000);
}
