/*#include <driverlib.h>
uint8_t seconds;
uint8_t sw1;
uint8_t ped;
uint8_t delay_and_poll(uint8_t seconds){
  uint8_t press =0;
  uint32_t i,j,k;
  k = 50;
  for(i = 0; i<seconds; i++){
    for(j = 0; j<20; j++){
      
      //delay(k);
      sw1 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
      if(sw1 = GPIO_INPUT_PIN_LOW){
        press = 1;
        //break;
        
      }
    }
  }
  return press;

}
void setup() {
  // put your setup code here, to run once:
    WDT_A_hold(WDT_A_BASE);
    GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN2);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1 );
    GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1 , GPIO_PIN1);
    //Serial.print("Hi");
   
    
}

void loop() {
  // put your main code here, to run repeatedly: 

    ped = delay_and_poll(8);
    if(ped == 1){
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1 | GPIO_PIN0);
    delay_and_poll(4);
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0 );
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1 );
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0 );
    delay_and_poll(8);
    }else{
     ped = delay_and_poll(1);
    }
}*/
#include <driverlib.h>

uint8_t ped = 0;


uint8_t delay_and_poll(uint8_t seconds){
    uint8_t press=0;
    uint32_t i, j, k;

    for(i=0; i <seconds; i++){
        for (j=0; j<20; j++){
                for (k=0; k<15000; k++){
                  delay(k);
                  int sw1 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
                  if( sw1 == GPIO_INPUT_PIN_LOW ){
                    press = 1;
                    break;
                  }
                  
                    // Delay loop using k of about 1/20 of a second
                    // Read SW1 and set press to 1 if SW1 pressed
            }
        }
    }
    return press;
}

void RGB_output(uint8_t count) {
      /*
       * Use this function to update the LEDs for both ped and car light
       * count == 0 ---> Car light green, ped light red 
       * count == 1 ---> Car light yellow, ped light red
       * count == 2 ---> Car light red, ped light greeen
       * 
       */
       
       if(count ==0){
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);
       }
       if(count == 1){
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1 | GPIO_PIN0);
        GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
       }
       if (count == 2){
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
       }
       
}

void setup()
{
    
    
    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    // Set P1.0 to output direction
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);

    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);

    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
    //Serial.begin(9600);
    
}

void loop ()
{

    /*
     * Implement the the flow chart in figure 1 here
     * This function will loop infinetly
     */
     RGB_output(0);
    ped = delay_and_poll(8);
  //  while (ped != 1){
    
  //  }
   
    while (ped == 0){
      ped = delay_and_poll(1);
    }
    //Serial.println(ped);
    if(ped ==  1){
    delay(2000);
    RGB_output(1);
    //delay_and_poll(4);
    delay(3000);
    RGB_output(2);
    //delay_and_poll(8);
    delay(6000);
    }else{
     ped = delay_and_poll(1);
    }

}
