//***************************************************************************************
//  COSMOS Lab 2, Part 2 - An Implementation of Morse Code
//
//
//
//***************************************************************************************

#include <driverlib.h>
#include <stdio.h>
uint8_t count = 0;          // Keeps track of number of presses
uint32_t time_passed = 0;   // Measure how long SW1 is pressed
uint8_t threshold = 3;     // Compare time_passed to this to see if input is dot or dash
uint8_t input [5];          // Store either dot or dash in this array
uint8_t sw1;

// Establish the Morse Code Reference Arrays
uint8_t zero  [5] = {1, 1, 1, 1, 1};
uint8_t one   [5] = {0, 1, 1, 1, 1};
uint8_t two   [5] = {0, 0, 1, 1, 1};
uint8_t three [5] = {0, 0, 0, 1, 1};
uint8_t four  [5] = {0, 0, 0, 0, 1};
uint8_t five  [5] = {0, 0, 0, 0, 0};
uint8_t six   [5] = {1, 0, 0, 0, 0};
uint8_t seven [5] = {1, 1, 0, 0, 0};
uint8_t eight [5] = {1, 1, 1, 0, 0};
uint8_t nine  [5] = {1, 1, 1, 1, 0};

uint8_t compare_array(uint8_t a[], uint8_t b[], uint8_t size) {
  uint8_t i;
  int c = 0;
  
    /*
       compare both array a and b here
       return 0 if both arrays are equal
      
    */
    if(a[0] == b[0]&&a[1] == b[1]&&a[2] == b[2]&&a[3] == b[3]&&a[4] == b[4]){
    //Serial.print("im here fam\n");
    return 0;
   
    }else{
      return 1;
    }
    
  
  

  //return 1 when both arrays are not equal
 
}
int timer(int num){
  time_passed = 0;
  sw1 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
  while(sw1 == GPIO_INPUT_PIN_HIGH){
    sw1 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
    
  }
  /*int check;
  check = 0;*/
 /* while(sw1 == GPIO_INPUT_PIN_HIGH){
    sw1 == GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
    delay(0.01);
  }*/
  /*while(sw1 == GPIO_INPUT_PIN_LOW){
    sw1 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
    delay(1000);
    time_passed ++;
    check = 1;
  }*/
  //Serial.println("un here");
  while( sw1 == GPIO_INPUT_PIN_LOW){
    sw1 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
    delay(1000);
    time_passed++;
    //check = 1;
  }
  return time_passed;
}
void setup()
{
  // Start Serial Terminal
  Serial.begin(115200);

  // Stop watchdog timer
  WDT_A_hold(WDT_A_BASE);

  GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
}

void loop()
{

  /*
     Add logic for Morse Code Detection here (figure 3)
     You must debounce SW1
     Acknowledge the duration that SW1 is pressed and compare this value to threshold
  */
  uint32_t number;
  for(int t = 0; t<5; t++){
    //Serial.print("got here\n");
    number = timer(input[t]);
    if(number >= threshold){
      input[t] = 1;
    }else{
      input[t] = 0;
    }
    Serial.print(input[t]);
    Serial.println();
  
    count++;
  }
  /*for(int f = 0; f<5; f++){
    Serial.print(input[f]);
    Serial.print(" ");
  }*/
   /*sw1 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);

  while(sw1 == GPIO_INPUT_PIN_HIGH){
    GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
  }*/
  

  // When Count = 5, we have recieved a valid input
  // The following code will output the morse code number detected
  count++;
  if (count >= 5) {
    count = 0;

    // Find Morse Code Value
    if (compare_array(input, zero, 5)==0) {
      Serial.println("0");
      Serial.println("======================");
    }
    else if (compare_array(input, one, 5)==0) {
      Serial.println("1");
      Serial.println("======================");
    }
    else if (compare_array(input, two, 5)==0) {
      Serial.println("2");
      Serial.println("======================");
    }
    else if (compare_array(input, three, 5)==0) {
      Serial.println("3");
      Serial.println("======================");
    }
    else if (compare_array(input, four, 5)==0) {
      Serial.println("4");
      Serial.println("======================");
    }
    else if (compare_array(input, five, 5)==0) {
      Serial.println("5");
      Serial.println("======================");
    }
    else if (compare_array(input, six, 5)==0) {
      Serial.println("6");
      Serial.println("======================");
    }
    else if (compare_array(input, seven, 5)==0) {
      Serial.println("7");
      Serial.println("======================");
    }
    else if (compare_array(input, eight, 5)==0) {
      Serial.println("8");
      Serial.println("======================");
    }
    else if (compare_array(input, nine, 5)==0) {
      Serial.println("9");
      Serial.println("======================");
    }
    else {
      Serial.println("Invalid Input!");
      Serial.println("======================");
    }

  }

}
