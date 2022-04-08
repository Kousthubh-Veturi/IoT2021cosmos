/*
 * 
 * COSMOS LAB5 Part 2
 * Make Sure to input your correct wifi and API credentials!!
 * 
 */



#include <driverlib.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <stdlib.h>
 
 /* LCD Screen libraries*/
#include <LCD_screen.h>
#include <LCD_screen_font.h>
#include <LCD_utilities.h>
#include <Screen_HX8353E.h>
#include <Terminal12e.h>
#include <Terminal6e.h>
#include <Terminal8e.h>
Screen_HX8353E myScreen;
volatile uint16_t y[5];
uint16_t mic_high = 600;
uint16_t mic_low = 512;
volatile uint16_t counter = 0;
char buf[3];
uint64_t status;
volatile uint16_t ledOut;
volatile uint16_t flag;
// your network name also called SSID
char ssid[] = "Veturi2";
// your network password
char password[] = "Login2veturi";
// MQTTServer to use
char server[] = "io.adafruit.com";

WiFiClient wifiClient;
PubSubClient client(server, 1883, callback, wifiClient);


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Received message for topic ");
  Serial.print(topic);
  Serial.print("with length ");
  Serial.println(length);
  Serial.println("Message:");
  Serial.write(payload, length);
  Serial.println();
}

uint8_t my_map (uint16_t in){
  
  uint8_t out;
  /*
   * Map ADC14 values to a range between 0 and 255
   */

   if (in < mic_low){
      out = 0;
   }
   else if (in > mic_high){
      out = 255;
   }
   else {
      // Place Mapping Logic Here
      out = (in-mic_low)*(mic_high-mic_low)/(255);
   }
  
  return out;
}


void LCD_init(){
  
  myScreen.begin();

  /* Let's make a title*/
  myScreen.gText(20, 20, "Microphone ADC:");
                  
}

void ADC14_IRQHandler (void){

    
    // Clear the interrupt flags
    status = ADC14_getEnabledInterruptStatus();
    ADC14_clearInterruptFlag(status);

    counter++;

    /* ADC_MEM2 conversion completed */
    if(status & ADC_INT0 & (flag==0))
    {
          flag = 1;
          for(int i = 4; i>0; i--){
             int temp = y[i-1];
            /* y[i+1] = y[i];
             y[i] = temp;*/
             y[i] = y[i-1];
          }
          y[0] = ADC14_getResult(ADC_MEM0);

         int k = (y[0] + y[1] + y[2] + y[3] + y[4])/5;
         ledOut = my_map(k);
         myScreen.gText(40,50,String(ledOut));
         analogWrite(39, ledOut);
        /*
         * 1. Insert Buffer Iteration Logic Here
         * 2. Sample Newest Microphone Value
         * 3. Normalize Input with this line: MicIn[0] = abs(MicIn[0] - 512) + 512;
         * 4. Average Buffer
         * 5. Display Value on LCD Screen
         */
        
         
    }
 }


void ADC_init(){
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN3, GPIO_TERTIARY_MODULE_FUNCTION);
  ADC14_registerInterrupt(ADC14_IRQHandler);

  /* Initializing ADC (ADCOSC/64/8) */
  ADC14_enableModule();
  ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

  /* Configuring ADC Memory */
  ADC14_configureSingleSampleMode(ADC_MEM0, true);
  ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A10, false);

  /* Enabling Interrupts */
  Interrupt_enableInterrupt(INT_ADC14);
  ADC14_enableInterrupt(ADC_INT0);
  Interrupt_enableMaster();

  /* Setting up the sample timer to automatically step through the sequence
   * convert.*/
  ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
  ADC14_setResolution(ADC_10BIT);

  /* Triggering the start of the sample */
  ADC14_enableConversion();
  ADC14_toggleConversionTrigger();
  
}

void setup() {


  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  flag = 0;

  Serial.print("Attempting to connect to Network named: ");
  // print the network name (SSID);
  Serial.println(ssid); 
  // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
  WiFi.begin(ssid, password);
  while ( WiFi.status() != WL_CONNECTED) {
    // print dots while we wait to connect
    Serial.print(".");
    delay(300);
  }
  Serial.println("\nYou're connected to the network");

  LCD_init();
  ADC_init();
}

void loop() {

    if (counter >= 300 && flag ==1){

      // The Interrupt should be disabled!!!
      // Connect to Adafruit IO
      // Publsih the most recent microphone reading to the cloud
      if (!client.connected()) {
           Serial.println("Disconnected. Reconnecting....");
  
        if(!client.connect("energia_Kousthubh", "Kousthubh", "aio_uEVY91XPHRNwHE7jBLF6smEVitTS")) {  
           Serial.println("Connection failed");
         } else {
          Serial.println("Connection success");
         
         }
       }
       if(client.publish("Kousthubh/feeds/micropublish",itoa(ledOut, buf, 10))) {
        Serial.println("Publish success");
       } else {
        Serial.println("Publish failed");
       }
          
      counter = 0;
      flag = 0;
      
    }
    else{
       // Go back to the interrupt handler function
       flag = 0;
    }
    delay(1000);
    
}
  
                      
