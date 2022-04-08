

#include <stdio.h>
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
int interupt_count = 0;
int old_gesture = 0;
int state = 0;
int timer = 0;
int gesture = 0;
int flag = 1;
uint8_t xAccel, prevXAccel, frequency;
uint8_t io1;
volatile uint16_t indices;
float delta_f = 8.97;
volatile uint32_t accel_datax[10];
volatile uint32_t accel_datay[10];
volatile uint32_t avg_xdata = 0;
volatile uint32_t avg_ydata = 0;
volatile uint32_t old_avg_xdata = 0;
volatile uint32_t old_avg_ydata = 0;
volatile uint32_t velocity_f = 0;
volatile uint32_t velocity_i = 0;
volatile uint32_t velocity_fy = 0;
volatile uint32_t velocity_iy = 0;
int seconds = 0;
int tau = 30;
volatile uint16_t minX;
volatile uint16_t maxX;
volatile uint16_t minY;
volatile uint16_t maxY;
volatile uint32_t displacement_f;
volatile uint32_t displacement_i;
volatile uint32_t displacement_fy;
volatile uint32_t displacement_iy;
int count = 0;
volatile uint32_t resultsBuffer[2];
char buf[3];
void drawAccelData(void);


// your network name also called SSID
char ssid[] = "Veturi2";
// your network password
char password[] = "Login2veturi";
// MQTTServer to use
char server[] = "io.adafruit.com";

WiFiClient wifiClient;
PubSubClient client(server, 1883, callback, wifiClient);

void callback(char* topic, byte* payload, unsigned int length) {

  // Type Cast Input Bytes to Char
  char* str = (char*)payload;

   if(str[1] == 'N'){
  io1 = 0;
  Serial.println("The button is on");
   
   }
   if(str[1] == 'F'){
  io1 = 1;
  Serial.println("The button is off");
   }


}

void LCD_init(){
 
  myScreen.begin();
 
  /* Let's make a title*/
  myScreen.gText(8, 5, "Acceleration Data");
  myScreen.gText(8, 35, "Velocity Data");
  myScreen.gText(8, 65, "Displacement Data");
  myScreen.gText(8, 95, "Gesture Detected");
             
}

void ADC_init(){
  /* Configures Pin 4.0, 4.2, and 6.1 ad ADC inputs */
  // ACC Z = P4.2
  // ACC Y = P4.0
  // ACC X = P6.1
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION);
  GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);

  ADC14_registerInterrupt(ADC14_IRQHandler);

  /* Initializing ADC (ADCOSC/64/8) */
  ADC14_enableModule();
  ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

  /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM2 (A11, A13, A14)  with no repeat)
   * with internal 2.5v reference */
  ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM2, true);
  ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, ADC_NONDIFFERENTIAL_INPUTS);

  ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS);

  ADC14_configureConversionMemory(ADC_MEM2, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A11, ADC_NONDIFFERENTIAL_INPUTS);


  ADC14_setResolution(ADC_10BIT); //IMPORTANT -> This seemed to give me the least noisey values (8 bit res was too small though)

  /* Enabling the interrupt when a conversion on channel 2 (end of sequence)
   *  is complete and enabling conversions */
   ADC14_enableInterrupt(ADC_INT2);

  /* Enabling Interrupts */
  Interrupt_enableInterrupt(INT_ADC14);
  Interrupt_enableMaster();

  /* Setting up the sample timer to automatically step through the sequence
   * convert.*/
  ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

  /* Triggering the start of the sample */
  ADC14_enableConversion();
  ADC14_toggleConversionTrigger();
}

void drawAccelData(void){
  myScreen.gText(40, 15,  "X: " + String(avg_xdata));
  myScreen.gText(40, 25,  "Y: " + String(avg_ydata));
  myScreen.gText(40, 45,  "X: " + String(velocity_f));
  myScreen.gText(40, 55,  "Y: " + String(velocity_fy));
  myScreen.gText(40, 75,  "X: " + String(displacement_f));
  myScreen.gText(40, 85,  "Y: " + String(displacement_fy));
  myScreen.gText(40, 105,  "G: " + String(gesture));
  myScreen.gText(40, 115,  "S: " + String(state));
 
}

void ADC14_IRQHandler(void)
{
  uint64_t status;

  status = MAP_ADC14_getEnabledInterruptStatus();
  MAP_ADC14_clearInterruptFlag(status);

  /* ADC_MEM2 conversion completed */
  if(status & ADC_INT2)
  {
     interupt_count++;
    // if ([DATA IS NOT BEING PUBLISHED TO SERVER])
   
   
    /* Store ADC14 conversion results */
    resultsBuffer[0] = ADC14_getResult(ADC_MEM0); // X Accelerometer
    resultsBuffer[1] = ADC14_getResult(ADC_MEM1); // Y Accelerometer
 
    accel_datax[indices] = resultsBuffer[0];
    accel_datay[indices] = resultsBuffer[1];
     
   
    
    /* Below implement logic for a smoothing filter for Acc readings, numReadings = 10 */
   
  /************************************************************************************/

    indices++;
    if (indices >= 10){
      for (int i = 0; i < 10; i++) {
        avg_xdata += accel_datax[i];
        avg_ydata += accel_datay[i];
      }
      avg_xdata/=10;
      avg_ydata/= 10;
      indices = 0;
  // }
    velocity_f =(10*(avg_xdata + old_avg_xdata)/(2*delta_f));
    velocity_fy = (10*(avg_ydata + old_avg_ydata)/(2*delta_f));
           
   
    // Calculate Displacement Readings
    displacement_f = displacement_i + (10*(velocity_f + velocity_i))/(2*delta_f);
    displacement_fy = displacement_iy + (10*(velocity_fy + velocity_iy))/(2*delta_f);

    displacement_i = 0;
    displacement_iy = 0;
    velocity_i = 0;
    velocity_iy = 0;
   
    old_avg_ydata = avg_ydata;
    old_avg_xdata = avg_xdata;
    /* IMPLEMENT GESTURE DETECTION STATE MACHINE */
  if (seconds < tau){
      seconds++;

       

        /* HERE IS CODE FOR MAX and MIN VALUE FOR BOTH X AND Y */
        if (displacement_f > maxX){
            maxX = displacement_f;
        }

        if (displacement_f < minX){
            minX = displacement_f;
        }

        if (displacement_fy > maxY){
            maxY = displacement_fy;
        }

        if (displacement_fy < minY){
            minY = displacement_fy;
        }        
    }
    else{
        /* After time of tau check max/min X and Y displacement values and decide next state*/
          seconds = 0;

        /* IMPLEMENT GESTURE DETECTION STATE MACHINE */
   
        // State 0 -> inital state
        if (state == 0){
          GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
          GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1 | GPIO_PIN2);
          if (((displacement_fy - minY) > 2) && ((maxX - displacement_f) < 4)){
              state = 1;
          }
          else {
              state = 0;
          }
        }

        // State 1 -> X-gesture detected
        else if (state == 1){
          GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
          GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN2);
          if (((maxX - displacement_f) > 2) && ((displacement_fy - minY) < 4)){
              state = 2;
          }
          else {
            state = 0;
          }
        }

        // State 2 -> L gesture detected
        else if (state == 2){
          GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);
          GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1 | GPIO_PIN0);
          gesture++;
          flag = 0;
          state = 0;
        }
       
        
        // Reset Max/Min storage values
        minX = 65535; // Max number for uint16
        maxX = 0;
        minY = 65535; // Max number for uint16
        maxY = 0;
        old_gesture = gesture;
    }
    drawAccelData();
    // Calculate Velocity Readings
    }
   
    }
}



void setup() {

  WDT_A_hold(WDT_A_BASE);

  Serial.begin(115200);

  // Start Ethernet with the build in MAC Address
  // attempt to connect to Wifi network:
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
  while(!client.connected()) {
           //Interrupt_disableMaster();
           Serial.println("Disconnected. Reconnecting....");
           //Interrupt_disableMaster();
        if(!client.connect("energia_Kousthubh", "Kousthubh", "aio_uEVY91XPHRNwHE7jBLF6smEVitTS")) {  
           Serial.println("Connection failed");
         } else {
          Serial.println("Connection success");
         if(client.subscribe("Kousthubh/feeds/micropublish")) {
            Serial.println("Subscription successful");
          
           }
         if(client.subscribe("Kousthubh/feeds/outtopic")) {
            Serial.println("Subscription successful");
          
          }
          if(client.subscribe("Kousthubh/feeds/status-finalproject")) {
            Serial.println("Subscription successful");
          
          }
          if(client.subscribe("Kousthubh/feeds/button1")) {
            Serial.println("Subscription successful");
          
           }
         }
  }

  /* Initialize all variables to 0 here  */

  resultsBuffer[0] = 0;
  resultsBuffer[1] = 0; 
  resultsBuffer[2] = 0;
  resultsBuffer[3] = 0;
  /***************************************/
  minX = 65535; // Max number for uint16
  maxX = 0;
  minY = 65535; // Max number for uint16
  maxY = 0;
  indices = 0;
  /* Initialize LCD and make some titles */
  LCD_init();
  /* Initialize ADC *SEE THIS FUNCTION IMPORTANT LINES ARE HIGHLIGHTED* */
  ADC_init();
}

void loop() {
 
  /* Everything for interacting with the MQTT server is in this loop */
  client.poll();
  if (interupt_count >= 20){ // This turns  out to be a delay of approx. 2.5 seconds
 
  if (!client.connected()) {
  Serial.println("Disconnected. Reconnecting....");

  if(!client.connect("kv", "Kousthubh", "aio_uEVY91XPHRNwHE7jBLF6smEVitTS")) { // Connect to API
    Serial.println("Connection failed");
  } else {
    Serial.println("Connection success");
    if(client.subscribe("Kousthubh/feeds/button1")) { // Subscribe to button1 feed - controls whether to publish X/Y data
      Serial.println("Subscription successful");
    }
  }
  }

  /* This loop triggers when gesture is detected */
  if (flag == 0){
  if(client.publish("Kousthubh/feeds/status-finalproject", "0")) { // Send 0 to gesture feed (indicator light becomes green)
        Serial.println("Publish success - gesture0");
    } else {
        Serial.println("Publish failed - gesture0");
    }

    delay(1000); // Allow indicator on dashboard to stay lit for a little

    if(client.publish("Kousthubh/feeds/status-finalproject", "1")) { // Send 1 to gesture feed (indicator light becomes red)
        Serial.println("Publish success - gesture1");
    } else {
        Serial.println("Publish failed - gesture1");
    }

    flag = 1; // Reset gesture flag to 1 - No gesture Detected
  }
  /* Publish to Xacc when choose_display == 0 (see callback function near top)*/
  else if (io1 == 0){
    if(client.publish("Kousthubh/feeds/micropublish",itoa(avg_xdata, buf, 10))) {
        Serial.println("Publish success - micropublish");
    } else {
        Serial.println("Publish failed - micropublish");
    }   
  /* Publish to Yacc when choose_display == 1 (see callback function near top)*/
  }else if(io1==1){
    if(client.publish("Kousthubh/feeds/y-axis", itoa(avg_ydata, buf, 10))) {
        Serial.println("Publish success - y-axis");
    } else {
        Serial.println("Publish failed - y-axis");
    }
  }

  client.poll(); //Function allows any subscription inputs from Adafruit IO
 
  interupt_count = 0; // Reset 2 second delay
  }
 
}
