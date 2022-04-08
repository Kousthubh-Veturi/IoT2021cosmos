
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

// For smoothing accelerometer data
const int numReadings = 10;
int readIndex = 0;

int16_t readingsX[numReadings];
int32_t totalX = 0;

int16_t readingsY[numReadings];
int32_t totalY = 0;

// For State Machine
const int tau = 30;
volatile uint16_t minX;
volatile uint16_t maxX;
volatile uint16_t minY;
volatile uint16_t maxY;
volatile uint32_t seconds;
volatile int gesture_detected;
volatile int current_state;


// For MQTT Server
volatile uint16_t server_seconds;
volatile int choose_display;
char buf[5];
volatile int gesture_flag;


// To initialize acceleration data
int init_flag = 0;


volatile int16_t Xdis;
volatile int16_t Ydis;

// Volatile value to poll ADC from interrupt
volatile int16_t resultsBuffer[3];

volatile int16_t Yvelocity[2];
volatile int16_t Xvelocity[2];
volatile int16_t Yacceleration[2];
volatile int16_t Xacceleration[2];
volatile int16_t Ydisplacement[2];
volatile int16_t Xdisplacement[2];
volatile int16_t Xval;
float delta_f = 8.47;  // Measured this value...


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

  /* Check the Second Character of the char* pointer
   *  str[1] == 'N' ---> IO_button = ON
   *  str[1] == 'F' ---> IO_button = OFF
   */

   /* button1 feed data */
   /* choose_display decides whether Xacc or Yacc is published to */
   if (str[1] == 'F'){
    choose_display = 0;
   }
   else if (str[1] == 'N'){
    choose_display = 1;
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
  myScreen.gText(40, 15,  "X: " + String(Xacceleration[1]));
  myScreen.gText(40, 25,  "Y: " + String(Yacceleration[1]));
  myScreen.gText(40, 45,  "X: " + String(Xvelocity[1]));
  myScreen.gText(40, 55,  "Y: " + String(Yvelocity[1]));
  myScreen.gText(40, 75,  "X: " + String(Xdisplacement[1]));
  myScreen.gText(40, 85,  "Y: " + String(Ydisplacement[1]));
  myScreen.gText(40, 105,  "G: " + String(gesture_detected));
  myScreen.gText(40, 115,  "S: " + String(current_state));
 
}

void ADC14_IRQHandler(void)
{
  uint64_t status;

  status = MAP_ADC14_getEnabledInterruptStatus();
  MAP_ADC14_clearInterruptFlag(status);

  /* ADC_MEM2 conversion completed */
  if(status & ADC_INT2)
  {

      if (server_seconds < 20){ // After 20 loops, we publish data and stop ADC things
      server_seconds++;
   
      /* Store ADC14 conversion results */
      resultsBuffer[0] = ADC14_getResult(ADC_MEM0); // X Accelerometer
      resultsBuffer[1] = ADC14_getResult(ADC_MEM1); // Y Accelerometer
     
    
      /* Below is logic for a smoothing filter for Acc readings, numReadings = 10 */
      // subtract the last reading:
      totalX = totalX - readingsX[readIndex];
      totalY = totalY - readingsY[readIndex];
      // read from the sensor:
      readingsX[readIndex] = resultsBuffer[0];
      readingsY[readIndex] = resultsBuffer[1];
      // add the reading to the total:
      totalX = totalX + readingsX[readIndex];
      totalY = totalY + readingsY[readIndex];
      // advance to the next position in the array:
      readIndex = readIndex + 1;

      // if we're at the end of the array...
      if (readIndex >= numReadings) {
      // ...wrap around to the beginning:
      readIndex = 0;
      }

      /* Acc is the average of smoothing filter */
      Xacceleration[1] = ((totalX) / numReadings);
      Yacceleration[1] = ((totalY) / numReadings);
     
     
      // Calculate Velocity Readings (100 helped scale so values we readable on LCD)
      Xvelocity[1] = Xvelocity[0] + (10*(Xacceleration[1] + Xacceleration[0])/(2*delta_f));
      Yvelocity[1] = Yvelocity[0] + (10*(Yacceleration[1] + Yacceleration[0])/(2*delta_f));
         

      // Calculate Displacement Readings (10000 helped scale so values were readable on LCD)
      Xdisplacement[1] = Xdisplacement[0] + (10*(Xvelocity[1] + Xvelocity[0])/(2*delta_f));
      Ydisplacement[1] = Ydisplacement[0] + (10*(Yvelocity[1] + Yvelocity[0])/(2*delta_f));

     
     
      // Setting inital conditions to 0 gave best results
      // ie when I used: Xdisplacement[0] = Xdisplacement[1] displacement changes were extremely small
      Xdisplacement[0] = 0;
      Ydisplacement[0] = 0;
      Xvelocity[0] = 0;
      Yvelocity[0] = 0;
     
      // Shift Buffer
      Xacceleration[0] = Xacceleration[1];
      Yacceleration[0] = Yacceleration[1];


     
      /* Find the MAX motion change within tau */
      if (seconds < tau){
          seconds++;

         

          /* HERE IS CODE FOR MAX and MIN VALUE FOR BOTH X AND Y */
          if (Xdisplacement[1] > maxX){
              maxX = Xdisplacement[1];
          }

          if (Xdisplacement[1] < minX){
              minX = Xdisplacement[1];
          }

          if (Ydisplacement[1] > maxY){
              maxY = Ydisplacement[1];
          }

          if (Ydisplacement[1] < minY){
              minY = Ydisplacement[1];
          }          
      }
      else{
          /* After time of tau check max/min X and Y displacement values and decide next state*/
          seconds = 0;

          /* IMPLEMENT GESTURE DETECTION STATE MACHINE */
     
          // State 0 -> inital state
          if (current_state == 0){
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0);
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1 | GPIO_PIN2);
            if (((Ydisplacement[1] - minY) > 4) && ((maxX - Xdisplacement[1]) < 4)){
                current_state = 1;
            }
            else {
                current_state = 0;
            }
          }

          // State 1 -> X-gesture detected
          else if (current_state == 1){
           GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1);
           GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN2);
            if (((maxX - Xdisplacement[1]) > 4) && ((Ydisplacement[1] - minY) < 4)){
                current_state = 2;
            }
            else {
              current_state = 0;
            }
          }

          // State 2 -> L gesture detected
          else if (current_state == 2){
            gesture_detected++;
            gesture_flag = 0;
            current_state = 0;
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1 | GPIO_PIN0);
          }
         

          // Reset Max/Min storage values
          minX = 65535; // Max number for uint16
          maxX = 0;
          minY = 65535; // Max number for uint16
          maxY = 0;
      }

     
      // Draw accelerometer data on display
      drawAccelData();
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


  /* Clear everything important */
  for (int i = 0; i<numReadings; i++){
  readingsX[i] = 0;
  readingsY[i] = 0;
  }

  /* Acceleration array */
  Xacceleration[0] = 0;
  Xacceleration[1] = 0;
  Yacceleration[0] = 0;
  Yacceleration[1] = 0;

  /* Velocity array */
  Yvelocity[0] = 0;
  Yvelocity[1] = 0;
  Xvelocity[0] = 0;
  Xvelocity[1] = 0;

  /* Displacement array */
  Ydisplacement[0] = 0;
  Ydisplacement[1] = 0;
  Xdisplacement[0] = 0;
  Xdisplacement[1] = 0;

  /* Used to find min/max movement displacement values within time < tau*/
  minX = 65535; // Max number for uint16
  maxX = 0;
  minY = 65535; // Max number for uint16
  maxY = 0;

  current_state = 0;  // Indicates current state in state machine
  gesture_detected = 0;  // Indicates how many gestures detected

  /* All of these are used for MQTT server code branches */
  choose_display = 0;
  server_seconds = 0;
  gesture_flag = 1; // Gesture Not Detected

  /* Initialize LCD and make some titles */
  LCD_init();
  /* Initialize ADC *SEE THIS FUNCTION IMPORTANT LINES ARE HIGHLIGHTED* */
  ADC_init();
}

void loop() {
 
  /* Everything for interacting with the MQTT server is in this loop */
 
  if (server_seconds >= 20){ // This turns  out to be a delay of approx. 2.5 seconds
 
  if (!client.connected()) {
  Serial.println("Disconnected. Reconnecting....");

  if(!client.connect("energiaClient45", "Kousthubh", "aio_uEVY91XPHRNwHE7jBLF6smEVitTS")) { // Connect to API
    Serial.println("Connection failed");
  } else {
    Serial.println("Connection success");
    if(client.subscribe("Kousthubh/feeds/button1")) { // Subscribe to button1 feed - controls whether to publish X/Y data
      Serial.println("Subscription successful");
    }
  }
  }

  /* This loop triggers when gesture is detected */
  if (gesture_flag == 0){
  if(client.publish("Kousthubh/feeds/status-finalproject", "0")) { // Send 0 to gesture feed (indicator light becomes green)
        Serial.println("Publish success - gesture0");
    } else {
        Serial.println("Publish failed - gesture0");
    }

    delay(5000); // Allow indicator on dashboard to stay lit for a little

    if(client.publish("Kousthubh/feeds/status-finalproject", "1")) { // Send 1 to gesture feed (indicator light becomes red)
        Serial.println("Publish success - gesture1");
    } else {
        Serial.println("Publish failed - gesture1");
    }

    gesture_flag = 1; // Reset gesture flag to 1 - No gesture Detected
  }
  /* Publish to Xacc when choose_display == 0 (see callback function near top)*/
  else if (choose_display == 0){
    if(client.publish("Kousthubh/feeds/micropublish",itoa(Xacceleration[1], buf, 10))) {
        Serial.println("Publish success - xacc");
    } else {
        Serial.println("Publish failed - xacc");
    }
  }
  /* Publish to Yacc when choose_display == 1 (see callback function near top)*/
  else{
    if(client.publish("Kousthubh/feeds/y-axis", itoa(Yacceleration[1], buf, 10))) {
        Serial.println("Publish success - yacc");
    } else {
        Serial.println("Publish failed - yacc");
    }
  }

  client.poll(); //Function allows any subscription inputs from Adafruit IO
 
  server_seconds = 0; // Reset 2 second delay
  }
 
}
