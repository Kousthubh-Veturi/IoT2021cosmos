

#include <driverlib.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <stdio.h>
#include <string.h>

uint8_t count = 0;
uint8_t IO_button = 1;
uint8_t io1;
uint8_t sw1;
// your network name also called SSID
char ssid[] = "Veturi2";
// your network password
char password[] = "Login2veturi";
// MQTTServer to use
char server[] = "io.adafruit.com";

void callback(char* topic, byte* payload, unsigned int length) {

  // Type Cast Input Bytes to Char
  char* str = (char*)payload;

  /* Check the Second Character of the char* pointer
   *  str[1] == 'N' ---> IO_button = ON 
   *  str[1] == 'F' ---> IO_button = OFF
   */
   if(str[1] == 'N'){
    io1 = 1;
    Serial.println("The button is on");
   
   }
   if(str[1] == 'F'){
    io1 = 0;
    Serial.println("The button is off");
   }

   
}

WiFiClient wifiClient;
PubSubClient client(server, 1883, callback, wifiClient);

void RGB_output(uint8_t count) {
  // Function to output to RGB LED based on count value between 0 and 7
   // Implement through simple switch statement''
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

void setup () {

    // Stop watchdog timer
    WDT_A_hold(WDT_A_BASE);

    // Set P1.0 to output direction
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);

    Serial.begin(9600);
  
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
    Serial.println("Waiting for an ip address");
  
    while (WiFi.localIP() == INADDR_NONE) {
      // print dots while we wait for an ip addresss
      Serial.print(".");
      delay(300);
    }

    Serial.println("\nIP Address obtained");
    // We are connected and have an IP address.
    // Print the WiFi status.
    printWifiStatus();
    if (!client.connected()) {
           Serial.println("Disconnected. Reconnecting....");
  
        if(!client.connect("energia_Kousthubh", "Kousthubh", "aio_uEVY91XPHRNwHE7jBLF6smEVitTS")) {  
           Serial.println("Connection failed");
         } else {
          Serial.println("Connection success");
         if(client.subscribe("Kousthubh/feeds/button1")) {
        Serial.println("Subscription successful");
          
           }
         }
       }
}

//analogWrite(39, led_out);
void loop () {
   /* if (!client.connected()) {
           Serial.println("Disconnected. Reconnecting....");
  
        if(!client.connect("Client", "Kousthubh", "aio_uEVY91XPHRNwHE7jBLF6smEVitTS")) {  
           Serial.println("Connection failed");
         } else {
          Serial.println("Connection success");
         if(client.subscribe("Kousthubh/feeds/button1")) {
        Serial.println("Subscription successful");
          
           }
          }
         }*/
    /* 
     *  Wait here until Sw1 on P1.1 or the IOT Button is pressed
     *  This should be a tight polling loop
     *  The processor will execute the loop until SW1 is pressed
     */
     
     sw1 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
     client.poll();

     while(sw1== GPIO_INPUT_PIN_HIGH &&  io1 == 0){
      if (!client.connected()) {
           Serial.println("Disconnected. Reconnecting....");
  
        if(!client.connect("Client", "Kousthubh", "aio_uEVY91XPHRNwHE7jBLF6smEVitTS")) {  
           Serial.println("Connection failed");
         } else {
          Serial.println("Connection success");
         if(client.subscribe("Kousthubh/feeds/button1")) {
        Serial.println("Subscription successful");
          
           }
          }
         }
        
        sw1 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
        
        client.poll();
        
        delay(300);
     }
    
     
    count++;
    if (count > 7){
        count = 0;
    }
    RGB_output(count);

    delay(100);                         // Delay

    /* 
     *  Wait here until Sw1 on P1.1 and the IOT Button is not pressed
     *  This should be a tight polling loop
     *  The processor will execute the loop until SW1 is not pressed
     */
    sw1 = GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
    client.poll();
    
    while((sw1==GPIO_INPUT_PIN_LOW)||(io1==1)){
      client.poll();
      sw1=GPIO_getInputPinValue(GPIO_PORT_P1,GPIO_PIN1);
      delay(300);
    }
    /*while(io1==1){
      client.poll();
      delay(300);
    }*/
   
    delay(100);                         // Delay

}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}
