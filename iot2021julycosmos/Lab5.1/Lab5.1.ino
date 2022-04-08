
 #include <driverlib.h>
 
 /* LCD Screen libraries*/
#include <LCD_screen.h>
#include <LCD_screen_font.h>
#include <LCD_utilities.h>
#include <Screen_HX8353E.h>
#include <Terminal12e.h>
#include <Terminal6e.h>
#include <Terminal8e.h>
Screen_HX8353E myScreen;

volatile uint16_t mic;
void LCD_init(){
  
  myScreen.begin();

  /* Let's make a title*/
  myScreen.gText(10, 40, "Microphone ADC:");
                  
}
  
void ADC14_IRQHandler (void){
  uint64_t status;
  int a;
    // Clear the interrupt flags
    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);

    /* ADC_MEM2 conversion completed */
    if(status & ADC_INT0)
    {
          mic = ADC14_getResult(ADC_MEM0);
          myScreen.gText(40, 70,String(mic));

         /*
          * 1. Grab the current output from ADC14
          * 2. Display this Value onto the LCD Screen
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

  /* Triggering the start of the sample */
  ADC14_enableConversion();
  ADC14_toggleConversionTrigger();
  
}

void setup() {

  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  LCD_init();
  ADC_init();
}

void loop() {
                       
}
