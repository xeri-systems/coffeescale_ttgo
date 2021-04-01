 /*************************************************************
  Xeri Coffee Scale - Load Cell based Coffee Scale using TTGO Display (240x135) and Sparkfun 500g Load Cell and HX711 Breakout

  Prototype.
  Features include (eventually):
  - Mode 1: Auto tare scale - when turned on AND when first weight detected - and display weight in grams (to one decimal point)
  - Mode 2: Auto tare scale (as Mode 1) and auto start timer when new weight is detected
  - Mode x
  
  
  The hardware is as follows:

  TTGO Display (ESP32 based module with integrated 240x135 display and charging circuitry for battery)
  Sparkfun Load Cell (500g) 
  SparkFun HX711 ADC
  LiPo for battery (uses TTGO Display onboard charging)
  Custom push buttons (too difficult to integrate the onboard TTGO onces in the enclosure (maybe for another version)
  
 *************************************************************
  CONNECTIONS:
    TTGO      <-> HX711
      PIN 26  <->   DATA (dout)
      PIN 27  <->   CLK (sck)
      3.3V    <->   VDD (reference voltage)
      5v      <->   VCC (power for breakout board)

    TTGO      <-> Buttons
      PIN 35        Onboard Button
      PIN 0         Onboard Button
      PIN 25  <->   Custom Button 1
      PIN 33  <->   Custom Button 2
      
  BLYNK:
    V0  xxx

  OTHER:
    TFT Pins has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
      #define TFT_MOSI            19
      #define TFT_SCLK            18
      #define TFT_CS              5
      #define TFT_DC              16
      #define TFT_RST             23
      #define TFT_BL              4   // Display backlight control pin

 *************************************************************
  CHANGE LOG:
      16 FEB 2021   Working version. Small issue with not activating weight when slow flow (eg when using single basket)
      17 MAR 2020   Reset calibration factor using standard weights.
                    Updated the logic that refreshes the display to use an array of previous readings and higher sensitivity seems to work ok
 
  NOTES:
  Please be sure to select the ESP Arduino/ESP Dev board (in Sketchbook) in the Tools -> Board menu!
  Port should be /dev/cu.SLAB_USBtoUART or /dev/cu.usbserial-NNN on latest SiLabs v6 driver (make sure that the SiLabs driver is working...finiky on OSX)

  Change WiFi ssid, pass, and Blynk auth token to run :)

  REFERENCES:
  
    List of all U8g2 fonts:       https://github.com/olikraus/u8g2/wiki/fntlistall
    TFT_eSPI library:             https://github.com/Bodmer/TFT_eSPI
    U8g2_for_TFT_eSPI library:    https://github.com/Bodmer/U8g2_for_TFT_eSPI

    Sparkfun Load Cell Amp HX711: https://www.sparkfun.com/products/13879
    Sparkfun Mini Load Cell:      https://cdn.sparkfun.com/assets/9/9/a/f/3/TAL221.pdf


  AUTHOR: Xeri Systems 2021
  
 *************************************************************/

/********************
 * LIBRARY INCLUDES
 ********************/

#include <SPI.h>
#include "TFT_eSPI.h"
#include "U8g2_for_TFT_eSPI.h"

#include <HX711_ADC.h>  // Arduino library for HX711 24-Bit Analog-to-Digital Converter for Weight Scales, Olav Kallhovd sept2017
#include <EEPROM.h>

#include <Wire.h>
#include "Button2.h"
#include "esp_adc_cal.h"
#include "bmp.h"


// Setup pins:
const int HX711_dout = 26; //mcu > HX711 dout pin
const int HX711_sck = 27; //mcu > HX711 sck pin

//HX711 constructor:
HX711_ADC Scale(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
long t;


#define DEFAULT_TEXT_SIZE 4
#define SMALL_TEXT_SIZE 2

#define ADC_EN              14  //ADC_EN is the ADC detection enable port
#define ADC_PIN             34

#define BUTTON_1            25 // 35
#define BUTTON_2            33 // 0

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library
U8g2_for_TFT_eSPI Display;         // U8g2 font instance

Button2 LowerButton(BUTTON_1);
Button2 UpperButton(BUTTON_2);

char buff[512];
int vref = 1100;
int btnCick = false;
bool TimerStarted = false;            // Used to start and stop the timer function
float TimerStartMillis;                // Used to record the time the timer was triggered


/*************************************************
 * FINITE STATE MACHINE (FSA) TO CONTROL COOP DOOR
 *************************************************/
enum scaleStates {
  SCALE_TARED,                        // Scale has just been tared (zeroed). THis is the initial state after power on.
  SCALE_CALCULATING_WEIGHT,           // Scale is in an unstable state trying to settle on a final weight
  SCALE_WEIGHT_CALCULATED,            // Scale has completed its measurement and is in a steady state with a weight applied (may be zero)

  SCALE_ERROR_STATE,                  // Oops something went wrong (debugging state)
  
  SCALE_CONFIG_MODE                   // CONFIG MODE: Keeping simple for now
};

enum scaleEvents {
  WEIGHT_ADDED,                       // Weight is added to the scale
  WEIGHT_REMOVED,                     // Weight is removed from the scale (some or all)
  TARE_BUTTON_PRESSED,                // Tare scale (zero reading with our without any weight applied)
  TIMER_BUTTON_PRESSED,               // Start and stop timer
  MODE_BUTTON_PRESSED,                // Enter configuration mode
  SLEEP_BUTTON_PRESSED                // Deep sleep the scale
};

enum scaleStates ScaleState = SCALE_TARED; // The scale is tared at startup

/*************************************************
 * SETUP
 *************************************************/
void setup()
{
    Serial.begin(115200);
    delay(10);
    Serial.println("Starting Coffee Scale...");

    /*
    ADC_EN is the ADC detection enable port
    If the USB port is used for power supply, it is turned on by default.
    If it is powered by battery, it needs to be set to high level
    */
    pinMode(ADC_EN, OUTPUT);
    digitalWrite(ADC_EN, HIGH);

    // SETUP THE TFT DISPLAY AND SHOW THE START UP SCREEN

    tft.begin();
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
  
    Display.begin(tft);                     // connect u8g2 procedures to TFT_eSPI Display

    Display.setFontMode(1);                 // use u8g2 none transparent mode
    Display.setFontDirection(0);            // 0 = left to right (this is default)
    Display.setForegroundColor(TFT_WHITE);  // apply color

    showMessage("Please wait...");
  
    // SETUP FROM ADAFRUIT HXC11 CALIBRATRION EXAMPLE
    
    Scale.begin();
    float calibrationValue;                       // calibration value (see example file "Calibration.ino")
   // calibrationValue = 3100.30;                 // For 500g Sparkfun Loadcell. Uncomment this if you want to set the calibration value in the sketch
                                                  // CALIBRATED Feb-21 using Lelit Tamper at 330g  VALUE calibrationValue = 3062.57;
                                                  // CALIBRATED Mar-21 using standard weights (from eBay) VALUE calibrationValue = 3100.30;
                                                  // DEFAULT Calibration value was 2550.0 from HX711 library
    #if defined(ESP8266)|| defined(ESP32)
      EEPROM.begin(512);                        // uncomment this if you use ESP8266/ESP32 and want to fetch the calibration value from eeprom
    #endif
    EEPROM.get(calVal_eepromAdress, calibrationValue); // uncomment this if you want to fetch the calibration value from eeprom
    
    // Initialise physical buttons
    button_init();

    // Setup the ADC and bits used to calculate battery voltage
    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
    //Check type of calibration value used to characterize ADC
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
        vref = adc_chars.vref;
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
    } else {
        Serial.println("Default Vref: 1100mV");
    }

    // Start the scale and allow some time for it to stabilise
    
    long stabilizingtime = 2000;                  // preciscion right after power-up can be improved by adding a few seconds of stabilizing time
    boolean _tare = true;                         // set this to false if you don't want tare to be performed in the next step
    
    Scale.start(stabilizingtime, _tare);

    if (Scale.getTareTimeoutFlag() || Scale.getSignalTimeoutFlag()) {
      Serial.println("ERROR: Timeout, check MCU>HX711 wiring and pin designations");
      displayText("ERROR: Check HX711 wiring", SMALL_TEXT_SIZE);
      showMessage("ERROR: HX711 wiring");
      while (1);
    }
    else {
      Scale.setCalFactor(calibrationValue);    // set calibration value (float)
      Serial.println("Startup is complete");

      showMessage("Ready");
      showBatteryLevel();
      showWeight(0.0);
      
      //String txtMsg = String("Ready [") + String(getVoltage()) + String("V]");
      //displayText(txtMsg, SMALL_TEXT_SIZE);      
    }

showBatteryLevel();


// DEBUGGIN
//Scale.refreshDataSet();
//Serial.print("Scale SPS = "); Serial.println(Scale.getSPS());



} // End Setup()

/******************************
 *** MAIN LOOP() 
 ******************************/
void loop()
{
  static bool dataIsStable = false;
  static bool refreshTheDisplay = false;
  static float weightData = 0.0;
  //static float previousWeightData = 0.0;
  static float previousWeightData[5] = {0.00,0.00,0.00,0.00,0.00};
  static boolean newDataReady = 0;
  static String weightStr;  
  static long int currentMillis, previousMillis;

  // Check for button events
  button_loop();                                  

  // Check for new data/start next conversion:
  if (Scale.update()) 
    newDataReady = true;

  // Get smoothed value from the dataset. Sample rate set in config.h to 16 (default) samples and to ignore high and low outliers
  if (newDataReady) {                     

    // Shift up the readings in the stored values
    previousWeightData[4] = previousWeightData[3];
    previousWeightData[3] = previousWeightData[2];
    previousWeightData[2] = previousWeightData[1];
    previousWeightData[1] = previousWeightData[0];    
    previousWeightData[0] = weightData;
    weightData = Scale.getData();

    if (abs(weightData-previousWeightData[4]) > 0.01) {           // Only update the display while the values are changing
        dataIsStable = false;
        refreshTheDisplay = true;
    } else {
        dataIsStable = true;
        refreshTheDisplay = false;
    }

    if (!dataIsStable && refreshTheDisplay) {                  // Only update the display when the readings stabilise
        
       // if (weightData < 0) { // CHECK THIS WORKS!
       //     weightData = 0;
            //Scale.tare();
            //Scale.refreshDataSet(); // is this necessary after TARE? Note it takes time (not much but essentially re-fills 16 readings)
       // }       
  
        showWeight(weightData);

        refreshTheDisplay = false;
        
    }
  }

  // Update the timer if it has been started
  if (TimerStarted) {
    currentMillis = millis();
    if(currentMillis - previousMillis > 100) {
      showTimer((currentMillis-TimerStartMillis)/1000.0);
      previousMillis = currentMillis;
    }
  }


  // Process commands from serial terminal
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') Scale.tareNoDelay(); //tare
    else if (inByte == 'r') calibrate(); //calibrate
    else if (inByte == 'c') changeSavedCalFactor(); //edit calibration value manually
  }

} // End Loop()


/*************************************
 * FUNCTIONS
 *************************************/


void scaleStateController (scaleEvents event) {
/*  
 *  This function is called whenever an event is triggered from the user (or eventually the Blynk app)
 *  eg. a weight is added or removed from the scale or a button has been pressed
 *  Uses global ScaleState
 */

  String tempStr = "";

  switch (ScaleState) {

    case SCALE_TARED:
        if (event == WEIGHT_ADDED || event == WEIGHT_REMOVED) {
            ScaleState = SCALE_CALCULATING_WEIGHT;
        } 
        break;

    case SCALE_CALCULATING_WEIGHT:
        break;

    case SCALE_WEIGHT_CALCULATED:
        break;

    case SCALE_CONFIG_MODE:
        break;

    case SCALE_ERROR_STATE:
    default:
        // Oops something went wrong
        break;   
  }
  
} 

void displayText (String textToDisplay, int sizeOfText) {
/* 
* Display text to the display
*/

    tft.setTextSize(sizeOfText);
    tft.fillScreen(TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(textToDisplay,  tft.width() / 2, tft.height() / 2 );
}

void showMessage(String textToDisplay) {
/*
 * This function displays the line of messages at the top of the display
 */

    Display.setFont(u8g2_font_helvR14_tf);

    tft.setViewport(2,2,236,40);
    tft.frameViewport(TFT_DARKGREY,  -2); // TFT_DARKGREEN TFT_NAVY TFT_WHITE TFT_SKYBLUE TFT_DARKGREY TFT_LIGHTGREY
  
    tft.fillScreen(TFT_BLACK);
    Display.setFont(u8g2_font_helvR14_tf);  
    Display.setFont(u8g2_font_helvB18_tf);  
    Display.drawStr(10,30,textToDisplay.c_str());
  
    tft.resetViewport();
}

void showWeight(float weight) { 
/* 
 *  This function displays the weight of the object
 */
    tft.setViewport(2,50,190,80);

//    tft.frameViewport(TFT_RED,  -2);
    
    tft.fillScreen(TFT_BLACK);
     
    Display.setFont(u8g2_font_fub42_tf);            // Other fonts for testing: u8g2_font_7Segments_26x42_mn u8g2_font_freedoomr25_mn  u8g2_font_fur42_tf
   
    int strLen = Display.drawStr(10,70,String(weight,1).c_str());
  
    Display.setFont(u8g2_font_helvR14_tf);
    Display.drawStr(strLen+20,65,"g");

    tft.resetViewport();
    
}

void showTimer(float counterInSeconds) {
/*
 * This function displays the timer to count the shot
 */
    Display.setFont(u8g2_font_helvR24_tf);

    tft.setViewport(2,2,236,40);
    
    tft.fillScreen(TFT_BLACK);
       
    int strLen = Display.drawStr(16,32,String(counterInSeconds,1).c_str());
  
    Display.drawStr(strLen+20,32,"s");

    tft.resetViewport();
    Display.setFont(u8g2_font_helvR14_tf);
}

void showBatteryLevel() { 
/* 
 *  This function displays the weight of the object
 */
    float batteryLevel = getVoltage();

    
    tft.setViewport(190,50,50,80);
 //   tft.frameViewport(TFT_RED,  2);
    Display.setFont(u8g2_font_battery19_tn);

    if (batteryLevel < 4.0) {
        Display.setForegroundColor(TFT_RED);
        Display.drawGlyph(20,70,0x31);  // Battery glyph starts at 0x30 (empty) to 0x35 (full)    
    } else if (batteryLevel < 4.5) {
        Display.drawGlyph(20,70,0x33);
    } else if (batteryLevel < 4.75) {
        Display.drawGlyph(20,70,0x34);
    } else {
        Display.drawGlyph(20,70,0x35);
    }
    
    Display.setForegroundColor(TFT_WHITE);  // apply color

    tft.resetViewport();
    Display.setFont(u8g2_font_helvR14_tf);

}
void espDelay(int ms)
/* 
* For long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
*/
{
    esp_sleep_enable_timer_wakeup(ms * 1000);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    esp_light_sleep_start();
}

float getVoltage()
/* 
* This function is used to return the battery voltage. May be useful as a prompt to recharge the unit.
*/
{
    uint16_t v = analogRead(ADC_PIN);
    float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
    return battery_voltage;
}

void button_init()
/* 
* Initiate the button classes for each button attached to the TTGO Display
*/
{
    UpperButton.setPressedHandler([](Button2 & b) {
        btnCick = false;
        Serial.println("EVENT: Upper button pressed. ACTION: Tare");

        showMessage("Tare");
        
        Scale.tare();
        
        showMessage("-----");

       // Scale.refreshDataSet(); // is this necessary after TARE? Note it takes time (not much but essentially re-fills 16 readings..10SPS so about 1.8s)

        showMessage("Ready");
        showWeight(0.0);

    });

 /*   
    UpperButton.setLongClickHandler([](Button2 & b) {
        Serial.println("EVENT: Upper button long pressed. Action: Sleep.");
        btnCick = false;
        int r = digitalRead(TFT_BL);
        
        // displayText("Sleep...", DEFAULT_TEXT_SIZE);
        showMessage("Sleeping...");

        espDelay(6000);
        digitalWrite(TFT_BL, !r);

        tft.writecommand(TFT_DISPOFF);
        tft.writecommand(TFT_SLPIN);
        //After using light sleep, you need to disable timer wake, because here use external IO port to wake up
        esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
        // esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
        // esp_sleep_enable_ext0_wakeup(GPIO_NUM_35, 0);
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 0);
        delay(200);
        esp_deep_sleep_start();
    });
 */   
    LowerButton.setPressedHandler([](Button2 & b) {
        Serial.println("EVENT: Lower button pressed. Action: Timer.");
        btnCick = true;

        // displayText("[TBC Timer]", SMALL_TEXT_SIZE);
        if (TimerStarted) {
          TimerStarted = false;          
        } else {
          TimerStarted = true;
          TimerStartMillis = millis();
        }
//        showMessage("Timer TBC!");

    });


    LowerButton.setLongClickHandler([](Button2 & b) {
        Serial.println("EVENT: Lower button long pressed. Action: Reset timer.");
        btnCick = false;

        showMessage("Timer reset");
        delay(500);
        TimerStarted = false;
        showMessage("Ready");
    });

}

void button_loop()
/* 
* Button handler used to detect which button has pressed. Values for each button updated within Button2 class.
*/
{
    LowerButton.loop();
    UpperButton.loop();
}

void calibrate() {
/* 
* This example file shows how to calibrate the load cell and optionally store the calibration
* value in EEPROM, and also how to change the value manually.
* The result value can then later be included in your project sketch or fetched from EEPROM.
* To implement calibration in your project sketch the simplified procedure is as follow:
*   Scale.tare();
*   Place known mass
*   Scale.refreshDataSet();
*   float newCalibrationValue = Scale.getNewCalibration(known_mass);
*/

  Serial.println("****************************************************");
  Serial.println("Start calibration:");
  Serial.println("Place the load cell an a level stable surface.");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("Send 't' from serial monitor to set the tare offset.");

  boolean _resume = false;
  while (_resume == false) {
    Scale.update();
    if (Serial.available() > 0) {
      if (Serial.available() > 0) {
        char inByte = Serial.read();
        if (inByte == 't') Scale.tareNoDelay();
      }
    }
    if (Scale.getTareStatus() == true) {
      Serial.println("Tare complete");
      _resume = true;
    }
  }

  Serial.println("");
  Serial.println("Now, place your known mass on the loadcell.");
  Serial.println("Then send the weight of this mass (i.e. 100.0) from serial monitor.");

  float known_mass = 0;
  _resume = false;
  while (_resume == false) {
    Scale.update();
    if (Serial.available() > 0) {
      known_mass = Serial.parseFloat();
      if (known_mass != 0) {
        Serial.print("Known mass is: ");
        Serial.println(known_mass);
        _resume = true;
      }
    }
  }

  Scale.refreshDataSet(); //refresh the dataset to be sure that the known mass is measured correct
  float newCalibrationValue = Scale.getNewCalibration(known_mass); //get the new calibration value

  Serial.print("New calibration value has been set to: ");
  Serial.print(newCalibrationValue);
  Serial.println(", use this as calibration value (calFactor) in your project sketch.");
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(calVal_eepromAdress);
  Serial.println("? y/n");

  _resume = false;
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
        _resume = true;

      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }

  Serial.println("End calibration");
  Serial.println("****************************************************");
  Serial.println("To re-calibrate, send 'r' from serial monitor.");
  Serial.println("For manual edit of the calibration value, send 'c' from serial monitor.");
  Serial.println("****************************************************");
}

void changeSavedCalFactor() {
/* 
* This function is used to change the calibration factor
*/
  float oldCalibrationValue = Scale.getCalFactor();
  boolean _resume = false;
  Serial.println("****************************************************");
  Serial.print("Current value is: ");
  Serial.println(oldCalibrationValue);
  Serial.println("Now, send the new value from serial monitor, i.e. 696.0");
  float newCalibrationValue;
  while (_resume == false) {
    if (Serial.available() > 0) {
      newCalibrationValue = Serial.parseFloat();
      if (newCalibrationValue != 0) {
        Serial.print("New calibration value is: ");
        Serial.println(newCalibrationValue);
        Scale.setCalFactor(newCalibrationValue);
        _resume = true;
      }
    }
  }
  _resume = false;
  Serial.print("Save this value to EEPROM adress ");
  Serial.print(calVal_eepromAdress);
  Serial.println("? y/n");
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
        _resume = true;
      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }
  Serial.println("End change calibration value");
  Serial.println("****************************************************");
}
