/*
OpenTherm Master Communication Example
By: Ihor Melnyk
Date: January 19th, 2018

Uses the OpenTherm library to get/set boiler status and water temperature
Open serial monitor at 115200 baud to see output.

Hardware Connections (OpenTherm Adapter (http://ihormelnyk.com/pages/OpenTherm) to Arduino/ESP8266):
-OT1/OT2 = Boiler X1/X2
-VCC = 5V or 3.3V
-GND = GND
-IN  = Arduino (3) / ESP8266 (5) Output Pin
-OUT = Arduino (2) / ESP8266 (4) Input Pin

Controller(Arduino/ESP8266) input pin should support interrupts.
Arduino digital pins usable for interrupts: Uno, Nano, Mini: 2,3; Mega: 2, 3, 18, 19, 20, 21
ESP8266: Interrupts may be attached to any GPIO pin except GPIO16,
but since GPIO6-GPIO11 are typically used to interface with the flash memory ICs on most esp8266 modules, applying interrupts to these pins are likely to cause problems
SSD1306 SPI

#define OLED_MOSI   PB15 //9
#define OLED_CLK    PB13 //10
#define OLED_DC     PA10 //11
#define OLED_CS     PB12 //12
#define OLED_RESET  PA9  //13
*/

#include <Arduino.h>
#include <OpenTherm.h>

//#include <SPI.h>
//#include <Wire.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>

#define inPin    PB13 // for STM32
#define outPin   PB12 // for STM32
#define heatOn   PB14 // for STM32
#define waterOn  PB15 // for STM32

//#define SCREEN_WIDTH 128 // OLED display width, in pixels
//#define SCREEN_HEIGHT 64 // OLED display height, in pixels

////#define SCREEN_WIDTH 128 // OLED display width, in pixels
////#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)  PB7, PB6
////#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//const int inPin = 2; //for Arduino, 4 for ESP8266 
//const int outPin = 3; //for Arduino, 5 for ESP8266 
OpenTherm ot(inPin, outPin);
   int uvrAnalogPin = PA0; //connect to 0-10V from UVR1611. Use 68K and 33K Spannungsteiler 10V -> 3.3V
   int tempSetInt = 0;
   int tempSetDHWInt = 0;
   int oldtempSetInt = 0;
   int heatOnVal;
   int waterOnVal;
   bool enableCentralHeating = false;
   bool enableHotWater = false;
   bool enableCooling = false;
   unsigned long previousMillis = 0;        // will store last time LED was updated
   // constants won't change:
   const long interval = 800;           // interval at which to blink (milliseconds)

void handleInterrupt() {
    ot.handleInterrupt();
}


//void ICACHE_RAM_ATTR handleInterrupt() {
//    ot.handleInterrupt();
//}

void setup()
{
    Serial.begin(115200);
    Serial.println("Start");
//     pinMode(inPin,INPUT);                      //Sets pin PA12 as input
//     pinMode(outPin,OUTPUT);                    //Sets pin PA13 as output
     pinMode(heatOn,INPUT_PULLUP);                     //Sets pin PA14 as output
     pinMode(waterOn,INPUT_PULLUP);                    //Sets pin PA15 as output
     digitalWrite(heatOn, HIGH);       // turn on pullup resistors
     digitalWrite(waterOn, HIGH);       // turn on pullup resistors
//
//  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
//    Serial.println(F("SSD1306 allocation failed"));
//    for(;;); // Don't proceed, loop forever
//  }
//  display.clearDisplay();
//  display.setTextSize(1);             // Normal 1:1 pixel scale
//  display.setTextColor(WHITE);        // Draw white text
//  display.setCursor(0,0);             // Start at top-left corner
  Serial.println(F("OpenTherm   Interface"));
  Serial.println(F("UVR16x -> ATAG i28ECZ"));
  Serial.println(F("0-10V  -> 0-100C"));
  Serial.println(F("Startup......"));
  
//  display.println(F("OpenTherm   Interface"));
//  display.println(F("UVR16x -> ATAG i28ECZ"));
//  display.println(F("0-10V  -> 0-100C"));
//  display.println(F("Startup......"));
//  display.display();
//  delay(5000); // wait for 5 seconds
  ot.begin(handleInterrupt);
}

void loop()
{
    //Read Temperature Boiler Temperature vom UVR1611/UVR2 0-10V = 0-100C  Use 68K and 33K Spannungsteiler
    //10V = 3,267V
    // VOLTAGE / STEP = REFERENCE VOLTAGE / 4096 = (3.3/4096= 8.056mV) per unit.
    // INPUT VOLTAGE = (ADC Value / ADC Resolution) * Reference Voltage
    int val = analogRead(uvrAnalogPin);    // read the ADC value from pin PA0
    //Voltage = (ADC Value / ADC Resolution) * Reference Voltage
    float voltage = ((((float(val)/4096) * 3.3)*3.267)*10); //formular to convert the ADC value to voltage

    tempSetInt = abs(voltage); // runde float to int
  //  Serial.println(highByte(tempSetInt));
   // Serial.println(lowByte(tempSetInt));
    //Set/Get Boiler Status
 //  Serial.println(millis());
   heatOnVal = digitalRead(heatOn);
   waterOnVal = digitalRead(waterOn);
  if (heatOnVal == false){
         enableCentralHeating = true;
         Serial.println(F("enableCentralHeating ON"));
    }
    if (heatOnVal == true){
         enableCentralHeating = false;
         Serial.println(F("enableCentralHeating OFF"));
    }
     if (waterOnVal == false){
         enableHotWater = true;
         //tempSetDHWInt = 45;
         Serial.println(F("enableHotWater ON"));
    }
   if (waterOnVal == true){
         enableHotWater = false;
       //  tempSetDHWInt = 0;
          Serial.println(F("enableHotWater OFF"));
    }
    enableCooling = false;
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
            previousMillis = currentMillis;
             // Serial.println(heatOnVal);

            unsigned long response = ot.setBoilerStatus(enableCentralHeating, enableHotWater, enableCooling);
            OpenThermResponseStatus responseStatus = ot.getLastResponseStatus();
//            display.clearDisplay();             //clear Dsiplay Buffer
//            display.setTextSize(1);             // Normal 1:1 pixel scale
//            display.setTextColor(WHITE);        // Draw white text
        if (responseStatus == OpenThermResponseStatus::SUCCESS) {
            Serial.println("Central Heating: " + String(ot.isCentralHeatingActive(response) ? "on" : "off"));
            Serial.println("Hot Water: " + String(ot.isHotWaterActive(response) ? "on" : "off"));
            Serial.println("Flame: " + String(ot.isFlameOn(response) ? "on" : "off"));
            Serial.println("Diagnostic: " + String(ot.isDiagnostic(response) ? "on" : "off"));
            Serial.println("Colling: " + String(ot.isCoolingActive(response) ? "on" : "off"));
            
//            display.setCursor(0,0);             // Start at top-left corner
//            display.print("H: " + String(ot.isCentralHeatingActive(response) ? "on" : "off"));
//            display.setCursor(45,0);             // Start at row 1 45px from left
//            display.print("W: " + String(ot.isHotWaterActive(response) ? "on" : "off"));
//            display.setCursor(90,0);             // Start at row 1 90px from left
//            display.print("F: " + String(ot.isFlameOn(response) ? "on" : "off"));
        }
        if (responseStatus == OpenThermResponseStatus::NONE) {
            Serial.println("Error: OpenTherm is not initialized");
//            display.setCursor(0,0);             // Start at top-left corner
//            display.print("Err: OpenTherm not initd");
        }
        else if (responseStatus == OpenThermResponseStatus::INVALID) {
            Serial.println("Error: Invalid response " + String(response, HEX));
//            display.setCursor(0,0);             // Start at top-left corner
//            display.print("Err: " + String(response, HEX));
        }
        else if (responseStatus == OpenThermResponseStatus::TIMEOUT) {
            Serial.println("Error: Response timeout");
//            display.setCursor(0,0);             // Start at top-left corner
//            display.print("Err: Response timeout");
        }
        //Set Boiler Temperature to 64 degrees C
        // ot.setBoilerTemperature(64);
        //if (oldtempSetInt != tempSetInt && (enableHotWater || enableCentralHeating)){  //use this for final hardware
      //  if (oldtempSetInt != tempSetInt ){  // only for testing
       //     oldtempSetInt = tempSetInt;
            ot.setBoilerTemperature(tempSetInt);
            Serial.println("Boiler temperature is set to " + String(tempSetInt) + " degrees C");
         //   ot.setDHWTemperature(tempSetDHWInt);
         //   Serial.println("DHW temperature is set to " + String(tempSetDHWInt) + " degrees C");
            
//            display.setCursor(0,16);             // Start at left row 3
//            display.print("Boiler set " + String(tempSetInt) + "C");
       // }
       // else {
       //     Serial.println("Boiler temperature is set to " + String(tempSetInt) + " degrees C");
//            display.setCursor(0,16);             // Start at  left row 3
//            display.print("No Setpoint change");
       // }
            //Get Boiler Temperature
            float temperature = ot.getBoilerTemperature();
            Serial.println("Boiler temperature is " + String(temperature) + " degrees C");
             float watertemperature = ot.getDHWTemperature();
            Serial.println("DHW temperature is " + String(watertemperature) + " degrees C");
            float pressure = ot.getBoilerPressure();
            Serial.println("Boiler Pressure is " + String(pressure) + " Bar");
            float flowrate = ot.getDHWFlowRate();
            Serial.println("DHW Flowrate is " + String(flowrate) + " Flow");
            float returntemperatur = ot.getBoilerReturnTemperature();
            Serial.println("Return temperature is " + String(returntemperatur) + " degrees C");
            float chstarts = ot.getCHBurnerStarts();
            Serial.println("Ch Burner starts " + String(chstarts) + "");
            float chburnerhours = ot.getCHBurnerHours();
            Serial.println("DHW Burner hours " + String(chburnerhours) + "");
           // float dhwstarts = ot.getDHWBurnerStarts();
           // Serial.println("DHW Burner starts " + String(dhwstarts) + "");
            float dhwburnerhours = ot.getDHWBurnerHours();
            Serial.println("Ch Burner hours " + String(dhwburnerhours) + "");
         //   float dhwpumprhours = ot.getDHWPumpHours();
         //   Serial.println("DHW PUMP hours " + String(dhwpumprhours) + "");
         //   float chpumprhours = ot.getCHPumpHours();
         //   Serial.println("CH PUMP hours " + String(chpumprhours) + "");
           // float exhaust = ot.getBoilerExhaustTemperature();
           // Serial.println("exhaust Temp " + String(exhaust) + "");
            float diagnostic = ot.getOEMDiagnostic();
            Serial.println("OEM Diagnostic " + String(diagnostic) + "");






            
//            display.setCursor(0,24);             // Start at  left row 4
//            display.println("Boiler get " + String(temperature) + "C");
        if (enableHotWater){
//            display.setCursor(0,8);             // Start at left row 2
//            display.print("RelW: on");
        }else{
//            display.setCursor(0,8);             // Start at left row 2
//            display.print("RelW: off");
        }
        if (enableCentralHeating){
//            display.setCursor(70,8);             // Start at middle row 2
//            display.print("RelH: on");
        }else{
//            display.setCursor(70,8);             // Start at middle row 2
//            display.print("RelH: off");
        }
   //         display.display();
            Serial.println();
    }
    
}
