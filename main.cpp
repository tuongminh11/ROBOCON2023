#include<Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include"robocon2023.c"

#include <Arduino.h>
#include "SPI.h"
// #include "Adafruit_GFX.h"
// #include "Adafruit_ILI9341.h"
#define TFT_RST    2    // we use the seesaw for resetting to save a pin
#define TFT_CS   5
#define TFT_DC   22

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);


float p = 3.1415926;

void setup(void) {

  tft.init(320, 240);           // Init ST7789 240x240
  tft.invertDisplay(true);
  tft.setTextWrap(true);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(26, 20);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(3);
  tft.println("RED!");
  tft.setCursor(26, 60);
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextSize(3);
  tft.println("BLUE!");
  tft.setCursor(26,100);
  tft.drawRGBBitmap(190, 0, robocon2023, 50, 50);
  tft.setCursor(10, 150);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.println("meo meo meo tra lai tam tri toi day");
  
}


void loop() {

  
  

}


