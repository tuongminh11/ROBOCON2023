#include <Arduino.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include "robocon2023.c"

#include <Arduino.h>
#include "SPI.h"
// #include "Adafruit_GFX.h"
// #include "Adafruit_ILI9341.h"
#define TFT_RST 2 // we use the seesaw for resetting to save a pin
#define TFT_CS 5
#define TFT_DC 22

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// void setup(void) {
//   tft.init(320, 240);           // Init ST7789 240x240
//   tft.invertDisplay(true);
//   tft.setTextWrap(true);
//   tft.fillScreen(ST77XX_BLACK);
//   tft.setCursor(26, 20);
//   tft.setTextColor(ST77XX_RED);
//   tft.setTextSize(3);
//   tft.println("RED!");
//   tft.setCursor(26, 60);
//   tft.setTextColor(ST77XX_BLUE);
//   tft.setTextSize(3);
//   tft.println("BLUE!");
//   tft.setCursor(26,100);
//   tft.drawRGBBitmap(190, 0, robocon2023, 50, 50);
//   // tft.setCursor(10, 150);
//   // tft.setTextColor(ST77XX_WHITE);
//   // tft.setTextSize(2);
//   // tft.println("meo meo meo tra lai tam tri toi day");
//     tft.setCursor(10,150);
//   tft.setTextColor(ST77XX_WHITE);
//   tft.setTextSize(2);
//   tft.println("Dem: ");

// }

// void loop() {

//   for(int i=0;i<10;i++){
//   tft.setCursor(100,150);
//  tft.fillRect(100, 150, 40, 40, ST77XX_BLACK);
//   tft.setTextColor(ST77XX_WHITE);
//   tft.setTextSize(2);
//   tft.print(i);
//   delay(2000);
//   tft.setCursor(100,150);

//   }
// }
// int check(int n){
// }
void setup()
{
  tft.init(320, 240); // chả biết này hàm gì, chắc là khởi tạo
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(3);
  tft.drawRGBBitmap(250, 0, robocon2023, 50, 50);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(2, 30);
  tft.print("Speed "); // tft.println(setSpeed);
  tft.setCursor(2, 50);
  tft.print("Steer "); // tft.println(setSteer);
  tft.setCursor(2, 70);
  tft.print("maxsp "); // tft.println(maxSpeed);
  tft.setCursor(2, 90);
  tft.print("maxst "); // tft.println(maxSteer);
  tft.setCursor(2, 110);
  // tft.println(sensorState[0], BIN);
  tft.setCursor(2, 130);
  tft.println("Goc x: ");         // tft.println(a.acceleration.x);
  tft.setCursor(2, 150);
  tft.println("Goc y: ");         // tft.println(a.acceleration.y);
  tft.setCursor(2, 170);
  tft.println("Gia toc goc x: "); // tft.println(g.gyro.x);
  tft.setCursor(2, 190);
  tft.println("Gia toc goc y: "); // tft.println(g.gyro.y);
  tft.setCursor(2, 210);
  tft.println("Nhiet do: ");      // tft.println(temp.temperature);
}
void loop()
{
  tft.setTextColor(ST77XX_WHITE);
  tft.fillRect(2, 2, 20, 20, ST77XX_BLACK);
  tft.setCursor(2, 2);
  tft.print(error1);
  tft.fillRect(20, 2, 20, 20, ST77XX_BLACK);
  tft.setCursor(20, 2);
  tft.print(error2);
  tft.fillRect(40, 2, 20, 20, ST77XX_BLACK);
  tft.setCursor(40, 2);
  tft.print(type_R);
  tft.fillRect(60, 2, 20, 20, ST77XX_BLACK);
  tft.setCursor(60, 2);
  tft.println(type_W);
//data
  tft.fillRect(70, 30, 20, 20, ST77XX_BLACK);
  tft.setCursor(70, 30);
  tft.print(setSpeed);//speed
  tft.fillRect(70, 50, 20, 20, ST77XX_BLACK);
  tft.setCursor(70, 50);
  tft.print(setSteer);//steer
  tft.fillRect(70, 70, 20, 20, ST77XX_BLACK);
  tft.setCursor(70, 70);
  tft.print(maxSpeed);//mspeed
  tft.fillRect(70, 90, 20, 20, ST77XX_BLACK);
  tft.setCursor(70, 90);
  tft.print(maxSteer);//msteer
  tft.setCursor(2, 110);
  tft.fillRect(2, 110, 20, 20, ST77XX_BLACK);
  tft.print(sensorState[0], BIN);
  tft.fillRect(70, 130, 20, 20, ST77XX_BLACK);
  tft.setCursor(70, 130);
  tft.print(a.acceleration.x);//goc x
  tft.fillRect(70, 150, 20, 20, ST77XX_BLACK);
  tft.setCursor(70, 150);
  tft.print(a.acceleration.y);//goc y
  tft.fillRect(170, 170, 20, 20, ST77XX_BLACK);
  tft.setCursor(170,170);
  tft.print(g.gyro.x);//gia toc x
  tft.fillRect(170, 190, 20, 20, ST77XX_BLACK);
  tft.setCursor(170,190);
  tft.print(g.gyro.y);//gia toc y
  tft.fillRect(110, 210, 20, 20, ST77XX_BLACK);
  tft.setCursor(110,210);
  tft.print(temp.temperature);//nhiet do
}
