#include <PS2X_lib.h>
#include <Adafruit_PCF8574.h>
#include <SoftwareSerial.h>
#include <Adafruit_GFX.h>       // include Adafruit graphics library
#include <Adafruit_ST7789.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "roboconIcon.h"

//Hoverboard
#define HOVER_SERIAL_BAUD 115200 // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD 115200       // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME 0xABCD       // [-] Start frme definition for reliable serial communication

//PS2
#define PS2_DAT 34
#define PS2_CMD 33
#define PS2_CLK 13
#define PS2_ATT_R 4
#define PS2_ATT_W 12
#define pressures false
#define rumble false

//OLED
#define TFT_CS    5    
#define TFT_RST   15    
#define TFT_DC    14

//struct hoverboard
typedef struct {
    uint16_t start;
    int16_t steer;
    int16_t speed;
    uint16_t checksum;
} SerialCommand;

typedef struct {
    uint16_t start;
    int16_t cmd1;
    int16_t cmd2;
    int16_t speedR_meas;
    int16_t speedL_meas;
    int16_t batVoltage;
    int16_t boardTemp;
    uint16_t cmdLed;
    uint16_t checksum;
} SerialFeedback;

//struct mạch công suất
struct Motor_Command{
	uint16_t Start ; //0xABCD
	uint8_t Motor; //stt motor i=1-6, config PID i+10
	uint8_t On; //on off
	uint8_t PID; //a
  
	int16_t Value;//1000
	uint8_t Acc;
  uint8_t Checksum;
};

struct Motor_Feedback {
    uint16_t Start;
    int32_t encoder0;
    int32_t encoder1;
    int32_t encoder2;
    int32_t encoder3;
    int16_t speed0;
    int16_t speed1;
    int16_t speed2;
    int16_t speed3;
    uint8_t Checksum;
};

struct Flag {
    bool ReadMotor, ReadHover, ReadSensor, ReadNUC, ReadController, Relay;
} fl = {0, 0, 1, 0, 0, 0};

//Biến trung gian hoverboard
SerialCommand Command;

uint8_t idx = 0;        // Index for new data pointer
uint16_t bufStartFrame; // Buffer Start Frame
uint8_t *p;                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

int16_t setSpeed;
int16_t setSteer;
uint16_t maxSpeed;
uint16_t maxSteer;

uint16_t gearSpeed[4] = {100, 200, 250, 300};
uint16_t gearSteer[4] = {30, 40, 50, 60};

//object OLED
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

//biến PCF
const uint8_t addressPCF = 0x20;  //PCF trên
const uint8_t addressPCF2 = 0x21; //PCF dưới
const uint8_t ESP_IRQ = 27;  //, ESP_IRQ2 = 11; // ngắt của readsensor
Adafruit_PCF8574 pcf1, pcf2;
uint8_t sensorState[2];
uint8_t prvSensor[2];

//PS2
PS2X ps2x_R; // create PS2 1
PS2X ps2x_W; //PS2 2
uint8_t controllerData0[4];
uint8_t controllerData1[4];

int error1 = -1;
int error2 = -1;
byte type_R = 0, type_W = 0;
byte vibrate = 0;
int tryNum = 1;

//Dual Core
TaskHandle_t Task1;
TaskHandle_t Task2;

//Cảm biến 
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

//Biến mạch công suất
Motor_Command command;
Motor_Feedback feedback, newFeedback;
uint8_t motorIN; //stt motor
uint8_t enIN; //on off
uint8_t pidIN; //
int16_t speedIN;//1000
int16_t SpUp = 700; //speed Up
int16_t SpRt = 1000; //speed Rotate
int16_t SpFi = 0;//speed fire
int16_t SpPu = 500; //speed pull
uint8_t gear = 0;
uint8_t up;
uint8_t rt;

//Hàm ngắt
void IRAM_ATTR readSensor() {
    fl.ReadSensor = 1;
}

void loadToMB(uint8_t motor, uint8_t EN, uint8_t pidEN, int16_t speed, uint8_t acc = 20) {
    command.Start = 0xABCD;
    command.Motor = motor;
    command.On = EN;
    command.PID = pidEN;
    command.Value = speed;
    command.Acc = acc;

    uint8_t buffer[sizeof(command)];
    memcpy(buffer, (uint8_t*) &command, sizeof(command));
    uint8_t checksum = 0;
    for(int i= 0; i < sizeof(command) - 1; i++){
      checksum += buffer[i];
    }
    buffer[sizeof(command) - 1] = ~checksum + 1;
    Serial2.write(buffer, sizeof(command));
}

void setup()
{
    Serial.begin(115200);

    // PCF8574
    if (!pcf1.begin(addressPCF, &Wire)) {
        //Serial.println("Couldn't find PCF8574 1");
    }
    if (!pcf2.begin(addressPCF2, &Wire)) {
        //Serial.println("Couldn't find PCF8574 2");
    }
    for (uint8_t p = 0; p < 8; p++) {
        pcf1.pinMode(p, INPUT_PULLUP);
        pcf2.pinMode(p, OUTPUT);
        pcf2.digitalWrite(p, HIGH);
    }
    pinMode(ESP_IRQ, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ESP_IRQ), readSensor, FALLING);

    // HOVERBOARD
    Serial1.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, 25, 26);
    setSpeed = 0;
    setSteer = 0;
    maxSpeed = gearSpeed[0];
    maxSteer = gearSteer[0];

    // Controller
    delay(300);
    error1 = ps2x_R.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT_R, PS2_DAT, pressures, rumble);
    Serial.println(error1);
    delay(300);
    error2 = ps2x_W.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT_W, PS2_DAT, pressures, rumble);
    Serial.println(error2);

    type_R = ps2x_R.readType();
    Serial.println(type_R);
    type_W = ps2x_W.readType();
    Serial.println(type_W);

    //tft
    tft.init(320, 240); 
    tft.fillScreen(ST77XX_WHITE);
    tft.setRotation(3);
    tft.drawRGBBitmap(250, 0, robocon2023, 50, 50);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_BLACK);
    tft.setCursor(2, 30);
    tft.print("Speed "); 
    tft.setCursor(2, 50);
    tft.print("Steer ");
    tft.setCursor(2, 70);
    tft.print("maxsp "); 
    tft.setCursor(2, 90);
    tft.print("maxst "); 
    tft.setCursor(2, 110);
    tft.print(sensorState[0], BIN);
    tft.setCursor(2, 130);
    tft.setCursor(2, 150);
    tft.println("Sp Rt: ");         
    tft.setCursor(2, 170);
    tft.println("Sp Fi: "); 
    tft.setCursor(2, 190);
    tft.println("Sp Up: "); 
    tft.setCursor(2, 210);
    tft.println("Gear : ");    
    tft.setCursor(2, 2);
    tft.print(error1);
    tft.setCursor(20, 2);
    tft.print(error2);
    tft.setCursor(40, 2);
    tft.print(type_R);
    tft.setCursor(60, 2);
    tft.println(type_W);
    //mpu
    mpu.begin();

    //stm32
    Serial2.begin(115200);
    speedIN = 0;
    pidIN = 1;
    enIN = 1;
    motorIN = 1;
    loadToMB(1, 0, 0, 0);
    loadToMB(2, 0, 0, 0);
    loadToMB(3, 0, 0, 0);
    loadToMB(4, 0, 0, 0);
    loadToMB(5, 0, 0, 0);
    loadToMB(6, 0, 0, 0);    
    xTaskCreatePinnedToCore(
                        Task1code,   /* Task function. */
                        "Task1",     /* name of task. */
                        10000,       /* Stack size of task */
                        NULL,        /* parameter of the task */
                        3,           /* priority of the task */
                        &Task1,      /* Task handle to keep track of created task */
                        0);          /* pin task to core 0 */                  
    delay(500);
    xTaskCreatePinnedToCore(
                        Task2code,   /* Task function. */
                        "Task2",     /* name of task. */
                        10000,       /* Stack size of task */
                        NULL,        /* parameter of the task */
                        1,           /* priority of the task */
                        &Task2,      /* Task handle to keep track of created task */
                        1);          /* pin task to core 1 */
    delay(500);
}

void readMB() {
    if(Serial2.available()) {
        incomingByte = Serial2.read();       
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;
    }
    else {
        return;
    }

    if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
        p       = (byte *)&newFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;	
    } else if (idx >= 2 && idx < sizeof(Motor_Feedback)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    }
    
    
    if (idx == sizeof(Motor_Feedback)) {
        uint8_t checksum  = 0;
        uint8_t* ptr = (uint8_t*) &newFeedback ;
        for(uint8_t i = 0; i < sizeof(Motor_Feedback) - 4; i++){
          checksum += *ptr++;
        }
        checksum = ~checksum + 1;
        if(checksum == newFeedback.Checksum) {
            memcpy(&feedback, &newFeedback, sizeof(Motor_Feedback));
            //  Serial.print("encoder 0: "); Serial.println(feedback.encoder0);
            //  Serial.print("encoder 1: "); Serial.println(feedback.encoder1);
            //  Serial.print("encoder 2: "); Serial.println(feedback.encoder2);
            //  Serial.print("encoder 3: "); Serial.println(feedback.encoder3);
            //  Serial.print("speed 0: "); Serial.println(feedback.speed0);
            //  Serial.print("speed 1: "); Serial.println(feedback.speed1);
            //  Serial.print("speed 2: "); Serial.println(feedback.speed2);
            //  Serial.print("speed 3: "); Serial.println(feedback.speed3);
            //  Serial.print("checksum: "); Serial.println(checksum);
        }
        idx = 0;
    }
    incomingBytePrev = incomingByte;
}

bool pad = 0;

void pressTypeButton(PS2X ps2x, byte type) {    
    if (ps2x.Button(PSB_PAD_RIGHT)) {
        setSpeed = 0;
        setSteer += 10;
        if(setSteer >= maxSteer) { setSteer = maxSteer;}
        //Serial.println("pad_right");
        pad = 1;
    }
    if (ps2x.Button(PSB_PAD_UP)) {
        setSteer = 0;
        setSpeed += 30;
        if(setSpeed >= maxSpeed) { setSpeed = maxSpeed;};
        //Serial.println("pad_up");
        pad = 1;
    }
    if (ps2x.Button(PSB_PAD_DOWN)) {
        setSpeed -= 30;
        if(setSpeed <= -maxSpeed) { setSpeed =  -maxSpeed;};
        setSteer = 0;//MAXSTEER;
        //Serial.println("pad_down");
        pad = 1;
    }
    if (ps2x.Button(PSB_PAD_LEFT)) {
        setSpeed = 0;
        setSteer -= 10;
        if(setSteer <= - maxSteer) { setSteer = -maxSteer;}
        //Serial.println("pad_left");
        pad = 1;
    }  
    
    if(ps2x.ButtonPressed(PSB_R2) && ((sensorState[0] & 0b00000100))) {
        loadToMB(1, 1, 0, -1000, 255);
        up = 0;
    }
    if(ps2x.ButtonReleased(PSB_R2)) {
        loadToMB(1, 0, 0, 0);
        up = 1;
    }
    if(ps2x.ButtonPressed(PSB_R1) && ((sensorState[0] & 0b00001000) && !(sensorState[0] & 0b01000000))) {
        loadToMB(1, 1, 0, SpUp, 255);
        up = 2;
    }
    if(ps2x.ButtonReleased(PSB_R1)) {
        loadToMB(1, 0, 0, 0);
        up = 1;
    }
    if(ps2x.ButtonPressed(PSB_L2) && ((sensorState[0] & 0b00000001))) {
        loadToMB(6, 1, 0, -SpRt, 255);
        rt = 0;
    }
    if(ps2x.ButtonReleased(PSB_L2)) {
        loadToMB(6, 0, 0, 0);
        rt = 1;
    }

    if(ps2x.ButtonPressed(PSB_L1) && ((sensorState[0] & 0b00000010))) {
        loadToMB(6, 1, 0, SpRt, 255);
        rt = 2;
    }    
    if(ps2x.ButtonReleased(PSB_L1)) {
        loadToMB(6, 0, 0, 0);
        rt = 1;
    }
    if (ps2x.ButtonPressed(PSB_TRIANGLE)){
        if(SpFi + 100 <= 1000) {
            SpFi += 100;
        }
        else SpFi = 1000;
        loadToMB(3, 1, 0, SpFi);
        loadToMB(4, 1, 0, SpFi);
        tft.fillRect(70, 170, 50, 20, ST77XX_WHITE);
        tft.setCursor(70,170);
        tft.print(SpFi);
    }
    if (ps2x.ButtonPressed(PSB_CROSS)) {
        //Serial.println("10");
        gear++;
        gear= gear%4;
        maxSpeed = gearSpeed[gear];
        maxSteer = gearSteer[gear];
        tft.fillRect(70, 70, 50, 20, ST77XX_WHITE);
        tft.setCursor(70, 70);
        tft.print(maxSpeed);//mspeed
        tft.fillRect(70, 90, 50, 20, ST77XX_WHITE);
        tft.setCursor(70, 90);
        tft.print(maxSteer);//msteer
        tft.fillRect(70, 210, 50, 20, ST77XX_WHITE);
        tft.setCursor(70, 210);
        tft.print(gear + 1);//nhiet do
    }
    if (ps2x.ButtonPressed(PSB_CIRCLE)) {
        if(SpFi - 100 >= -1000) {
            SpFi -= 100;
        }
        else  SpFi = -1000;
        loadToMB(3, 1, 0, SpFi);
        loadToMB(4, 1, 0, SpFi);
        tft.fillRect(70, 170, 50, 20, ST77XX_WHITE);
        tft.setCursor(70,170);
        tft.print(SpFi);
    }
    if (ps2x.ButtonPressed(PSB_SQUARE)) {
        //Serial.println("12");
        if(!(sensorState[0] & 0b00010000)) {
            setRelay(0, 1, 1);
        }
    }
}

void loadToHover(int16_t uSteer, int16_t uSpeed)
{
    // Create command
    Command.start = (uint16_t)START_FRAME;
    Command.steer = (int16_t)uSteer;
    Command.speed = (int16_t)uSpeed;
    Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);
    // Write to Serial
    Serial1.write((uint8_t *)&Command, sizeof(Command));
}

void processHoverboard(uint8_t data[4]) {
    uint8_t x = data[1];
    uint8_t y = data[0];
    setSpeed = map(y, 0, 255, maxSpeed, -maxSpeed);
    if(setSpeed == -1) setSpeed = 0;
    setSteer = map(x, 0, 255, -maxSteer, maxSteer);
}

void readController()
{
        controllerData0[0] = ps2x_R.Analog(PSS_LY);
        controllerData0[1] = ps2x_R.Analog(PSS_LX);
        controllerData0[2] = ps2x_R.Analog(PSS_RY);
        controllerData0[3] = ps2x_R.Analog(PSS_RX);
        
        controllerData1[0] = ps2x_W.Analog(PSS_LY);
        controllerData1[1] = ps2x_W.Analog(PSS_LX);
        controllerData1[2] = ps2x_W.Analog(PSS_RY);
        controllerData1[3] = ps2x_W.Analog(PSS_RX);
}

void setRelay(uint8_t pinA, uint8_t pinB, uint8_t St) {
    if(St == 1) {
        pcf2.digitalWrite(pinA, HIGH);
        pcf2.digitalWrite(pinB, LOW);
    }
    if(St == 2) {
        pcf2.digitalWrite(pinA, LOW);
        pcf2.digitalWrite(pinB, HIGH);
    }
    if(St == 0) {
        pcf2.digitalWrite(pinA, HIGH);
        pcf2.digitalWrite(pinB, HIGH);
    }
}

void lcd() {
    tft.setTextColor(ST77XX_BLACK);
    tft.fillRect(70, 30, 50, 20, ST77XX_WHITE);
    tft.setCursor(70, 30);
    tft.print(setSpeed);//speed
    tft.fillRect(70, 50, 50, 20, ST77XX_WHITE);
    tft.setCursor(70, 50);
    tft.print(setSteer);//steer
    tft.fillRect(70, 150, 50, 20, ST77XX_WHITE);
    tft.setCursor(70, 150);
    tft.print(rt);
    // tft.fillRect(70, 170, 50, 20, ST77XX_WHITE);
    // tft.setCursor(70,170);
    // tft.print(SpFi);
    tft.fillRect(70, 190, 50, 20, ST77XX_WHITE);
    tft.setCursor(70,190);
    tft.print(up);
    // tft.fillRect(70, 210, 50, 20, ST77XX_WHITE);
    // tft.setCursor(70,210);
    // tft.print(gear);//nhiet do
}

unsigned long last = 0;
unsigned long lastOfLCD = 0;
unsigned long delayForMotor = 0;
void loop() {

}

//Task1code
void Task1code( void * pvParameters ){
    for(;;){
    unsigned long time1 = millis();
    if(type_R == 1 || type_W == 1){
        pad = 0;
        ps2x_R.read_gamepad(false, vibrate);
        ps2x_W.read_gamepad(false, vibrate);
        readController();
        if(type_R == 1) {
            pressTypeButton(ps2x_R, type_R);
        }
        if(type_W == 1){
            pressTypeButton(ps2x_W, type_W);
        }
    }
    if(pad == 0) processHoverboard(controllerData0);
    unsigned long time2 = millis();
    //Serial.println(time2 - time1);
    }
}

//Task2code
void Task2code( void * pvParameters ){
    for(;;){
    mpu.getEvent(&a, &g, &temp);
    if (fl.ReadSensor == 1){
        prvSensor[0] = sensorState[0];
        sensorState[0] = pcf1.digitalReadByte();
        tft.setCursor(2, 110);
        tft.fillRect(2, 110, 120, 20, ST77XX_WHITE);
        tft.print(sensorState[0], BIN);
        //Serial.println(sensorState[0], BIN);
        if((prvSensor[0] & 0b00100000) && !(sensorState[0] & 0b00100000)) setRelay(0, 1, 2);
        if((prvSensor[0] & 0b00010000) && !(sensorState[0] & 0b00010000)) {
            setRelay(0, 1, 0);
        }
        if(!(prvSensor[0] & 0b01000000) && (sensorState[0] & 0b01000000)) loadToMB(1, 0, 0, 0);     //0 -> 1
        if((prvSensor[0] & 0b00000100) && !(sensorState[0] & 0b00000100)) loadToMB(1, 0, 0, 0);
        if((prvSensor[0] & 0b00001000) && !(sensorState[0] & 0b00001000)) loadToMB(1, 0, 0, 0);
        if((prvSensor[0] & 0b00000010) && !(sensorState[0] & 0b00000010)) loadToMB(6, 0, 0, 0);
        if((prvSensor[0] & 0b00000001) && !(sensorState[0] & 0b00000001)) loadToMB(6, 0, 0, 0);      
        fl.ReadSensor = 0;
        //Serial.println(sensorState[0], BIN);
    }
    
    unsigned long now = millis();
    
    if (now - last >= 100)
    {   
        last = now;    
        loadToHover(-setSpeed, setSteer);
        
    }
    readMB();
    if (millis() - lastOfLCD >= 100){
        lastOfLCD = millis();
        lcd();
    }
    }
}