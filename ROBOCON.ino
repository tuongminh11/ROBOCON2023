
#include <PS2X_lib.h>
#include <Adafruit_PCF8574.h>
#include <SoftwareSerial.h>
#include <Adafruit_GFX.h>       // include Adafruit graphics library
#include <Adafruit_ST7789.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "roboconIcon.h"

#define HOVER_SERIAL_BAUD 115200 // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD 115200       // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME 0xABCD       // [-] Start frme definition for reliable serial communication
#define SPEED_MAX 1000      // [-] Maximum speed
#define SPEED_MIN 60      // [-] Maximum speed
#define SPEED_STEP 20            // [-] Speed step
#define STEER_MAX 100      // [-] Maximum steer
#define STEER_MIN 10      // [-] Maximum steer
#define STEER_STEP 10            // [-] Steerstep

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

SerialCommand Command;

SerialFeedback Feedback;
SerialFeedback NewFeedback;

uint8_t idx = 0;        // Index for new data pointer
uint16_t bufStartFrame; // Buffer Start Frame
byte *p;                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

int16_t setSpeed;
int16_t setSteer;
uint16_t maxSpeed;
uint16_t maxSteer;


#define PS2_DAT 34
#define PS2_CMD 33
#define PS2_CLK 13

#define PS2_ATT_R 4
#define PS2_ATT_W 12

#define pressures false
#define rumble false

#define TFT_CS    5    
#define TFT_RST   15    
#define TFT_DC    14     
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

const uint8_t addressPCF = 0x20;  //
// const uint8_t addressPCF2 = 0x00; //

const uint8_t ESP_IRQ = 27;  //, ESP_IRQ2 = 11; // ngắt của readsensor
Adafruit_PCF8574 pcf1;//, pcf2;

struct Flag {
    bool ReadMotor, ReadHover, ReadSensor, ReadNUC, ReadController;
} fl = {0, 0, 1, 0, 0};

// biến trung gian
uint8_t sensorState[2];

struct NUC_DATA {
};

uint8_t controllerData0[4];
uint8_t controllerData1[4];

int error1 = -1;
int error2 = -1;
byte type_R = 0, type_W = 0;
byte vibrate = 0;
int tryNum = 1;

TaskHandle_t Task1;
TaskHandle_t Task2;

//HardwareSerial HoverSerial(1); // unknown

PS2X ps2x_R; // create PS2 1
PS2X ps2x_W; //PS2 2

Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;

struct Motor_Command{
	uint16_t Start ; //0xABCD
	uint8_t Motor; //stt motor i=1-6, config PID i+10
	uint8_t on; //on off
	uint8_t PID; //
	int16_t Value;//9999
	uint8_t Checksum;
};
Motor_Command command;
uint8_t motorIN; //stt motor
uint8_t enIN; //on off
uint8_t pidIN; //
int16_t speedIN;//9999
uint16_t SpUp = 100; //speed Up
uint16_t SpRt = 100; //speed Rotate
uint16_t SpFi = 1000;//speed fire
uint16_t SpPu = 100; //speed pull
uint8_t gear = 0;

void IRAM_ATTR readSensor()
{
    //11111111
    // sensorState[1] = pcf2.digitalReadByte();
    fl.ReadSensor = 1;
}

void setup()
{
    Serial.begin(115200);
    // create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    // xTaskCreatePinnedToCore(
    //     Task1code, /* Task function. */
    //     "Task1",   /* name of task. */
    //     10000,     /* Stack size of task */
    //     NULL,      /* parameter of the task */
    //     1,         /* priority of the task */
    //     &Task1,    /* Task handle to keep track of created task */
    //     0);        /* pin task to core 0 */
    // delay(500);

    // // create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
    // xTaskCreatePinnedToCore(
    //     Task2code, /* Task function. */
    //     "Task2",   /* name of task. */
    //     10000,     /* Stack size of task */
    //     NULL,      /* parameter of the task */
    //     1,         /* priority of the task */
    //     &Task2,    /* Task handle to keep track of created task */
    //     1);        /* pin task to core 1 */
    // delay(500);

    // PCF8574
    if (!pcf1.begin(addressPCF, &Wire)) {
        Serial.println("Couldn't find PCF8574");
    }
    
    for (uint8_t p = 0; p < 8; p++) {
        pcf1.pinMode(p, INPUT_PULLUP);
        //pcf2.pinMode(p, INPUT_PULLUP);
    }
    pinMode(ESP_IRQ, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ESP_IRQ), readSensor, FALLING);
    // attachInterrupt(digitalPinToInterrupt(ESP_IRQ2), readSensor, CHANGE);

    // HOVERBOARD
    Serial.println("Hoverboard Serial v1.0");
    Serial1.begin(HOVER_SERIAL_BAUD, SERIAL_8N1, 25, 26);

    setSpeed = 0;
    setSteer = 0;
    maxSpeed = SPEED_MIN;
    maxSteer = STEER_MIN;
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
    tft.init(320, 240); // chả biết này hàm gì, chắc là khởi tạo
    tft.fillScreen(ST77XX_BLACK);
    tft.setRotation(3);
    tft.drawRGBBitmap(270, 0, robocon2023, 50, 50);
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
    tft.println("Sp Up: ");         // tft.println(a.acceleration.x);
    tft.setCursor(2, 150);
    tft.println("Sp Rt: ");         // tft.println(a.acceleration.y);
    tft.setCursor(2, 170);
    tft.println("Sp Fi: "); // tft.println(g.gyro.x);
    tft.setCursor(2, 190);
    tft.println("Sp Pu: "); // tft.println(g.gyro.y);
    tft.setCursor(2, 210);
    tft.println("Gear : ");      // tft.println(temp.temperature);

    //mpu
    mpu.begin();

    //stm32
    Serial2.begin(9600);
    speedIN = 0;
    pidIN = 1;
    enIN = 1;
    motorIN = 1;
}

void loadToMB(uint8_t motor, uint8_t EN, uint8_t pidEN, int16_t speed) {
    command.Start = 0xABCD;
    command.Motor = motor;
    command.on = EN;
    command.PID = pidEN;
    command.Value = speed;
    command.Checksum = 16;
    Serial2.write((uint8_t *)&command, sizeof(command));
}

bool pad = 0;
void pressTypeButton(PS2X ps2x, byte type)
{

    // if (ps2x.Button(PSB_PAD_RIGHT)) {
    //     setSpeed = 0;
    //     setSteer += 10;
    //     if(setSteer >= maxSteer) { setSteer = maxSteer;}
    //     Serial.println("1");
    //     pad = 1;
    // }
    // if (ps2x.Button(PSB_PAD_UP)) {
    //     setSteer = 0;
    //     setSpeed += 30;
    //     if(setSpeed >= maxSpeed) { setSpeed = maxSpeed;};
    //     Serial.println("2");
    //     pad = 1;
    // }
    // if (ps2x.Button(PSB_PAD_DOWN)) {
    //     setSpeed -= 30;
    //     if(setSpeed <= -maxSpeed) { setSpeed =  -maxSpeed;};
    //     setSteer = 0;//MAXSTEER;
    //     Serial.println("3");
    //     pad = 1;
    // }
    // if (ps2x.Button(PSB_PAD_LEFT)) {
    //     setSpeed = 0;
    //     setSteer -= 10;
    //     if(setSteer <= - maxSteer) { setSteer = -maxSteer;}
    //     Serial.println("4");
    //     pad = 1;
    // }

    if (ps2x.Button(PSB_L2)) {
        if (!(sensorState[0] || 0b1111110))
            loadToMB(1, 1, 1, 0);
        else 
            loadToMB(1, 1, 1, -SpRt);
    }
    else loadToMB(1, 0, 1, 0);
    if (ps2x.Button(PSB_R2)) {
        if(!(sensorState[0] || 0b11111011))
            loadToMB(2, 1, 1, 0);
        else
            loadToMB(2, 1, 1, -SpUp);
    }
    else loadToMB(2, 0, 1, 0);
    if (ps2x.Button(PSB_L1)) {
        if(!(sensorState[0] || 0b11111101)) 
            loadToMB(1, 1, 1, 0);
        else 
            loadToMB(1, 1, 1, SpRt);       
    }
    else loadToMB(1, 0, 1, 0);
    if (ps2x.Button(PSB_R1)) {
        if(!(sensorState[0] || 0b11110111))
            loadToMB(2, 1, 1, 0);
        else
            loadToMB(2, 1, 1, SpUp);
    }
    else loadToMB(2, 0, 1, 0);

    if (ps2x.NewButtonState()) {
        if (ps2x.Button(PSB_R3)) {
            //Code here 
            gear++;
            gear= gear%5;
            loadToMB(3, 1, 1, SpFi/4*gear);
        }
        if (ps2x.Button(PSB_PAD_RIGHT)) {
            if(maxSteer + STEER_STEP <= STEER_MAX) {
                maxSteer += STEER_STEP;
            }
            else    maxSteer = STEER_MAX;
        }
        if (ps2x.Button(PSB_PAD_UP)) {
            if(maxSpeed + SPEED_STEP <= SPEED_MAX) {
                maxSpeed += SPEED_STEP;
            }
            else    maxSpeed = SPEED_MAX;
        }
        if (ps2x.Button(PSB_PAD_DOWN)) {
            if(maxSpeed - SPEED_STEP >= SPEED_MIN) {
                maxSpeed -= SPEED_STEP;
            }
            else    maxSpeed = SPEED_MIN;
        }
        if (ps2x.Button(PSB_PAD_LEFT)) {
            if(maxSteer - STEER_STEP >= STEER_MIN) {
                maxSteer -= STEER_STEP;
            }
            else    maxSteer = STEER_MIN;
        }
        if (ps2x.Button(PSB_TRIANGLE)){
            if(SpFi + 100 <= 9999) {
                SpFi += 100;
            }
            else SpFi = 9999;
        }
        if (ps2x.Button(PSB_CIRCLE)) {
            Serial.println("10");
        }
        if (ps2x.Button(PSB_CROSS)) {
            if(SpFi - 100 >= -9999) {
                SpFi -= 100;
            }
            else  SpFi = -9999;
        }
        if (ps2x.Button(PSB_SQUARE)) {
            Serial.println("12");
        }
    }
}

//

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

void readHover()
{
    // Check for new data availability in the Serial buffer
    // if (HoverSerial.available())
    // {
    //     incomingByte = HoverSerial.read();                                  // Read the incoming byte
    //     bufStartFrame = ((uint16_t)(incomingByte) << 8) | incomingBytePrev; // Construct the start frame
    // }
    // else
    // {
    //     return;
    // }

// If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
    Serial.print(incomingByte);
    return;
#endif

    // Copy received data
    if (bufStartFrame == START_FRAME)
    { // Initialize if new data is detected
        p = (byte *)&NewFeedback;
        *p++ = incomingBytePrev;
        *p++ = incomingByte;
        idx = 2;
    }
    else if (idx >= 2 && idx < sizeof(SerialFeedback))
    { // Save the new received data
        *p++ = incomingByte;
        idx++;
    }

    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback))
    {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum)
        {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

            // Print data to built-in Serial
            Serial.print("1: ");
            Serial.print(Feedback.cmd1);
            Serial.print(" 2: ");
            Serial.print(Feedback.cmd2);
            Serial.print(" 3: ");
            Serial.print(Feedback.speedR_meas);
            Serial.print(" 4: ");
            Serial.print(Feedback.speedL_meas);
            Serial.print(" 5: ");
            Serial.print(Feedback.batVoltage);
            Serial.print(" 6: ");
            Serial.print(Feedback.boardTemp);
            Serial.print(" 7: ");
            Serial.println(Feedback.cmdLed);
        }
        else
        {
            Serial.println("Non-valid data skipped");
        }
        idx = 0; // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

void loadToNUC()
{
}

void readNUC()
{
}

void processHoverboard(uint8_t data[4])
{
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

void loadToMotor()
{
}

void readMotor()
{
}

void lcd() {
    tft.setTextColor(ST77XX_WHITE);
    tft.fillRect(2, 2, 50, 20, ST77XX_BLACK);
    tft.setCursor(2, 2);
    tft.print(error1);
    tft.fillRect(20, 2, 50, 20, ST77XX_BLACK);
    tft.setCursor(20, 2);
    tft.print(error2);
    tft.fillRect(40, 2, 50, 20, ST77XX_BLACK);
    tft.setCursor(40, 2);
    tft.print(type_R);
    tft.fillRect(60, 2, 50, 20, ST77XX_BLACK);
    tft.setCursor(60, 2);
    tft.println(type_W);
    //data
    tft.fillRect(70, 30, 50, 20, ST77XX_BLACK);
    tft.setCursor(70, 30);
    tft.print(setSpeed);//speed
    tft.fillRect(70, 50, 50, 20, ST77XX_BLACK);
    tft.setCursor(70, 50);
    tft.print(setSteer);//steer
    tft.fillRect(70, 70, 50, 20, ST77XX_BLACK);
    tft.setCursor(70, 70);
    tft.print(maxSpeed);//mspeed
    tft.fillRect(70, 90, 50, 20, ST77XX_BLACK);
    tft.setCursor(70, 90);
    tft.print(maxSteer);//msteer
    tft.setCursor(2, 110);
    tft.fillRect(2, 110, 120, 20, ST77XX_BLACK);
    tft.print(sensorState[0], BIN);
    tft.fillRect(70, 130, 50, 20, ST77XX_BLACK);
    tft.setCursor(70, 130);
    tft.print(SpUp);//goc x
    tft.fillRect(70, 150, 50, 20, ST77XX_BLACK);
    tft.setCursor(70, 150);
    tft.print(SpRt);//goc y
    tft.fillRect(70, 170, 50, 20, ST77XX_BLACK);
    tft.setCursor(70,170);
    tft.print(SpFi);//gia toc x
    tft.fillRect(70, 190, 50, 20, ST77XX_BLACK);
    tft.setCursor(70,190);
    tft.print(SpPu);//gia toc y
    tft.fillRect(700, 210, 50, 20, ST77XX_BLACK);
    tft.setCursor(70,210);
    tft.print(gear);//nhiet do
}

// Task1code:
// void Task1code(void *pvParameters)
// {
//     while (fl.ReadSensor * fl.ReadHover * fl.ReadMotor)
//     {
//         loadToLCD();
//         loadToNUC();
//         fl.ReadSensor = 0;
//         fl.ReadHover = 0;
//         fl.ReadMotor = 0;
//     }
//     readController();
//     Serial.print("controller 1 ");
//     for (int i = 0; i < 4; i++){ 
//         Serial.print(controllerData0[i]);
//         Serial.print(',');
//     }
//     Serial.print("controller 2 ");
//     for (int i = 0; i < 4; i++){ 
//         Serial.print(controllerData1[i]);
//         Serial.print(',');
//     }
//     while (fl.ReadNUC)
//     {
//         readNUC();
//     }
// }

// Task2code:
// void Task2code(void *pvParameters)
// {
//     readSensor();
//     fl.ReadSensor = 1;
//     // loadToHover();
//     // readHover();
//     fl.ReadHover = 1;
//     loadToMotor();
//     readMotor();
//     fl.ReadMotor = 1;
// }

unsigned long last = 0;
unsigned long lastOfLCD = 0;

void loop() {
    mpu.getEvent(&a, &g, &temp);

    // while(Serial.available() > 0){
    //     String command = Serial.readString();
    //     String funccode = command.substring(0,command.indexOf(':'));
    //     String send_value = command.substring(command.indexOf(':')+1);
    // }

    if(type_R == 1 || type_W == 1){
        pad = 0;
        ps2x_R.read_gamepad(false, vibrate);
        ps2x_W.read_gamepad(false, vibrate);
        readController();
        if(type_R == 1) {
            pressTypeButton(ps2x_R, type_R);
            // if(ps2x_R.Button(PSB_L1) || ps2x_R.Button(PSB_R1)) { //print joystick
            // Serial.println();
            // Serial.print("controller 1 ");
            // for (int i = 0; i < 4; i++){ 
            //     Serial.print(controllerData0[i]);
            //     Serial.print(',');
            // }
            // }
        }
        if(type_W == 1){
            pressTypeButton(ps2x_W, type_W);
        // if(ps2x_W.Button(PSB_L1) || ps2x_W.Button(PSB_R1)) { //print joystick
        // Serial.println();
        // Serial.print("controller 2 ");
        // for (int i = 0; i < 4; i++){ 
        //     Serial.print(controllerData1[i]);
        //     Serial.print(',');
        // }
        // }
    }
    }
    unsigned long now = millis();
    if(pad == 0) processHoverboard(controllerData0);
    if (now - last >= 100)
    {   
        last = now;    
        loadToHover(setSpeed, setSteer);
    }
    if (millis() - lastOfLCD >= 100){
        lastOfLCD = millis();
        lcd();
    }
    // while (fl.ReadNUC)
    // {
    //     readNUC();
    // }

    if (fl.ReadSensor = 1) 
    sensorState[0] = pcf1.digitalReadByte();
    fl.ReadSensor = 0;
    // // loadToHover();
    readHover();
    // fl.ReadHover = 1;
    // loadToMotor();
    // readMotor();
    // fl.ReadMotor = 1;
}
