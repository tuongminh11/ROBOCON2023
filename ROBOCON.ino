#include <SoftwareSerial.h>
#include <PS2X_lib.h>
#include <Adafruit_PCF8574.h>
#include <SoftwareSerial.h>
#include <Adafruit_GFX.h>       // include Adafruit graphics library
#include <Adafruit_ILI9341.h>   // include Adafruit ILI9341 TFT library 

#define HOVER_SERIAL_BAUD 115200 // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD 115200       // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME 0xABCD       // [-] Start frme definition for reliable serial communication
#define SPEED_MAX 1000      // [-] Maximum speed
#define SPEED_MIN 60      // [-] Maximum speed
#define SPEED_STEP 20            // [-] Speed step
#define STEER_MAX 100      // [-] Maximum steer
#define STEER_MIN 10      // [-] Maximum steer
#define STEER_STEP 10            // [-] Steerstep
//okokoko
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
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// const uint8_t addressPCF = 0x20;  //
// const uint8_t addressPCF2 = 0x00; //

const uint8_t ESP_IRQ = 27;  //, ESP_IRQ2 = 11; // ngắt của readsensor
// Adafruit_PCF8574 pcf1;//, pcf2;

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

void IRAM_ATTR readSensor()
{
    // sensorState[0] = pcf1.digitalReadByte();
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
    // if (!pcf1.begin(addressPCF, &Wire)) {
    //     Serial.println("Couldn't find PCF8574");
    // }
    
    // for (uint8_t p = 0; p < 8; p++) {
    //     pcf1.pinMode(p, INPUT_PULLUP);
    //     //pcf2.pinMode(p, INPUT_PULLUP);
    // }
    // pinMode(ESP_IRQ, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(ESP_IRQ), readSensor, FALLING);
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
    tft.begin();
}

void pressTypeButton(PS2X ps2x, byte type)
{

    if (ps2x.Button(PSB_PAD_RIGHT))
        Serial.println("1");
    if (ps2x.Button(PSB_PAD_UP))
        Serial.println("2");
    if (ps2x.Button(PSB_PAD_DOWN))
        Serial.println("3");
    if (ps2x.Button(PSB_PAD_LEFT))
        Serial.println("4");
    if (ps2x.NewButtonState()) {
        if (ps2x.Button(PSB_L2)) {
            Serial.println("5");
        }
        if (ps2x.Button(PSB_R2))
            Serial.println("6");
        if (ps2x.Button(PSB_L1))
            Serial.println("7");
        if (ps2x.Button(PSB_R1))
            Serial.println("8");
        if (ps2x.Button(PSB_TRIANGLE)){
            Serial.println("9");
            if(maxSpeed + SPEED_STEP <= SPEED_MAX) {
                maxSpeed += SPEED_STEP;
            }
            else    maxSpeed = SPEED_MAX;
        }
        if (ps2x.Button(PSB_CIRCLE)) {
            Serial.println("10");
            if(maxSteer + STEER_STEP <= STEER_MAX) {
                maxSteer += STEER_STEP;
            }
            else    maxSteer = STEER_MAX;
        }
        if (ps2x.Button(PSB_CROSS)) {
            Serial.println("11");
            if(maxSpeed - SPEED_STEP >= SPEED_MIN) {
                maxSpeed -= SPEED_STEP;
            }
            else    maxSpeed = SPEED_MIN;
        }
        if (ps2x.Button(PSB_SQUARE)) {
            Serial.println("12");
            if(maxSteer - STEER_STEP >= STEER_MIN) {
                maxSteer -= STEER_STEP;
            }
            else    maxSteer = STEER_MIN;
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

void loadToLCD()
{
}

void processHoverboard(uint8_t data[4])
{
    uint8_t x = data[1];
    uint8_t y = data[0];
    setSpeed = map(y, 0, 255, maxSpeed, -maxSpeed);
    if(setSpeed == -1) setSpeed = 0;
    setSteer = map(x, 0, 255, -maxSteer, maxSteer);
    loadToHover(setSpeed,setSteer);
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
    tft.setRotation(2);
    //tft.clear();
    tft.fillScreen(ILI9341_BLACK);
    tft.setTextSize(2);
    tft.setCursor(2, 2);
    tft.print(error1);
    tft.print(" ");
    tft.print(error2);
    tft.print(" ");
    tft.print(type_R);
    tft.print(" ");
    tft.println(type_W);
    tft.print("Speed "); tft.println(setSpeed);
    tft.print("Steer "); tft.println(setSteer);
    tft.print("maxsp "); tft.println(maxSpeed); 
    tft.print("maxst "); tft.println(maxSteer); 
    tft.print(sensorState[0], BIN);

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
void loop()
{
    // while (fl.ReadSensor || fl.ReadHover || fl.ReadMotor)
    // {
    //     loadToLCD();
    //     loadToNUC();
    //     fl.ReadSensor = 0;
    //     fl.ReadHover = 0;
    //     fl.ReadMotor = 0;
    // }    
    // if(fl.ReadSensor) {
    //     sensorState[0] = pcf1.digitalReadByte();
    //     fl.ReadSensor = 0;
    // }
    if(type_R == 1 || type_W == 1){
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
        unsigned long now = millis();
        if (now - last >= 100)
        {   
            last = now;
            lcd();
            processHoverboard(controllerData0);
        }
    }
    // while (fl.ReadNUC)
    // {
    //     readNUC();
    // }

    // fl.ReadSensor = 1;
    // // loadToHover();
    readHover();
    // fl.ReadHover = 1;
    // loadToMotor();
    // readMotor();
    // fl.ReadMotor = 1;
}
