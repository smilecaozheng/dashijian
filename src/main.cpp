#include <Adafruit_ADXL345_U.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Audio.h>
#include <FS.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <Wire.h>
#include <esp_task_wdt.h>

#define DEBUG 1  // DEBUG模式

// MAX98357数字功放模块参数
#define I2S_MAX98357_DOUT 11
#define I2S_MAX98357_BCLK 12
#define I2S_MAX98357_LRC 13

// 微型震动马达
#define MOTOR_PIN 1  // 震动马达mos栅极

// WS2812灯带实例化
#define LED_PIN 41
#define LED_COUNT 52
#define LED_BRIGHT 50

// ADXL345加速度传感器模块参数
#define I2C_ADXL345_SCL 36
#define I2C_ADXL345_SDA 37

// 功能按钮
#define BTN 18

// 大师剑的几种状态
enum SwordStatus {
    STATUS_INIT,    // 开机状态 剑身不亮，单击触发震动，逐渐点亮蓝色剑身，并播放001.wav 之后进入通常状态
    STATUS_NORMAL,  // 通常状态 剑身蓝色，挥动触发特效，双击更改特效，长按进入战斗状态，触发震动，并播放002.wav
    STATUS_FIGHT    // 战斗状态 剑身红色，挥动触发特效，双击更改特效，单击返回普通模式，触发震动，并播放003.wav
};

// 触发器的几种状态
enum Trigger {
    TRIG_CLICK,   // 单击
    TRIG_DBCLCK,  // 双击
    TRIG_PRESS,   // 长按
    TRIG_SWING,   // 挥动
    TRIG_IDLE     // 空闲状态
};



// 实例对象
Audio audio;                                                        // 音频对象
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);   // 加速度传感器对象
sensors_event_t accel_event;                                        // 加速度传感器事件
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);  // WS2812灯带对象

// 全局变量
volatile SwordStatus status;          // 大师剑状态
volatile Trigger trigger;             // 触发器
volatile boolean btnState, btn_flag;  // 按键状态

// 加速度参数
static float prevAccel[3] = {0.0f, 0.0f, 0.0f};      // 当前加速度值
static float currAccel[3] = {0.0f, 0.0f, 0.0f};      // 上一次加速度值

// 上次状态更改的计时器
static unsigned long ADXL_Timer = 0;            // 加速度传感器
static unsigned long LED_Timer = 0;             // LED灯
static unsigned long MOTOR_Timer = 0;           // 震动马达
static unsigned long BTN_Timer = 0;             // 按钮
static unsigned long BTN_Timer = 0;             // 按钮

// 平滑加速度数据的滤波器系数
const float FILTER_ALPHA = 0.1f;

// 挥动动作的阈值 忽略XYZ 关注加速度变化
const float SWING_THRESHOLD = 1.0f;

// 状态冷却时间（毫秒）
const unsigned long SWING_COOLDOWN = 2000;          // 挥动冷却
const unsigned long BTN_COOLDOWN = 2000;

// LittleFS中需要调用的音频文件列表
const char* const music_list[] = {
    "/001.wav", "/002.wav", "/003.wav"};





// 函数声明
void detectBottons();
void detectActions();
void updateActions();
void setLEDColor(uint8_t r, uint8_t g, uint8_t b);
void trigMotor();
void adjustBrightnessForSwing();
void printAccelData(float prev[], float current[]);
void printStatus();
void printTrigger();

void setup() {

    // 开启串口debug输出
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    delay(5000);
    Serial.println("Setup started");

    // 初始化WS2812
    strip.begin();
    strip.setBrightness(0);
    strip.show();  // 关闭所有灯
    delay(200);

    // 初始化ADXL
    Wire.begin(I2C_ADXL345_SDA, I2C_ADXL345_SCL);
    if (!accel.begin()) {
        Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
        while (1)
            ;
    }
    accel.setRange(ADXL345_RANGE_16_G);  // 设置ADXL 量程

    // 初始化I2S MAX98357数字功放模块参数
    audio.setPinout(I2S_MAX98357_BCLK, I2S_MAX98357_LRC, I2S_MAX98357_DOUT);
    audio.setVolume(21);                   // 音量范围0...21
    audio.setBufsize(0, 1 * 1024 * 1024);  // 0表示使用PSRAM
    audio.connecttoFS(LittleFS, "/001.wav");

    // 初始化功能按钮
    pinMode(BTN, INPUT_PULLUP);

    // 初始化震动马达
    pinMode(MOTOR_PIN, OUTPUT);

    // 初始化大师剑状态和触发器
    status = STATUS_INIT;
    trigger = TRIG_IDLE;

    // 打印大师剑和触发器状态
    printStatus();
    printTrigger();

    // strip.setBrightness(50);
    // Serial.println("led start...");
    // for (int i = 0; i < strip.numPixels(); i++) {
    //     strip.setPixelColor(i, 0, 255, 255);  // 冰蓝色
    //     strip.show();
    //     delay(50);  // 每个LED之间的延迟
    // }
    // Serial.println("Shaking motor...");
    // for (int i = 0; i < 2; i++) {
    //     digitalWrite(MOTOR_PIN, HIGH);
    //     delay(250);
    //     digitalWrite(MOTOR_PIN, LOW);
    //     delay(250);
    // }

    // delay(1000);  // 每个LED之间的延迟
    // status = STATUS_NORMAL;


    // // 锁对象初始化
    // xMutex = xSemaphoreCreateMutex();

    // // 线程规划
    // xTaskCreate(task_musicplay, "Task 1", 8 * 1024, NULL, 1, NULL);
    // xTaskCreate(task_sens, "Task 2", 8 * 1024, NULL, 2, NULL);
    // vTaskStartScheduler();
}

void loop() {
    // 喂看门狗定时器
    esp_task_wdt_reset();

    // 检测功能按键
    detectBottons();

    // 检测手势
    detectActions();

    // 根据状态更新LED灯带和执行其他逻辑
    updateActions();
}

// 检测功能按键
void detectBottons() {

    // 判断功能按键冷却时间
    if (millis() - BTN_Timer > BTN_COOLDOWN && trigger==TRIG_IDLE){


    }

    btnState = !digitalRead(BTN);  // 获取按键状态
    if (btnState && !btn_flag)     // 按下
    {
        if (DEBUG)
            Serial.println(F("BTN PRESS"));
        btn_flag = 1;  // 按下的状态
        btn_counter++;
        btn_timer = millis();  // 按下的时间
    }
    if (!btnState && btn_flag) {  // 按键释放
        btn_flag = 0;
        hold_flag = 0;
    }
    // 长按 开关光剑
    if (btn_flag && btnState && (millis() - btn_timer > BTN_TIMEOUT) && !hold_flag) {
        ls_chg_state = 1;  // flag to change saber state (on/off)
        hold_flag = 1;
    }
    // 连按 触发
    if ((millis() - btn_timer > BTN_TIMEOUT) && (btn_counter != 0)) {
        if (ls_state) {
            if (DEBUG)
                Serial.println("Change color");
            if (btn_counter >= 3) {  // 3 press count
                nowColor++;          // change color
                if (nowColor >= 3)
                    nowColor = 0;
                setAll(red, green, blue);
                eeprom_flag = 1;
            }
        }
        btn_counter = 0;
    }
}

// 检测手势
void detectActions() {

    // 判断挥动冷却时间
    if (millis() - ADXL_Timer > SWING_COOLDOWN && trigger==TRIG_IDLE)
    {
        // 取得加速度
        accel.getEvent(&accel_event);

        // 平滑加速度数据
        currAccel[0] = FILTER_ALPHA * accel_event.acceleration.x + (1 - FILTER_ALPHA) * prevAccel[0];
        currAccel[1] = FILTER_ALPHA * accel_event.acceleration.y + (1 - FILTER_ALPHA) * prevAccel[1];
        currAccel[2] = FILTER_ALPHA * accel_event.acceleration.z + (1 - FILTER_ALPHA) * prevAccel[2];

        // 检查XYZ轴的总体变化
        if ((fabs(currAccel[0] - prevAccel[0]) > SWING_THRESHOLD) ||
            (fabs(currAccel[1] - prevAccel[1]) > SWING_THRESHOLD) ||
            (fabs(currAccel[2] - prevAccel[2]) > SWING_THRESHOLD) ){

            // 触发挥动
            trigger = TRIG_SWING;

            // 更新挥动的最后时间
            ADXL_Timer = millis();

            // 打印大师剑和触发器状态，触发时间，加速度参数
            printStatus();
            printTrigger();
            Serial.println(millis());
            printAccelData(prevAccel, currAccel);

        }

        // 更新上一次加速度值
        prevAccel[0] = currAccel[0];
        prevAccel[1] = currAccel[1];
        prevAccel[2] = currAccel[2];
    }
}

// 更新动作
void updateActions() {
    // 根据状态更新LED灯带
    switch(status){

        case STATUS_INIT:

            switch(trigger){
                case TRIG_CLICK:
                    Serial.println("TRIG_CLICK");
                    break;
                case TRIG_DBCLCK:
                    break;
                case TRIG_PRESS:
                    break;
                case TRIG_SWING:
                    break;
                case TRIG_IDLE:
                    break;
                default:
                    break;
            }
            break;

        case STATUS_NORMAL:

            switch(trigger){
                case TRIG_CLICK:
                    Serial.println("TRIG_CLICK");
                    break;
                case TRIG_DBCLCK:
                    Serial.println("TRIG_DBCLCK");
                    break;
                case TRIG_PRESS:
                    Serial.println("TRIG_PRESS");
                    break;
                case TRIG_SWING:
                    Serial.println("TRIG_SWING");
                    break;
                case TRIG_IDLE:
                    Serial.println("TRIG_IDLE");
                    break;
                default:
                    break;
            }
            break;

        case STATUS_FIGHT:

            switch(trigger){
                case TRIG_CLICK:
                    Serial.println("TRIG_CLICK");
                    break;
                case TRIG_DBCLCK:
                    Serial.println("TRIG_DBCLCK");
                    break;
                case TRIG_PRESS:
                    Serial.println("TRIG_PRESS");
                    break;
                case TRIG_SWING:
                    Serial.println("TRIG_SWING");
                    break;
                case TRIG_IDLE:
                    Serial.println("TRIG_IDLE");
                    break;
                default:
                    break;
            }
            break;

        default:
            break;
    }
    trigger = TRIG_IDLE;
}

void setLEDColor(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, r, g, b);
    }
    strip.show();
}

void trigMotor() {
    digitalWrite(MOTOR_PIN, HIGH);
    delay(500);
    digitalWrite(MOTOR_PIN, LOW);
}

void adjustBrightnessForSwing() {
    // 临时增加亮度，然后恢复原状
    Serial.println("led Brightness");
    strip.setBrightness(150);
    strip.show();
    delay(500);               // 亮0.5秒
    strip.setBrightness(50);  // 恢复原来的亮度
    strip.show();
}

// 打印大师剑状态
void printStatus(){
    switch(status){
        case STATUS_INIT:
            Serial.println("STATUS_INIT");
            break;
        case STATUS_NORMAL:
            Serial.println("STATUS_NORMAL");
            break;
        case STATUS_FIGHT:
            Serial.println("STATUS_FIGHT");
            break;
        default:
            break;
    }
}

// 打印触发器状态
void printTrigger(){
    switch(trigger){
        case TRIG_CLICK:
            Serial.println("TRIG_CLICK");
            break;
        case TRIG_DBCLCK:
            Serial.println("TRIG_DBCLCK");
            break;
        case TRIG_PRESS:
            Serial.println("TRIG_PRESS");
            break;
        case TRIG_SWING:
            Serial.println("TRIG_SWING");
            break;
        case TRIG_IDLE:
            Serial.println("TRIG_IDLE");
            break;
        default:
            break;
    }
}

// 打印加速度值
void printAccelData(float prev[], float current[]) {
    Serial.print("prevAccel: X: ");
    Serial.print(prev[0]);
    Serial.print("  Y: ");
    Serial.print(prev[1]);
    Serial.print("  Z: ");
    Serial.print(prev[2]);
    Serial.println(" m/s^2");

    Serial.print("currAccel: X: ");
    Serial.print(current[0]);
    Serial.print("  Y: ");
    Serial.print(current[1]);
    Serial.print("  Z: ");
    Serial.print(current[2]);
    Serial.println(" m/s^2");
}
// // 线程1 独立控制音频播放，避免因delay而产生的音频播放错误
// void task_musicplay(void* pvParameters) {
//     while (1) {
//         // 获取互斥锁
//         if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
//             // 释放互斥锁
//             xSemaphoreGive(xMutex);
// vTaskDelete(NULL); // 删除线程
//         }
//     }
// }

// void task_sens(void* pvParameters) {
