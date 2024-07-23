
/*******************Arduino通用库************************/
#include <Arduino.h>

/*******************音频文件处理库***********************/
#include <Audio.h>
#include <FS.h>
#include <LittleFS.h>

/*******************I2C ADXL传感器库*********************/
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/*******************LED灯库*****************************/
#include <Adafruit_NeoPixel.h>

/*******************NVS库******************************/
#include <Preferences.h>
#include <esp_system.h>

#define DEBUG 1  // DEBUG模式

// MAX98357数字功放模块参数
#define I2S_DOUT 11
#define I2S_BCLK 12
#define I2S_LRC 13

// 微型震动马达
#define MOTOR_PIN 1  // 震动马达mos栅极

// WS2812灯带实例化
#define LED_PIN 41
#define LED_COUNT 52
#define MAXBRIGHT 50

// ADXL345加速度传感器模块参数
#define I2C_MASTER_SCL_IO 36
#define I2C_MASTER_SDA_IO 37

// 大师剑的几种状态
enum SwordStatus {
    STATUS_INIT,    // 开机状态
    STATUS_NORMAL,  // 通常状态
    STATUS_FIGHT,   // 战斗状态
    STATUS_IDLE     // 空闲状态
};

// LittleFS中需要调用的音频文件列表
const char* const music_list[] = {
    "/001.wav", "/002.wav", "/003.wav"};

// 对象实例
Audio audio;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
sensors_event_t accel_event;
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// 全局变量
volatile SwordStatus music_status;

// 顺时针扭转 X轴基本不变化，Y轴和Z轴会增加
// 逆时针扭转 X轴基本不变化，Y轴和Z轴会减小
// 前刺 X轴减小，Y和Z基本不变化
// 挥动 XYZ都会有变化

// 加速度参数
static float prevAccel[3] = {0.0f, 0.0f, 0.0f};      // 当前加速度值
static float currAccel[3] = {0.0f, 0.0f, 0.0f};      // 上一次加速度值
static float filteredAccel[3] = {0.0f, 0.0f, 0.0f};  // 平滑后的加速度值

// 平滑加速度数据的滤波器系数
const float FILTER_ALPHA = 0.1f;

// 扭转加速度变化阈值
const float TWIST_THRESHOLD = 5.0f;          // 顺时针翻转阈值
const float REVERSE_TWIST_THRESHOLD = 5.0f;  // 逆时针翻转阈值

// 前刺加速度变化阈值
const float THRUST_Y_THRESHOLD = 3.0f;
const float THRUST_Z_THRESHOLD = 3.0f;

// 挥动动作的阈值
const float SWING_THRESHOLD = 5.0f;

// 触发器
bool fightTriggered = false;   // 战斗触发器
bool normalTriggered = false;  // 普通触发器
bool swingTriggered = false;   // 挥动触发器
bool thrustTriggered = false;  // 前刺触发器

const float TWIST_X_THRESHOLD = 5.0f;

// 状态冷却时间（毫秒）
const unsigned long TWIST_COOLDOWN = 1000;          // 顺时针扭转冷却
const unsigned long REVERSE_TWIST_COOLDOWN = 1000;  // 逆时针扭转冷却
const unsigned long THRUST_COOLDOWN = 1000;         // 前刺冷却
const unsigned long SWING_COOLDOWN = 1000;          // 挥动冷却

// 上次状态更改的时间戳
static unsigned long currentTime = 0;
static unsigned long lastActionTime = 0;

// 函数声明
void detectActions();
void updateLEDAndActions();
void setLEDColor(uint8_t r, uint8_t g, uint8_t b);
void triggerMotor();
void adjustBrightnessForSwing();
void getAccel();
void detectClockwiseTwist(unsigned long currentTime);
void detectCounterClockwiseTwist(unsigned long currentTime);
void detectThrust(unsigned long currentTime);
void detectSwing(unsigned long currentTime);
// 删除未使用的变量和函数声明
// SemaphoreHandle_t xMutex;
// Preferences prefs;
// volatile SwordStatus led_status;

// void task_musicplay(void* pvParameters);
// void task_sens(void* pvParameters);

void setup() {
    Serial.begin(115200);
    Serial.setDebugOutput(true);

    delay(5000);
    Serial.println("Setup started");

    // 开机状态
    music_status = STATUS_INIT;

    // 初始化WS2812
    strip.begin();
    strip.setBrightness(0);
    strip.show();  // 关闭所有灯
    delay(200);

    // 初始化ADXL
    Wire.begin(I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
    if (!accel.begin()) {
        Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    }
    accel.setRange(ADXL345_RANGE_16_G);  // 设置ADXL 量程

    // // 初始化I2S MAX98357数字功放模块参数
    // audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    // audio.setVolume(21);                   // 音量范围0...21
    // audio.setBufsize(0, 1 * 1024 * 1024);  // 0表示使用PSRAM
    // audio.connecttoFS(LittleFS, "/001.wav");

    // 初始化震动马达
    pinMode(MOTOR_PIN, OUTPUT);

    music_status = STATUS_INIT;
    strip.setBrightness(50);
    for (int i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, 0, 255, 255);  // 冰蓝色
        strip.show();
        delay(50);  // 每个LED之间的延迟
    }
    Serial.println("Shaking motor...");
    for (int i = 0; i < 2; i++) {
        digitalWrite(MOTOR_PIN, HIGH);
        delay(250);
        digitalWrite(MOTOR_PIN, LOW);
        delay(250);
    }
    music_status = STATUS_NORMAL;
    // // 锁对象初始化
    // xMutex = xSemaphoreCreateMutex();

    // // 线程规划
    // xTaskCreate(task_musicplay, "Task 1", 8 * 1024, NULL, 1, NULL);
    // xTaskCreate(task_sens, "Task 2", 8 * 1024, NULL, 2, NULL);
    // vTaskStartScheduler();
}

void loop() {
    // 检测状态
    detectActions();

    // 根据状态更新LED灯带和执行其他逻辑
    updateLEDAndActions();

    // 更新上一次加速度值
    prevAccel[0] = currAccel[0];
    prevAccel[1] = currAccel[1];
    prevAccel[2] = currAccel[2];

    // 打印加速度数据 (仅在DEBUG模式下)
#if DEBUG
    Serial.print("X: ");
    Serial.print(accel_event.acceleration.x);
    Serial.print("  Y: ");
    Serial.print(accel_event.acceleration.y);
    Serial.print("  Z: ");
    Serial.print(accel_event.acceleration.z);
    Serial.println(" m/s^2");
#endif
}

void detectActions() {
    // 检测加速度
    getAccel();
    // 当前时间
    currentTime = millis();

    // 检测顺时针扭转
    detectClockwiseTwist(currentTime);

    // 检测逆时针扭转
    detectCounterClockwiseTwist(currentTime);

    // 检测前刺
    detectThrust(currentTime);

    // 检测挥动
    detectSwing(currentTime);

}

// 检测加速度
void getAccel() {

    // 从ADXL345传感器获取最新的加速度数据
    accel.getEvent(&accel_event);

    // 当前加速度值
    currAccel[0] = accel_event.acceleration.x;
    currAccel[1] = accel_event.acceleration.y;
    currAccel[2] = accel_event.acceleration.z;

    // 用于平滑加速度数据的函数
    filteredAccel[0] = FILTER_ALPHA * currAccel[0] + (1 - FILTER_ALPHA) * filteredAccel[0];
    filteredAccel[1] = FILTER_ALPHA * currAccel[1] + (1 - FILTER_ALPHA) * filteredAccel[1];
    filteredAccel[2] = FILTER_ALPHA * currAccel[2] + (1 - FILTER_ALPHA) * filteredAccel[2];
}

// 检测顺时针扭转
void detectClockwiseTwist(unsigned long currentTime) {
    // 检查Y和Z轴的增量
    if ((currAccel[1] > prevAccel[1]) &&
        (currAccel[2] > prevAccel[2]) &&
        (currAccel[0] - prevAccel[0] < TWIST_X_THRESHOLD) &&
        (currentTime - lastActionTime > TWIST_COOLDOWN)) {
        if (music_status == STATUS_NORMAL) {
            music_status = STATUS_FIGHT;  // 或者根据你的逻辑更改状态
            fightTriggered = true;
            lastActionTime = currentTime;
        }
    }
}

// 检测逆时针扭转
void detectCounterClockwiseTwist(unsigned long currentTime) {
    // 检查Y和Z轴的减量
    if ((currAccel[1] < prevAccel[1]) &&
        (currAccel[2] < prevAccel[2]) &&
        (currAccel[0] - prevAccel[0] < TWIST_X_THRESHOLD) &&
        (currentTime - lastActionTime > REVERSE_TWIST_COOLDOWN)) {
        if (music_status == STATUS_FIGHT) {
            music_status = STATUS_NORMAL;  // 或者根据你的逻辑更改状态
            normalTriggered = true;
            lastActionTime = currentTime;
        }
    }
}

// 检测前刺
void detectThrust(unsigned long currentTime) {
    // 检查X轴的减量
    if ((currAccel[0] < prevAccel[0]) &&
        (fabs(currAccel[1] - prevAccel[1]) < THRUST_Y_THRESHOLD) &&
        (fabs(currAccel[2] - prevAccel[2]) < THRUST_Z_THRESHOLD) &&
        (currentTime - lastActionTime > THRUST_COOLDOWN)) {
        // 触发前刺动作
        // 这里可以加入你的前刺动作逻辑
        thrustTriggered = true;
        lastActionTime = currentTime;
    }
}

// 检测挥动
void detectSwing(unsigned long currentTime) {
    // 检查XYZ轴的总体变化
    if ((fabs(currAccel[0] - prevAccel[0]) > SWING_THRESHOLD) ||
        (fabs(currAccel[1] - prevAccel[1]) > SWING_THRESHOLD) ||
        (fabs(currAccel[2] - prevAccel[2]) > SWING_THRESHOLD) &&
            (currentTime - lastActionTime > SWING_COOLDOWN)) {
        // 触发挥动动作
        // 这里可以加入你的挥动动作逻辑
        swingTriggered = true;
        lastActionTime = currentTime;
    }
}

void updateLEDAndActions() {
    // 根据状态更新LED灯带
    switch (music_status) {
        case STATUS_NORMAL:

            if (fightTriggered) {
                setLEDColor(255, 0, 0);
                triggerMotor();
            }
            if (swingTriggered) {
                adjustBrightnessForSwing();
            }
            if (thrustTriggered) {
            }
            break;
        case STATUS_FIGHT:

            if (normalTriggered) {
                setLEDColor(0, 255, 255);
                triggerMotor();
            }
            if (swingTriggered) {
                adjustBrightnessForSwing();
            }
            if (thrustTriggered) {
            }
            break;
        default:
            // 默认状态或空闲状态 - 保持不变
            break;
    }
    normalTriggered = false;
    fightTriggered = false;
    swingTriggered = false;
    thrustTriggered = false;
}

void setLEDColor(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, r, g, b);
    }
    strip.show();
}

void triggerMotor() {
    digitalWrite(MOTOR_PIN, HIGH);
    delay(500);
    digitalWrite(MOTOR_PIN, LOW);
}

void adjustBrightnessForSwing() {
    // 临时增加亮度，然后恢复原状
    strip.setBrightness(255);
    strip.show();
    delay(500);               // 亮0.5秒
    strip.setBrightness(50);  // 恢复原来的亮度
    strip.show();
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
//
// 大师剑灯光的几种状态  只有完成了动作之后，才能触发其他状态
// 开机状态，0.5秒内震动2次，缓慢逐渐点亮剑身至全量为冰蓝色，同时播放001.wav音频
// 通常状态，剑身冰蓝色
// 战斗状态，剑身红色
// 空闲状态，表示音频或者效果触发结束

// 顺时针扭转触发器，在通常状态，顺时针扭转剑身，剑身变为红色，状态变为战斗状态，0.5秒内震动1次，同时播放002.wav音频
// 逆时针扭转触发器，在战斗状态，逆时针扭转剑身，剑身变为冰蓝色，状态变为战斗状态，0.5秒内震动1次，同时播放003.wav音频
// 挥动触发器，剑身变亮0.5秒之后返回原来亮度
// 前刺触发器，在原有状态颜色的基础上，增加1条极光色的追踪效果，追踪方向从第一个灯珠移动到结尾，保持4个led长度不变，同时播放004.wav音频

// 本代码为基于platformio的esp32s3通过arduino平台制作的塞尔达大师光剑
// 本项目通过adxl345检测加速度 ，I2S MAX98357数字功放模块 播放音频,52颗ws2812灯带作为光剑效果，并使用GPIO通过mos管控制微型震动马达
// [env:esp32-s3-devkitc-1]
// platform = espressif32
// board = esp32-s3-devkitc-1
// framework = arduino
// board_build.filesystem = littlefs
// board_build.arduino.partitions = ./default_16MB.csv
// board_build.arduino.memory_type = qio_opi
// build_flags =
// 	-DBOARD_HAS_PSRAM
// 	-DARDUINO_USB_CDC_ON_BOOT=1
// board_upload.flash_size = 16MB
// monitor_speed = 115200
// lib_deps =
// 	adafruit/Adafruit NeoPixel@^1.12.2
// 	adafruit/Adafruit Unified Sensor@^1.1.14
// 	adafruit/Adafruit ADXL345@^1.3.4
// 	briand/LibBriandIDF@^1.5.0
// 	esphome/ESP32-audioI2S@^2.0.7
// 需求如下
// 大师剑灯光的几种状态  只有完成了动作之后，才能触发其他状态
// 开机状态，0.5秒内震动2次，缓慢逐渐点亮剑身至全量为冰蓝色
// 通常状态，剑身冰蓝色
// 战斗状态，剑身红色
// 空闲状态，表示音频或者效果触发结束
// 顺时针扭转触发器，在通常状态，顺时针扭转剑身，剑身变为红色，状态变为战斗状态，0.5秒内震动1次
// 逆时针扭转触发器，在战斗状态，逆时针扭转剑身，剑身变为冰蓝色，状态变为战斗状态，0.5秒内震动1次
// 挥动触发器，剑身变亮0.5秒之后返回原来亮度
// 前刺触发器，在原有状态颜色的基础上，增加1条极光色的追踪效果，追踪方向从第一个灯珠移动到结尾，保持4个led长度不变