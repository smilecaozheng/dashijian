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
#define BUTTON_PIN 18

// 预定颜色定义
#define ICE_BLUE 0, 255, 255       // 冰蓝色
#define TIFFANY_GREEN 0, 255, 153  // 蒂芙尼绿色
#define RED 255, 0, 0              // 红色

// 大师剑的几种状态
enum SwordStatus {
    STATUS_INIT,    // 开机状态 剑身不亮，单击触发震动，逐渐点亮蓝色剑身，并播放001.wav 之后进入通常状态，闲时超过60秒进入展示状态
    STATUS_NORMAL,  // 通常状态 剑身蓝色，挥动触发特效，双击更改挥动特效，长按进入战斗状态，触发震动，并播放002.wav
    STATUS_FIGHT,   // 战斗状态 剑身红色，挥动触发特效，双击更改挥动特效，单击返回普通模式，触发震动，并播放003.wav
    STATUS_DISPLAY  // 展示状态 剑身炫彩，双击拾音灯功能，单击返回普通状态，触发震动，并播放003.wav
};

// 触发器的几种状态
enum TriggerState {
    TRIG_CLICK,   // 单击
    TRIG_DBCLCK,  // 双击
    TRIG_PRESS,   // 长按
    TRIG_SWING,   // 挥动
    TRIG_IDLE     // 空闲状态
};

// LED的几种效果
enum LedEffect {
    LED_ON,       // 常量
    LED_OFF,      // 熄灭
    INCREASE_UP,  // 逐渐点亮
    BRIGHTNESS,   // 闪耀
    COLORFUL      // 炫彩
};

// 实例对象
Audio audio;                                                        // 音频对象
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);   // 加速度传感器对象
sensors_event_t accel_event;                                        // 加速度传感器事件
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);  // WS2812灯带对象

// 全局变量
volatile SwordStatus swordStatus = STATUS_INIT;  // 大师剑状态
volatile TriggerState triggerState = TRIG_IDLE;  // 触发器
volatile LedEffect ledEffect = LED_OFF;          // LED效果

// 加速度参数
static float prevAccel[3] = {0.0f, 0.0f, 0.0f};  // 当前加速度值
static float currAccel[3] = {0.0f, 0.0f, 0.0f};  // 上一次加速度值


// 按键参数
volatile boolean buttonPressed = false;   // 按键按下
volatile boolean buttonReleased = false;  // 按键释放
volatile boolean longPress = false;       // 按键长按
volatile boolean dobleClicked = false;    // 按键双击

static unsigned long lastButtonTime = 0;  // 上次按键时间
volatile int buttonLevel = 0;                // 按键电平
volatile int clickCount = 0;  // 按键计数器
const unsigned long BUTTON_TIMEOUT = 50;         // 按键超时时间 (毫秒)
const unsigned long DOUBLE_CLICK_TIMEOUT = 300;  // 双击超时时间 (毫秒)
const unsigned long PRESS_TIMEOUT = 1000;        // 长按超时时间 (毫秒)



// 动作开始计时器
static unsigned long motorTimer = 0;  // 震动马达开始时间
static unsigned long audioTimer = 0;  // 音频播放开始时间
static unsigned long ledTimer = 0;    // Led效果开始时间
static unsigned long idleTimer = 0;   // 闲时计时开始时间

// 上次检测时间

static unsigned long lastSwingTime = 0;   // 上次挥动时间
static unsigned long lastClickTime = 0;   // 上次单击时间

// 计数器

volatile int ledCount = 0;    // led计数器

// 平滑加速度数据的滤波器系数
const float FILTER_ALPHA = 0.1f;

// 挥动动作的阈值 忽略XYZ 关注加速度变化
const float SWING_THRESHOLD = 1.0f;

// 状态冷却时间（毫秒）

const unsigned long SWING_TIMEOUT = 2000;        // 挥动超时时间 (毫秒)

const unsigned long MOTOR_TIMEOUT = 500;         // 马达超时时间 (毫秒)
const unsigned long IDLE_TIMEOUT = 60000;        // 闲时超时时间 (毫秒)
const unsigned long BRIGHTNESS_TIMEOUT = 500;    // 闪耀特效超时时间 (毫秒)
// LED延时时间（毫秒）
const unsigned long INCREASE_UP_TIMEOUT = 30;  // 逐渐点亮延时时间 (毫秒)

// LittleFS中需要调用的音频文件列表，使用方式例：调用002.wav 使用music_list[1]
volatile int audioNumber = 0;  // 音频No
const char* const music_list[] = {
    "/001.wav", "/002.wav", "/003.wav"};

// 函数声明
void detectBottons();
void detectADXL();
void judgeActions();
void motorHandler();
void audioHandler();
void ledHandler();

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

    // 初始化功能按钮
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // 初始化震动马达
    pinMode(MOTOR_PIN, OUTPUT);

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
}

void loop() {
    // 喂看门狗定时器
    esp_task_wdt_reset();

    // 音频循环
    audio.loop();

    // 检测功能按键
    detectBottons();

    // 检测加速度
    detectADXL();

    // 根据状态更新LED灯带和执行其他逻辑
    judgeActions();
}

// 检测功能按键
void detectBottons() {

    // 读取按键电平
    buttonLevel = digitalRead(BUTTON_PIN);

    // 消抖处理
    if (buttonLevel != buttonPressed) {
        lastButtonTime = millis();
    }

    // 如果过了冷却时间
    if ((millis() - lastButtonTime) > BUTTON_TIMEOUT) {
        // 如果按键被按下
        if (buttonLevel == LOW && !buttonPressed) {
            buttonPressed = true;
            buttonReleased = false;
            clickCount++;
            lastClickTime = millis();
        }
        // 如果按键被释放
        else if (buttonLevel == HIGH && buttonPressed) {
            buttonPressed = false;
            buttonReleased = true;

            // 检查是否为长按
            if (millis() - lastClickTime > PRESS_TIMEOUT) {
                longPress = true;
            }
            // 检查是否为双击
            else if (clickCount == 2 && (millis() - lastClickTime) < DOUBLE_CLICK_TIMEOUT) {
                dobleClicked = true;
            } else {
                clickCount = 0;
            }
        }
    }

    // 检查是否为单击
    if (buttonReleased && !longPress && !dobleClicked) {
        triggerState = TRIG_CLICK;  // 触发单击
        printStatus();
        printTrigger();
        buttonReleased = false;  // 重置释放标志
    }

        // 检查是否为双击
    if (dobleClicked) {
        triggerState = TRIG_DBCLCK;  // 触发双击
        printStatus();
        printTrigger();
        dobleClicked = false;  // 重置双击标志
    }
        // 检查是否为长按
    if (longPress) {
        triggerState = TRIG_PRESS;  // 触发长按
        printStatus();
        printTrigger();
        longPress = false;  // 重置长按标志
    }
}

// 检测加速度
void detectADXL() {
    // 如果开机状态 或者任意计时器有值则 不检测加速度
    if (swordStatus == STATUS_INIT || motorTimer != 0 || audioTimer != 0 || ledTimer != 0)
        return;

    // 取得加速度
    accel.getEvent(&accel_event);

    // 判断挥动冷却时间
    if (millis() - lastSwingTime > SWING_TIMEOUT) {
        // 平滑加速度数据
        currAccel[0] = FILTER_ALPHA * accel_event.acceleration.x + (1 - FILTER_ALPHA) * prevAccel[0];
        currAccel[1] = FILTER_ALPHA * accel_event.acceleration.y + (1 - FILTER_ALPHA) * prevAccel[1];
        currAccel[2] = FILTER_ALPHA * accel_event.acceleration.z + (1 - FILTER_ALPHA) * prevAccel[2];

        // 检查XYZ轴的总体变化
        if ((fabs(currAccel[0] - prevAccel[0]) > SWING_THRESHOLD) ||
            (fabs(currAccel[1] - prevAccel[1]) > SWING_THRESHOLD) ||
            (fabs(currAccel[2] - prevAccel[2]) > SWING_THRESHOLD)) {
            // 触发挥动
            triggerState = TRIG_SWING;

            // 更新挥动的最后时间
            lastSwingTime = millis();

            // 打印大师剑和触发器状态，触发时间，加速度参数
            printStatus();
            printTrigger();
            Serial.println(millis());
            printAccelData(prevAccel, currAccel);
        }
    }

    // 更新上一次加速度值
    prevAccel[0] = accel_event.acceleration.x;
    prevAccel[1] = accel_event.acceleration.y;
    prevAccel[2] = accel_event.acceleration.z;
}

// 判断动作
void judgeActions() {
    // 根据状态更新LED灯带
    switch (swordStatus) {
        // 开机状态
        case STATUS_INIT:

            switch (triggerState) {
                // 单击触发开机效果
                case TRIG_CLICK:
                    // 延时启动震动电机
                    if (motorTimer == 0) {
                        motorTimer = millis() + 5000;
                    }

                    // 逐渐点亮LED灯
                    if (ledTimer == 0) {
                        ledTimer = millis();
                        ledEffect = INCREASE_UP;
                    }

                    // 当灯全部点亮播放开机声音
                    if (audioTimer == 0) {
                        audioTimer = millis() + 3000;
                        audioNumber = 0;
                    }
                    // 进入普通状态
                    swordStatus = STATUS_NORMAL;
                    break;

                // 如果空闲超过60秒进入展示状态
                case TRIG_IDLE:
                    if (idleTimer == 0) {
                        idleTimer = millis();

                    } else if (millis() - idleTimer > IDLE_TIMEOUT) {
                        idleTimer = 0;
                        ledEffect = COLORFUL;
                        // 进入展示状态
                        swordStatus = STATUS_DISPLAY;
                    }
                    break;
            }
        // 普通状态
        case STATUS_NORMAL:

            switch (triggerState) {
                case TRIG_DBCLCK:

                    // TODO:切换挥动特效
                    break;

                case TRIG_PRESS:
                    // 进入战斗模式

                    // 播放战斗声音
                    if (audioTimer == 0) {
                        audioTimer = millis();
                        audioNumber = 0;
                    }

                    // 延时启动震动电机
                    if (motorTimer == 0) {
                        motorTimer = millis() + 5000;
                    }
                    break;
                case TRIG_SWING:

                    // LED灯增加亮度
                    if (ledTimer == 0) {
                        ledTimer = millis();
                        ledEffect = BRIGHTNESS;
                    }
                    break;
                default:
                    break;
            }
            break;

        // 战斗状态
        case STATUS_FIGHT:

            switch (triggerState) {
                case TRIG_CLICK:
                    Serial.println("TRIG_CLICK");
                    break;
                case TRIG_DBCLCK:
                    // TODO:切换挥动特效
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
    triggerState = TRIG_IDLE;
}

// 处理音频
void audioHandler() {
    // 判断计时器有值
    if (audioTimer != 0 && millis() > audioTimer) {
        // 如果没有播放音频
        if (audio.isRunning() == false) {
            Serial.print("Connecting to ");
            Serial.print(music_list[audioNumber]);
            Serial.println(" to play audio...");
            audio.connecttoFS(LittleFS, music_list[audioNumber]);  // 使用LittleFS和MP3文件路径进行连接
        } else {
            audioTimer = 0;
        }
    }
}

// 处理震动
void motorHandler() {
    // 判断计时器有值
    if (motorTimer != 0 && millis() > motorTimer) {
        // 如果电机低电平，打开电机
        if (digitalRead(MOTOR_PIN) == LOW) {
            digitalWrite(MOTOR_PIN, HIGH);
            // 如果电机高电平，到达了震动时长，计时器清零并关闭电机
        } else if (digitalRead(MOTOR_PIN) == HIGH && millis() - motorTimer > MOTOR_TIMEOUT) {
            digitalWrite(MOTOR_PIN, LOW);
            motorTimer = 0;
        }
    }
}

// 处理LED
void ledHandler() {
    // 判断计时器有值
    if (ledTimer != 0 && millis() > ledTimer) {
        // 逐渐点亮特效
        if (ledEffect == INCREASE_UP) {
            // 设置led亮度为默认
            strip.setBrightness(LED_BRIGHT);

            // 判断灯珠位置
            if (ledCount < strip.numPixels()) {
                // 判断点亮时间
                if (millis() > ledTimer + ledCount * INCREASE_UP_TIMEOUT) {
                    strip.setPixelColor(ledCount, ICE_BLUE);
                    strip.show();
                    ledCount++;
                }
            } else {
                ledTimer = 0;
                // ledEffect = LED_ON;
            }
        }

        // 增加亮度特效
        if (ledEffect == BRIGHTNESS) {
            // 判断增加亮度超时时间
            if (millis() < ledTimer + BRIGHTNESS_TIMEOUT) {
                Serial.println("led Brightness");
                strip.setBrightness(150);
                strip.show();
            } else {
                strip.setBrightness(LED_BRIGHT);
                strip.show();
                ledTimer = 0;
            }
        }

    } else if (ledTimer == 0) {
        // 常亮
        if (ledEffect == LED_ON) {
            strip.setBrightness(LED_BRIGHT);
            strip.show();
        }

        // 关闭
        if (ledEffect == LED_OFF) {
            /* code */
        }

        // 炫彩
        if (ledEffect == COLORFUL) {
            // hue += 2;
            // if (hue >= 256) {
            //     hue = 0;  // 当色调值达到最大值时重置为0
            // }
            // for (int i = 0; i < strip.numPixels(); i++) {
            //     strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(hue)));
            // }
            // strip.show();
            // delay(50); // 每次循环之间等待50毫秒
        }

        // strip.setBrightness(LED_BRIGHT);
        // strip.show();
    }
}

// 打印大师剑状态
void printStatus() {
    switch (swordStatus) {
        case STATUS_INIT:
            Serial.println("STATUS_INIT");
            break;
        case STATUS_NORMAL:
            Serial.println("STATUS_NORMAL");
            break;
        case STATUS_FIGHT:
            Serial.println("STATUS_FIGHT");
            break;
        case STATUS_DISPLAY:
            Serial.println("STATUS_DISPLAY");
            break;
        default:
            break;
    }
}

// 打印触发器状态
void printTrigger() {
    switch (triggerState) {
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
