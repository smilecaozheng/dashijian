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

// WS2812灯带实例化
#define LED_PIN 41
#define LED_COUNT 52
#define LED_BRIGHT 50

// 功能按钮
#define BUTTON_PIN 18

// 预定颜色定义
#define ICE_BLUE (0x00FFFFFF)       // 冰蓝色
#define GOLDEN (0xFFD700FF)         // 金色
#define TIFFANY_GREEN (0x009999FF)  // 蒂芙尼绿色
#define RED (0xFF0000FF)            // 红色

// 大师剑的几种状态

// 触发器的几种状态
enum TriggerState {
    TRIG_CLICK,   // 单击
    TRIG_DBCLCK,  // 双击
    TRIG_PRESS,   // 长按
    TRIG_IDLE     // 空闲状态
};

// LED的几种效果
enum LedEffect {
    LED_OFF,      // 熄灭
    INCREASE_UP,  // 逐渐点亮
    BRIGHTNESS,   // 闪耀
    COLORFUL      // 炫彩
};

// 实例对象
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);  // WS2812灯带对象

// 全局变量
volatile TriggerState triggerState = TRIG_IDLE;  // 触发器
volatile LedEffect ledEffect = LED_OFF;          // LED效果

// 按键参数
volatile boolean buttonPressed = false;          // 按键按下
volatile boolean buttonReleased = false;         // 按键释放
volatile boolean longPress = false;              // 按键长按
volatile boolean doubleClicked = false;          // 按键双击
static unsigned long lastButtonTime = 0;         // 上次按键时间
volatile int buttonLevel = 0;                    // 按键电平
volatile int clickCount = 0;                     // 按键计数器
const unsigned long BUTTON_TIMEOUT = 50;         // 按键超时时间 (毫秒)
const unsigned long DOUBLE_CLICK_TIMEOUT = 300;  // 双击超时时间 (毫秒)
const unsigned long PRESS_TIMEOUT = 1000;        // 长按超时时间 (毫秒)

// 动作开始计时器
static unsigned long ledTimer = 0;   // LED效果开始时间
static unsigned long idleTimer = 0;  // 闲时计时开始时间

// 上次检测时间
static unsigned long lastClickTime = 0;  // 上次单击时间

// 计数器
volatile int ledCount = 0;  // led计数器

// 状态冷却时间（毫秒）

const unsigned long IDLE_TIMEOUT = 60000;      // 闲时超时时间 (毫秒)
const unsigned long BRIGHTNESS_TIMEOUT = 500;  // 闪耀特效超时时间 (毫秒)
// LED延时时间（毫秒）
const unsigned long INCREASE_UP_TIMEOUT = 30;  // 逐渐点亮延时时间 (毫秒)

// 函数声明
void detectButtons();

void judgeActions();

void ledHandler();

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

    // 初始化功能按钮
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // 打印触发器状态
    printTrigger();
}

void loop() {
    // 喂看门狗定时器
    esp_task_wdt_reset();

    // 检测功能按键
    detectButtons();

    // 根据状态更新LED灯带和执行其他逻辑
    judgeActions();

    // 处理LED
    ledHandler();
}

// 检测功能按键
void detectButtons() {
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
            clickCount = 1;
            lastClickTime = millis();
        }
        // 如果按键被释放
        else if (buttonLevel == HIGH && buttonPressed) {
            buttonPressed = false;
            buttonReleased = true;

            // 检查是否为长按
            if (millis() - lastClickTime > PRESS_TIMEOUT) {
                longPress = true;
                buttonReleased = false;  // 清除释放标志，防止长按时误触发单击
            }
            // 检查是否为双击
            else if (clickCount == 1 && (millis() - lastClickTime) < DOUBLE_CLICK_TIMEOUT) {
                // 等待第二次点击
                clickCount = 2;  // 标记为第二次点击
            } else {
                clickCount = 0;
            }
        }
    }

    // 检查是否为单击
    if (buttonReleased && !longPress && clickCount == 1) {
        // 确保从上次点击到现在的时间超过双击超时时间
        if (millis() - lastClickTime >= DOUBLE_CLICK_TIMEOUT) {
            triggerState = TRIG_CLICK;  // 触发单击

            printTrigger();
            buttonReleased = false;  // 重置释放标志
            clickCount = 0;          // 清除点击计数
        }
    }

    // 检查是否为双击
    if (buttonReleased && !longPress && clickCount == 2) {
        triggerState = TRIG_DBCLCK;  // 触发双击

        printTrigger();
        buttonReleased = false;  // 重置释放标志
        clickCount = 0;          // 清除点击计数
    }
    // 检查是否为长按
    if (longPress) {
        triggerState = TRIG_PRESS;  // 触发长按
        printTrigger();
        longPress = false;          // 重置长按标志
        buttonReleased = false;     // 清除释放标志，防止长按时误触发单击
    }
}

// 处理LED
void ledHandler() {
    switch (ledEffect) {
        case LED_OFF:
            // 灯熄灭
            strip.setBrightness(0);
            strip.show();
            break;
        case INCREASE_UP:
            if (ledCount < LED_COUNT) {
                strip.setPixelColor(ledCount++, ICE_BLUE);  // 冰蓝色
                strip.show();
                delay(INCREASE_UP_TIMEOUT);
            } else {
                strip.fill(ICE_BLUE, 0, LED_COUNT);
                strip.show();
                ledEffect = LED_OFF;
            }
            break;
        case BRIGHTNESS:
            strip.setBrightness(LED_BRIGHT);
            strip.show();
            delay(BRIGHTNESS_TIMEOUT);
            ledEffect = LED_OFF;
            break;
        case COLORFUL:
            // 炫彩效果
            // 这里可以添加炫彩效果的代码
            break;
        default:
            break;
    }
}

// 判断并设置LED的效果
void judgeActions() {
    if (triggerState == TRIG_CLICK) {
        ledEffect = INCREASE_UP;
        ledCount = 0;
        ledTimer = millis();
    } else if (triggerState == TRIG_DBCLCK) {
        ledEffect = BRIGHTNESS;
        ledTimer = millis();
    } else if (triggerState == TRIG_PRESS) {
        ledEffect = INCREASE_UP;
        ledCount = 0;
        for (int i = 0; i < LED_COUNT; i++) {
            strip.setPixelColor(i, GOLDEN);  // 金色
        }
        ledTimer = millis();
    } else if (triggerState == TRIG_IDLE) {
        if (millis() - idleTimer > IDLE_TIMEOUT) {
            ledEffect = COLORFUL;
            idleTimer = millis();
        }
    }
    triggerState = TRIG_IDLE;
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
        case TRIG_IDLE:
            Serial.println("TRIG_IDLE");
            break;
        default:
            break;
    }
}