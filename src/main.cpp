#include <Adafruit_ADXL345_U.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Audio.h>
#include <FS.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <SPI.h>
#include <esp_heap_caps.h>  // ESP-IDF 提供的头文件
// #include <esp_task_wdt.h>  // 使用任务看门狗库

#define DEBUG 1  // DEBUG模式

// 功能按钮
#define BUTTON_PIN 18

// 触发器的几种状态
enum TriggerState {
    TRIG_CLICK,   // 单击
    TRIG_DBCLCK,  // 双击
    TRIG_PRESS,   // 长按
    TRIG_SWING,   // 挥动
    TRIG_SLEEP,   // 休眠状态
    TRIG_IDLE     // 空闲状态
};

// 按键全局变量
TriggerState triggerState = TRIG_IDLE;        // 初始状态为空闲
unsigned long lastPressTime = 0;              // 上次长按时间
unsigned long lastReleaseTime = 0;            // 上次按键释放时间
unsigned long lastActionTime = 0;             // 上次动作时间
bool buttonState = false;                     // 按键状态，低电平：1 ，高电平：0
bool lastButtonState = false;                 // 上次按键状态
bool waitingForDoubleClick = false;           // 等待双击标志
bool longPressTriggered = false;              // 长按触发标志
bool sleepTriggered = false;                  // 睡眠触发标志
const unsigned long DOUBLE_CLICK_TIME = 500;  // 双击间隔时间 (毫秒)
const unsigned long LONG_PRESS_TIME = 2000;   // 长按时间 (毫秒)
const unsigned long SLEEP_TIME = 10000;       // 休眠时间 (毫秒)

// ADXL345加速度传感器
#define I2C_ADXL345_CS 40
#define I2C_ADXL345_SCL 36   // SCK
#define I2C_ADXL345_SDA 37   // MOSI
#define I2C_ADXL345_MISO 39  // MISO
// #define I2C_ADXL345_ADDR 0x53  // 0x1D if SDO = HIGH

// 加速度传感器全局变量
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(I2C_ADXL345_SCL, I2C_ADXL345_MISO, I2C_ADXL345_SDA, I2C_ADXL345_CS, 12345);  // 加速度传感器对象
sensors_event_t accel_event;                                                                                                           // 加速度传感器事件
static float prevAccel[3] = {0.0f, 0.0f, 0.0f};                                                                                        // 当前加速度值
static float currAccel[3] = {0.0f, 0.0f, 0.0f};                                                                                        // 上一次加速度值
const float FILTER_ALPHA = 0.1f;                                                                                                       // 平滑加速度数据的滤波器系数
const float SWING_THRESHOLD = 1.0f;                                                                                                    // 挥动动作的阈值 忽略XYZ 关注加速度变化
const unsigned long SWING_TIME = 2000;                                                                                                 // 挥动消抖时间 (毫秒)

// 大师剑的几种状态
enum SwordStatus {
    STATUS_INIT,
    STATUS_NORMAL,
    STATUS_FIGHT,
    STATUS_DISPLAY
};

// 全局变量大师剑状态
volatile SwordStatus swordStatus = STATUS_INIT;      // 大师剑状态
volatile SwordStatus lastSwordStatus = STATUS_INIT;  // 上一次大师剑状态

// WS2812灯带实例化
#define LED_PIN 41
#define LED_COUNT 52
#define LED_BRIGHT 50

// 定义颜色
#define ICE_BLUE 0, 255, 255       // 冰蓝色
#define GOLDEN 255, 215, 0         // 金色
#define TIFFANY_GREEN 0, 153, 153  // 蒂芙尼绿色
#define RED 255, 0, 0              // 红色
#define OFF 0, 0, 0                // 不亮

// LED的几种效果
enum LedEffect {
    LED_OFF,        // 熄灭
    INCREASE_UP,    // 逐渐点亮
    FIGHT_IN,       // 进入战斗
    RETURN_NORMAL,  // 返回普通
    RETURN_LAST,    // 换回上一次效果
    BRIGHTNESS,     // 闪耀
    COLORFUL,       // 炫彩
    LED_ON,         // 熄灭
    EFFECT_DONE     // 效果切换完成

};

// 全局变量LED灯
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);  // WS2812灯带对象
volatile LedEffect ledEffect = LED_OFF;                                                 // LED效果
volatile int ledCount = 0;                                                              // led计数器
unsigned long lastLedCountTime = 0;                                                     // 上次led计数时间
volatile int hueCount = 0;                                                              // hue计数器
unsigned long lastHueTime = 0;                                                          // 上次hue变化时间
unsigned long brightEndTime = 0;                                                        // 闪耀结束时间
const unsigned long INCREASE_UP_TIME = 30;                                              // 逐渐点亮延时时间 (毫秒)
const unsigned long HUE_UPDATE_TIME = 20;                                               // 炫彩变换延时时间 (毫秒)
const unsigned long BRIGHTNESS_TIME = 500;                                              // 闪耀特效超时时间 (毫秒)

// 微型震动马达
#define MOTOR_PIN 1  // 震动马达mos栅极

unsigned long motorEndTime = 0;        // 震动结束时间
const unsigned long MOTOR_TIME = 300;  // 震动超时时间 (毫秒)

// MAX98357数字功放模块参数
#define I2S_MAX98357_DOUT 11
#define I2S_MAX98357_BCLK 12
#define I2S_MAX98357_LRC 13

Audio audio = Audio(false, 3, I2S_NUM_1);
unsigned long audioStartTime = 0;             // 音频开始时间
const unsigned long AUDIO_DELAY_TIME = 1000;  // 延时播放时间 (毫秒)

int audioNumber = 0;  // 音频No
const char* music_list[] = {
    "/", "/001.mp3", "/002.mp3", "/003.mp3"};

// 函数声明
void detectButtons();
void detectADXL();
void judgeActions();
void ledHandler();
void motorHandler();
void audioHandler();
void setTriggerState(TriggerState state);
void setSwordStatus(SwordStatus swordStatus);
void setLedEffect(LedEffect ledEffect);
void printAccelData(float prev[], float current[]);
void setup() {
    // 开启串口debug输出
    Serial.begin(115200);
    Serial.setDebugOutput(true);

    delay(2000);
    Serial.println("Serial started");

    Serial.printf("Deafult free size: %d\n", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    Serial.printf("PSRAM free size: %d\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    Serial.printf("Flash size: %d bytes\n", ESP.getFlashChipSize());

    // 初始化按键
    Serial.println("Initializing Button");
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    lastActionTime = millis();

    // 初始化LED
    Serial.println("Initializing WS2812");
    strip.begin();
    strip.show();  // 初始化所有像素为'off'
    lastLedCountTime = millis();

    // 初始化震动马达
    Serial.println("Initializing Motor");
    pinMode(MOTOR_PIN, OUTPUT);

    // 初始化 SPI
    SPI.begin(I2C_ADXL345_SCL, I2C_ADXL345_MISO, I2C_ADXL345_SDA, I2C_ADXL345_CS);

    // 初始化ADXL
    Serial.println("Initializing ADXL");
    // Wire.begin(I2C_ADXL345_SDA, I2C_ADXL345_SCL, 100000);
    if (!accel.begin()) {
        Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
        while (1);
    }
    accel.setRange(ADXL345_RANGE_16_G);  // 设置ADXL 量程

    delay(2000);  // 添加延时

    // 初始化LittleFS
    Serial.println("Initializing LittleFS");
    if (!LittleFS.begin()) {
        Serial.println("LittleFS mount failed");
        return;
    }

    // 初始化I2S MAX98357数字功放模块参数
    Serial.println("Initializing I2S audio module");
    if (!audio.setPinout(I2S_MAX98357_BCLK, I2S_MAX98357_LRC, I2S_MAX98357_DOUT, I2S_GPIO_UNUSED)) {
        Serial.println("Failed to initialize I2S pins");
        return;
    }
    audio.setBufsize(0, 200 * 1024);  // 0表示使用PSRAM
    audio.setVolume(21);              // 音量范围0...21
    // Serial.println("Connecting to /001.mp3");
    // if (!audio.connecttohost("http://downsc.chinaz.net/Files/DownLoad/sound1/201906/11582.mp3")) {
    //     // if (!audio.connecttoFS(LittleFS, music_list[1])) {
    //     Serial.println("Failed to connect to /001.mp3");
    //     return;  // 如果音频连接失败，退出setup
    // }
    // audio.setFileLoop(1);

    Serial.println("Setup complete");
}

void loop() {
    // audio.loop();  // 音频循环

    detectButtons();  // 检测按键
    detectADXL();     // 检测加速度传感器
    judgeActions();   // 判断动作
    ledHandler();     // 处理LED
    motorHandler();   // 处理震动马达
    audioHandler();   // 处理音频
}

// 检测按键
void detectButtons() {
    // 记录当前时间
    unsigned long currentTime = millis();

    // 判断按键状态，低电平：1 ，高电平：0
    buttonState = digitalRead(BUTTON_PIN) == LOW;

    // 检查按钮状态
    if (buttonState != lastButtonState) {
        // 按键按下
        if (buttonState) {
            lastPressTime = currentTime;  // 更新上次动作时间
            longPressTriggered = false;   // 按下按钮时重置长按标志

            // 按键松开
        } else {
            lastReleaseTime = currentTime;  // 更新上次动作时间

            // 判断等待双击标志
            if (waitingForDoubleClick) {
                // 判断双击条件
                if (currentTime - lastReleaseTime < DOUBLE_CLICK_TIME && !longPressTriggered) {
                    setTriggerState(TRIG_DBCLCK);   // 设置状态：双击
                    waitingForDoubleClick = false;  // 重置等待双击标志
                }
            } else {
                //  防止长按后检测到单击或双击
                if (!longPressTriggered) {
                    waitingForDoubleClick = true;  // 重置等待双击标志
                    lastActionTime = currentTime;  // 更新上次动作时间
                }
            }
            // 如果长按已触发，松开时不检测单击
            if (longPressTriggered) {
                waitingForDoubleClick = false;  // 重置等待双击标志
            }
        }
        lastButtonState = buttonState;  // 更新上次按键状态
    }

    // 长按检测
    if (buttonState && (currentTime - lastPressTime > LONG_PRESS_TIME) && !longPressTriggered) {
        setTriggerState(TRIG_PRESS);    // 设置状态：长按
        longPressTriggered = true;      // 触发长按
        waitingForDoubleClick = false;  // 重置等待双击标志
        lastActionTime = currentTime;   // 更新上次动作时间 防止进入睡眠
    }

    // 睡眠检测
    if (!buttonState && (currentTime - lastActionTime > SLEEP_TIME) && !sleepTriggered) {
        setTriggerState(TRIG_SLEEP);   // 设置状态：睡眠
        sleepTriggered = true;         // 触发睡眠
        lastActionTime = currentTime;  // 更新上次动作时间 防止进入睡眠
    }

    // 单击检测
    if (waitingForDoubleClick && (currentTime - lastReleaseTime > DOUBLE_CLICK_TIME)) {
        setTriggerState(TRIG_CLICK);    // 设置状态：单击
        waitingForDoubleClick = false;  // 重置等待双击标志 防止单击后检测到双击
        lastActionTime = currentTime;   // 更新 lastActionTime 防止进入睡眠
    }

    // 如果有按键触发，则不触发睡眠状态
    if (buttonState || waitingForDoubleClick || longPressTriggered) {
        sleepTriggered = false;  // 重置睡眠标志
    }

    // 进入睡眠状态后重置活动时间，防止反复触发
    if (triggerState == TRIG_SLEEP) {
        lastActionTime = currentTime;  // 更新上次动作时间
    }

    // 闲置检测
    if (!buttonState && !waitingForDoubleClick && !longPressTriggered && !sleepTriggered) {
        // setTriggerState(TRIG_IDLE);  // 设置状态：空闲
    }
}

// 检测加速度
void detectADXL() {
    // 记录当前时间
    unsigned long currentTime = millis();

    // 非空闲状态，开机状态，展示状态，跳出检测
    if (triggerState == TRIG_CLICK || triggerState == TRIG_PRESS || triggerState == TRIG_DBCLCK || swordStatus == STATUS_INIT)
        return;

    // 取得加速度
    accel.getEvent(&accel_event);

    // 判断挥动冷却时间
    if (currentTime - lastActionTime > SWING_TIME) {
        // 平滑加速度数据
        currAccel[0] = FILTER_ALPHA * accel_event.acceleration.x + (1 - FILTER_ALPHA) * prevAccel[0];
        currAccel[1] = FILTER_ALPHA * accel_event.acceleration.y + (1 - FILTER_ALPHA) * prevAccel[1];
        currAccel[2] = FILTER_ALPHA * accel_event.acceleration.z + (1 - FILTER_ALPHA) * prevAccel[2];

        // 检查XYZ轴的总体变化
        if ((fabs(currAccel[0] - prevAccel[0]) > SWING_THRESHOLD) ||
            (fabs(currAccel[1] - prevAccel[1]) > SWING_THRESHOLD) ||
            (fabs(currAccel[2] - prevAccel[2]) > SWING_THRESHOLD)) {
            // 触发挥动
            setTriggerState(TRIG_SWING);  // 设置状态：挥动

            // 更新挥动的最后时间
            lastActionTime = currentTime;  // 更新 lastActionTime 防止进入睡眠

            // 打印大师剑和触发器状态，触发时间，加速度参数
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
    // 开机状态
    if (swordStatus == STATUS_INIT) {
        // 长按
        if (triggerState == TRIG_PRESS) {
            motorEndTime = millis() + MOTOR_TIME;          // 指定电机结束震动时间
            audioStartTime = millis() + AUDIO_DELAY_TIME;  // 指定音频播放开始时间
            audioNumber = 1;                               // 指定播放音频
            setLedEffect(INCREASE_UP);                     // 开机处理
            setSwordStatus(STATUS_NORMAL);                 // 进入普通状态

            // 睡眠
        } else if (triggerState == TRIG_SLEEP) {
            lastSwordStatus = swordStatus;   // 记录上一次状态
            setLedEffect(COLORFUL);          // 渐变炫彩
            setSwordStatus(STATUS_DISPLAY);  // 进入展示状态
        }

        // 普通状态
    } else if (swordStatus == STATUS_NORMAL && ledEffect == EFFECT_DONE) {
        // 长按
        if (triggerState == TRIG_PRESS) {
            audioStartTime = millis() + AUDIO_DELAY_TIME;  // 指定音频播放开始时间
            audioNumber = 3;                               // 指定播放音频
            motorEndTime = millis() + MOTOR_TIME;          // 指定电机结束震动时间
            setLedEffect(FIGHT_IN);                        // 进入战斗
            setSwordStatus(STATUS_FIGHT);                  // 进入战斗状态

            // 双击
        } else if (triggerState == TRIG_DBCLCK) {
            // 切换挥动效果

            // 单击
        } else if (triggerState == TRIG_CLICK) {
            // 熄灭处理
            audioStartTime = millis() + AUDIO_DELAY_TIME;  // 指定音频播放开始时间
            audioNumber = 2;                               // 指定播放音频
            setLedEffect(LED_OFF);                         // 关灯
            setSwordStatus(STATUS_INIT);                   // 返回开机状态

            // 挥动
        } else if (triggerState == TRIG_SWING) {
            // 变亮特效
            setLedEffect(BRIGHTNESS);  // 关灯

            // 睡眠
        } else if (triggerState == TRIG_SLEEP) {
            lastSwordStatus = swordStatus;   // 记录上一次状态
            setLedEffect(COLORFUL);          // 渐变炫彩
            setSwordStatus(STATUS_DISPLAY);  // 进入展示状态
        }

        // 战斗状态
    } else if (swordStatus == STATUS_FIGHT && ledEffect == EFFECT_DONE) {
        // 单击
        if (triggerState == TRIG_CLICK) {
            audioStartTime = millis() + AUDIO_DELAY_TIME;  // 指定音频播放开始时间
            audioNumber = 2;                               // 指定播放音频
            setLedEffect(RETURN_NORMAL);                   // 返回普通
            setSwordStatus(STATUS_NORMAL);                 // 返回正常状态

            // 双击
        } else if (triggerState == TRIG_DBCLCK) {
            // 切换挥动效果

            // 挥动
        } else if (triggerState == TRIG_SWING) {
            setLedEffect(BRIGHTNESS);  // 变亮特效
            // 睡眠
        } else if (triggerState == TRIG_SLEEP) {
            lastSwordStatus = swordStatus;   // 记录上一次状态
            setLedEffect(COLORFUL);          // 渐变炫彩
            setSwordStatus(STATUS_DISPLAY);  // 进入展示状态
        }

        // 展示状态
    } else if (swordStatus == STATUS_DISPLAY) {
        // 双击
        if (triggerState == TRIG_DBCLCK) {
            // 切换拾音灯 模式

        } else if (triggerState != TRIG_SLEEP) {
            setLedEffect(RETURN_LAST);        // 返回上一次效果
            setSwordStatus(lastSwordStatus);  // 返回上一次状态
        }
    }
}

// 处理LED
void ledHandler() {
    // 记录当前时间
    unsigned long currentTime = millis();

    if (ledEffect == LED_OFF) {
        if (ledCount < strip.numPixels() && (currentTime - lastLedCountTime > INCREASE_UP_TIME)) {
            strip.setPixelColor(strip.numPixels() - ledCount - 1, OFF);  // 不亮
            ledCount++;
            lastLedCountTime = currentTime;
            strip.show();
        }
    }

    if (ledEffect == INCREASE_UP) {
        if (ledCount < strip.numPixels() && (currentTime - lastLedCountTime > INCREASE_UP_TIME)) {
            strip.setBrightness(LED_BRIGHT);
            strip.setPixelColor(ledCount, ICE_BLUE);  // 冰蓝色
            ledCount++;
            lastLedCountTime = currentTime;
            strip.show();
            Serial.println(ledCount);
        } else if (ledCount >= strip.numPixels()) {
            setLedEffect(EFFECT_DONE);   // 效果完成
            setTriggerState(TRIG_IDLE);  // 触发完成
        }
    }

    if (ledEffect == FIGHT_IN) {
        if (ledCount < strip.numPixels() && (currentTime - lastLedCountTime > INCREASE_UP_TIME)) {
            strip.setPixelColor(ledCount, GOLDEN);  // 金色
            ledCount++;
            lastLedCountTime = currentTime;
            strip.show();
            Serial.println(ledCount);
        } else if (ledCount >= strip.numPixels()) {
            setLedEffect(EFFECT_DONE);   // 效果完成
            setTriggerState(TRIG_IDLE);  // 触发完成
        }
    }

    if (ledEffect == RETURN_NORMAL) {
        if (ledCount < strip.numPixels() && (currentTime - lastLedCountTime > INCREASE_UP_TIME)) {
            strip.setPixelColor(strip.numPixels() - ledCount - 1, ICE_BLUE);  // 不亮
            ledCount++;
            lastLedCountTime = currentTime;
            strip.show();
        } else if (ledCount >= strip.numPixels()) {
            setLedEffect(EFFECT_DONE);   // 效果完成
            setTriggerState(TRIG_IDLE);  // 触发完成
        }
    }

    if (ledEffect == BRIGHTNESS) {
        if (currentTime < brightEndTime) {
            strip.setBrightness(100);
            strip.show();
        } else {
            strip.setBrightness(LED_BRIGHT);  // 恢复亮度
            strip.show();
            setLedEffect(EFFECT_DONE);   // 效果完成
            setTriggerState(TRIG_IDLE);  // 触发完成
        }
    }

    if (ledEffect == COLORFUL) {
        strip.setBrightness(LED_BRIGHT);  // 恢复亮度
        if (hueCount < 256 * 5 && (currentTime - lastHueTime > HUE_UPDATE_TIME)) {
            for (ledCount; ledCount < strip.numPixels(); ledCount++) {
                uint16_t hue = (ledCount * 65536L / strip.numPixels()) - (hueCount * 256);
                strip.setPixelColor(ledCount, strip.gamma32(strip.ColorHSV(hue)));
            }
            strip.show();
            ledCount = 0;
            hueCount++;
            lastHueTime = currentTime;
        } else if (hueCount >= 256 * 5) {
            hueCount = 0;
        }
    }

    if (ledEffect == RETURN_LAST) {
        if (lastSwordStatus == STATUS_INIT) {
            setLedEffect(LED_OFF);
        } else if (lastSwordStatus == STATUS_NORMAL) {
            setLedEffect(RETURN_NORMAL);
        } else if (lastSwordStatus == STATUS_FIGHT) {
            setLedEffect(FIGHT_IN);
        }
    }
}

// 处理震动
void motorHandler() {
    // 记录当前时间
    unsigned long currentTime = millis();
    if (currentTime < motorEndTime && digitalRead(MOTOR_PIN) == LOW) {
        digitalWrite(MOTOR_PIN, HIGH);
        Serial.println("Motor vibration");
    } else if (currentTime >= motorEndTime) {
        digitalWrite(MOTOR_PIN, LOW);
    }
}

// 处理音频
void audioHandler() {
    // 记录当前时间
    unsigned long currentTime = millis();
    int number;

    if (currentTime < audioStartTime && audio.isRunning() == false && audioNumber != 0) {
        number = audioNumber;
        audioNumber = 0;
        Serial.print("Connecting to ");
        Serial.print(music_list[number]);
        Serial.println(" to play audio...");
        audio.connecttoFS(LittleFS, music_list[number]);  // 使用LittleFS和MP3文件路径进行连接
    }
}

// 设置触发器
void setTriggerState(TriggerState state) {
    triggerState = state;
    switch (state) {
        case TRIG_CLICK:
            Serial.println("TRIG_CLICK");
            break;
        case TRIG_DBCLCK:
            Serial.println("TRIG_DBCLCK");
            break;
        case TRIG_PRESS:
            Serial.println("TRIG_PRESS");
            break;
        case TRIG_SLEEP:
            Serial.println("TRIG_SLEEP");
            break;
        case TRIG_SWING:
            Serial.println("TRIG_SWING");
            break;
        case TRIG_IDLE:
            break;
    }
}

// 设置大师剑状态
void setSwordStatus(SwordStatus status) {
    swordStatus = status;
    Serial.print("last status: ");
    switch (lastSwordStatus) {
        case STATUS_INIT:
            Serial.print("STATUS_INIT");
            break;
        case STATUS_NORMAL:
            Serial.print("STATUS_NORMAL");
            break;
        case STATUS_FIGHT:
            Serial.print("STATUS_FIGHT");
            break;
        case STATUS_DISPLAY:
            Serial.print("STATUS_DISPLAY");
            break;
    }
    Serial.print("   ");
    Serial.print("current status: ");
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
    }
}

// 设置LED效果
void setLedEffect(LedEffect effect) {
    ledCount = 0;
    if (effect == BRIGHTNESS) brightEndTime = millis() + BRIGHTNESS_TIME;
    ledEffect = effect;
    Serial.print("current effect: ");
    switch (ledEffect) {
        case INCREASE_UP:
            Serial.println("INCREASE_UP");
            break;
        case FIGHT_IN:
            Serial.println("FIGHT_IN");
            break;
        case RETURN_NORMAL:
            Serial.println("RETURN_NORMAL");
            break;
        case BRIGHTNESS:
            Serial.println("BRIGHTNESS");
            break;
        case COLORFUL:
            Serial.println("COLORFUL");
            break;
        case LED_OFF:
            Serial.println("LED_OFF");
            break;
        case EFFECT_DONE:
            Serial.println("EFFECT_DONE");
            break;
        case RETURN_LAST:
            Serial.println("RETURN_LAST");
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
