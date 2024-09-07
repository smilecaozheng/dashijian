#include <Adafruit_ADXL345_U.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Audio.h>
#include <FS.h>
#include <Preferences.h>
#include <SPI.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <driver/i2s.h>
#include <esp_heap_caps.h>  // ESP-IDF 提供的头文件
#include <esp_task_wdt.h>   // 使用任务看门狗库
#include <math.h>

#define DEBUG 1  // DEBUG模式

// 功能按钮
#define BUTTON_PIN 18

const char* ssid = "2703";          // 替换为你的 Wi-Fi SSID
const char* password = "13763706";  // 替换为你的 Wi-Fi 密码

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
#define SPI_ADXL345_CS 40
#define SPI_ADXL345_SCL 36   // SCK
#define SPI_ADXL345_SDA 37   // MOSI
#define SPI_ADXL345_MISO 39  // MISO
// #define I2C_ADXL345_ADDR 0x53  // 0x1D if SDO = HIGH

// 加速度传感器全局变量
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(SPI_ADXL345_SCL, SPI_ADXL345_MISO, SPI_ADXL345_SDA, SPI_ADXL345_CS, 12345);  // 加速度传感器对象
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

// WS2812灯带
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
    EFFECT_DONE,    // 效果切换完成
    SOUND           // 声音
};

// 全局变量LED灯
Adafruit_NeoPixel strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);  // WS2812灯带对象
volatile LedEffect ledEffect = LED_OFF;                                                 // LED效果
volatile int ledCount = 0;                                                              // led计数器
unsigned long lastLedCountTime = 0;                                                     // 上次led计数时间
int previousLitLEDs = 0;                                                                // 之前点亮的 LED 数量
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

Audio audio;
unsigned long audioStartTime = 0;             // 音频开始时间
const unsigned long AUDIO_DELAY_TIME = 1000;  // 延时播放时间 (毫秒)

int audioNumber = 0;  // 音频No
const char* music_list[] = {
    "/", "/001.mp3", "/002.mp3", "/003.mp3"};

// INMP441麦克风模块
#define I2S_INMP441_WS 33
#define I2S_INMP441_SD 42
#define I2S_INMP441_SCK 35
#define I2S_INMP441_PORT I2S_NUM_1

// INMP441麦克风 全局变量
int volumePercentage = 0;              // 音量百分比 值 1-100
float noiseLevel = 40.0;               // 底噪水平
int32_t maxVolume = 0;                 // 最大音量
unsigned long lastMicroTime = 0;       // 上次麦克风时间
const unsigned long TIME_WINDOW = 10;  // 时间窗口，单位为毫秒
const int BUFFER_SIZE = 512;           // 缓冲区大小
const float MICRO_SENSITIVITY = 0.8;   // 灵敏度 0.1~1
const float LED_SMOOTH = 0.3;          // 0.1 是平滑系数，值越大平滑过渡越快

// 函数声明
void detectButtons();
void detectADXL();
void detectMicrophone();
void judgeActions();
void ledHandler();
void motorHandler();
void audioHandler();
void setTriggerState(TriggerState state);
void setSwordStatus(SwordStatus swordStatus);
void setLedEffect(LedEffect ledEffect);
void printAccelData(float prev[], float current[]);
void i2s_install_inmp441();
void setup() {
    // 开启串口debug输出
    Serial.begin(115200);
    Serial.setDebugOutput(true);

    // 初始化任务看门狗定时器，设置超时时间为 5 秒
    esp_task_wdt_init(5, true);

    delay(2000);
    Serial.println("Serial started");

    Serial.printf("Deafult free size: %d\n", heap_caps_get_free_size(MALLOC_CAP_DEFAULT));
    Serial.printf("PSRAM free size: %d\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    Serial.printf("Flash size: %d bytes\n", ESP.getFlashChipSize());

    // 连接到 Wi-Fi 网络
    // WiFi.begin(ssid, password);
    // while (WiFi.status() != WL_CONNECTED) {
    //     delay(500);
    //     Serial.print(".");
    // }
    // Serial.println("");
    // Serial.println("Connected to Wi-Fi");
    // Serial.println("IP address: ");
    // Serial.println(WiFi.localIP());

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

    // 初始化SPI ADXL模块
    Serial.println("Initializing ADXL");
    SPI.begin(SPI_ADXL345_SCL, SPI_ADXL345_MISO, SPI_ADXL345_SDA, SPI_ADXL345_CS);
    if (!accel.begin()) {
        Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
        while (1);
    }
    accel.setRange(ADXL345_RANGE_16_G);  // 设置ADXL 量程

    // 初始化LittleFS
    Serial.println("Initializing LittleFS");
    // if (!LittleFS.begin()) {
    if (!SPIFFS.begin()) {
        Serial.println("LittleFS mount failed");
        return;
    }

    // 初始化I2S MAX98357数字功放模块
    Serial.println("Initializing I2S audio module");
    if (!audio.setPinout(I2S_MAX98357_BCLK, I2S_MAX98357_LRC, I2S_MAX98357_DOUT, I2S_GPIO_UNUSED)) {
        Serial.println("Failed to initialize I2S pins");
        return;
    }
    // audio.setBufsize(0, 200 * 1024);  // 0表示使用PSRAM
    audio.setVolume(21);  // 音量范围0...21

    // 初始化I2S INMP441麦克风
    Serial.println("Initializing I2S Microphone module");
    i2s_install_inmp441();

    Serial.println("Setup complete");
}

void loop() {
    esp_task_wdt_reset();  // 喂狗操作
    detectButtons();       // 检测按键
    detectMicrophone();    // 处理麦克风
    detectADXL();          // 检测加速度传感器
    judgeActions();        // 判断动作
    ledHandler();          // 处理LED
    motorHandler();        // 处理震动马达
    audioHandler();        // 处理音频
}

// 检测按键
void detectButtons() {
    // 记录当前时间
    unsigned long currentTime = millis();

    int doubleCount = 0;

    // 判断按键状态，低电平：0，高电平：1
    buttonState = digitalRead(BUTTON_PIN) == LOW;
    buttonState = !buttonState;

    setTriggerState(TRIG_IDLE);

    // 按钮状态有变化
    if (buttonState != lastButtonState) {
        // 按键按下
        if (buttonState == 0) {
            lastPressTime = currentTime;  // 更新上次按下时间
            longPressTriggered = false;   // 按下按钮时重置长按标志

            // 按键松开
        } else {
            lastReleaseTime = currentTime;  // 更新上次释放时间

            // 判断等待双击标志
            if (waitingForDoubleClick == true) {
                // 判断双击条件
                if (currentTime - lastReleaseTime < DOUBLE_CLICK_TIME && !longPressTriggered) {
                    setTriggerState(TRIG_DBCLCK);   // 设置状态：双击
                    waitingForDoubleClick = false;  // 重置等待双击标志
                }
            } else {
                //  防止长按后检测到单击或双击
                if (!longPressTriggered) {
                    waitingForDoubleClick = true;  // 等待双击
                    lastActionTime = currentTime;  // 更新上次动作时间
                }
            }
            // 如果长按已触发，松开时不检测单击
            if (longPressTriggered) {
                waitingForDoubleClick = false;  // 重置等待双击标志
                longPressTriggered = false;     // 重置长按
                lastActionTime = currentTime;   // 更新上次动作时间 防止进入睡眠
            }
        }
        lastButtonState = buttonState;  // 更新上次按键状态
    }

    // 长按检测
    // 按键低电平 长按表示没有被触发
    if (buttonState == 0 && (currentTime - lastPressTime > LONG_PRESS_TIME) && longPressTriggered == false) {
        setTriggerState(TRIG_PRESS);    // 设置状态：长按
        longPressTriggered = true;      // 触发长按
        waitingForDoubleClick = false;  // 重置等待双击标志
        lastActionTime = currentTime;   // 更新上次动作时间 防止进入睡眠
    }

    // 睡眠检测
    // 按键高电平，超过睡眠时间，睡眠等待标志没有被触发
    if (buttonState == 1 && (currentTime - lastActionTime > SLEEP_TIME) && sleepTriggered == false) {
        setTriggerState(TRIG_SLEEP);   // 设置状态：睡眠
        sleepTriggered = true;         // 触发睡眠
        lastActionTime = currentTime;  // 更新上次动作时间 防止进入睡眠
    }

    // 单击检测
    // 按键释放等待第二次按下，超过双击等待时间
    if (waitingForDoubleClick && (currentTime - lastReleaseTime > DOUBLE_CLICK_TIME)) {
        setTriggerState(TRIG_CLICK);    // 设置状态：单击
        waitingForDoubleClick = false;  // 重置等待双击标志 防止单击后检测到双击
        lastActionTime = currentTime;   // 更新 lastActionTime 防止进入睡眠
    }

    // 如果有按键触发，则不触发睡眠状态
    if (buttonState == 0 || waitingForDoubleClick || longPressTriggered) {
        sleepTriggered = false;  // 重置睡眠标志
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
            sleepTriggered = false;

            // 打印大师剑和触发器状态，触发时间，加速度参数
            printAccelData(prevAccel, currAccel);
        }
    }

    // 更新上一次加速度值
    prevAccel[0] = accel_event.acceleration.x;
    prevAccel[1] = accel_event.acceleration.y;
    prevAccel[2] = accel_event.acceleration.z;
}
void detectMicrophone() {
    if (swordStatus != STATUS_DISPLAY || ledEffect != SOUND) return;
    unsigned long currentTime = millis();

    int16_t buffer[BUFFER_SIZE];
    size_t bytes_read;

    // 读取音频数据
    esp_err_t err = i2s_read(I2S_INMP441_PORT, buffer, sizeof(buffer), &bytes_read, portMAX_DELAY);
    if (err != ESP_OK) {
        Serial.printf("I2S read error: %d\n", err);
        return;
    }

    // 计算当前时间窗口的最大音量
    int16_t currentMaxVolume = 0;
    int32_t sampleCount = bytes_read / sizeof(int16_t);

    for (int i = 0; i < sampleCount; i++) {
        int16_t sample = abs(buffer[i]);
        if (sample > currentMaxVolume) {
            currentMaxVolume = sample;
        }
    }

    // 计算底噪水平并输出到串口
    // 只在调试时计算底噪水平，不在计算音量时使用
    if (currentTime - lastMicroTime >= TIME_WINDOW) {
        Serial.print("Noise Level: ");
        Serial.println(currentMaxVolume);  // 输出底噪水平供手动调整

        lastMicroTime = currentTime;

        // 更新最大音量
        maxVolume = currentMaxVolume;

        // 计算实际音量（减去底噪）
        int16_t adjustedVolume = maxVolume - (int16_t)noiseLevel;
        if (adjustedVolume < 0) {
            adjustedVolume = 0;  // 避免负值
        }

        // 将调整后的音量转换为百分比
        // 对于 16 位音频，最大值为 32767
        volumePercentage = (int)round((adjustedVolume * MICRO_SENSITIVITY * 327.7 / 32767.0) * 100.0);
        if (volumePercentage > 100) {
            volumePercentage = 100;  // 确保百分比不超过100%
        }

        // 重置最大音量以便于下一时间窗口的计算
        maxVolume = 0;

        // 输出调整后的音量百分比到串口
        // Serial.print("Volume Percentage: ");
        // Serial.println(volumePercentage);
    }
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
    } else if (swordStatus == STATUS_NORMAL) {
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
    } else if (swordStatus == STATUS_FIGHT) {
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
        if (triggerState == TRIG_SWING || triggerState == TRIG_CLICK) {
            setLedEffect(RETURN_LAST);        // 返回上一次效果
            setSwordStatus(lastSwordStatus);  // 返回上一次状态
        } else if (triggerState == TRIG_DBCLCK) {
            setLedEffect(SOUND);  // 拾音灯效果
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

    if (ledEffect == SOUND) {
        strip.setBrightness(LED_BRIGHT);  // 恢复亮度
        // 计算当前应点亮的 LED 数量
        int targetLitLEDs = round(strip.numPixels() * volumePercentage / 100);

        // 计算点亮的 LED 数量的平滑过渡
        int litLEDs = previousLitLEDs + (targetLitLEDs - previousLitLEDs) * LED_SMOOTH;  // 0.1 是平滑系数，值越大平滑过渡越快

        if (hueCount < 256 * 5 && (currentTime - lastHueTime > HUE_UPDATE_TIME)) {
            for (ledCount; ledCount < strip.numPixels(); ledCount++) {
                uint16_t hue = (ledCount * 65536L / strip.numPixels()) - (hueCount * 256);
                if (ledCount < strip.numPixels() - litLEDs) {
                    strip.setPixelColor(ledCount, OFF);
                } else {
                    strip.setPixelColor(ledCount, strip.gamma32(strip.ColorHSV(hue)));
                }
            }
            strip.show();
            ledCount = 0;
            hueCount++;
            lastHueTime = currentTime;
        } else if (hueCount >= 256 * 5) {
            hueCount = 0;
        }

        // 更新之前的点亮 LED 数量
        previousLitLEDs = litLEDs;
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
    if (swordStatus == STATUS_DISPLAY) return;
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
    if (swordStatus == STATUS_DISPLAY) return;
    unsigned long currentTime = millis();
    int number;
    audio.loop();
    if (currentTime < audioStartTime && audio.isRunning() == false && audioNumber != 0) {
        number = audioNumber;
        audioNumber = 0;
        Serial.print("Connecting to ");
        Serial.print(music_list[number]);
        Serial.println(" to play audio...");
        audio.connecttoFS(SPIFFS, music_list[number]);  // 使用LittleFS和MP3文件路径进行连接
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
            // Serial.println("TRIG_IDLE");
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
        case SOUND:
            Serial.println("SOUND");
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

void i2s_install_inmp441() {
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 8000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,  // 修改为 16 位
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false};

    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_INMP441_SCK,
        .ws_io_num = I2S_INMP441_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_INMP441_SD};

    esp_err_t err;
    err = i2s_driver_install(I2S_INMP441_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Failed to install I2S driver for INMP441: %d\n", err);
        return;
    }

    err = i2s_set_pin(I2S_INMP441_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("Failed to set I2S pins for INMP441: %d\n", err);
        return;
    }

    // err = i2s_set_clk(I2S_INMP441_PORT, 8000, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
    // if (err != ESP_OK) {
    //     Serial.printf("Failed to set I2S clock for INMP441: %d\n", err);
    //     return;
    // }
}
