本代码为基于platformio的esp32s3通过arduino平台制作的塞尔达大师光剑
只有完成了动作之后，才能触发其他状态
开机状态 剑身不亮，单击触发震动，逐渐点亮蓝色剑身，并播放001.wav 之后进入通常状态
通常状态 剑身蓝色，挥动触发LED灯效，双击更改LED特效，长按进入战斗状态，触发震动，并播放002.wav
战斗状态 剑身红色，挥动触发LED灯效，双击更改LED特效，单击返回普通模式，触发震动，并播放003.wav
本项目通过adxl345检测加速度 ，I2S MAX98357数字功放模块 播放音频,52颗ws2812灯带作为光剑效果，并使用GPIO通过mos管控制微型震动马达
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino
board_build.filesystem = littlefs
board_build.arduino.partitions = ./default_16MB.csv
board_build.arduino.memory_type = qio_opi
build_flags =
	-DBOARD_HAS_PSRAM
	-DARDUINO_USB_CDC_ON_BOOT=1
board_upload.flash_size = 16MB
monitor_speed = 115200
lib_deps =
	adafruit/Adafruit NeoPixel@^1.12.2
	adafruit/Adafruit Unified Sensor@^1.1.14
	adafruit/Adafruit ADXL345@^1.3.4
	briand/LibBriandIDF@^1.5.0
	esphome/ESP32-audioI2S@^2.0.7
需求如下
大师剑灯光的几种状态
开机状态，0.5秒内震动2次，缓慢逐渐点亮剑身至全量为冰蓝色
通常状态，剑身冰蓝色
战斗状态，剑身红色
空闲状态，表示音频或者效果触发结束
顺时针扭转触发器，在通常状态，顺时针扭转剑身，剑身变为红色，状态变为战斗状态，0.5秒内震动1次
逆时针扭转触发器，在战斗状态，逆时针扭转剑身，剑身变为冰蓝色，状态变为战斗状态，0.5秒内震动1次
挥动触发器，剑身变亮0.5秒之后返回原来亮度
前刺触发器，在原有状态颜色的基础上，增加1条极光色的追踪效果，追踪方向从第一个灯珠移动到结尾，保持4个led长度不变