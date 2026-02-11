#ifndef __ASR_MODULE_H
#define __ASR_MODULE_H

#include <Wire.h>

#define I2C_ADDR		0x34
//识别结果存放处，通过不断读取此地址的值判断是否识别到语音，不同的值对应不同的语音
#define ASR_RESULT_ADDR   100
#define ASR_SPEAK_ADDR    110

#define ASR_CMDMAND    0x00
#define ASR_ANNOUNCER  0xFF

#define SDA_PIN 48  // 定义 SDA 引脚
#define SCL_PIN 47  // 定义 SCL 引脚

class ASR_MOUDLE
{
  public:
    ASR_MOUDLE(void);
    uint8_t rec_recognition(void);
    void speak(uint8_t cmd , uint8_t id);
    void ASR_init();

  private:
    uint8_t send[2];
};

#endif //__ASR_MODULE_H

