#ifndef UART_IN_H
#define UART_IN_H
#include <Arduino.h>

// UART 命令字（从语音模块过来的）
#define CMD_test_1   0x0001
#define CMD_test_2   0x0004
#define CMD_test_3  0x0005
#define CMD_test_4      0x0006
#define CMD_BROADCAST_ON  0x0900
#define CMD_BROADCAST_OFF 0x0A00

// 返回动作编号（内部系统动作）
#define test_1  1
#define test_2 2
#define test_3     3
#define test_4   4
#define ACT_LED_OFF  5
#define LED_ON       6
#define ACT_UNKNOWN  0

uint8_t parseAction(uint16_t cmd);
bool readCmdFromUART(uint16_t &cmd);
void uartInInit(uint32_t baud, int rxPin, int txPin);





#endif