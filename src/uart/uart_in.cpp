#include "uart_in.h"
HardwareSerial VoiceSerial(2);

void uartInInit(uint32_t baud, int rxPin, int txPin) {
    VoiceSerial.begin(baud, SERIAL_8N1, rxPin, txPin);
}

uint8_t parseAction(uint16_t cmd) {
  switch (cmd) {
        case CMD_test_1:     return test_1;
        case CMD_test_2:     return test_2;
        case CMD_test_3:     return test_3;
        case CMD_test_4:     return test_4;
        default:               return ACT_UNKNOWN;
  }
}



bool readCmdFromUART(uint16_t &cmd) {
    static uint8_t frame[5];
    static uint8_t index = 0;

    while (VoiceSerial.available() > 0) {
        uint8_t b = VoiceSerial.read();

        if (index == 0 && b != 0xAA) continue;
        if (index == 1 && b != 0x55) { index = 0; continue; }
        if (index == 4 && b != 0xFB) { index = 0; continue; }

        frame[index++] = b;

        if (index == 5) {
            index = 0;
            cmd = (frame[2] << 8) | frame[3];
            return true;
        }
    }
    return false;
}
