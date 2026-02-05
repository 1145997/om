#include "ASR_module.h"



static bool WireWriteByte(uint8_t val)
{
    Wire.beginTransmission(I2C_ADDR);
    Wire.write(val);
    if( Wire.endTransmission() != 0 ) {
        return false;
    }
    return true;
}

static bool WireWriteDataArray(  uint8_t reg,uint8_t *val,unsigned int len)
{
    unsigned int i;

    Wire.beginTransmission(I2C_ADDR);
    Wire.write(reg);
    for(i = 0; i < len; i++) {
        Wire.write(val[i]);
    }
    if( Wire.endTransmission() != 0 ) {
        return false;
    }
    return true;
}

static int WireReadDataArray(   uint8_t reg, uint8_t *val, unsigned int len)
{
    unsigned char i = 0;  
    /* Indicate which register we want to read from */
    if (!WireWriteByte(reg)) {
        return -1;
    }
    Wire.requestFrom(I2C_ADDR, len);
    while (Wire.available()) {
        if (i >= len) {
            return -1;
        }
        val[i] = Wire.read();
        i++;
    }
    /* Read block data */    
    return i;
}

ASR_MOUDLE::ASR_MOUDLE(void)
{
  send[0] = 0;
  send[1] = 0;
  Wire.begin();
}

uint8_t ASR_MOUDLE::rec_recognition(void)
{
  uint8_t result = 0;
  WireReadDataArray(ASR_RESULT_ADDR,&result,1);
  return result;
}

void ASR_MOUDLE::speak(uint8_t cmd , uint8_t id)
{
  if(cmd == 0xFF || cmd == 0x00)
  {
    send[0] = cmd;
    send[1] = id;
    WireWriteDataArray(ASR_SPEAK_ADDR , send , 2);
  }
}

void ASR_MOUDLE::ASR_init(){
    Wire.begin(SDA_PIN, SCL_PIN); 
}
