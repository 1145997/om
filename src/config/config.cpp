#include "config.h"

void config_init(){
    pinMode(Leaser_pin_1,OUTPUT);
    pinMode(Leaser_pin_2,OUTPUT);
    pinMode(Leaser_pin_3,OUTPUT);
    pinMode(Radar_pin,OUTPUT);
    digitalWrite(Leaser_pin_1, LOW);
    digitalWrite(Leaser_pin_2, LOW);
    digitalWrite(Leaser_pin_3, LOW);
    digitalWrite(Radar_pin, HIGH);
}


