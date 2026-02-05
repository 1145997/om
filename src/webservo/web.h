#ifndef web_in_h
#define web_in_h
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include "../servo/servo_in.h"

extern Servo E; 
extern Servo Y; 
extern Servo Z; 


void wifi_init();




#endif