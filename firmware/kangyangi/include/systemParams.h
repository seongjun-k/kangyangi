#include <Arduino.h>

// Dynamixel Variables
bool recordData = false;
size_t masterSize;
size_t smallerSize = 4;
uint16_t* rData = nullptr;

// Debug mode
bool debugMode = false;
