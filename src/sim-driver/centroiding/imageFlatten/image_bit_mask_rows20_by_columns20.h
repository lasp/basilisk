#include <cstdint> 
typedef struct { 
uint16_t imageRows = 20; 
uint16_t imageColumns = 20; 
volatile uint8_t imageBitMask[50] = {0, 0, 0, 6, 0, 3, 252, 0, 127, 224, 15, 255, 1, 255, 248, 63, 255, 195, 255, 252, 63, 255, 199, 255, 254, 127, 255, 227, 255, 252, 63, 255, 195, 255, 252, 31, 255, 128, 255, 240, 7, 254, 0, 63, 192, 0, 96, 0, 0, 0}; 
} Image; 
