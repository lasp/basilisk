#include <cstdint> 
typedef struct { 
uint16_t imageRows = 8; 
uint16_t imageColumns = 8; 
volatile uint8_t imageBitMask[8] = {24, 126, 126, 255, 255, 126, 126, 24}; 
} Image; 
