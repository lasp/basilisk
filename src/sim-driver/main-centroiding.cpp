#include "simulationDriver.h"
#include "imageFlatten/image_bit_mask_rows1000_by_columns1000.h"
//#include "imageFlatten/image_bit_mask_rows8_by_columns8.h"
//#include "imageFlatten/image_bit_mask_rows20_by_columns20.h"
//#include "imageFlatten/image_bit_mask_rows270_by_columns480.h"

uint8_t getPixelFromImage(uint16_t row, uint16_t column, Image *image);

int main (int argc, char* argv[] ) {
    auto image = Image{};

    uint32_t sum_x = 0;
    uint32_t sum_y = 0;
    uint32_t sum_image = 0;
    for (uint16_t row = 0; row < image.imageRows; ++row) {
        for (uint16_t col = 0; col < image.imageColumns; ++col) {
            uint8_t value = getPixelFromImage(row, col, &image);
            sum_x += col*value;
            sum_y += row*value;
            sum_image += value;
        }
    }
    uint32_t centroid_x = sum_x/sum_image;
    uint32_t centroid_y = sum_y/sum_image;
    std::cout << "centroid_x: " << centroid_x << std::endl;
    std::cout << "centroid_y: " << centroid_y << std::endl;
}

uint8_t getPixelFromImage(uint16_t row, uint16_t column, Image *image) {
    // TODO assert row and column are in range
    // Get byte index into image array
    uint32_t byte_idx = (row*image->imageColumns + column) / 8;
    // Get bit index into the byte
    uint8_t bit_idx = (row*image->imageColumns + column) % 8;
    // Get the byte
    uint8_t value_byte = image->imageBitMask[byte_idx];
    // Get the bit
    uint8_t value_bit = (value_byte >> (7 - bit_idx)) & 0x1;
    return value_bit;
}
