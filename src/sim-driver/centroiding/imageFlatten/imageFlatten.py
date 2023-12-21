import numpy as np
import cv2


if __name__ == '__main__':
    image_path = "full_circle_8.jpg"
    image_data = np.array(cv2.imread(image_path))
    [rows, columns, depth] = image_data.shape
    bw_image = cv2.cvtColor(image_data, cv2.COLOR_BGR2GRAY)
    ret, bit_mask = cv2.threshold(bw_image, 50, 1, cv2.THRESH_BINARY)

    # calculate x,y coordinate of center for validation later
    sum_x = 0
    sum_y = 0
    sum_image = 0
    for row in range(rows):
        for col in range(columns):
            pixel = bw_image[row, col]
            sum_x += col*pixel
            sum_y += row*pixel
            sum_image += pixel
    centroid_x = sum_x/sum_image
    centroid_y = sum_y/sum_image
    print(centroid_x, centroid_y)

    flattened_bit_mask = bit_mask.flatten()
    # pad the final byte
    num_bytes = int(flattened_bit_mask.size/8)
    binary_string = ''.join(map(str, flattened_bit_mask))
    bits_to_pad = flattened_bit_mask.size % 8
    if bits_to_pad > 0:
        num_bytes += 1
        for i in range(bits_to_pad):
            binary_string += '0'

    bit_mask = []
    for i in range(num_bytes):
        bits = binary_string[i*8:i*8+8]
        value = int(bits, 2)
        bit_mask.append(value)

    f = open("image_bit_mask_rows{0}_by_columns{1}.h".format(rows, columns), "w")
    f.write('#include <cstdint> \n')

    f.write('typedef struct { \n')
    f.write('uint16_t imageRows = {0}; \n'.format(str(rows)))
    f.write('uint16_t imageColumns = {0}; \n'.format(str(columns)))
    f.write('volatile uint8_t imageBitMask[{size}] = '.format(size=str(num_bytes)))
    f.write('{')
    f.write('{value}'.format(value=str(bit_mask[0])))
    for i in range(1, num_bytes):
        f.write(', {value}'.format(value=str(bit_mask[i])))
    f.write('}; \n')
    f.write('}')
    f.write(' Image; \n'.format(rows, columns))
    f.close()
