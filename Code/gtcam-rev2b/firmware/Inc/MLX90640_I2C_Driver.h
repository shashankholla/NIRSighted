#ifndef _MLX90640_I2C_DRIVER_H
#define _MLX90640_I2C_DRIVER_H

#include <stdint.h>

int  MLX90640_I2CRead(uint8_t slaveAddr,uint16_t startAddress, uint16_t nMemAddressRead, uint16_t *data);
int  MLX90640_I2CWrite(uint8_t slaveAddr,uint16_t writeAddress, uint16_t data);

#endif
