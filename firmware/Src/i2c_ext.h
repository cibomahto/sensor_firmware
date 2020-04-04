#ifndef __I2C_EXT_H
#define __I2C_EXT_H

#define I2C_BUFFER_SIZE 6

#define TRUE  1
#define FALSE 0

/* I2C timout in tick unit */
#define I2C_TIMEOUT_TICK 100

/* I2C state */
typedef enum {
  I2C_OK = 0,
  I2C_TIMEOUT = 1,
  I2C_ERROR = 2,
  I2C_BUSY = 3
} i2c_status_e;

i2c_status_e i2c_master_write_it(I2C_HandleTypeDef hi2c, uint16_t dev_address, uint8_t *data, uint16_t size);
i2c_status_e i2c_master_read_it(I2C_HandleTypeDef hi2c, uint16_t dev_address, uint8_t *data, uint16_t size);

void I2C_Timeout_Handler(void);
void I2C_Error_Handler(void);


#endif /* __I2C_EXT_H */
