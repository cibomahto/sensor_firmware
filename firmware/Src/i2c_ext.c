#include "i2c.h"
#include "i2c_ext.h"

/**
  * @brief  Write bytes at a given address
  * @param  hi2c : handle of I2C
  * @param  dev_address: specifies the address of the device.
  * @param  data: pointer to data to be write
  * @param  size: number of bytes to be write.
  * @retval read status
  */
i2c_status_e i2c_master_write_it(I2C_HandleTypeDef hi2c, uint16_t dev_address,
                        uint8_t *data, uint16_t size)
{
  i2c_status_e ret = I2C_ERROR;
  uint32_t tick_start = HAL_GetTick();

  if (HAL_I2C_Master_Transmit_IT(&hi2c1, dev_address, data, size) == HAL_OK) {
    ret = I2C_OK;

    /* Wait for transfer completion */
    while ((HAL_I2C_GetState(&hi2c) != HAL_I2C_STATE_READY)
           && (ret != I2C_TIMEOUT)) {
      if ((HAL_GetTick() - tick_start) > I2C_TIMEOUT_TICK) {
        ret = I2C_TIMEOUT; /* timeout */
      }
    }
  }
  else {
    ret = I2C_ERROR;
  }

  return ret;
}

/**
  * @brief  read bytes in master mode at a given address
  * @param  hi2c: handle of I2C
  * @param  dev_address: specifies the address of the device.
  * @param  data: pointer to data to be read
  * @param  size: number of bytes to be read.
  * @retval read status
  */
i2c_status_e i2c_master_read_it(I2C_HandleTypeDef hi2c, uint16_t dev_address, uint8_t *data, uint16_t size)
{
  i2c_status_e ret = I2C_ERROR;
  uint32_t tick_start = HAL_GetTick();

  if (HAL_I2C_Master_Receive_IT(&hi2c1, dev_address, data, size) == HAL_OK) {
    ret = I2C_OK;
    // wait for transfer completion
    while ( (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
           && (ret != I2C_TIMEOUT) ) {
      if ((HAL_GetTick() - tick_start) > I2C_TIMEOUT_TICK) {
        ret = I2C_TIMEOUT;
      }
    }
  }
  else {
    ret = I2C_ERROR;
  }

  return ret;
}

void I2C_Error_Handler(void)
{
  /* TODO: a proper handling method */
}

void I2C_Timeout_Handler(void)
{
  /* TODO: a proper handling method */
}

