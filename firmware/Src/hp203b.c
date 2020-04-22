#include "hp203b.h"
#include "i2c.h"

#define HP203B_I2C_ADDR 0xEE ///!< CSB pulled to GND
//#define HP203B_I2C_ADDR 0xEC ///!< CSB pulled to VCC


#define HP203B_CMD_SOFT_RST 0x06  ///!< Soft reset the device
///! Perform an ADC conversion
#define HP203B_CMD_ADC_CVT(osr, channel) \
     ((0b010 << 5) | ((osr & 0b111) << 2) | (channel & 0b11))

#define HP203B_CMD_READ_PT 0x10 ///!< Read the temperature and pressure values
//#define HP203B_CMD_READ_AT 0x11 ///!< Read the temperature and altiutude values
//#define HP203B_CMD_READ_P 0x30 ///!< Read the pressure value only
//#define HP203B_CMD_READ_A 0x31 ///!< Read the altitude value only
//#define HP203B_CMD_READ_T 0x32 ///!< Read the temperature value only
//#define HP203B_CMD_ANA_CAL 0x28 ///!< Re-calibrate the internal analog blocks
///!< Read out the control registers
#define HP203B_CMD_READ_RED(address) \
    ((0b10 << 6) | (address & 0b111111))
///!< Write in the control registers
#define HP203B_CMD_WRITE_REG(address) \
    ((0b11 << 6) | (address & 0b111111))

#define HP203B_ADC_OSR_128 0b101 ///!< Fastest conversion (4.1ms for temp+pressure)
#define HP203B_ADC_OSR_256 0b100 ///!<
#define HP203B_ADC_OSR_512 0b011 ///!<
#define HP203B_ADC_OSR_1024 0b010 ///!<
#define HP203B_ADC_OSR_2048 0b001 ///!<
#define HP203B_ADC_OSR_4096 0b000 ///!< Slowest conversion (131.1ms for temp + pressure)

#define HP203B_ADC_CHNL_P_T 0b00 ///!< Measure pressure and temperature
#define HP203B_ADC_CHNL_T 0b10 ///!< Measure only the temperature

hp203b_error_t hp203b_init() {
  HAL_StatusTypeDef ret;

  // Init sequence from HP203B Product Development Guide
  HAL_Delay(100);

  const uint8_t cmd[] = {HP203B_CMD_SOFT_RST};
  ret = HAL_I2C_Master_Transmit(&hi2c1, HP203B_I2C_ADDR, cmd, sizeof(cmd), HAL_MAX_DELAY);
  if ( ret != HAL_OK )
    return HP203B_ERROR_COMMS;

  HAL_Delay(10);

  // TODO: Check DEV_RDY = 1, retry if failed
  //return HP203B_ERROR_FAIL;
  
  return HP203B_ERROR_OK;
}

hp203b_error_t hp203b_start_read_temp_pressure() {
  HAL_StatusTypeDef ret;

  // Make a fast conversion, then wait for a result
  const uint8_t cmd[] = {HP203B_CMD_ADC_CVT(HP203B_ADC_OSR_128, HP203B_ADC_CHNL_P_T)};
  ret = HAL_I2C_Master_Transmit(&hi2c1, HP203B_I2C_ADDR, cmd, sizeof(cmd), HAL_MAX_DELAY);
  if ( ret != HAL_OK )
    return HP203B_ERROR_COMMS;
}
  
hp203b_error_t hp203b_read_temp_pressure(uint32_t *temperature, uint32_t *pressure) {
  HAL_StatusTypeDef ret;

  // TODO: Check that the conversion was completed

  const uint8_t cmd[] = {HP203B_CMD_READ_PT};
  ret = HAL_I2C_Master_Transmit(&hi2c1, HP203B_I2C_ADDR, cmd, sizeof(cmd), HAL_MAX_DELAY);
  if ( ret != HAL_OK )
    return HP203B_ERROR_COMMS;

  uint8_t buff[6];
  ret = HAL_I2C_Master_Receive(&hi2c1, HP203B_I2C_ADDR, buff, sizeof(buff), HAL_MAX_DELAY);
  if ( ret != HAL_OK )
    return HP203B_ERROR_COMMS;

  // First 3 bytes are temperature (TODO: sign extend this
  *temperature = (buff[0] << 16) | (buff[1] << 8) | (buff[2]);
  /// Next 3 bytes are pressure
  *pressure = (buff[3] << 16) | (buff[4] << 8) | (buff[5]);
  

  return HP203B_ERROR_OK;
}
