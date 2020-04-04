#include "hp203b.h"
#include "i2c.h"
#include "i2c_ext.h"

/*
 * ref
 * https://github.com/viktorpanasiuk/HP203B/blob/master/hp203b.c
 *
 */

/* Public variables ---------------------------------------------------------*/
HP203x_TH_TypeDef    HP203B_TH_Struct;
HP203x_CR_TypeDef    HP203B_CR_Struct;
HP203x_Data_TypeDef  hp203b_data;

uint8_t i2c_rx_buffer[I2C_BUFFER_SIZE];
uint8_t i2c_tx_buffer[I2C_BUFFER_SIZE];

/************************************************************
 * Initialize the structure of altitude offset and thresholds
 * - read comments for understanding values metric
 ***********************************************************/
static void hp203b_th_struct_init(HP203x_TH_TypeDef *HP203B_TH_InitStruct)
{
  HP203B_TH_InitStruct->ALT_OFF = 0;  // Altitude offset (cm)
  HP203B_TH_InitStruct->P_H_TH  = 0;  // Pressure upper bound threshold (Pa)
  HP203B_TH_InitStruct->P_M_TH  = 0;  // Pressure middle bound threshold (Pa)
  HP203B_TH_InitStruct->P_L_TH  = 0;  // Pressure lower bound threshold (Pa)
  HP203B_TH_InitStruct->A_H_TH  = 0;  // Altitude upper bound threshold (m)
  HP203B_TH_InitStruct->A_M_TH  = 0;  // Altitude middle bound threshold (m)
  HP203B_TH_InitStruct->A_L_TH  = 0;  // Altitude lower bound threshold (m)
  HP203B_TH_InitStruct->T_H_TH  = 0;  // Temperature upper bound threshold (°C)
  HP203B_TH_InitStruct->T_M_TH  = 0;  // Temperature middle bound threshold (°C)
  HP203B_TH_InitStruct->T_L_TH  = 0;  // Temperature lower bound threshold (°C)
  HP203B_TH_InitStruct->P_or_A  = 1;  // 0 for pressure, 1 for altitude
}

/************************************************************
 * Initialize the HP203B control registers structure
 * - read comments for understanding changes which you are can adjust
 ***********************************************************/
static void hp203b_cr_struct_init(HP203x_CR_TypeDef *HP203B_CR_InitStruct, HP203x_TH_TypeDef *HP203B_TH_Struct)
{
  hp203b_th_struct_init(HP203B_TH_Struct); // Configure the structure of HP203B thresholds
  HP203B_CR_InitStruct->ALT_OFF_LSB = (uint8_t)(HP203B_TH_Struct->ALT_OFF);      // Write LSB of altitude offset (1cm per count)
  HP203B_CR_InitStruct->ALT_OFF_MSB = (uint8_t)(HP203B_TH_Struct->ALT_OFF >> 8); // Write MSB of altitude offset

  if (!HP203B_TH_Struct->P_or_A) { // If P_or_A = 0 (pressure)
    HP203B_CR_InitStruct->PA_H_TH_LSB = (uint8_t)(HP203B_TH_Struct->P_H_TH >> 1);  // Write LSB for upper bound of pressure (0.02mbar or 2Pa per count)
    HP203B_CR_InitStruct->PA_H_TH_MSB = (uint8_t)(HP203B_TH_Struct->P_H_TH >> 9);  // Write MSB for upper bound of pressure
    HP203B_CR_InitStruct->PA_M_TH_LSB = (uint8_t)(HP203B_TH_Struct->P_M_TH >> 1);  // Write LSB for middle bound of pressure (0.02mbar or 2Pa per count)
    HP203B_CR_InitStruct->PA_M_TH_MSB = (uint8_t)(HP203B_TH_Struct->P_M_TH >> 9);  // Write MSB for middle bound of pressure
    HP203B_CR_InitStruct->PA_L_TH_LSB = (uint8_t)(HP203B_TH_Struct->P_L_TH >> 1);  // Write LSB for lower bound of pressure (0.02mbar or 2Pa per count)
    HP203B_CR_InitStruct->PA_L_TH_MSB = (uint8_t)(HP203B_TH_Struct->P_L_TH >> 9);  // Write MSB for lower bound of pressure
  }
  else { // else if P_or_A = 1 (altitude)
    HP203B_CR_InitStruct->PA_H_TH_LSB = (uint8_t)(HP203B_TH_Struct->A_H_TH);       // Write LSB for upper bound of altitude (1m per count)
    HP203B_CR_InitStruct->PA_H_TH_MSB = (uint8_t)(HP203B_TH_Struct->A_H_TH >> 8);  // Write MSB for upper bound of altitude
    HP203B_CR_InitStruct->PA_M_TH_LSB = (uint8_t)(HP203B_TH_Struct->A_M_TH);       // Write LSB for middle bound of altitude (1m per count)
    HP203B_CR_InitStruct->PA_M_TH_MSB = (uint8_t)(HP203B_TH_Struct->A_M_TH >> 8);  // Write MSB for middle bound of altitude
    HP203B_CR_InitStruct->PA_L_TH_LSB = (uint8_t)(HP203B_TH_Struct->A_L_TH);       // Write LSB for lower bound of altitude (1m per count)
    HP203B_CR_InitStruct->PA_L_TH_MSB = (uint8_t)(HP203B_TH_Struct->A_L_TH >> 8);  // Write MSB for lower bound of altitude
  }

  HP203B_CR_InitStruct->T_H_TH = HP203B_TH_Struct->T_H_TH;  // Write upper bound of temperature (1°C per count)
  HP203B_CR_InitStruct->T_M_TH = HP203B_TH_Struct->T_M_TH;  // Write middle bound of temperature (1°C per count)
  HP203B_CR_InitStruct->T_L_TH = HP203B_TH_Struct->T_L_TH;  // Write lower bound of temperature (1°C per count)

  HP203B_CR_InitStruct->INT_EN = // Disable/enable interrupt (0:disable,1:enable)
                               1 << PA_RDY_EN  | // Pressure/altitude is ready to reading
                               0 << T_RDY_EN   | // Temperature is ready to reading
                               0 << PA_TRAV_EN | // Pressure/altitude traversed the middle threshold
                               0 << T_TRAV_EN  | // Temperature traversed the middle threshold
                               0 << PA_WIN_EN  | // Pressure/altitude is outside predefined window
                               0 << T_WIN_EN;    // Temperature is outside predefined window

  HP203B_CR_InitStruct->INT_CFG = // Select pin for interrupt output (0:INT0,1:INT1)
                                HP203B_TH_Struct->P_or_A << PA_MODE |	// Selects whether the event detection parameters and the interrupts registers prefixed by
                                // a ‘PA_’ corresponds to the pressure or the altitude measurement (0:pressure,1:altitude)
                                0 << PA_RDY_CFG  |  // Pressure/altitude is ready to reading
                                0 << T_RDY_CFG   |  // Temperature is ready to reading
                                0 << PA_TRAV_CFG |  // Pressure/altitude traversed the middle threshold
                                0 << T_TRAV_CFG  |  // Temperature traversed the middle threshold
                                0 << PA_WIN_CFG  |  // Pressure/altitude is outside predefined window
                                0 << T_WIN_CFG;     // Temperature is outside predefined window

  HP203B_CR_InitStruct->INT_SRC =    // Flags that indicates interrupt status (read only)
                                0 << TH_ERR     |   // Indicates that improper settings for thresholds are set (lower bound above higher bound for example)
                                0 << DEV_RDY    |   // Indicates whether the HP203B is ready (in the sleep state and not performing any operation) or not (0:busy,1:ready)
                                0 << PA_RDY     |   // Pressure/altitude is ready to reading
                                0 << T_RDY      |   // Temperature is ready to reading
                                0 << PA_TRAV    |   // Pressure/altitude traversed the middle threshold
                                0 << T_TRAV     |   // Temperature traversed the middle threshold
                                0 << PA_WIN     |   // Pressure/altitude is outside predefined window
                                0 << T_WIN;         // Temperature is outside predefined window

  HP203B_CR_InitStruct->INT_DIR =    // Check details of traversal or window interrupt events (read only)
                                0 << CMPS_EN    |   // The data compensation enabled or disabled (0:disabled,1:enabled)
                                0 << P_TRAV_DIR |   // 1 if pressure rising from low to high, 0 if pressure falling from high to low (through middle level)
                                0 << T_TRAV_DIR |   // 1 if temperature rising from low to high, 0 if temperature falling from high to low (through middle level)
                                0 << P_WIN_DIR  |   // 1 if pressure above the window, 0 if pressure below the window
                                0 << T_WIN_DIR;     // 1 if temperature above the window, 0 if temperature below the window

  HP203B_CR_InitStruct->PARA = 1 << CMPS_EN;        // Enable/disable data compensation (0:disable,1:enable)
}

/************************************************************
 * Soft reset
 * - executed immediately and no matter what it has doing
 * - all memories reset to their default values
 * - automatically performed power-up sequence
 ***********************************************************/
void hp203b_soft_reset(void)
{
  i2c_status_e ret;
  uint8_t num_of_bytes = 1;

  i2c_tx_buffer[0] = SOFT_RST;

  // wait for the bus
  //while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY); // blocking

  ret = i2c_master_write_it(hi2c1, HP203B_ADDR, i2c_tx_buffer, num_of_bytes);
}


/************************************************************
 * Start convert
 * - start convert temperature/pressure/altitude to digital value
 * - OSR must be one of following:
					OSR4096
					OSR2048
					OSR1024
					OSR512
					OSR256
					OSR128
 * - ch(channel) must be one of following:
 * 					PT_CH
					T_CH
 ***********************************************************/
void hp203b_start_convert(uint8_t OSR, uint8_t ch)
{
  i2c_status_e ret;
  uint8_t num_of_bytes = 1;

  i2c_tx_buffer[0] = ADC_CVT | (OSR << 2) | ch; // command

  // wait for the bus
  //while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY); // blocking

  ret = i2c_master_write_it(hi2c1, HP203B_ADDR, i2c_tx_buffer, num_of_bytes);

  switch (ret) {
      case I2C_ERROR:
        I2C_Error_Handler();
        break;
      case I2C_TIMEOUT:
        I2C_Timeout_Handler();
        break;
      case I2C_BUSY:
        // TODO 
        break;
      case I2C_OK:
        break;
  }
}

/************************************************************
 * Get pressure and temperature
 * - read the temperature and the pressure values
 * - 3 bytes for temperature MSB first
 * - 3 bytes for pressure MSB first
 ***********************************************************/
void hp203b_read_pt(void)
{
  i2c_status_e ret;
  uint8_t num_of_bytes = 1;
  uint8_t num_bytes_to_read = 6;

  i2c_tx_buffer[0] = READ_PT; // command
  ret = i2c_master_write_it(hi2c1, HP203B_ADDR, i2c_tx_buffer, num_of_bytes); //write 1 byte

  ret = i2c_master_read_it(hi2c1, HP203B_ADDR, i2c_rx_buffer, num_bytes_to_read);

  // Save the temperature value to the data structure
  hp203b_data.T = (int32_t)i2c_rx_buffer[0]<<16 | (int32_t)i2c_rx_buffer[1]<<8 | (int32_t)i2c_rx_buffer[2];
  if (hp203b_data.T & (1 << 23)) // If temperature below 0
    hp203b_data.T |= 0xFF << 24; // rewrite 8 MSBs by 1

  // save the pressure value to the data structure
  hp203b_data.P = (uint32_t)i2c_rx_buffer[3]<<16 | (uint32_t)i2c_rx_buffer[4]<<8 | (uint32_t)i2c_rx_buffer[5];
}

/************************************************************
 * Get pressure
 * - read the pressure value
 * - 3 bytes for pressure MSB first
 ***********************************************************/
void hp203b_read_p(void)
{
  i2c_status_e ret;
  uint8_t num_of_bytes = 1;
  uint8_t num_bytes_to_read = 3;

  i2c_tx_buffer[0] = READ_P; // command
  // write 1 byte
  ret = i2c_master_write_it(hi2c1, HP203B_ADDR, i2c_tx_buffer, num_of_bytes);

  // then read 3 bytes
  ret = i2c_master_read_it(hi2c1, HP203B_ADDR, i2c_rx_buffer, num_bytes_to_read);

  // save the pressure value to the data structure
  hp203b_data.P = (uint32_t)i2c_rx_buffer[0]<<16 | (uint32_t)i2c_rx_buffer[1]<<8 | (uint32_t)i2c_rx_buffer[2];
}

/************************************************************
 * Get altitude
 * - read the altitude value
 * - 3 bytes for altitude MSB first
 ***********************************************************/
void hp203B_read_a(void)
{
  i2c_status_e ret;
  uint8_t num_of_bytes = 1;
  uint8_t num_bytes_to_read = 3;

  i2c_tx_buffer[0] = READ_A; // command
  // write 1 byte
  ret = i2c_master_write_it(hi2c1, HP203B_ADDR, i2c_tx_buffer, num_of_bytes);

  // then read 3 bytes
  ret = i2c_master_read_it(hi2c1, HP203B_ADDR, i2c_rx_buffer, num_bytes_to_read);

  // Save the altitude value to the data structure
  hp203b_data.H = (int32_t)i2c_rx_buffer[0]<<16 | (int32_t)i2c_rx_buffer[1]<<8 | (int32_t)i2c_rx_buffer[2];

  // If altitude below 0
  if (hp203b_data.H & (1<<23))
    hp203b_data.H |= 0xFF<<24; // Rewrite 8 MSBs by 1
}


/************************************************************
 * Get temperature
 * - read the temperature value
 * - 3 bytes for temperature MSB first
 ***********************************************************/
void hp203b_read_t(void)
{
  i2c_status_e ret;
  uint8_t num_of_bytes = 1;
  uint8_t num_bytes_to_read = 3;

  i2c_tx_buffer[0] = READ_T; // command
  // write 1 byte
  ret = i2c_master_write_it(hi2c1, HP203B_ADDR, i2c_tx_buffer, num_of_bytes);

  // then read 3 bytes
  ret = i2c_master_read_it(hi2c1, HP203B_ADDR, i2c_rx_buffer, num_bytes_to_read);

  // save the temperature value to the data structure
  hp203b_data.T = (int32_t)i2c_rx_buffer[0]<<16 | (int32_t)i2c_rx_buffer[1]<<8 | (int32_t)i2c_rx_buffer[2];

  if (hp203b_data.T & (1<<23)) // if temperature below 0
    hp203b_data.T |= 0xFF<<24; // rewrite 8 MSBs by 1
}

/************************************************************
 * Start internal calibration
 * - re-calibrate internal circuits
 * - use when environment rapidly changed
 * - this command allows increase the accurate
 * - no need execute it if environment is stable
 * - after finishes sensor enters to sleep mode
 ***********************************************************/
void hp203b_calibration(HP203x_CR_TypeDef *HP203B_CR_Struct)
{
  i2c_status_e ret;
  uint8_t num_of_bytes = 1;

  i2c_tx_buffer[0] = ANA_CAL; // command
  // write 1 byte
  ret = i2c_master_write_it(hi2c1, HP203B_ADDR, i2c_tx_buffer, num_of_bytes);

  // Checking DEV_RDY bit of the INT_SRC register and wait until it will set to 1
  while (1) {
    hp203b_read_reg(HP203B_CR_Struct, INT_SRC); // Read INT_SRC register from HP203B
    if (HP203B_CR_Struct->INT_SRC & (1<<DEV_RDY)) // Check DEV_RDY bit
      break; // Break if HP203B is ready (DEV_RDY = 1)
  }
}

/************************************************************
 * Read register from HP203B to control registers structure by address
 * - read internal register of HP203B by address
 * - save it to structure by appropriate address
 ***********************************************************/
void hp203b_read_reg(HP203x_CR_TypeDef *HP203B_CR_Struct, uint8_t address)
{
  i2c_status_e ret;
  uint8_t num_of_bytes = 1;
  uint8_t num_bytes_to_read = 1;

  i2c_tx_buffer[0] = READ_REG | address; // command
  // write 1 byte
  ret = i2c_master_write_it(hi2c1, HP203B_ADDR, i2c_tx_buffer, num_of_bytes);

  // read 1 byte
  ret = i2c_master_read_it(hi2c1, HP203B_ADDR, i2c_rx_buffer, num_bytes_to_read);

  // Save data of register to the structure by appropriate address
  *((uint8_t*)HP203B_CR_Struct + address) = i2c_rx_buffer[0];
}

/************************************************************
 * Write register to HP203B from control registers structure by address
 * - write data to internal register from the structure by address
 ***********************************************************/
void hp203b_write_reg(HP203x_CR_TypeDef *HP203B_CR_Struct, uint8_t address)
{
  i2c_status_e ret;
  uint8_t num_of_bytes = 2;

  i2c_tx_buffer[0] = WRITE_REG | address; // command
  i2c_tx_buffer[1] = *((uint8_t*)HP203B_CR_Struct + address); // data

  // write 2 byte
  ret = i2c_master_write_it(hi2c1, HP203B_ADDR, i2c_tx_buffer, num_of_bytes);
}

/************************************************************
 * Read all registers from HP203B to control registers structure
 ***********************************************************/
void hp203b_read_all_reg(HP203x_CR_TypeDef *HP203B_CR_Struct)
{
  uint8_t address;

  for (address = ALT_OFF_LSB; address <= PARA; address++)// For all registers
    hp203b_read_reg(HP203B_CR_Struct, address); // Save their to the structure one by one
}

/************************************************************
 * Write all registers from the structure to HP203B
 ***********************************************************/
void hp203b_write_all_reg(HP203x_CR_TypeDef *HP203B_CR_Struct)
{
  uint8_t address;

  // For addresses from LT_OFF_LSB to INT_SRC
  for (address = ALT_OFF_LSB; address <= INT_SRC; address++) {
    // Write data from the structure to appropriate register by address
    hp203b_write_reg(HP203B_CR_Struct, address);
  }

  // Write data from the structure to PARA register
  hp203b_write_reg(HP203B_CR_Struct, PARA);
}


/* Initialization sequence for HP203B sensor */
void hp203b_init(HP203x_CR_TypeDef *HP203B_CR_Struct, HP203x_TH_TypeDef *HP203B_TH_Struct)
{
  // Checking DEV_RDY bit of the INT_SRC register and wait until it will set to 1
  while (1) {
    hp203b_read_reg(HP203B_CR_Struct, INT_SRC);     // Read INT_SRC register from HP203B
    if (HP203B_CR_Struct->INT_SRC & (1 << DEV_RDY)) // Check DEV_RDY bit
      break;                                        // Break if HP203B is ready (DEV_RDY = 1)
  }

  hp203b_cr_struct_init(HP203B_CR_Struct, HP203B_TH_Struct); // Configure the structure of HP203B registers
  hp203b_write_all_reg(HP203B_CR_Struct);  // Copy this structure into internal registers of HP203B
}

