#pragma once

#include <stdint.h>

typedef enum {
    HP203B_ERROR_OK, //!< No error
    HP203B_ERROR_FAIL, //!< Generic error
    HP203B_ERROR_COMMS,	//!< Error in I2C communication
} hp203b_error_t;

hp203b_error_t hp203b_init();

hp203b_error_t hp203b_start_read_temp_pressure();

//! @brief Makes a blocking call to read the temperature and pressure.
//!
//! @param[out] temperature Current temperature (units?)
//! @param[out] pressure Current pressure (units?)
//! @return HP203B_ERROR_OK on success
hp203b_error_t hp203b_read_temp_pressure(uint32_t *temperature, uint32_t *pressure);
