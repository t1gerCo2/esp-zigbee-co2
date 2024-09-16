#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "ulp_lp_core_i2c.h"
#include "ulp_lp_core_uart.h" /* ULP serial output */
#include "ulp_lp_core_utils.h"

#include "../SensirionI2CScd4x.h"

#define LP_UART_PORT_NUM    LP_UART_NUM_0


#define LP_I2C_TRANS_TIMEOUT_CYCLES 5000
#define LP_I2C_TRANS_WAIT_FOREVER   -1
