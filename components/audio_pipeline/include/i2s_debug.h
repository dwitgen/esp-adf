#ifndef _I2S_DEBUG_H_
#define _I2S_DEBUG_H_

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Debug functions for I2S
void debug_i2s_data_flow(void);
i2s_chan_handle_t tx_handle = NULL;

#ifdef __cplusplus
}
#endif

#endif // _I2S_DEBUG_H_
