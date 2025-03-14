/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2017 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS products only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#ifndef _IOT_I2C_BUS_H_
#define _IOT_I2C_BUS_H_

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

#include "audio_idf_version.h"

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
#include "driver/i2c_master.h"
#endif  /* (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)) */

#include "driver/i2c.h"

typedef void *i2c_bus_handle_t;
typedef void (*i2c_run_cb_t)(i2c_port_t port, void *arg);

/**
 * @brief  Create and init I2C bus and return a I2C bus handle
 *
 * @param  port  I2C port number
 * @param  conf  Pointer to I2C parameters
 *
 * @return
 *       - I2C  bus handle
 */
i2c_bus_handle_t i2c_bus_create(i2c_port_t port, i2c_config_t *conf);

#if (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0))
/**
 * @brief  Set I2S master bus handle
 *
 * @param  port           I2C port number
 * @param  master_handle  I2C master bus handle.
 *
 * @return
 *       - ESP_OK  on success
 */
esp_err_t i2c_bus_set_master_handle(i2c_port_t port, i2c_master_bus_handle_t master_handle);

/**
 * @brief  Get I2S master bus handle
 *
 * @param  port  I2C port number
 *
 * @return
 *       - I2C  master bus handle.
 */
i2c_master_bus_handle_t i2c_bus_get_master_handle(i2c_port_t port);

/**
 * @brief  Read certain bytes of data from I2C bus by address
 *
 *         ___________________________________________________
 *         | start | slave_addr + rd_bit + ack | .....  | stop |
 *         --------|---------------------------|-- -----|------|
 *
 * @note  Directly reads the data in the registers address without a write action
 *
 * @param  bus      I2C bus handle
 * @param  addr     The address of the device
 * @param  outdata  The outdata pointer
 * @param  datalen  The length of outdata
 *
 * @return
 *       - ESP_OK    Success
 *       - ESP_FAIL  Fail
 */
esp_err_t i2c_bus_read_bytes_directly(i2c_bus_handle_t bus, int addr, uint8_t *outdata, int datalen);

/**
 * @brief  Set I2C bus clock frequency
 *
 * @note  This function need called before any i2c bus read or write operation
 *
 * @param  bus        I2C bus handle
 * @param  clk_speed  I2C clk frequency
 *
 * @return
 *       - ESP_OK    Success
 *       - ESP_FAIL  Fail
 */
esp_err_t i2c_bus_set_clk(i2c_bus_handle_t bus, uint32_t clk_speed);

#else

/**
 * @brief  I2C start sending buffered commands
 *
 * @param  bus            I2C bus handle
 * @param  cmd            I2C cmd handle
 * @param  ticks_to_wait  Maximum blocking time
 *
 * @return
 *       - ESP_OK    Success
 *       - ESP_FAIL  Fail
 */
esp_err_t i2c_bus_cmd_begin(i2c_bus_handle_t bus, i2c_cmd_handle_t cmd, portBASE_TYPE ticks_to_wait);

#endif  /* (ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)) */

/**
 * @brief  Write bytes to I2C bus
 *
 * @param  bus      I2C bus handle
 * @param  addr     The address of the device
 * @param  reg      The register of the device
 * @param  regLen   The length of register
 * @param  data     The data pointer
 * @param  datalen  The length of data
 *
 * @return
 *       - NULL    Fail
 *       - Others  Success
 */
esp_err_t i2c_bus_write_bytes(i2c_bus_handle_t bus, int addr, uint8_t *reg, int regLen, uint8_t *data, int datalen);

/**
 * @brief  Write data to I2C bus
 *
 * @param  bus      I2C bus handle
 * @param  addr     The address of the device
 * @param  data     The data pointer
 * @param  datalen  The length of data
 *
 * @return
 *       - NULL    Fail
 *       - Others  Success
 */
esp_err_t i2c_bus_write_data(i2c_bus_handle_t bus, int addr, uint8_t *data, int datalen);

/**
 * @brief  Read bytes to I2C bus
 *
 * @param  bus      I2C bus handle
 * @param  addr     The address of the device
 * @param  reg      The register of the device
 * @param  regLen   The length of register
 * @param  outdata  The outdata pointer
 * @param  datalen  The length of outdata
 *
 * @return
 *       - NULL    Fail
 *       - Others  Success
 */
esp_err_t i2c_bus_read_bytes(i2c_bus_handle_t bus, int addr, uint8_t *reg, int reglen, uint8_t *outdata, int datalen);

/**
 * @brief  Delete and release the I2C bus object
 *
 * @param  bus  I2C bus handle
 *
 * @return
 *       - ESP_OK    Success
 *       - ESP_FAIL  Fail
 */
esp_err_t i2c_bus_delete(i2c_bus_handle_t bus);

/**
 * @brief  Auto probe the I2C device
 *
 * @param  bus   I2C bus handle
 * @param  addr  I2C 8bit address
 *
 * @return
 *       - ESP_OK    Found a I2C device
 *       - ESP_FAIL  Fail
 */
esp_err_t i2c_bus_probe_addr(i2c_bus_handle_t bus, uint8_t addr);

/**
 * @brief  Lock the I2C bus while executing the given callback
 *
 * @param  bus  I2C bus handle
 * @param  cb   The callback to execute
 * @param  arg  The argument for the callback
 *
 * @return
 *       - ESP_OK    Done calling callback function
 *       - ESP_FAIL  Fail
 */
esp_err_t i2c_bus_run_cb(i2c_bus_handle_t bus, i2c_run_cb_t cb, void *arg);

#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif  /* _IOT_I2C_BUS_H_ */
