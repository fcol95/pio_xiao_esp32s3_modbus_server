/*
 * SPDX-FileCopyrightText: 2016-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/*=====================================================================================
 * Description:
 *   C file to define parameter storage instances
 *====================================================================================*/
#include "modbus_params.h"

#include <math.h>

#include "freertos/semphr.h"

// Defines below are used to define register start address for each type of Modbus registers
#define MODBUS_PARAMS_MUTEX_TIMEOUT_MS 10U

// Here are the user defined instances for device parameters packed by 1 byte
// These are keep the values that can be accessed from Modbus master
#pragma pack(push, 1)
typedef struct
{
    float    floats[MODBUS_PARAMS_INPUT_REGISTER_FLOAT_COUNT];
    uint16_t uints[MODBUS_PARAMS_INPUT_REGISTER_UINT_COUNT];
} input_reg_params_t;
#pragma pack(pop)
typedef struct
{
    SemaphoreHandle_t floats[MODBUS_PARAMS_INPUT_REGISTER_FLOAT_COUNT];
    SemaphoreHandle_t uints[MODBUS_PARAMS_INPUT_REGISTER_UINT_COUNT];
} input_reg_params_mutexes_t;

#pragma pack(push, 1)
typedef struct
{
    float    floats[MODBUS_PARAMS_HOLDING_REGISTER_FLOAT_COUNT];
    uint16_t uints[MODBUS_PARAMS_HOLDING_REGISTER_UINT_COUNT];
} holding_reg_params_t;
#pragma pack(pop)
typedef struct
{
    SemaphoreHandle_t floats[MODBUS_PARAMS_HOLDING_REGISTER_FLOAT_COUNT];
    SemaphoreHandle_t uints[MODBUS_PARAMS_HOLDING_REGISTER_UINT_COUNT];
} holding_reg_params_mutexes_t;

#pragma pack(push, 1)
typedef struct
{
    bool ports[MODBUS_PARAMS_COIL_PORTS_COUNT];
} coil_reg_params_t;
#pragma pack(pop)
typedef struct
{
    SemaphoreHandle_t ports[MODBUS_PARAMS_COIL_PORTS_COUNT];
} coil_reg_params_mutexes_t;

#pragma pack(push, 1)
typedef struct
{
    bool ports[MODBUS_PARAMS_DISCRETE_INPUT_PORTS_COUNT];
} discrete_input_reg_params_t;
#pragma pack(pop)
typedef struct
{
    SemaphoreHandle_t ports[MODBUS_PARAMS_DISCRETE_INPUT_PORTS_COUNT];
} discrete_input_reg_params_mutexes_t;

input_reg_params_t         input_reg_params = {0};
input_reg_params_mutexes_t input_reg_params_mutexes = {0};

holding_reg_params_t         holding_reg_params = {0};
holding_reg_params_mutexes_t holding_reg_params_mutexes = {0};

coil_reg_params_t         coil_params = {0};
coil_reg_params_mutexes_t coil_params_mutexes = {0};

discrete_input_reg_params_t         discrete_input_params = {0};
discrete_input_reg_params_mutexes_t discrete_input_params_mutexes = {0};

// TODO: Add logic to have min/max values for parameters

// Utils
// Get a single bit's value
// Usage:
//    assert(get_bit(0x01, 0) == 0b1);
//    assert(get_bit(0x01, 1) == 0b0);
static inline uint32_t get_bit(uint32_t in, int8_t bit_index)
{
    return (((in) >> (bit_index)) & 1UL); //
}

// Set a single bit's value
// Usage:
//    assert(set_bit(0x0F, 1, 7) == 0b10001111);
//    assert(set_bit(0x0F, 0, 0) == 0b00001110);
static inline uint32_t set_bit(uint32_t in, bool bit_value, uint8_t bit_index)
{
    return ((in) ^ ((-(bit_value) ^ (in)) & (1U << (bit_index))));
}

// Set register values into known state
static esp_err_t setup_reg_data(void)
{
    // Define initial state of parameters
    for (ModbusParams_InReg_Float_t float_ind = (ModbusParams_InReg_Float_t)0;
         float_ind < MODBUS_PARAMS_INPUT_REGISTER_FLOAT_COUNT;
         float_ind++)
    {
        input_reg_params.floats[float_ind] = NAN;
        input_reg_params_mutexes.floats[float_ind] = xSemaphoreCreateMutex();
        if (input_reg_params_mutexes.floats[float_ind] == NULL) return ESP_FAIL;
    }
    for (ModbusParams_InReg_UInt_t uint_ind = (ModbusParams_InReg_UInt_t)0;
         uint_ind < MODBUS_PARAMS_INPUT_REGISTER_UINT_COUNT;
         uint_ind++)
    {
        input_reg_params.uints[uint_ind] = 0x0;
        input_reg_params_mutexes.uints[uint_ind] = xSemaphoreCreateMutex();
        if (input_reg_params_mutexes.uints[uint_ind] == NULL) return ESP_FAIL;
    }
    for (ModbusParams_HoldReg_UInt_t uint_ind = (ModbusParams_HoldReg_UInt_t)0;
         uint_ind < MODBUS_PARAMS_HOLDING_REGISTER_UINT_COUNT;
         uint_ind++)
    {
        holding_reg_params.uints[uint_ind] = 0x0;
        holding_reg_params_mutexes.uints[uint_ind] = xSemaphoreCreateMutex();
        if (holding_reg_params_mutexes.uints[uint_ind] == NULL) return ESP_FAIL;
    }
    for (ModbusParams_HoldReg_Float_t float_ind = (ModbusParams_HoldReg_Float_t)0;
         float_ind < MODBUS_PARAMS_HOLDING_REGISTER_FLOAT_COUNT;
         float_ind++)
    {
        holding_reg_params.floats[float_ind] = NAN;
        holding_reg_params_mutexes.floats[float_ind] = xSemaphoreCreateMutex();
        if (holding_reg_params_mutexes.floats[float_ind] == NULL) return ESP_FAIL;
    }
    for (uint8_t coil_port_ind = 0; coil_port_ind < MODBUS_PARAMS_COIL_PORTS_COUNT; coil_port_ind++)
    {
        coil_params.ports[coil_port_ind] = 0x0;
        coil_params_mutexes.ports[coil_port_ind] = xSemaphoreCreateMutex();
        if (coil_params_mutexes.ports[coil_port_ind] == NULL) return ESP_FAIL;
    }
    for (uint8_t discrete_port_ind = 0; discrete_port_ind < MODBUS_PARAMS_DISCRETE_INPUT_PORTS_COUNT;
         discrete_port_ind++)
    {
        discrete_input_params.ports[discrete_port_ind] = 0x0;
        discrete_input_params_mutexes.ports[discrete_port_ind] = xSemaphoreCreateMutex();
        if (discrete_input_params_mutexes.ports[discrete_port_ind] == NULL) return ESP_FAIL;
    }
    return ESP_OK;
}

void *slave_handler_ctx = NULL;

esp_err_t modbus_params_init(void *slave_handler)
{
    if (slave_handler == NULL) return ESP_FAIL;

    esp_err_t ret = ESP_OK;

    slave_handler_ctx = slave_handler;

    ret = setup_reg_data();

    return ret;
}

esp_err_t modbus_params_get_input_register_float_reg_area(ModbusParams_InReg_Float_t           index,
                                                          mb_register_area_descriptor_t *const reg_area)
{
    if (index >= MODBUS_PARAMS_INPUT_REGISTER_FLOAT_COUNT) return ESP_FAIL;
    if (reg_area == NULL) return ESP_FAIL;
    if (input_reg_params_mutexes.floats[index] == NULL) return ESP_FAIL;

    reg_area->type = MB_PARAM_INPUT;
    reg_area->start_offset = (offsetof(input_reg_params_t, floats) + index * (sizeof(float) << 1));
    reg_area->address = (void *)&input_reg_params.floats[index];
    reg_area->size = (sizeof(float) << 1);
    reg_area->access = MB_ACCESS_RO;

    return ESP_OK;
}

esp_err_t modbus_params_get_input_register_uint_reg_area(ModbusParams_InReg_UInt_t            index,
                                                         mb_register_area_descriptor_t *const reg_area)
{
    if (index >= MODBUS_PARAMS_INPUT_REGISTER_UINT_COUNT) return ESP_FAIL;
    if (reg_area == NULL) return ESP_FAIL;
    if (input_reg_params_mutexes.uints[index] == NULL) return ESP_FAIL;

    reg_area->type = MB_PARAM_INPUT;
    reg_area->start_offset = (offsetof(input_reg_params_t, uints) + index * (sizeof(uint16_t) << 1));
    reg_area->address = (void *)&input_reg_params.uints[index];
    reg_area->size = (sizeof(uint16_t) << 1);
    reg_area->access = MB_ACCESS_RO;

    return ESP_OK;
}

esp_err_t modbus_params_get_holding_register_float_reg_area(ModbusParams_HoldReg_Float_t         index,
                                                            mb_register_area_descriptor_t *const reg_area)
{
    if (index >= MODBUS_PARAMS_HOLDING_REGISTER_FLOAT_COUNT) return ESP_FAIL;
    if (reg_area == NULL) return ESP_FAIL;
    if (holding_reg_params_mutexes.floats[index] == NULL) return ESP_FAIL;

    reg_area->type = MB_PARAM_HOLDING;
    reg_area->start_offset = (offsetof(holding_reg_params_t, floats) + index * (sizeof(float) << 1));
    reg_area->address = (void *)&holding_reg_params.floats[index];
    reg_area->size = (sizeof(float) << 1);
    reg_area->access = MB_ACCESS_RW;

    return ESP_OK;
}

esp_err_t modbus_params_get_holding_register_uint_reg_area(ModbusParams_HoldReg_UInt_t          index,
                                                           mb_register_area_descriptor_t *const reg_area)
{
    if (index >= MODBUS_PARAMS_HOLDING_REGISTER_UINT_COUNT) return ESP_FAIL;
    if (reg_area == NULL) return ESP_FAIL;
    if (holding_reg_params_mutexes.uints[index] == NULL) return ESP_FAIL;

    reg_area->type = MB_PARAM_HOLDING;
    reg_area->start_offset = (offsetof(holding_reg_params_t, uints) + index * (sizeof(uint16_t) << 1));
    reg_area->address = (void *)&holding_reg_params.uints[index];
    reg_area->size = (sizeof(uint16_t) << 1);
    reg_area->access = MB_ACCESS_RW;

    return ESP_OK;
}

esp_err_t modbus_params_get_coil_port_reg_area(uint8_t index, mb_register_area_descriptor_t *const reg_area)
{
    if (index >= MODBUS_PARAMS_COIL_PORTS_COUNT) return ESP_FAIL;
    if (reg_area == NULL) return ESP_FAIL;
    if (coil_params_mutexes.ports[index] == NULL) return ESP_FAIL;

    reg_area->type = MB_PARAM_COIL;
    reg_area->start_offset = (offsetof(coil_reg_params_t, ports) + index * (sizeof(bool) << 1));
    reg_area->address = (void *)&coil_params.ports[index];
    reg_area->size = (sizeof(bool) << 1);
    reg_area->access = MB_ACCESS_RW;

    return ESP_OK;
}

esp_err_t modbus_params_get_discrete_input_port_reg_area(ModbusParams_DiscreteInput_t         index,
                                                         mb_register_area_descriptor_t *const reg_area)
{
    if (index >= MODBUS_PARAMS_DISCRETE_INPUT_COUNT) return ESP_FAIL;
    if (reg_area == NULL) return ESP_FAIL;
    if (discrete_input_params_mutexes.ports[index] == NULL) return ESP_FAIL;

    reg_area->type = MB_PARAM_DISCRETE;
    reg_area->start_offset = (offsetof(discrete_input_reg_params_t, ports) + index * (sizeof(bool) << 1));
    reg_area->address = (void *)&discrete_input_params.ports[index];
    reg_area->size = (sizeof(bool) << 1);
    reg_area->access = MB_ACCESS_RO;

    return ESP_OK;
}

esp_err_t modbus_params_set_input_register_float(ModbusParams_InReg_Float_t index, float value)
{
    if (index >= MODBUS_PARAMS_INPUT_REGISTER_FLOAT_COUNT) return ESP_FAIL;
    if (input_reg_params_mutexes.floats[index] == NULL) return ESP_FAIL;

    esp_err_t ret = ESP_OK;
    if (xSemaphoreTake(input_reg_params_mutexes.floats[index], pdMS_TO_TICKS(MODBUS_PARAMS_MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_FAIL;
    ret = mbc_slave_lock(slave_handler_ctx);
    if (ret != ESP_OK)
    {
        xSemaphoreGive(holding_reg_params_mutexes.uints[index]);
        return ret;
    }
    input_reg_params.floats[index] = value;
    ret = mbc_slave_unlock(slave_handler_ctx);
    xSemaphoreGive(input_reg_params_mutexes.floats[index]);
    if (ret != ESP_OK) return ret;
    return ESP_OK;
}

esp_err_t modbus_params_set_input_register_uint(ModbusParams_InReg_UInt_t index, uint16_t value)
{
    if (index >= MODBUS_PARAMS_INPUT_REGISTER_UINT_COUNT) return ESP_FAIL;
    if (input_reg_params_mutexes.uints[index] == NULL) return ESP_FAIL;

    esp_err_t ret = ESP_OK;
    if (xSemaphoreTake(input_reg_params_mutexes.uints[index], pdMS_TO_TICKS(MODBUS_PARAMS_MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_FAIL;
    ret = mbc_slave_lock(slave_handler_ctx);
    if (ret != ESP_OK)
    {
        xSemaphoreGive(holding_reg_params_mutexes.uints[index]);
        return ret;
    }
    input_reg_params.uints[index] = value;
    ret = mbc_slave_unlock(slave_handler_ctx);
    xSemaphoreGive(input_reg_params_mutexes.uints[index]);
    if (ret != ESP_OK) return ret;
    return ESP_OK;
}

esp_err_t modbus_params_set_holding_register_uint(ModbusParams_HoldReg_UInt_t index, uint16_t value)
{
    if (index >= MODBUS_PARAMS_HOLDING_REGISTER_UINT_COUNT) return ESP_FAIL;
    if (holding_reg_params_mutexes.uints[index] == NULL) return ESP_FAIL;

    esp_err_t ret = ESP_OK;
    if (xSemaphoreTake(holding_reg_params_mutexes.uints[index], pdMS_TO_TICKS(MODBUS_PARAMS_MUTEX_TIMEOUT_MS))
        != pdTRUE)
        return ESP_FAIL;
    ret = mbc_slave_lock(slave_handler_ctx);
    if (ret != ESP_OK)
    {
        xSemaphoreGive(holding_reg_params_mutexes.uints[index]);
        return ret;
    }
    holding_reg_params.uints[index] = value;
    ret = mbc_slave_unlock(slave_handler_ctx);
    xSemaphoreGive(holding_reg_params_mutexes.uints[index]);
    if (ret != ESP_OK) return ret;
    return ESP_OK;
}

esp_err_t modbus_params_get_holding_register_uint(ModbusParams_HoldReg_UInt_t index, uint16_t *const value)
{
    if (index >= MODBUS_PARAMS_HOLDING_REGISTER_UINT_COUNT) return ESP_FAIL;
    if (value == NULL) return ESP_FAIL;
    if (holding_reg_params_mutexes.uints[index] == NULL) return ESP_FAIL;

    esp_err_t ret = ESP_OK;
    if (xSemaphoreTake(holding_reg_params_mutexes.uints[index], pdMS_TO_TICKS(MODBUS_PARAMS_MUTEX_TIMEOUT_MS))
        != pdTRUE)
        return ESP_FAIL;
    ret = mbc_slave_lock(slave_handler_ctx);
    if (ret != ESP_OK)
    {
        xSemaphoreGive(holding_reg_params_mutexes.uints[index]);
        return ret;
    }
    *value = holding_reg_params.uints[index];
    ret = mbc_slave_unlock(slave_handler_ctx);
    xSemaphoreGive(holding_reg_params_mutexes.uints[index]);
    if (ret != ESP_OK) return ret;
    return ESP_OK;
}

esp_err_t modbus_params_set_holding_register_float(ModbusParams_HoldReg_Float_t index, float value)
{
    if (index >= MODBUS_PARAMS_HOLDING_REGISTER_FLOAT_COUNT) return ESP_FAIL;
    if (holding_reg_params_mutexes.floats[index] == NULL) return ESP_FAIL;

    esp_err_t ret = ESP_OK;
    if (xSemaphoreTake(holding_reg_params_mutexes.floats[index], pdMS_TO_TICKS(MODBUS_PARAMS_MUTEX_TIMEOUT_MS))
        != pdTRUE)
        return ESP_FAIL;
    ret = mbc_slave_lock(slave_handler_ctx);
    if (ret != ESP_OK)
    {
        xSemaphoreGive(holding_reg_params_mutexes.floats[index]);
        return ret;
    }
    holding_reg_params.floats[index] = value;
    ret = mbc_slave_unlock(slave_handler_ctx);
    xSemaphoreGive(holding_reg_params_mutexes.floats[index]);
    if (ret != ESP_OK) return ret;
    return ESP_OK;
}

esp_err_t modbus_params_get_holding_register_float(ModbusParams_HoldReg_Float_t index, float *const value)
{
    if (index >= MODBUS_PARAMS_HOLDING_REGISTER_FLOAT_COUNT) return ESP_FAIL;
    if (value == NULL) return ESP_FAIL;
    if (holding_reg_params_mutexes.floats[index] == NULL) return ESP_FAIL;

    esp_err_t ret = ESP_OK;
    if (xSemaphoreTake(holding_reg_params_mutexes.floats[index], pdMS_TO_TICKS(MODBUS_PARAMS_MUTEX_TIMEOUT_MS))
        != pdTRUE)
        return ESP_FAIL;
    ret = mbc_slave_lock(slave_handler_ctx);
    if (ret != ESP_OK)
    {
        xSemaphoreGive(holding_reg_params_mutexes.floats[index]);
        return ret;
    }
    *value = holding_reg_params.floats[index];
    ret = mbc_slave_unlock(slave_handler_ctx);
    xSemaphoreGive(holding_reg_params_mutexes.floats[index]);
    if (ret != ESP_OK) return ret;
    return ESP_OK;
}

esp_err_t modbus_params_set_coil_state(ModbusParams_Coil_t index, bool state)
{
    if (index >= MODBUS_PARAMS_COIL_COUNT) return ESP_FAIL;

    uint8_t port_index = (uint8_t)(index / 8);
    uint8_t port_offset = (uint8_t)(index % 8);

    if (coil_params_mutexes.ports[port_index] == NULL) return ESP_FAIL;

    esp_err_t ret = ESP_OK;
    if (xSemaphoreTake(coil_params_mutexes.ports[port_index], pdMS_TO_TICKS(MODBUS_PARAMS_MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_FAIL;
    ret = mbc_slave_lock(slave_handler_ctx);
    if (ret != ESP_OK)
    {
        xSemaphoreGive(coil_params_mutexes.ports[port_index]);
        return ret;
    }
    coil_params.ports[port_index] = set_bit(coil_params.ports[port_index], state, port_offset);
    ret = mbc_slave_unlock(slave_handler_ctx);
    xSemaphoreGive(coil_params_mutexes.ports[port_index]);
    if (ret != ESP_OK) return ret;
    return ESP_OK;
}

esp_err_t modbus_params_get_coil_state(ModbusParams_Coil_t index, bool *const state)
{
    if (index >= MODBUS_PARAMS_COIL_COUNT) return ESP_FAIL;
    if (state == NULL) return ESP_FAIL;

    uint8_t port_index = (uint8_t)(index / 8);
    uint8_t port_offset = (uint8_t)(index % 8);

    if (coil_params_mutexes.ports[port_index] == NULL) return ESP_FAIL;

    esp_err_t ret = ESP_OK;
    if (xSemaphoreTake(coil_params_mutexes.ports[port_index], pdMS_TO_TICKS(MODBUS_PARAMS_MUTEX_TIMEOUT_MS)) != pdTRUE)
        return ESP_FAIL;
    ret = mbc_slave_lock(slave_handler_ctx);
    if (ret != ESP_OK)
    {
        xSemaphoreGive(coil_params_mutexes.ports[port_index]);
        return ret;
    }
    *state = get_bit(coil_params.ports[port_index], port_offset);
    ret = mbc_slave_unlock(slave_handler_ctx);
    xSemaphoreGive(coil_params_mutexes.ports[port_index]);
    if (ret != ESP_OK) return ret;
    return ESP_OK;
}

esp_err_t modbus_params_set_discrete_input_state(ModbusParams_DiscreteInput_t index, bool state)
{
    if (index >= MODBUS_PARAMS_DISCRETE_INPUT_COUNT) return ESP_FAIL;

    uint8_t port_index = (uint8_t)(index / 8);
    uint8_t port_offset = (uint8_t)(index % 8);

    if (discrete_input_params_mutexes.ports[port_index] == NULL) return ESP_FAIL;

    esp_err_t ret = ESP_OK;
    if (xSemaphoreTake(discrete_input_params_mutexes.ports[port_index], pdMS_TO_TICKS(MODBUS_PARAMS_MUTEX_TIMEOUT_MS))
        != pdTRUE)
        return ESP_FAIL;
    ret = mbc_slave_lock(slave_handler_ctx);
    if (ret != ESP_OK)
    {
        xSemaphoreGive(discrete_input_params_mutexes.ports[port_index]);
        return ret;
    }
    discrete_input_params.ports[port_index] = set_bit(discrete_input_params.ports[port_index], state, port_offset);
    ret = mbc_slave_unlock(slave_handler_ctx);
    xSemaphoreGive(discrete_input_params_mutexes.ports[port_index]);
    if (ret != ESP_OK) return ret;
    return ESP_OK;
}
