#ifndef MODBUS_SERVER__H__
#define MODBUS_SERVER__H__

#include "esp_err.h"

esp_err_t modbus_server_init(void);
void modbus_server_task(void *pvParameter);

#endif // MODBUS_SERVER__H__