#include "sdkconfig.h"
#include <stdio.h>

#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_wifi.h"

#include "mdns.h"
#include "nvs_flash.h"

#include "protocol_examples_common.h"

#include "mbcontroller.h" // for mbcontroller defines and api

#include "modbus_params.h" // for modbus parameters structures

static const char *LOG_TAG = "modbus_server";

#define MB_TCP_PORT_NUMBER   (CONFIG_FMB_TCP_PORT_DEFAULT)
#define MB_SLAVE_ADDR        (CONFIG_MB_SLAVE_ADDR)
#define MB_TCP_TIMEOUT       1000

#define MB_MDNS_PORT         (502)

#define MB_PAR_INFO_GET_TOUT (10) // Timeout for get parameter info

#define MB_READ_MASK         (MB_EVENT_INPUT_REG_RD | MB_EVENT_HOLDING_REG_RD | MB_EVENT_DISCRETE_RD | MB_EVENT_COILS_RD)
#define MB_WRITE_MASK        (MB_EVENT_HOLDING_REG_WR | MB_EVENT_COILS_WR)
#define MB_READ_WRITE_MASK   (MB_READ_MASK | MB_WRITE_MASK)

#if CONFIG_MB_MDNS_IP_RESOLVER

    #if CONFIG_FMB_CONTROLLER_SLAVE_ID_SUPPORT
        #define MB_DEVICE_ID (uint32_t) CONFIG_FMB_CONTROLLER_SLAVE_ID
    #endif // CONFIG_FMB_CONTROLLER_SLAVE_ID_SUPPORT

    #define MB_ID_BYTE0(id)        ((uint8_t)(id))
    #define MB_ID_BYTE1(id)        ((uint8_t)(((uint16_t)(id) >> 8) & 0xFF))
    #define MB_ID_BYTE2(id)        ((uint8_t)(((uint32_t)(id) >> 16) & 0xFF))
    #define MB_ID_BYTE3(id)        ((uint8_t)(((uint32_t)(id) >> 24) & 0xFF))

    #define MB_ID2STR(id)          MB_ID_BYTE0(id), MB_ID_BYTE1(id), MB_ID_BYTE2(id), MB_ID_BYTE3(id)

    #define MB_MDNS_INSTANCE(pref) pref "mb_slave_tcp"

char dns_hostname_buffer[32] = {0};

mb_communication_info_t comm_info = {
    .tcp_opts = {
        .mode = MB_TCP,
        .port = MB_TCP_PORT_NUMBER,
        .response_tout_ms = MB_TCP_TIMEOUT,
    #if !CONFIG_EXAMPLE_CONNECT_IPV6
        .addr_type = MB_IPV4,
    #else
        .addr_type = MB_IPV6,
    #endif // CONFIG_EXAMPLE_CONNECT_IPV6
        .ip_addr_table = NULL,       //< Bind to any address
        .ip_netif_ptr = NULL,        //< Set during slave_init
        .dns_name = NULL,            //< Master only option
        .start_disconnected = false, //< Master only option
        .uid = MB_SLAVE_ADDR,
    },
};
void *slave_handler = NULL;

// convert mac from binary format to string
static inline char *gen_mac_str(const uint8_t *mac, char *pref, char *mac_str)
{
    sprintf(mac_str, "%s%02X%02X%02X%02X%02X%02X", pref, MAC2STR(mac));
    return mac_str;
}

static inline char *gen_id_str(char *service_name, char *slave_id_str)
{
    sprintf(slave_id_str, "%s%02X%02X%02X%02X", service_name, MB_ID2STR(MB_DEVICE_ID));
    return slave_id_str;
}

static inline char *gen_host_name_str(char *service_name, char *name)
{
    sprintf(name, "%s_%02X", service_name, MB_SLAVE_ADDR);
    return name;
}

static void start_mdns_service(void)
{
    uint8_t sta_mac[6] = {0};
    ESP_ERROR_CHECK(esp_read_mac(sta_mac, ESP_MAC_WIFI_STA));
    char *hostname = gen_host_name_str(MB_MDNS_INSTANCE(""), dns_hostname_buffer);
    // initialize mDNS
    ESP_ERROR_CHECK(mdns_init());
    // set mDNS hostname (required if you want to advertise services)
    ESP_ERROR_CHECK(mdns_hostname_set(hostname));
    ESP_LOGI(LOG_TAG, "mdns hostname set to: [%s]", hostname);

    // set default mDNS instance name
    ESP_ERROR_CHECK(mdns_instance_name_set(MB_MDNS_INSTANCE("esp32_")));

    // structure with TXT records
    mdns_txt_item_t serviceTxtData[] = {{"board", "esp32"}};

    // initialize service
    ESP_ERROR_CHECK(mdns_service_add(hostname, "_modbus", "_tcp", MB_MDNS_PORT, serviceTxtData, 1));

    char temp_str[32] = {0};
    // add mac key string text item
    ESP_ERROR_CHECK(mdns_service_txt_item_set("_modbus", "_tcp", "mac", gen_mac_str(sta_mac, "\0", temp_str)));
    // add slave id key txt item
    ESP_ERROR_CHECK(mdns_service_txt_item_set("_modbus", "_tcp", "mb_id", gen_id_str("\0", temp_str)));
}

static void stop_mdns_service(void) { mdns_free(); }

#endif // CONFIG_MB_MDNS_IP_RESOLVER

static esp_err_t init_services(void)
{
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        result = nvs_flash_init();
    }
    MB_RETURN_ON_FALSE((result == ESP_OK),
                       ESP_ERR_INVALID_STATE,
                       LOG_TAG,
                       "nvs_flash_init fail, returns(0x%x).",
                       (int)result);
    result = esp_netif_init();
    MB_RETURN_ON_FALSE((result == ESP_OK),
                       ESP_ERR_INVALID_STATE,
                       LOG_TAG,
                       "esp_netif_init fail, returns(0x%x).",
                       (int)result);
    result = esp_event_loop_create_default();
    MB_RETURN_ON_FALSE((result == ESP_OK),
                       ESP_ERR_INVALID_STATE,
                       LOG_TAG,
                       "esp_event_loop_create_default fail, returns(0x%x).",
                       (int)result);
#if CONFIG_MB_MDNS_IP_RESOLVER
    // Start mdns service and register device
    start_mdns_service();
#endif
    // This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
    // Read "Establishing Wi-Fi or Ethernet Connection" section in
    // examples/protocols/README.md for more information about this function.
    result = example_connect();
    MB_RETURN_ON_FALSE((result == ESP_OK),
                       ESP_ERR_INVALID_STATE,
                       LOG_TAG,
                       "example_connect fail, returns(0x%x).",
                       (int)result);
#if CONFIG_EXAMPLE_CONNECT_WIFI
    result = esp_wifi_set_ps(WIFI_PS_NONE);
    MB_RETURN_ON_FALSE((result == ESP_OK),
                       ESP_ERR_INVALID_STATE,
                       LOG_TAG,
                       "esp_wifi_set_ps fail, returns(0x%x).",
                       (int)result);
#endif
    return ESP_OK;
}

static esp_err_t destroy_services(void)
{
    esp_err_t ret = ESP_OK;

    ret = example_disconnect();
    MB_RETURN_ON_FALSE((ret == ESP_OK),
                       ESP_ERR_INVALID_STATE,
                       LOG_TAG,
                       "example_disconnect fail, returns(0x%x).",
                       (int)ret);
    ret = esp_event_loop_delete_default();
    MB_RETURN_ON_FALSE((ret == ESP_OK),
                       ESP_ERR_INVALID_STATE,
                       LOG_TAG,
                       "esp_event_loop_delete_default fail, returns(0x%x).",
                       (int)ret);
    ret = esp_netif_deinit();
    MB_RETURN_ON_FALSE((ret == ESP_OK || ret == ESP_ERR_NOT_SUPPORTED),
                       ESP_ERR_INVALID_STATE,
                       LOG_TAG,
                       "esp_netif_deinit fail, returns(0x%x).",
                       (int)ret);
    ret = nvs_flash_deinit();
    MB_RETURN_ON_FALSE((ret == ESP_OK),
                       ESP_ERR_INVALID_STATE,
                       LOG_TAG,
                       "nvs_flash_deinit fail, returns(0x%x).",
                       (int)ret);
#if CONFIG_MB_MDNS_IP_RESOLVER
    stop_mdns_service();
#endif
    return ret;
}

// Modbus slave initialization
static esp_err_t slave_init()
{
    comm_info.tcp_opts.ip_netif_ptr = (void *)get_example_netif();

    // Initialization of Modbus controller - Setup communication parameters and start stack
    esp_err_t ret = mbc_slave_create_tcp(&comm_info, &slave_handler);
    MB_RETURN_ON_FALSE((ret == ESP_OK && slave_handler != NULL),
                       ESP_ERR_INVALID_STATE,
                       LOG_TAG,
                       "mbc_slave_create_tcp failed!");

    // Init Modbus slave controller interface handle (reset descriptors list)
    mbc_slave_init_iface(slave_handler);

    // Init Modbus parameters and register area descriptors
    ret = modbus_params_init(slave_handler);
    MB_RETURN_ON_FALSE((ret == ESP_OK), ESP_ERR_INVALID_STATE, LOG_TAG, "modbus_params_init failed!");
    // The code below initializes Modbus register area descriptors
    // for Modbus Holding Registers, Input Registers, Coils and Discrete Inputs
    // Initialization should be done for each supported Modbus register area according to register map.
    // When external master trying to access the register in the area that is not initialized
    // by mbc_slave_set_descriptor() API call then Modbus stack
    // will send exception response for this register area.
    // Initialization of Input Registers area
    for (ModbusParams_InReg_Float_t float_ind = (ModbusParams_InReg_Float_t)0;
         float_ind < MODBUS_PARAMS_INPUT_REGISTER_FLOAT_COUNT;
         float_ind++)
    {
        mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure
        ret = modbus_params_get_input_register_float_reg_area(float_ind, &reg_area);
        MB_RETURN_ON_FALSE(
            (ret == ESP_OK),
            ESP_ERR_INVALID_STATE,
            LOG_TAG,
            "modbus_params_get_input_register_float_reg_area fail for input reg float index %d, returns(0x%x).",
            float_ind,
            (int)ret);

        ret = mbc_slave_set_descriptor(slave_handler, reg_area);
        MB_RETURN_ON_FALSE((ret == ESP_OK),
                           ESP_ERR_INVALID_STATE,
                           LOG_TAG,
                           "mbc_slave_set_descriptor fail for input reg float index %d, returns(0x%x).",
                           float_ind,
                           (int)ret);
    }
    for (ModbusParams_InReg_UInt_t uint_ind = (ModbusParams_InReg_UInt_t)0;
         uint_ind < MODBUS_PARAMS_INPUT_REGISTER_UINT_COUNT;
         uint_ind++)
    {
        mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure
        ret = modbus_params_get_input_register_uint_reg_area(uint_ind, &reg_area);
        MB_RETURN_ON_FALSE(
            (ret == ESP_OK),
            ESP_ERR_INVALID_STATE,
            LOG_TAG,
            "modbus_params_get_input_register_uint_reg_area fail for input reg uint index %d, returns(0x%x).",
            uint_ind,
            (int)ret);

        ret = mbc_slave_set_descriptor(slave_handler, reg_area);
        MB_RETURN_ON_FALSE((ret == ESP_OK),
                           ESP_ERR_INVALID_STATE,
                           LOG_TAG,
                           "mbc_slave_set_descriptor fail for input reg uint index %d, returns(0x%x).",
                           uint_ind,
                           (int)ret);
    }
    // Initialization of Holding Registers area
    for (ModbusParams_HoldReg_Float_t float_ind = (ModbusParams_HoldReg_Float_t)0;
         float_ind < MODBUS_PARAMS_HOLDING_REGISTER_FLOAT_COUNT;
         float_ind++)
    {
        mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure
        ret = modbus_params_get_holding_register_float_reg_area(float_ind, &reg_area);
        MB_RETURN_ON_FALSE(
            (ret == ESP_OK),
            ESP_ERR_INVALID_STATE,
            LOG_TAG,
            "modbus_params_get_holding_register_float_reg_area fail for hold reg float index %d, returns(0x%x).",
            float_ind,
            (int)ret);

        ret = mbc_slave_set_descriptor(slave_handler, reg_area);
        MB_RETURN_ON_FALSE((ret == ESP_OK),
                           ESP_ERR_INVALID_STATE,
                           LOG_TAG,
                           "mbc_slave_set_descriptor fail for hold reg float index %d, returns(0x%x).",
                           float_ind,
                           (int)ret);
    }
    for (ModbusParams_HoldReg_UInt_t uint_ind = (ModbusParams_HoldReg_UInt_t)0;
         uint_ind < MODBUS_PARAMS_HOLDING_REGISTER_UINT_COUNT;
         uint_ind++)
    {
        mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure
        ret = modbus_params_get_holding_register_uint_reg_area(uint_ind, &reg_area);
        MB_RETURN_ON_FALSE(
            (ret == ESP_OK),
            ESP_ERR_INVALID_STATE,
            LOG_TAG,
            "modbus_params_get_holding_register_uint_reg_area fail for hold reg uint index %d, returns(0x%x).",
            uint_ind,
            (int)ret);

        ret = mbc_slave_set_descriptor(slave_handler, reg_area);
        MB_RETURN_ON_FALSE((ret == ESP_OK),
                           ESP_ERR_INVALID_STATE,
                           LOG_TAG,
                           "mbc_slave_set_descriptor fail for hold reg uint index %d, returns(0x%x).",
                           uint_ind,
                           (int)ret);
    }

    // Initialization of Coils ports registers area
    for (uint8_t coil_port_ind = 0; coil_port_ind < MODBUS_PARAMS_COIL_PORTS_COUNT; coil_port_ind++)
    {
        mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure
        ret = modbus_params_get_coil_port_reg_area(coil_port_ind, &reg_area);
        MB_RETURN_ON_FALSE((ret == ESP_OK),
                           ESP_ERR_INVALID_STATE,
                           LOG_TAG,
                           "modbus_params_get_coil_port_reg_area fail for coil port index %d, returns(0x%x).",
                           coil_port_ind,
                           (int)ret);

        ret = mbc_slave_set_descriptor(slave_handler, reg_area);
        MB_RETURN_ON_FALSE((ret == ESP_OK),
                           ESP_ERR_INVALID_STATE,
                           LOG_TAG,
                           "mbc_slave_set_descriptor fail for coil port index %d, returns(0x%x).",
                           coil_port_ind,
                           (int)ret);
    }
    // Initialization of Discrete Input ports registers area
    for (uint8_t discrete_port_ind = 0; discrete_port_ind < MODBUS_PARAMS_DISCRETE_INPUT_PORTS_COUNT;
         discrete_port_ind++)
    {
        mb_register_area_descriptor_t reg_area; // Modbus register area descriptor structure
        ret = modbus_params_get_discrete_input_port_reg_area(discrete_port_ind, &reg_area);
        MB_RETURN_ON_FALSE(
            (ret == ESP_OK),
            ESP_ERR_INVALID_STATE,
            LOG_TAG,
            "modbus_params_get_discrete_input_port_reg_area fail for discrete port index %d, returns(0x%x).",
            discrete_port_ind,
            (int)ret);

        ret = mbc_slave_set_descriptor(slave_handler, reg_area);
        MB_RETURN_ON_FALSE((ret == ESP_OK),
                           ESP_ERR_INVALID_STATE,
                           LOG_TAG,
                           "mbc_slave_set_descriptor fail for discrete port index %d, returns(0x%x).",
                           discrete_port_ind,
                           (int)ret);
    }

    // Starts of modbus controller and stack
    ret = mbc_slave_start(slave_handler);
    MB_RETURN_ON_FALSE((ret == ESP_OK),
                       ESP_ERR_INVALID_STATE,
                       LOG_TAG,
                       "mbc_slave_start fail, returns(0x%x).",
                       (int)ret);
    vTaskDelay(pdMS_TO_TICKS(5)); // TODO: Check why we need this delay?
    return ret;
}

static esp_err_t slave_destroy(void)
{
    esp_err_t ret = ESP_OK;
    ret = mbc_slave_stop(slave_handler);
    MB_RETURN_ON_FALSE((ret == ESP_OK),
                       ESP_ERR_INVALID_STATE,
                       LOG_TAG,
                       "mbc_slave_stop fail, returns(0x%x).",
                       (int)ret);
    ret = mbc_slave_delete(slave_handler);
    MB_RETURN_ON_FALSE((ret == ESP_OK),
                       ESP_ERR_INVALID_STATE,
                       LOG_TAG,
                       "mbc_slave_delete fail, returns(0x%x).",
                       (int)ret);
    return ret;
}

// An example application of Modbus slave. It is based on freemodbus stack.
// See deviceparams.h file for more information about assigned Modbus parameters.
// These parameters can be accessed from main application and also can be changed
// by external Modbus master host.
esp_err_t modbus_server_init(void)
{
    esp_err_t ret = ESP_OK;
    ret = init_services();
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "init_services failed!");
        return ret;
    }

    ret = slave_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "slave_init failed!");
        return ret;
    }
    return ret;
}

void modbus_server_task(void *pvParameter)
{
    // The Modbus server(slave) logic is located in this function (user handling of Modbus)
    mb_param_info_t reg_info; // keeps the Modbus registers access information

    ESP_LOGI(LOG_TAG, "Modbus slave stack initialized.");
    ESP_LOGI(LOG_TAG, "Start modbus server loop...");

    // Currently this loop only log in verbose the requests from the master
    /*TODO: Add a way to handle following error (happens when pymodbus master doesn't properly close/disconnect - i.e.
script is killed):

I (57756) port.utils: Socket (#58), accept client connection from address[port]: 192.168.10.30[62889]
E (57756) mb_driver: 0x3fcb7a28, unable to open node: 192.168.10.30

    */
    while (1)
    {
        // Check for read/write events of Modbus master for certain events
        (void)mbc_slave_check_event(slave_handler, MB_READ_WRITE_MASK);
        ESP_ERROR_CHECK_WITHOUT_ABORT(mbc_slave_get_param_info(slave_handler, &reg_info, MB_PAR_INFO_GET_TOUT));
        const char *rw_str = (reg_info.type & MB_READ_MASK) ? "READ" : "WRITE";
        // Filter events and process them accordingly
        if (reg_info.type & (MB_EVENT_HOLDING_REG_WR | MB_EVENT_HOLDING_REG_RD))
        {
            // Get parameter information from parameter queue
            ESP_LOGV(LOG_TAG,
                     "HOLDING %s (%" PRIu32 " us), ADDR:%u, TYPE:%u, INST_ADDR:0x%" PRIx32 ", SIZE:%u",
                     rw_str,
                     reg_info.time_stamp,
                     (unsigned)reg_info.mb_offset,
                     (unsigned)reg_info.type,
                     (uint32_t)reg_info.address,
                     (unsigned)reg_info.size);
        }
        else if (reg_info.type & MB_EVENT_INPUT_REG_RD)
        {
            ESP_LOGV(LOG_TAG,
                     "INPUT READ (%" PRIu32 " us), ADDR:%u, TYPE:%u, INST_ADDR:0x%" PRIx32 ", SIZE:%u",
                     reg_info.time_stamp,
                     (unsigned)reg_info.mb_offset,
                     (unsigned)reg_info.type,
                     (uint32_t)reg_info.address,
                     (unsigned)reg_info.size);
        }
        else if (reg_info.type & MB_EVENT_DISCRETE_RD)
        {
            ESP_LOGV(LOG_TAG,
                     "DISCRETE READ (%" PRIu32 " us), ADDR:%u, TYPE:%u, INST_ADDR:0x%" PRIx32 ", SIZE:%u",
                     reg_info.time_stamp,
                     (unsigned)reg_info.mb_offset,
                     (unsigned)reg_info.type,
                     (uint32_t)reg_info.address,
                     (unsigned)reg_info.size);
        }
        else if (reg_info.type & (MB_EVENT_COILS_RD | MB_EVENT_COILS_WR))
        {
            ESP_LOGV(LOG_TAG,
                     "COILS %s (%" PRIu32 " us), ADDR:%u, TYPE:%u, INST_ADDR:0x%" PRIx32 ", SIZE:%u",
                     rw_str,
                     reg_info.time_stamp,
                     (unsigned)reg_info.mb_offset,
                     (unsigned)reg_info.type,
                     (uint32_t)reg_info.address,
                     (unsigned)reg_info.size);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    // Destroy of Modbus controller on alarm
    ESP_LOGI(LOG_TAG, "Modbus controller destroyed.");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(slave_destroy());
    ESP_ERROR_CHECK(destroy_services());
}