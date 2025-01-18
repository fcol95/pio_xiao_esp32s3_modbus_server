#include "ambient_sense.h"

#include "esp_log.h"
#include "esp_rom_sys.h" //< For BME688 delay_us port
#include "driver/i2c.h"  //< For BME688 I2C communication port

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bme68x.h"

static const char *LOG_TAG = "ambient_sense";

static esp_err_t i2c_master_init(void);
static void bme68x_delay_us(uint32_t period, void *intf_ptr);
static BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
static BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);

esp_err_t ambient_sense_init(void)
{
    esp_err_t i2c_ret = i2c_master_init();
    if (i2c_ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "I2C Master initialization failed");
        return i2c_ret;
    }
    return i2c_ret;
}

void ambient_sense_task(void *pvParameter)
{
    uint8_t i2c_address = 0x76; // Replace with your sensor's I2C address

    struct bme68x_dev bme688_handle =
        {
            .intf = BME68X_I2C_INTF,
            // Port Functions and Pointer
            .intf_ptr = &i2c_address,
            .delay_us = bme68x_delay_us,
            .read = bme68x_i2c_read,
            .write = bme68x_i2c_write,
            .amb_temp = 25, // Ambient temperature in degrees Celsius
        };

    int8_t ret = bme68x_init(&bme688_handle);
    if (ret != BME68X_OK)
    {
        ESP_LOGE(LOG_TAG, "BME68x initialization failed");
        return;
    }
    else
    {
        ESP_LOGI(LOG_TAG, "BME68x initialization succeeded");
    }

    struct bme68x_data data;
    uint8_t n_fields;

    // Set sensor configuration
    struct bme68x_conf conf =
        {
            .os_hum = BME68X_OS_16X,
            .os_pres = BME68X_OS_1X,
            .os_temp = BME68X_OS_2X,
            .filter = BME68X_FILTER_OFF,
            .odr = BME68X_ODR_NONE,
        };
    ret = bme68x_set_conf(&conf, &bme688_handle);
    if (ret != BME68X_OK)
    {
        ESP_LOGE(LOG_TAG, "BME68x configuration failed");
        return;
    }
    else
    {
        ESP_LOGI(LOG_TAG, "BME68x configuration succeeded");
    }

    // Set heater configuration
    struct bme68x_heatr_conf heatr_conf =
        {
            .enable = BME68X_DISABLE,
            .heatr_temp = 320, // Target temperature in degree Celsius
            .heatr_dur = 150,  // Duration in milliseconds
        };
    ret = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme688_handle);
    if (ret != BME68X_OK)
    {
        ESP_LOGE(LOG_TAG, "BME68x heater configuration failed");
        return;
    }
    else
    {
        ESP_LOGI(LOG_TAG, "BME68x heater configuration succeeded");
    }

    while (1)
    {
        // Set sensor to forced mode
        ret = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme688_handle);
        if (ret != BME68X_OK)
        {
            ESP_LOGE(LOG_TAG, "BME68x setting operation mode failed");
            return;
        }

        // Wait for the measurement to complete
        vTaskDelay(pdMS_TO_TICKS(1 + (bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme688_handle) / 1000)));

        // Get sensor data
        ret = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme688_handle);
        if (ret == BME68X_OK && n_fields > 0)
        {
            ESP_LOGI(LOG_TAG, "Temperature: %.1f°C, Pressure: %.1fhPa, Humidity: %.1f%%, Gas Resistance: %.2fMOhms.", data.temperature, data.pressure / 100.0, data.humidity, data.gas_resistance / 1e6);
        }
        else
        {
            ESP_LOGE(LOG_TAG, "Failed to get sensor data");
            return;
        }

        // Wait for 1 second before the next read
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = GPIO_NUM_5, // XIAO ESP32S3 SDA GPIO
        .scl_io_num = GPIO_NUM_6, // XIAO ESP32S3 SCL GPIO
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000, // I2C clock speed (400 kHz)
    };
    esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "I2C parameter configuration failed");
        return ESP_FAIL;
    }
    ret = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE(LOG_TAG, "I2C parameter configuration failed");
        return ESP_FAIL;
    }
    return ESP_OK;
}

// BME688 microseconds delay function implementation
static void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
    // Use esp_rom_delay_us to delay for the specified period
    esp_rom_delay_us(period);
}

// BME688 I2C read function implementation
static BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    // Cast the interface pointer to the I2C address
    uint8_t i2c_addr = *(uint8_t *)intf_ptr;

    // Create an I2C command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL)
    {
        return BME68X_E_COM_FAIL; // Communication failure
    }

    // Queue the write of the register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);

    // Queue the read of the data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, true);
    if (length > 1)
    {
        i2c_master_read(cmd, reg_data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, reg_data + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    // Execute the I2C command
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));

    // Delete the command link
    i2c_cmd_link_delete(cmd);

    // Return success or failure
    return (err == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}

// BME688 I2C write function implementation
static BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    // Cast the interface pointer to the I2C address
    uint8_t i2c_addr = *(uint8_t *)intf_ptr;

    // Create an I2C command link
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL)
    {
        return BME68X_E_COM_FAIL; // Communication failure
    }

    // Queue the write of the register address and data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, reg_data, length, true);
    i2c_master_stop(cmd);

    // Execute the I2C command
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000));

    // Delete the command link
    i2c_cmd_link_delete(cmd);

    // Return success or failure
    return (err == ESP_OK) ? BME68X_OK : BME68X_E_COM_FAIL;
}
