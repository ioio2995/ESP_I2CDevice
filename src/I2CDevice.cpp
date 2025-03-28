#include "I2CDevice.hpp"
#include <string.h> 
#include "esp_log.h"

I2CDevice::I2CDevice(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint16_t addr, uint32_t freq)
    : i2c_bus_config{
        .i2c_port = port,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = { 
            .enable_internal_pullup = true 
        }
    },
    i2c_dev_cfg{
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = freq,
    },
    owns_bus(true)
{
    esp_err_t err;

    err = i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE("I2CDevice", "I2C bus initialization failed: %s", esp_err_to_name(err));
        return ;
    }
    err = i2c_master_bus_add_device(i2c_bus_handle, &i2c_dev_cfg, &i2c_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE("I2CDevice", "I2C add device failed: %s", esp_err_to_name(err));
        return ;
    }
    
    err = CreateMutex(Lock);
    if (err != ESP_OK) {
        ESP_LOGE("I2CDevice", "I2C mutex creation failed: %s", esp_err_to_name(err));
        return ;
    }
}

I2CDevice::I2CDevice(i2c_master_bus_handle_t bus, uint16_t addr, uint32_t freq)
    : i2c_bus_handle(bus), 
    i2c_dev_cfg{ 
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, 
        .device_address = addr, 
        .scl_speed_hz = freq,
    }
{

    esp_err_t err;

    err = i2c_master_bus_add_device(i2c_bus_handle, &i2c_dev_cfg, &i2c_dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE("I2CDevice", "I2C add device failed: %s", esp_err_to_name(err));
        return ;
    }

    err = CreateMutex(Lock);
    if (err != ESP_OK) {
        ESP_LOGE("I2CDevice", "I2C mutex creation failed: %s", esp_err_to_name(err));
        return ;
    }
}

I2CDevice::~I2CDevice() {
    if (i2c_dev_handle) i2c_master_bus_rm_device(i2c_dev_handle);
    if (owns_bus && i2c_bus_handle) i2c_del_master_bus(i2c_bus_handle);
    if (Lock) vSemaphoreDelete(Lock);
}

esp_err_t I2CDevice::write(uint8_t reg, const uint8_t* data, size_t len) {

    if (xSemaphoreTake(Lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE("I2CDevice", "I2C write: mutex lock failed");
        return ESP_ERR_NO_MEM;     
    }

    uint8_t buffer[len + 1];
    buffer[0] = reg;
    memcpy(buffer + 1, data, len);

    esp_err_t err = i2c_master_transmit(i2c_dev_handle, buffer, len + 1, 100);
    xSemaphoreGive(Lock);

    if (err != ESP_OK) {
        ESP_LOGE("I2CDevice", "I2C write failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t I2CDevice::read(uint8_t reg, uint8_t* data, size_t len) {

    if (xSemaphoreTake(Lock, portMAX_DELAY) != pdTRUE) {
        ESP_LOGE("I2CDevice", "I2C read: mutex lock failed");
        return ESP_ERR_NO_MEM;     
    }
    
    esp_err_t err = i2c_master_transmit_receive(i2c_dev_handle, &reg, 1, data, len, 100);
    xSemaphoreGive(Lock);

    if (err != ESP_OK) {
        ESP_LOGE("I2CDevice", "I2C read failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t I2CDevice::CreateMutex(SemaphoreHandle_t &mutex) {
    mutex = xSemaphoreCreateMutex();
    if (!mutex) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}