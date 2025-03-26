#pragma once

#include "driver/i2c_master.h"
#include "freertos/semphr.h"
#include <expected>
#include <stdexcept>

class I2CDevice {
public:
    I2CDevice(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freq, uint16_t addr);
    I2CDevice(i2c_master_bus_handle_t bus, uint16_t addr, uint32_t freq = 100000);
    virtual ~I2CDevice();

    std::expected<void, std::runtime_error> write(uint8_t reg, const uint8_t* data, size_t len);
    std::expected<void, std::runtime_error> read(uint8_t reg, uint8_t* data, size_t len);

protected:
    SemaphoreHandle_t mutex;
    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_device_config_t dev_cfg;
    bool owns_bus = false;
};
