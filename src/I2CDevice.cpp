#include "I2CDevice.hpp"

I2CDevice::I2CDevice(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freq, uint16_t addr)
    : dev_cfg{
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = freq,
    },
    owns_bus(true)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = port,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = { .enable_internal_pullup = true }
    };

    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    mutex = xSemaphoreCreateMutex();
    if (!mutex) throw std::runtime_error("Failed to create I2C mutex");
}

I2CDevice::I2CDevice(i2c_master_bus_handle_t bus, uint16_t addr, uint32_t freq)
    : bus_handle(bus), dev_cfg{ .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = addr, .scl_speed_hz = freq }
{
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

    mutex = xSemaphoreCreateMutex();
    if (!mutex) throw std::runtime_error("Failed to create I2C mutex");
}

I2CDevice::~I2CDevice() {
    if (dev_handle) i2c_master_bus_rm_device(dev_handle);
    if (owns_bus && bus_handle) i2c_del_master_bus(bus_handle);
    if (mutex) vSemaphoreDelete(mutex);
}

std::expected<void, std::runtime_error> I2CDevice::write(uint8_t reg, const uint8_t* data, size_t len) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
        return std::unexpected(std::runtime_error("I2C write: mutex lock failed"));

    uint8_t buffer[len + 1];
    buffer[0] = reg;
    memcpy(buffer + 1, data, len);

    esp_err_t err = i2c_master_transmit(dev_handle, buffer, len + 1, 100);
    xSemaphoreGive(mutex);

    if (err != ESP_OK)
        return std::unexpected(std::runtime_error("I2C write failed: " + std::to_string(err)));
    return {};
}

std::expected<void, std::runtime_error> I2CDevice::read(uint8_t reg, uint8_t* data, size_t len) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) != pdTRUE)
        return std::unexpected(std::runtime_error("I2C read: mutex lock failed"));

    esp_err_t err = i2c_master_transmit_receive(dev_handle, &reg, 1, data, len, 100);
    xSemaphoreGive(mutex);

    if (err != ESP_OK)
        return std::unexpected(std::runtime_error("I2C read failed: " + std::to_string(err)));
    return {};
}
