#pragma once
#include <cstdint>
#include <mutex>

namespace TCA9548A {
    constexpr uint8_t kMuxAddress = 0x70;
    constexpr uint8_t kCh3 = 0x08;  // PCA9685
    constexpr uint8_t kCh4 = 0x10;  // IMU (MPU6050)

    // Global mutex — must be held for the duration of any I2C operation.
    extern std::mutex g_mutex;

    // Must be called with g_mutex held.
    // Addresses the mux, writes the channel mask, then restores the slave
    // address on bus_fd so subsequent smbus calls go to the right device.
    void selectChannel(int bus_fd, uint8_t channel_mask, uint8_t device_address);
}
