#pragma once
#include <cstdint>
#include <mutex>

namespace TCA9548A {
    constexpr uint8_t kMuxAddress = 0x70;
    constexpr uint8_t kCh3 = 0x08;  // PCA9685
    constexpr uint8_t kCh4 = 0x10;  // IMU (MPU6050)

    // Global mutex — must be held for the duration of any I2C operation.
    extern std::mutex g_mutex;

    // Tracks the last channel written to the mux so redundant selects are skipped.
    // Reset to 0xFF on startup (no channel selected).
    extern uint8_t g_active_channel;

    // Must be called with g_mutex held.
    // Only writes to the mux when channel_mask differs from g_active_channel,
    // then restores the slave address on bus_fd for subsequent smbus calls.
    void selectChannel(int bus_fd, uint8_t channel_mask, uint8_t device_address);
}
