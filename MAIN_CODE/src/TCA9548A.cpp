#include "TCA9548A.h"
#include <sys/ioctl.h>
extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

namespace TCA9548A {
    std::mutex g_mutex;
    uint8_t g_active_channel = 0xFF;  // 0xFF = no channel selected yet

    void selectChannel(int bus_fd, uint8_t channel_mask, uint8_t device_address) {
        if (channel_mask != g_active_channel) {
            ioctl(bus_fd, I2C_SLAVE, kMuxAddress);
            i2c_smbus_write_byte(bus_fd, channel_mask);
            g_active_channel = channel_mask;
        }
        ioctl(bus_fd, I2C_SLAVE, device_address);
    }
}
