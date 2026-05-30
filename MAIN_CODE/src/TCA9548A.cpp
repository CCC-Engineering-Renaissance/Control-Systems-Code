#include "TCA9548A.h"
#include <sys/ioctl.h>
extern "C" {
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>
}

namespace TCA9548A {
    std::mutex g_mutex;

    void selectChannel(int bus_fd, uint8_t channel_mask, uint8_t device_address) {
        ioctl(bus_fd, I2C_SLAVE, kMuxAddress);
        i2c_smbus_write_byte(bus_fd, channel_mask);
        ioctl(bus_fd, I2C_SLAVE, device_address);
    }
}
