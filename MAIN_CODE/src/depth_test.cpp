// Standalone depth/pressure sensor tester — isolates the MS5837 from the full
// ROV stack. Build:  make depthtest    Run on the Pi:  ./depthtest [/dev/i2c-5]
//
// Prints the exact init failure reason (bus/wiring/CRC) if it can't start,
// otherwise streams pressure / temperature / depth at ~5 Hz until Ctrl-C.
#include "Depth_Sensor.h"
#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

static volatile sig_atomic_t run = 1;
static void onSig(int) { run = 0; }

int main(int argc, char** argv) {
  std::signal(SIGINT, onSig);
  const std::string bus = (argc > 1) ? argv[1] : "/dev/i2c-5";
  std::cout << "Opening MS5837 on " << bus << " at 0x76...\n";

  DepthSensor sensor(bus, 0x76, DepthSensor::MS5837_30BA);  // Bar30
  if (!sensor.init()) {
    std::cerr << "INIT FAILED — see reason above.\n";
    return 1;
  }
  std::cout << "INIT OK — PROM read and CRC valid.\n";

  if (sensor.read()) {
    sensor.setSurfacePressure(sensor.getPressureMbar());
    std::cout << "Zeroed at surface: " << sensor.getPressureMbar() << " mbar\n";
  }

  while (run) {
    if (sensor.read()) {
      std::cout << "P=" << sensor.getPressureMbar() << " mbar  "
                << "T=" << sensor.getTemperatureC() << " C  "
                << "depth=" << sensor.getDepthMeters() << " m\n";
    } else {
      std::cerr << "read() failed\n";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  std::cout << "\nDone.\n";
  return 0;
}
