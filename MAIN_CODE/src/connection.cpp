#include "connection.h"

#include <boost/asio.hpp>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/select.h>

using boost::asio::ip::udp;

static boost::asio::io_context io_context;
static std::atomic<bool> g_stop{false};

POVState state{};

// Mutex protects both "state" and "last_packet" so reads/writes can't interleave.
static std::mutex state_mutex;

// Time of last successfully parsed packet.
// Initialized to "min" so is_Fresh() returns false until the first valid packet arrives.
static std::chrono::steady_clock::time_point last_packet = std::chrono::steady_clock::time_point::min();


//         READ API


POVState get_State() {
  // lock_guard locks the mutex now, unlocks automatically when function returns.
  // This makes the returned state a consistent snapshot.
  std::lock_guard<std::mutex> lock(state_mutex);
  return state;
}

bool is_Fresh(int max_age_ms) {
  // main calls this to decide whether to keep driving thrusters or stop them.
  std::lock_guard<std::mutex> lock(state_mutex);

  // No packet ever received => stale by definition.
  if (last_packet == std::chrono::steady_clock::time_point::min()) {
    return false;
  }

  // Compute age of the last packet in milliseconds.
  auto age_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - last_packet
    ).count();

  // Fresh if it is newer than the allowed max age.
  return age_ms <= max_age_ms;
}

//         WRITE API


void set_State(const POVState& s){

  // setting up a place to update state amd timestamp within the same lock

  std::lock_guard<std::mutex> lock(state_mutex);
  state = s;
  last_packet = std::chrono::steady_clock::now();

}

void stopServer() {
  g_stop.store(true, std::memory_order_relaxed);
}

//        UDP RECEIVER


void server(unsigned short port) {
  try {
    g_stop.store(false, std::memory_order_relaxed);

    udp::socket sock(io_context, udp::endpoint(udp::v4(), port));
    sock.non_blocking(true);
    int fd = static_cast<int>(sock.native_handle());

    for (;;) {
      if (g_stop.load(std::memory_order_relaxed)) break;

      // Wait up to 100 ms for a datagram to arrive.
      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(fd, &read_fds);
      struct timeval tv;
      tv.tv_sec  = 0;
      tv.tv_usec = 100'000;  // 100 ms

      int ready = select(fd + 1, &read_fds, nullptr, nullptr, &tv);

      if (ready < 0) {
        if (errno == EINTR) continue;  // signal interrupted select — retry
        throw std::runtime_error(std::string("select() failed: ") + std::strerror(errno));
      }

      if (ready == 0) continue;  // timeout — loop back and re-check g_stop

      // A datagram is waiting; receive_from returns immediately (non-blocking).
      char data[1024];
      udp::endpoint remote_endpoint;
      boost::system::error_code error;
      std::size_t length = sock.receive_from(
          boost::asio::buffer(data), remote_endpoint, 0, error);

      if (error == boost::asio::error::would_block) continue;
      if (error && error != boost::asio::error::message_size)
        throw boost::system::system_error(error);

      std::string msg(data, length);
      std::stringstream ss(msg);

      POVState temp{};
      int alsInt = 0;

      if (!(ss >> temp.forward >> temp.strafe >> temp.vertical >> temp.yaw >> temp.pitch >> temp.roll
               >> temp.clawRotate >> temp.clawOpen >> temp.pitchAngle
               >> temp.yawAngle >> alsInt)) {
        continue;  // malformed packet — keep last good state
      }

      temp.als = (alsInt != 0);
      set_State(temp);
    }
  } catch (std::exception& e) {
    std::cerr << "Server error: " << e.what() << std::endl;
  }
}  // void server()

