#include "connection.h"

#include <boost/asio.hpp> // UDP socket
#include <chrono>
#include <iostream>
#include <mutex> // used for thread safety 
#include <sstream>
#include <string>

using boost::asio::ip::udp;

static boost::asio::io_context io_context;

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
  io_context.stop();
}

//        UDP RECEIVER


void server(unsigned short port){

  try {

    udp::socket sock(io_context, udp::endpoint(udp::v4(), port));
     
    for(;;) {

      char data[1024];

      udp::endpoint remote_endpoint;
      boost::system::error_code error;

      //receive one UDP datagram (blocking)
    std::size_t length = sock.receive_from(boost::asio::buffer(data), remote_endpoint, 0, error);

  //   Ignore oversized error messages but throw on any other errors 
      if(error && error != boost::asio::error::message_size) {

        throw boost::system::system_error(error);

      }

      // convert to string for parsing with sstream
      std::string msg(data, length);
      std::stringstream ss(msg);

      // Prevent partially overwriting global state if the packet isnt fully developed by parsing into a temporary state
      POVState temp{};
      int alsInt = 0;

      // if (bad packet){ ignore it and we can keep the last good state} 
      if (!(ss >> temp.forward >> temp.strafe >> temp.vertical >> temp.yaw >> temp.pitch >> temp.roll
             >> temp.clawRotate >> temp.clawPitch >> temp.pitchAngle
             >> temp.yawAngle >> alsInt)) {
        
        continue; // ignore it

      }

      // convert last field (0/1) to bool 
      temp.als = (alsInt != 0);

      // commit as one update (lock + timestamp)
      set_State(temp);

    }//for{;;}

  }/*try*/ catch (std::exception& e) {
    // if this comes up, control loop should do isFressh()==false and stop
           std::cerr << "Server error: " << e.what() << std::endl;
  } 
}// void server()

