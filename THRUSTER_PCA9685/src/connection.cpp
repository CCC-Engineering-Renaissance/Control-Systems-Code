#include "include/connection.h"

#include <boost/asio.hpp> // UDP socket
#include <chrono>
#include <iostream>
#include <mutex> // used for thread safety 
#include <sstream>
#include <string>

using boost::asio::ip::udp;

POVState state{};

static std::mutex state_Mutex;

// isFresh() returns false until first packet because last_Packet starts as "never"

static std::chrono::steady_clock::time_point last_Packet = 
  std::chrono::steady_clock::time_point::min();


//         READ API

POVState get_State(){
  //allows for a stable snapshot even while a server thread informs new packets
  
  std::lock_guard<std::mutex> lock(state_Mutex);
  return state;
}

bool is_Fresh(int Max_Age_ms){
  // If packets stop being received (crash via wifi or python), the control code can stop movements instead of holding them indefinitely 

  std::lock_guard<std::mutex> lock(state_Mutex);

  if (last_Packet == std::chrono::steady_clock::time_point::min()){
    return false;
  }   

  auto age = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last_Packet).count();

  return age <= Max_Age_ms;

}

//         WRITE API

void set_State(const POVState& s){

  // setting up a place to update state amd timestamp within the same lock

  std::lock_guard<std::mutex> lock(state_Mutex);
  state = s;
  last_Packet = std::chrono::steady_clock::now();

}

//        UDP RECEIVER

void sever(unsigned short port){

  try {

    boost::asio::io_context io_context;
    udp::socket socket(io_context, udp::endpoint(udp::v4(), port));
     
    for(;;) {

      char data[1024];

      udp::endpoint remote_endpoint;
      boost::system::error_code error;

      //receive one UDP datagram (blocking)
    std::size_t length = socket.receive_from(boost::asio::buffer(data), remote_endpoint, 0, error);

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
             >> temp.clawRotate >> temp.clawOpen >> temp.clawPitch >> temp.claw1Open >> temp.pitchAngle 
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
           std::cerr << "Server error: " << e.what() << endl;
  } 
}// void server()
