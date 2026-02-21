#include "connection.h"

#include <boost/asio.hpp>
#include <chrono>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

using boost::asio::ip::udp;

POVState state{};
static std::mutex state_mutex;
static std::chrono::steady_clock::time_point last_packet =
    std::chrono::steady_clock::time_point::min();


// ================= READ API =================

POVState get_State() {
    std::lock_guard<std::mutex> lock(state_mutex);
    return state;
}

bool is_Fresh(int max_age_ms) {
    std::lock_guard<std::mutex> lock(state_mutex);

    if (last_packet == std::chrono::steady_clock::time_point::min())
        return false;

    auto age_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_packet).count();

    return age_ms <= max_age_ms;
}


// ================= WRITE API =================

void set_State(const POVState& s) {
    std::lock_guard<std::mutex> lock(state_mutex);
    state = s;
    last_packet = std::chrono::steady_clock::now();
}


// ================= UDP SERVER =================

void server(unsigned short port) {
    try {
        boost::asio::io_context io_context;
        udp::socket sock(io_context, udp::endpoint(udp::v4(), port));

        std::cout << "UDP server listening on port " << port << std::endl;

        for (;;) {
            char data[1024];
            udp::endpoint remote_endpoint;
            boost::system::error_code error;

            std::size_t length =
                sock.receive_from(boost::asio::buffer(data),
                                  remote_endpoint, 0, error);

            if (error && error != boost::asio::error::message_size)
                throw boost::system::system_error(error);

            std::string msg(data, length);

            POVState temp = get_State();  // start from last state

            std::stringstream ss(msg);

            // ===============================
            // CASE 1: 13 whitespace numbers
            // ===============================
            int alsInt = 0;

            if (ss >> temp.forward >> temp.strafe >> temp.vertical
                   >> temp.yaw >> temp.pitch >> temp.roll
                   >> temp.clawPitch >> temp.clawOpen
                   >> temp.claw1Open >> temp.clawRotate
                   >> temp.pitchAngle >> temp.yawAngle
                   >> alsInt)
            {
                temp.als = (alsInt != 0);
                set_State(temp);
                continue;
            }

            // ===============================
            // CASE 2: key=value format
            // Example: "pitch=0.53"
            // ===============================
            size_t eq = msg.find('=');
            if (eq != std::string::npos) {
                std::string key = msg.substr(0, eq);
                std::string val = msg.substr(eq + 1);

                try {
                    float f = std::stof(val);

                    if (key == "pitch")
                        temp.pitch = f;
                    else if (key == "yaw")
                        temp.yaw = f;
                    else if (key == "forward")
                        temp.forward = f;
                    else if (key == "vertical")
                        temp.vertical = f;
                    else
                        continue; // unknown key

                    set_State(temp);
                }
                catch (...) {
                    continue; // ignore bad float
                }
            }
        }

    } catch (std::exception& e) {
        std::cerr << "Server error: " << e.what() << std::endl;
    }
}
