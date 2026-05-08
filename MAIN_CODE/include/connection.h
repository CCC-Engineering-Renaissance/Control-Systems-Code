#pragma once

#include <chrono> // used for tracking last packet time to detect controller disconnecting and when packets stop

// create spaces that coorespond to values sent in pythons MESSAGE
// using 0.0f is a default value that delivers no motion before another packet arrives
struct POVState{
  float forward = 0.0f;
  float strafe = 0.0f;
  float vertical = 0.0f;
  float yaw = 0.0f;
  float pitch = 0.0f;
  float roll = 0.0f;

  float clawRotate = 0.0f;
  float clawOpen = 0.0f;
  float clawPitch = 0.0f;
  float claw1Open = 0.0f;

  float pitchAngle = 0.0f;
  float yawAngle = 0.0f;

  bool als = false;
};

extern POVState state;

// Start UDP receiver and run it in its own thread 
void server(unsigned short port = 12345);

// Return true if a packet was recently received 
bool is_Fresh(int Max_Age_ms = 250);

// Thread-safe insight of controller state
POVState get_State();

// Setter that may be useful for tests or simulation
void set_State(const POVState& s);

// Signal the UDP server thread to stop
void stopServer();

