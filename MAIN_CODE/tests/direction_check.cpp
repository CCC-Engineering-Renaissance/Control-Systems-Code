// direction_check.cpp — verifies spin directions for every control input,
// anchored on two facts confirmed on the real ROV:
//   1. Forward/backward works (left stick Y).
//   2. Ascend/descend works (left/right triggers).
// Replicates main_One_Servo.cpp exactly: channel assignments, inversion
// flags, the manual-yaw negation, depth-PID negation, and kMaxThrustCoeff.
//
// Spin convention (same as main_One_Servo.cpp): PWM > 1500us = CW spin.
// Blade config: ch0 CW=fwd, ch1 CCW=fwd, ch6/ch7 CW=fwd.
//               Rise spins: ch2 CW, ch3 CCW, ch4 CCW, ch5 CW.

#include "PCA9685.h"
#include "Thruster.h"
#include "Thruster_Mixer.h"
#include "PID.h"
#include "connection.h"

#include <iostream>
#include <stdexcept>
#include <string>

namespace {

constexpr float kMaxThrustCoeff = 0.8f;
constexpr int   kNeutral = 1500;

struct Rig {
  PiPCA9685::PCA9685 driver;
  Thruster ch0{0}, ch1{1}, ch6{6}, ch7{7};  // horizontals
  Thruster ch2{2}, ch3{3}, ch4{4}, ch5{5};  // verticals
  Thruster_Mixer mixer;

  Rig() {
    // Same inversion flags as main_One_Servo.cpp
    ch1.setInverted(true);  // frontRightHorizontal
    ch3.setInverted(true);  // rightVertical
    ch4.setInverted(true);  // leftVertical2
  }

  // Mirrors the OneServo control loop: negate manual yaw, mix, apply power.
  void drive(POVState in) {
    in.yaw = -in.yaw;
    const Thruster_Outputs o = mixer.mix(in);
    ch0.setPower(o.frontLeftHorizontal  * kMaxThrustCoeff, driver);
    ch1.setPower(o.frontRightHorizontal * kMaxThrustCoeff, driver);
    ch6.setPower(o.rearLeftHorizontal   * kMaxThrustCoeff, driver);
    ch7.setPower(o.rearRightHorizontal  * kMaxThrustCoeff, driver);
    ch2.setPower(o.leftVertical         * kMaxThrustCoeff, driver);
    ch3.setPower(o.rightVertical        * kMaxThrustCoeff, driver);
    ch4.setPower(o.leftVertical2        * kMaxThrustCoeff, driver);
    ch5.setPower(o.rightVertical2       * kMaxThrustCoeff, driver);
  }
};

void require(bool cond, const std::string& what) {
  if (!cond) throw std::runtime_error("FAILED: " + what);
  std::cout << "  ok: " << what << "\n";
}

// CW means PWM above neutral.
bool cw (const Thruster& t) { return t.getPWM() > kNeutral; }
bool ccw(const Thruster& t) { return t.getPWM() < kNeutral; }

void anchor_forward_backward() {
  std::cout << "[anchor] left stick up = forward (verified in water)\n";
  Rig r;
  POVState s{};
  s.forward = 0.75f;  // stick up, thruster.py sends +
  r.drive(s);
  require(cw (r.ch0), "forward: ch0 CW (=fwd)");
  require(ccw(r.ch1), "forward: ch1 CCW (=fwd)");
  require(cw (r.ch6), "forward: ch6 CW (=fwd)");
  require(cw (r.ch7), "forward: ch7 CW (=fwd)");
  s.forward = -0.75f;
  r.drive(s);
  require(ccw(r.ch0) && cw(r.ch1) && ccw(r.ch6) && ccw(r.ch7),
          "backward: all four horizontals reversed");
}

void anchor_ascend_descend() {
  std::cout << "[anchor] left trigger = ascend (verified in water)\n";
  Rig r;
  POVState s{};
  s.vertical = 0.75f;  // LT, thruster.py sends +
  r.drive(s);
  require(cw (r.ch2), "ascend: ch2 CW (=rise)");
  require(ccw(r.ch3), "ascend: ch3 CCW (=rise)");
  require(ccw(r.ch4), "ascend: ch4 CCW (=rise)");
  require(cw (r.ch5), "ascend: ch5 CW (=rise)");
  s.vertical = -0.75f;  // RT
  r.drive(s);
  require(ccw(r.ch2) && cw(r.ch3) && cw(r.ch4) && ccw(r.ch5),
          "descend: all four verticals reversed");
}

void derived_roll() {
  std::cout << "[derived] RB = roll right, LB = roll left\n";
  Rig r;
  POVState s{};
  s.roll = 0.75f;  // RB, thruster.py sends +
  r.drive(s);
  // Roll right = left side rises, right side dips. Rise/dip spins per anchor 2.
  require(cw (r.ch2), "RB: ch2 (front-left)  CW  = rise");
  require(ccw(r.ch4), "RB: ch4 (rear-left)   CCW = rise");
  require(cw (r.ch3), "RB: ch3 (front-right) CW  = dip");
  require(ccw(r.ch5), "RB: ch5 (rear-right)  CCW = dip");
  s.roll = -0.75f;  // LB
  r.drive(s);
  require(ccw(r.ch2) && cw(r.ch4) && ccw(r.ch3) && cw(r.ch5),
          "LB: mirror of RB = roll left");
}

void derived_pitch() {
  std::cout << "[derived] right stick back = pitch up (nose up)\n";
  Rig r;
  POVState s{};
  // Stick pulled back: rjy=-1 -> pitch_out = rjy*scale*-1 = +0.75 in thruster.py
  s.pitch = 0.75f;
  r.drive(s);
  // Nose up = front pair (ch2/ch3) rises, rear pair (ch4/ch5) dips.
  require(cw (r.ch2), "pitch up: ch2 (front-left)  CW  = rise");
  require(ccw(r.ch3), "pitch up: ch3 (front-right) CCW = rise");
  require(cw (r.ch4), "pitch up: ch4 (rear-left)   CW  = dip");
  require(ccw(r.ch5), "pitch up: ch5 (rear-right)  CCW = dip");
  s.pitch = -0.75f;  // stick pushed forward
  r.drive(s);
  require(ccw(r.ch2) && cw(r.ch3) && ccw(r.ch4) && cw(r.ch5),
          "pitch down: mirror of pitch up");
}

void derived_depth_hold() {
  std::cout << "[derived] ALS depth hold pushes back toward setpoint\n";
  PID depthPID(0.50f, 0.0f, 0.10f, -1.0f, 1.0f);
  // Too deep: measured 5 m, setpoint 2 m (depth is positive-down).
  float out = -depthPID.update(2.0f, 5.0f, 0.02f);  // negation as in OneServo
  require(out > 0.0f, "too deep  -> +vertical command (rise)");
  PID depthPID2(0.50f, 0.0f, 0.10f, -1.0f, 1.0f);
  out = -depthPID2.update(5.0f, 2.0f, 0.02f);
  require(out < 0.0f, "too shallow -> -vertical command (descend)");
}

void report_unverifiable() {
  Rig r;
  POVState s{};
  s.strafe = 0.75f;  // left stick right
  r.drive(s);
  std::cout << "[info] strafe right: front pair "
            << (cw(r.ch0) ? "fwd-thrust" : "rev-thrust") << "..."
            << " ch0=" << r.ch0.getPWM() << " ch1=" << r.ch1.getPWM()
            << " ch6=" << r.ch6.getPWM() << " ch7=" << r.ch7.getPWM()
            << "  (lateral direction depends on mount angles — water test)\n";
  s = POVState{};
  s.yaw = 0.75f;  // right stick right
  r.drive(s);
  std::cout << "[info] yaw stick right: left side ch0=" << r.ch0.getPWM()
            << " ch6=" << r.ch6.getPWM()
            << ", right side ch1=" << r.ch1.getPWM()
            << " ch7=" << r.ch7.getPWM()
            << "  (left reverses, right forward => nose LEFT; flip the"
               " yaw negation in main_One_Servo if wrong in water)\n";
}

}  // namespace

int main() {
  try {
    anchor_forward_backward();
    anchor_ascend_descend();
    derived_roll();
    derived_pitch();
    derived_depth_hold();
    report_unverifiable();
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
  std::cout << "direction_check passed\n";
  return 0;
}
