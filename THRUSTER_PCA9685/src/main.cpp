<<<<<<< HEAD
// main.cpp
// - UDP receiver with timeout (SO_RCVTIMEO)
// - Parses Python message (13 fields)
// - Eigen 6DOF -> 8 thrusters allocation (damped least-squares)
// - Saturation / normalization
// - PCA9685 + Thruster output
// - Safety stop if no commands received
//
// Expected Python UDP payload (space-separated 13 values):
// 0 forward, 1 strafe, 2 vertical, 3 yaw, 4 pitch, 5 roll,
// 6 clawRotate, 7 clawOpen, 8 clawPitch, 9 claw1Open,
// 10 pitchAngle, 11 yawAngle, 12 alsInt
//
// NOTE: Tune:
// - PWM frequency (50Hz vs 100Hz) for your ESCs
// - Thruster channel mapping and rest/offset/limits in Thruster
// - Allocation matrix A to match your mechanical layout
// - Axis mixing signs and scaling in desired vector

=======
##include "Constants.h"
#include "I2CPeripheral.h"
#include "PCA9685.h"
#include "Thruster.h"
#include "connection.h"
#include <Eigen/Dense>
#include <Eigen/QR>
#include <arpa/inet.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace std;

int main() {
  const int PORT = 5005;
  //const int BUF_SIZE = 2048;

  // Start UDP receiver in background
  thread net([&] { server(PORT); });



/*
  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    perror("socket");
    return 1;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(PORT);
    if (bind(sockfd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(sockfd);
        return 1;
    }
*/
    cout << "Listening on UDP port using Boost.Asio " << PORT << "..." << endl;

    while (true) {
  
      if (!is_Fresh(250)) {
  
      cout << "STALE (recevied no packets)\r";
      cout.flush();

      }else {
        
        POVState s = get_State();

        cout
        << "fwd=" << s.forward
        << " str=" << s.strafe
        << " vert=" << s.vertical
        << " yaw=" << s.yaw
        << " pitch=" << s.pitch
        << " roll=" << s.roll
        << " clawR=" << s.clawRotate
        << " clawO=" << s.clawOpen
        << " clawP=" << s.clawPitch
        << " claw1=" << s.claw1Open
        << " pAng=" << s.pitchAngle
        << " yAng=" << s.yawAngle
        << " als=" << s.als
        << "        \r";
        
        cout.flush();

      this_thread::sleep_for(chrono::milliseconds(50));

    }
    
    net.join();
    return 0;

  }


/*
        char buf[BUF_SIZE];
        sockaddr_in from{};
        socklen_t from_len = sizeof(from);

        ssize_t n = recvfrom(sockfd, buf, BUF_SIZE - 1, 0, (sockaddr*)&from, &from_len);
        if (n < 0) { perror("recvfrom"); continue; }

        buf[n] = '\0'; // make it a C-string
        std::cout << "Received: " << buf << std::endl;
    }

    close(sockfd);
    return 0;
}


/*
>>>>>>> ff67be4c (added connection code in main)
#include "Constants.h"
#include "I2CPeripheral.h"
#include "PCA9685.h"
#include "Thruster.h"

#include <eigen3/Eigen/Dense>

#include <arpa/inet.h>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

using Clock = std::chrono::steady_clock;

// ------------------------- Utilities -------------------------

static inline double clampd(double v, double lo, double hi) {
  return (v < lo) ? lo : (v > hi) ? hi : v;
}

static inline bool parse_13_doubles(const char* s, double out[13]) {
  // Fast-ish, robust parsing using strtod; rejects if not exactly 13 numbers.
  // Accepts extra whitespace at end.
  const char* p = s;
  char* end = nullptr;

  for (int i = 0; i < 13; ++i) {
    // skip leading spaces
    while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') ++p;
    if (*p == '\0') return false;

    errno = 0;
    out[i] = std::strtod(p, &end);
    if (end == p || errno == ERANGE) return false;
    p = end;
  }

  // After 13 numbers, allow only whitespace
  while (*p) {
    if (!std::isspace(static_cast<unsigned char>(*p))) return false;
    ++p;
  }
  return true;
}

static inline void stop_all(Thruster thrusters[], int n, PiPCA9685::PCA9685& pca) {
  for (int i = 0; i < n; ++i) thrusters[i].stop(pca);
}

// Damped least squares: u = A^T (A A^T + λI)^-1 * x
// Stable for non-square / near-singular A.
static Eigen::Matrix<double, 8, 1>
solve_damped_ls(const Eigen::Matrix<double, 6, 8>& A,
                const Eigen::Matrix<double, 6, 1>& x,
                double lambda) {
  Eigen::Matrix<double, 6, 6> M = A * A.transpose();
  M.diagonal().array() += (lambda * lambda);

  // Solve M y = x, then u = A^T y
  Eigen::Matrix<double, 6, 1> y = M.ldlt().solve(x);
  return A.transpose() * y;
}

// ------------------------- MAIN -------------------------

int main() {
  // ---------- UDP settings ----------
  constexpr int PORT = 5005;
  constexpr int BUF_SIZE = 2048;

  // Safety watchdog
  constexpr int RX_TIMEOUT_MS = 30;      // socket read timeout
  constexpr int STALE_STOP_MS = 300;     // stop thrusters if no valid packet in this time

  // ---------- PCA9685 / ESC settings ----------
  // Typical ESC: 50 Hz. If you truly want 100 Hz and your ESC supports it, set 100.0.
  constexpr double PWM_FREQ_HZ = 50.0;

  // ---------- Thruster layout ----------
  // Map 8 thrusters to PCA9685 channels 0..15
  // Edit these to match your wiring.
  // Example naming: 0 FL, 1 FR, 2 RL, 3 RR, 4 V1, 5 V2, 6 V3, 7 V4 (or however you wired)
  const int thruster_pin[8] = {0, 1, 2, 3, 4, 5, 6, 7};

  // ---------- Allocation matrix ----------
  // A maps thruster outputs u (8x1) to body wrench x (6x1): x = A u
  // Rows: [surge Fx, sway Fy, heave Fz, roll τx, pitch τy, yaw τz]
  //
  // THIS MUST MATCH YOUR REAL GEOMETRY.
  // The below is a reasonable starting point for:
  // - 4 horizontal thrusters at 45° for surge/sway/yaw
  // - 4 vertical thrusters for heave/roll/pitch
  //
  // Tune signs for your motor directions (you can flip columns if a motor is reversed).
  Eigen::Matrix<double, 6, 8> A;
  A <<
      //  H0     H1     H2     H3     V0   V1   V2   V3
       0.707,  0.707,  0.707,  0.707,  0.0, 0.0, 0.0, 0.0,  // surge
      -0.707,  0.707,  0.707, -0.707,  0.0, 0.0, 0.0, 0.0,  // sway
       0.0,    0.0,    0.0,    0.0,    1.0, 1.0, 1.0, 1.0,  // heave
       0.0,    0.0,    0.0,    0.0,   -1.0, 1.0,-1.0, 1.0,  // roll
       0.0,    0.0,    0.0,    0.0,   -1.0,-1.0, 1.0, 1.0,  // pitch (example)
       1.0,   -1.0,    1.0,   -1.0,    0.0, 0.0, 0.0, 0.0;  // yaw

  // Damping factor for damped least squares (bigger = more stable, less aggressive)
  constexpr double LAMBDA = 0.15;

  // Optional: scale each DOF (makes “easy-to-tune”)
  // If one axis feels too strong/weak in water, adjust these.
  Eigen::Matrix<double, 6, 1> dof_gain;
  dof_gain << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

  // ---------- Initialize UDP socket ----------
  int sockfd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    perror("socket");
    return 1;
  }

  int yes = 1;
  (void)setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  // Receive timeout so we can run watchdog logic without blocking forever
  timeval tv{};
  tv.tv_sec = 0;
  tv.tv_usec = RX_TIMEOUT_MS * 1000;
  if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    perror("setsockopt(SO_RCVTIMEO)");
    // not fatal, continue
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(sockfd, (sockaddr*)&addr, sizeof(addr)) < 0) {
    perror("bind");
    close(sockfd);
    return 1;
  }

  std::cout << "Listening on UDP port " << PORT << " (timeout " << RX_TIMEOUT_MS << "ms)\n";

  // ---------- Initialize PCA9685 + Thrusters ----------
  PiPCA9685::PCA9685 pca("/dev/i2c-1", 0x40);
  pca.set_pwm_freq(PWM_FREQ_HZ);

  Thruster thrusters[8];
  for (int i = 0; i < 8; ++i) {
    thrusters[i] = Thruster(thruster_pin[i], &pca);
    // Optional: per-thruster calibration hooks if needed:
    // thrusters[i].setRest(1500);
    // thrusters[i].setOffset(400);
    // thrusters[i].setLimits(1100, 1900);
    thrusters[i].stop(pca);
  }

  // ---------- Main loop ----------
  auto last_good = Clock::now();
  bool ever_received = false;

  char buf[BUF_SIZE];

  while (true) {
    sockaddr_in from{};
    socklen_t from_len = sizeof(from);

    ssize_t n = recvfrom(sockfd, buf, BUF_SIZE - 1, 0, (sockaddr*)&from, &from_len);

    bool got_valid = false;
    double v[13]{};

    if (n > 0) {
      buf[n] = '\0';
      if (parse_13_doubles(buf, v)) {
        got_valid = true;
      } else {
        // Bad packet format: ignore (don’t update last_good)
        got_valid = false;
      }
    } else if (n < 0) {
      // Timeout is expected (EAGAIN/EWOULDBLOCK) due to SO_RCVTIMEO
      if (errno != EAGAIN && errno != EWOULDBLOCK) {
        perror("recvfrom");
      }
    }

    if (got_valid) {
      ever_received = true;
      last_good = Clock::now();

      // Map Python values to desired 6DOF:
      // Python message fields:
      // v[0]=forward, v[1]=strafe, v[2]=vertical, v[3]=yaw, v[4]=pitch, v[5]=roll
      Eigen::Matrix<double, 6, 1> desired;
      desired <<
        v[0], // surge
        v[1], // sway
        v[2], // heave
        v[5], // roll
        v[4], // pitch
        v[3]; // yaw

      // Apply gains (easy tuning)
      desired = desired.cwiseProduct(dof_gain);

      // Clamp desired to sane range [-1, 1] (if sender already clamps, this is extra safety)
      for (int i = 0; i < 6; ++i) desired(i) = clampd(desired(i), -1.0, 1.0);

      // Solve for thruster commands u in [-1, 1] roughly
      Eigen::Matrix<double, 8, 1> u = solve_damped_ls(A, desired, LAMBDA);
      // Normalize if any exceeds 1.0
      const double maxAbs = u.cwiseAbs().maxCoeff();
      if (maxAbs > 1.0) u /= maxAbs;

      // Final clamp
      for (int i = 0; i < 8; ++i) u(i) = clampd(u(i), -1.0, 1.0);

      // Output to thrusters
      for (int i = 0; i < 8; ++i) {
        thrusters[i].setPower(u(i), pca);
      }
    }

    // Safety stop if stale
    const auto now = Clock::now();
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_good).count();

    if (!ever_received || age_ms > STALE_STOP_MS) {
      stop_all(thrusters, 8, pca);
      // Don’t spam prints; only print occasionally if you want
      // std::cerr << "[SAFETY] Stopping thrusters (stale: " << age_ms << "ms)\n";
    }
  }

  // Unreachable, but keep tidy
  close(sockfd);
  return 0;
}
<<<<<<< HEAD

=======
*/
>>>>>>> ff67be4c (added connection code in main)
