#include "Constants.h"
#include "I2CPeripheral.h"
#include "PCA9685.h"
#include "Thruster.h"
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
  int PORT = 12345;

  int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
  if (sockfd < 0) {
    perror("socket");
    return 1;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(PORT);
  inet_pton(AF_INET, "192.168.8.161", &addr.sin_addr);
}
