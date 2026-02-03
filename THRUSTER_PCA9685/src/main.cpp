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
  const int PORT = 5005;
  const int BUF_SIZE = 2048;

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

    std::cout << "Listening on UDP port " << PORT << "...\n";

    while (true) {
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
