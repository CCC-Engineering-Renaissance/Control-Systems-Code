TARGET := rov

CXX := g++
CXXFLAGS := -std=c++17 -g -O0 -Wall -Wextra -pthread
INCLUDES := -I./include
LIBS := -lboost_system

SRCS := \
  src/main.cpp \
  src/connection.cpp \
  src/Thruster.cpp \
  src/PCA9685.cpp \
  src/I2CPeripheral.cpp

OBJS := \
  out/main.o \
  out/connection.o \
  out/Thruster.o \
  out/PCA9685.o \
  out/I2CPeripheral.o

all: out $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) $(LIBS) -o $(TARGET)

out/main.o: src/main.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/connection.o: src/connection.cpp include/connection.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/Thruster.o: src/Thruster.cpp include/Thruster.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/PCA9685.o: src/PCA9685.cpp include/PCA9685.h include/I2CPeripheral.h include/Constants.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/I2CPeripheral.o: src/I2CPeripheral.cpp include/I2CPeripheral.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out:
	mkdir -p out

clean:
	rm -f $(TARGET) out/*.o
