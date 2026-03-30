TARGET := rov
MANIP  := manipulator

CXX := g++
CXXFLAGS := -std=c++17 -g -O0 -Wall -Wextra -pthread
INCLUDES := -I./include
LIBS := -lboost_system -li2c

ifeq ($(OS),Windows_NT)
MKDIR_P := if not exist out mkdir out
RM_F := del /Q
else
MKDIR_P := mkdir -p out
RM_F := rm -f
endif

# rov (main thruster control)
ROV_OBJS := out/main.o out/connection.o out/Thruster_Mixer.o out/Thruster.o out/PCA9685.o out/I2CPeripheral.o

# manipulator (arm servo control)
MANIP_OBJS := out/manipulator.o out/connection.o out/Claw.o out/PCA9685.o out/I2CPeripheral.o

all: out $(TARGET)

$(TARGET): $(ROV_OBJS)
	$(CXX) $(CXXFLAGS) $(ROV_OBJS) $(LIBS) -o $(TARGET)

$(MANIP): $(MANIP_OBJS)
	$(CXX) $(CXXFLAGS) $(MANIP_OBJS) $(LIBS) -o $(MANIP)

out/main.o: src/main.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/manipulator.o: src/manipulator.cpp include/Claw.h include/PCA9685.h include/connection.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/connection.o: src/connection.cpp include/connection.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/Thruster.o: src/Thruster.cpp include/Thruster.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/Thruster_Mixer.o: src/Thruster_Mixer.cpp include/Thruster_Mixer.h include/connection.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/Claw.o: src/Claw.cpp include/Claw.h include/PCA9685.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/PCA9685.o: src/PCA9685.cpp include/PCA9685.h include/I2CPeripheral.h include/Constants.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/I2CPeripheral.o: src/I2CPeripheral.cpp include/I2CPeripheral.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out:
	$(MKDIR_P)

clean:
	-$(RM_F) $(TARGET) $(MANIP) out\*.o out/*.o
