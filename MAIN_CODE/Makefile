TARGET     := rov
TARGET_ONE := OneServo
TARGET_SPINCLAW := spinClaw
CXX      := g++
CXXFLAGS := -std=c++17 -g -O0 -Wall -Wextra -pthread
INCLUDES := -I./include
LIBS     := -lboost_system -li2c

ROV_OBJS       := out/main.o out/connection.o out/Thruster_Mixer.o out/Thruster.o out/PID.o out/Claw.o out/PCA9685.o out/I2CPeripheral.o
ONE_SERVO_OBJS := out/main_One_Servo.o out/connection.o out/Thruster_Mixer.o out/Thruster.o out/PID.o out/Claw.o out/PCA9685.o out/I2CPeripheral.o
SPINCLAW_OBJS := out/spinClaw.o out/PCA9685.o out/I2CPeripheral.o

all: out $(TARGET) $(TARGET_ONE)

all: out $(TARGET) $(TARGET_ONE) $(TARGET_SPINCLAW)

$(TARGET_SPINCLAW): $(SPINCLAW_OBJS)
	$(CXX) $(CXXFLAGS) $(SPINCLAW_OBJS) $(LIBS) -o $(TARGET_SPINCLAW)

$(TARGET): $(ROV_OBJS)
	$(CXX) $(CXXFLAGS) $(ROV_OBJS) $(LIBS) -o $(TARGET)

$(TARGET_ONE): $(ONE_SERVO_OBJS)
	$(CXX) $(CXXFLAGS) $(ONE_SERVO_OBJS) $(LIBS) -o $(TARGET_ONE)

out/main.o: src/main.cpp include/Thruster.h include/Thruster_Mixer.h include/Claw.h include/PCA9685.h include/connection.h include/PID.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/main_One_Servo.o: src/main_One_Servo.cpp include/Thruster.h include/Thruster_Mixer.h include/Claw.h include/PCA9685.h include/connection.h include/PID.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/connection.o: src/connection.cpp include/connection.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/Thruster.o: src/Thruster.cpp include/Thruster.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/Thruster_Mixer.o: src/Thruster_Mixer.cpp include/Thruster_Mixer.h include/connection.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/PID.o: src/PID.cpp include/PID.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/Claw.o: src/Claw.cpp include/Claw.h include/PCA9685.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/PCA9685.o: src/PCA9685.cpp include/PCA9685.h include/I2CPeripheral.h include/Constants.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out/I2CPeripheral.o: src/I2CPeripheral.cpp include/I2CPeripheral.h
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

out:
	mkdir -p out

clean:
	rm -f $(TARGET) $(TARGET_ONE) out/*.o
