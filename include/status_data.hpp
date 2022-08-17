#include <chrono>
#pragma once
namespace RoverRobotics {
struct motorData {
  signed short int id;
  float rpm;
  unsigned int encoder_count;
  signed short int current;
  signed short int temp;
  signed short int mos_temp;
  };

struct batteryData {
  float voltage;
  signed short int temp;
  unsigned short int current;
  bool SOC;
  unsigned short int fault_flag;
  };

struct robotVelocity {
  double linear;
  double angular;
};

struct robotData {
  // Motor Infos
  motorData motor1;
  motorData motor2;
  motorData motor3;
  motorData motor4;

  // Battery Infos
  batteryData battery1;
  batteryData battery2;

  // Robot FEEDBACK Infos
  unsigned short int robot_guid;
  unsigned short int robot_firmware;
  unsigned short int robot_fault_flag;
  unsigned short int robot_fan_speed;
  unsigned short int robot_speed_limit;

  // Flipper Infos
  unsigned short int motor3_angle;
  unsigned short int motor3_sensor1;
  unsigned short int motor3_sensor2;

  // Robot Info
  robotVelocity motor_fb_vel;
  robotVelocity ext_fb_vel;
  robotVelocity cmd_vel;  

  std::chrono::milliseconds cmd_ts;

  // estop info
  bool estop;
};
}  // namespace RoverRobotics
