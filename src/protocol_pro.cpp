#include "protocol_pro.hpp"

namespace RoverRobotics {

ProProtocolObject::ProProtocolObject(const char *device,
                                     std::string new_comm_type,
                                     Control::robot_motion_mode_t robot_mode,
                                     Control::pid_gains pid): use_ext_fb_(false) {
  comm_type_ = new_comm_type;
  robot_mode_ = robot_mode;
  robotstatus_ = {0};
  motors_speeds_[LEFT_MOTOR] = MOTOR_NEUTRAL_;
  motors_speeds_[RIGHT_MOTOR] = MOTOR_NEUTRAL_;
  motors_speeds_[FLIPPER_MOTOR] = MOTOR_NEUTRAL_;
  std::vector<uint32_t> fast_data = {REG_MOTOR_FB_RPM_LEFT,
                                     REG_MOTOR_FB_RPM_RIGHT, EncoderInterval_0,
                                     EncoderInterval_1};
  std::vector<uint32_t> slow_data = {
      REG_MOTOR_FB_CURRENT_LEFT, REG_MOTOR_FB_CURRENT_RIGHT,
      REG_MOTOR_TEMP_LEFT,       REG_MOTOR_TEMP_RIGHT,
      REG_MOTOR_CHARGER_STATE,   BuildNO,
      BATTERY_VOLTAGE_A};
  pid_ = pid;
  PidGains oldgain = {pid_.kp, pid_.ki, pid_.kd};
  if (robot_mode_ != Control::OPEN_LOOP)
    closed_loop_ = true;
  else
    closed_loop_ = false;
  motor1_control_ = OdomControl(closed_loop_, oldgain, 1.5, 0);
  motor2_control_ = OdomControl(closed_loop_, oldgain, 1.5, 0);

  std::cout << "Pro constructor, going to register_comm_base" << std::endl;
   register_comm_base(device);
 

  std::cout << "Pro constructor, going to do thread 1" << std::endl;
   // Create a New Thread with 30 mili seconds sleep timer
   fast_data_write_thread_ =
       std::thread([this, fast_data]() { this->send_command(30, fast_data); });
   // Create a new Thread with 50 mili seconds sleep timer
  std::cout << "Pro constructor, going to do thread 2" << std::endl;
   slow_data_write_thread_ =
       std::thread([this, slow_data]() { this->send_command(50, slow_data); });
   // Create a motor update thread with 30 mili second sleep timer
  std::cout << "Pro constructor, going to do thread 3" << std::endl;
   motor_commands_update_thread_ =
       std::thread([this]() { this->motors_control_loop(30); });
  std::cout << "Pro constructor finished" << std::endl;

}

void ProProtocolObject::update_drivetrim(double value) { trimvalue_ += value; }

void ProProtocolObject::send_estop(bool estop) {
  robotstatus_mutex_.lock();
  robotstatus_.estop = estop;
  robotstatus_mutex_.unlock();
}

robotData ProProtocolObject::status_request() {
  return robotstatus_;
}

robotData ProProtocolObject::info_request() { return robotstatus_; }

void ProProtocolObject::set_robot_velocity(double *controlarray) {
  robotstatus_mutex_.lock();
  robotstatus_.cmd_vel.linear = controlarray[0];
  robotstatus_.cmd_vel.angular = controlarray[1];
  motors_speeds_[FLIPPER_MOTOR] =
      (int)round(controlarray[2] + MOTOR_NEUTRAL_) % MOTOR_MAX_;
  robotstatus_.cmd_ts = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch());
  robotstatus_mutex_.unlock();
}

void ProProtocolObject::set_robot_fb_velocity(double linear_vel, double angular_vel) {
  std::cout << "set_robot_fb_velocity: linear_vel = " << linear_vel << " angular_vel = " << angular_vel << std::endl;
  robotstatus_mutex_.lock();
  robotstatus_.ext_fb_vel.linear = linear_vel;
  robotstatus_.ext_fb_vel.angular = angular_vel;
  robotstatus_mutex_.unlock();
}


void ProProtocolObject::motors_control_loop(int sleeptime) {
  robotVelocity ref_vel; //linear_vel, angular_vel; // reference
  double rpm1, rpm2; // observer motor rotations
  robotVelocity ext_fb_vel;

  std::chrono::milliseconds time_last =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch());
  std::chrono::milliseconds time_from_msg;

  while (true) {
    std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
    std::chrono::milliseconds time_now =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
    
    // get robot data
    robotstatus_mutex_.lock();

    int firmware = robotstatus_.robot_firmware;
    time_from_msg = robotstatus_.cmd_ts;

    ref_vel.linear = robotstatus_.cmd_vel.linear;
    ref_vel.angular = robotstatus_.cmd_vel.angular;
    ext_fb_vel.linear = robotstatus_.ext_fb_vel.linear;
    ext_fb_vel.angular = robotstatus_.ext_fb_vel.angular;

    rpm1 = robotstatus_.motor1.rpm;
    rpm2 = robotstatus_.motor2.rpm;

    robotstatus_mutex_.unlock();

    float ctrl_update_elapsedtime = (time_now - time_from_msg).count();
    float pid_update_elapsedtime = (time_now - time_last).count();

    if (ctrl_update_elapsedtime > CONTROL_LOOP_TIMEOUT_MS_ || robotstatus_.estop) {
      robotstatus_mutex_.lock();
      motors_speeds_[LEFT_MOTOR] = MOTOR_NEUTRAL_;
      motors_speeds_[RIGHT_MOTOR] = MOTOR_NEUTRAL_;
      motors_speeds_[FLIPPER_MOTOR] = MOTOR_NEUTRAL_;
      motor1_control_.reset();
      motor2_control_.reset();
      robotstatus_mutex_.unlock();
      time_last = time_now;
      continue;
    }

    if (ref_vel.angular == 0) {
      if (ref_vel.linear > 0) {
        ref_vel.angular = trimvalue_;
      } else if (ref_vel.linear < 0) {
        ref_vel.angular = -trimvalue_;
      }
    }
    // !Applying some Skid-steer math
    double motor1_vel = ref_vel.linear - 0.5 * wheel2wheelDistance * ref_vel.angular;
    double motor2_vel = ref_vel.linear + 0.5 * wheel2wheelDistance * ref_vel.angular;
    if (motor1_vel == 0) motor1_control_.reset();
    if (motor2_vel == 0) motor2_control_.reset();
    if (firmware == OVF_FIXED_FIRM_VER_) {  // check firmware version
      rpm1 = rpm1 * 2;
      rpm2 = rpm2 * 2;
    }

    double motor1_measured_vel; // left motor
    double motor2_measured_vel; // right motor
    std::cout << "fb loop: use_ext_fb_ = " << use_ext_fb_ << std::endl;

    if(use_ext_fb_) { // TODO make sure we have recent data; otherwise switch to internal fb and throw a warning
      // compute motor vel based on externally observed robot vel
      motor1_measured_vel = ext_fb_vel.linear - 0.5 * wheel2wheelDistance * ext_fb_vel.angular;
      motor2_measured_vel = ext_fb_vel.linear + 0.5 * wheel2wheelDistance * ext_fb_vel.angular;
    } else {
      motor1_measured_vel = rpm1 / MOTOR_RPM_TO_MPS_RATIO_;
      motor2_measured_vel = rpm2 / MOTOR_RPM_TO_MPS_RATIO_;
    }
    robotstatus_mutex_.lock();
    // motor speeds in m/s
    motors_speeds_[LEFT_MOTOR] =
        motor1_control_.run(motor1_vel, motor1_measured_vel,
                            pid_update_elapsedtime / 1000, firmware);
    motors_speeds_[RIGHT_MOTOR] =
        motor2_control_.run(motor2_vel, motor2_measured_vel,
                            pid_update_elapsedtime / 1000, firmware);

    // Convert to 8 bit Command
    motors_speeds_[LEFT_MOTOR] = motor1_control_.boundMotorSpeed(
        int(round(motors_speeds_[LEFT_MOTOR] * 50 + MOTOR_NEUTRAL_)),
        MOTOR_MAX_, MOTOR_MIN_);

    motors_speeds_[RIGHT_MOTOR] = motor2_control_.boundMotorSpeed(
        int(round(motors_speeds_[RIGHT_MOTOR] * 50 + MOTOR_NEUTRAL_)),
        MOTOR_MAX_, MOTOR_MIN_);
    robotstatus_mutex_.unlock();
    time_last = time_now;
  }
}
void ProProtocolObject::unpack_comm_response(std::vector<uint8_t> robotmsg) {
  static std::vector<uint32_t> msgqueue;
  robotstatus_mutex_.lock();
  msgqueue.insert(msgqueue.end(), robotmsg.begin(),
                  robotmsg.end());  // insert robotmsg to msg list
  // ! Delete bytes until valid start byte is found
  if ((unsigned char)msgqueue[0] != startbyte_ &&
      msgqueue.size() > RECEIVE_MSG_LEN_) {
    int startbyte_index = 0;
    // !Did not find valid start byte in buffer
    while (msgqueue[startbyte_index] != startbyte_ &&
           startbyte_index < msgqueue.size())
      startbyte_index++;
    if (startbyte_index >= msgqueue.size()) {
      msgqueue.clear();
      return;
    } else {
      // !Reconstruct the vector so that the start byte is at the 0 position
      std::vector<uint32_t> temp;
      for (int x = startbyte_index; x < msgqueue.size(); x++) {
        temp.push_back(msgqueue[x]);
      }
      msgqueue.clear();
      msgqueue.resize(0);
      msgqueue = temp;
      temp.clear();
    }
  }
  if ((unsigned char)msgqueue[0] == startbyte_ &&
      msgqueue.size() >= RECEIVE_MSG_LEN_) {  // if valid start byte
    unsigned char start_byte_read, data1, data2, dataNO, checksum,
        read_checksum;
    start_byte_read = (unsigned char)msgqueue[0];
    dataNO = (unsigned char)msgqueue[1];
    data1 = (unsigned char)msgqueue[2];
    data2 = (unsigned char)msgqueue[3];
    checksum = 255 - (dataNO + data1 + data2) % 255;
    read_checksum = (unsigned char)msgqueue[4];
    if (checksum == read_checksum) {  // verify checksum
      int16_t b = (data1 << 8) + data2;
      switch (int(dataNO)) {
        case REG_PWR_TOTAL_CURRENT:
          break;
        case REG_MOTOR_FB_RPM_LEFT:
          robotstatus_.motor1.rpm = b;
          break;
        case REG_MOTOR_FB_RPM_RIGHT:  // motor2_rpm;
          robotstatus_.motor2.rpm = b;
          break;
        case REG_FLIPPER_FB_POSITION_POT1:
          robotstatus_.motor3_sensor1 = b;
          break;
        case REG_FLIPPER_FB_POSITION_POT2:
          robotstatus_.motor3_sensor2 = b;
          break;
        case REG_MOTOR_FB_CURRENT_LEFT:
          robotstatus_.motor1.current = b;
          break;
        case REG_MOTOR_FB_CURRENT_RIGHT:
          robotstatus_.motor2.current = b;
          break;
        case REG_MOTOR_ENCODER_COUNT_LEFT:
          break;
        case REG_MOTOR_ENCODER_COUNT_RIGHT:
          break;
        case REG_MOTOR_FAULT_FLAG_LEFT:
          robotstatus_.robot_fault_flag = b;
          break;
        case REG_MOTOR_TEMP_LEFT:
          robotstatus_.motor1.temp = b;
          break;
        case REG_MOTOR_TEMP_RIGHT:
          robotstatus_.motor2.temp = b;
          break;
        case REG_PWR_BAT_VOLTAGE_A:
          break;
        case REG_PWR_BAT_VOLTAGE_B:
          break;
        case EncoderInterval_0:
          break;
        case EncoderInterval_1:
          break;
        case EncoderInterval_2:
          break;
        case REG_ROBOT_REL_SOC_A:
          robotstatus_.battery1.SOC = b;
          break;
        case REG_ROBOT_REL_SOC_B:
          break;
        case REG_MOTOR_CHARGER_STATE:
          break;
        case BuildNO:
          robotstatus_.robot_firmware = b;
          break;
        case REG_PWR_A_CURRENT:
          break;
        case REG_PWR_B_CURRENT:
          break;
        case REG_MOTOR_FLIPPER_ANGLE:
          robotstatus_.motor3_angle = b;
          break;
        case to_computer_REG_MOTOR_SIDE_FAN_SPEED:
          robotstatus_.robot_fan_speed = b;
          break;
        case to_computer_REG_MOTOR_SLOW_SPEED:
          break;
        case BATTERY_STATUS_A:
          break;
        case BATTERY_STATUS_B:
          break;
        case BATTERY_MODE_A:
          robotstatus_.battery1.fault_flag = b;
          break;
        case BATTERY_MODE_B:
          robotstatus_.battery2.fault_flag = b;
          break;
        case BATTERY_TEMP_A:
          robotstatus_.battery1.temp = b;
          break;
        case BATTERY_TEMP_B:
          robotstatus_.battery2.temp = b;
          break;
        case BATTERY_VOLTAGE_A:
          robotstatus_.battery1.voltage = b;
          break;
        case BATTERY_VOLTAGE_B:
          robotstatus_.battery2.voltage = b;
          break;
        case BATTERY_CURRENT_A:
          robotstatus_.battery1.current = b;
          break;
        case BATTERY_CURRENT_B:
          robotstatus_.battery2.current = b;
          break;
      }
      // !Same battery system for both A and B on this robot
      robotstatus_.battery2.SOC = robotstatus_.battery1.SOC;
      // !THESE VALUES ARE NOT AVAILABLE ON ROVER PRO
      robotstatus_.motor1.id = 0;
      robotstatus_.motor1.mos_temp = 0;
      robotstatus_.motor2.id = 0;
      robotstatus_.motor2.mos_temp = 0;
      robotstatus_.motor3.id = 0;
      robotstatus_.motor3.rpm = 0;
      robotstatus_.motor3.current = 0;
      robotstatus_.motor3.temp = 0;
      robotstatus_.motor3.mos_temp = 0;
      robotstatus_.motor4.id = 0;
      robotstatus_.motor4.rpm = 0;
      robotstatus_.motor4.current = 0;
      robotstatus_.motor4.temp = 0;
      robotstatus_.motor4.mos_temp = 0;
      robotstatus_.robot_guid = 0;
      robotstatus_.robot_speed_limit = 0;
      if (robotstatus_.robot_firmware == OVF_FIXED_FIRM_VER_) {  // check firmware version
        robotstatus_.motor_fb_vel.linear =
            0.5 * (robotstatus_.motor1.rpm * 2 / MOTOR_RPM_TO_MPS_RATIO_ +
                   robotstatus_.motor2.rpm * 2 / MOTOR_RPM_TO_MPS_RATIO_);

        robotstatus_.motor_fb_vel.angular =
            ((robotstatus_.motor2.rpm * 2 / MOTOR_RPM_TO_MPS_RATIO_) -
             (robotstatus_.motor1.rpm * 2 / MOTOR_RPM_TO_MPS_RATIO_)) *
            odom_angular_coef_ * odom_traction_factor_;
      } else {
        robotstatus_.motor_fb_vel.linear =
            0.5 * (robotstatus_.motor1.rpm / MOTOR_RPM_TO_MPS_RATIO_ +
                   robotstatus_.motor2.rpm / MOTOR_RPM_TO_MPS_RATIO_);

        robotstatus_.motor_fb_vel.angular =
            ((robotstatus_.motor2.rpm / MOTOR_RPM_TO_MPS_RATIO_) -
             (robotstatus_.motor1.rpm / MOTOR_RPM_TO_MPS_RATIO_)) *
            odom_angular_coef_ * odom_traction_factor_;
      }

      std::vector<uint32_t> temp;
      // !Remove processed msg from queue
      for (int x = RECEIVE_MSG_LEN_; x < msgqueue.size(); x++) {
        temp.push_back(msgqueue[x]);
      }
      msgqueue.clear();
      msgqueue.resize(0);
      msgqueue = temp;
      temp.clear();
    } else {  // !Found start byte but the msg contents were invalid, throw away
              // broken message
      std::vector<uint32_t> temp;
      for (int x = 1; x < msgqueue.size(); x++) {
        temp.push_back(msgqueue[x]);
      }
      msgqueue.clear();
      msgqueue.resize(0);
      msgqueue = temp;
      temp.clear();
    }

  } else {
    // !ran out of data; waiting for more
  }
  robotstatus_mutex_.unlock();
}

bool ProProtocolObject::is_connected() { return comm_base_->is_connected(); }

int ProProtocolObject::cycle_robot_mode() {
  // TODO
  return 0;
}
void ProProtocolObject::register_comm_base(const char *device) {
	std::cout << "register comm base, comm_type_ = " << comm_type_ << std::endl;
	if (comm_type_ == "serial") {
	  std::cout <<"Comm_type_ is serial" << std::endl;
    std::vector<uint8_t> setting;
    setting.push_back(static_cast<uint8_t>(termios_baud_code_ >> 24));
    setting.push_back(static_cast<uint8_t>(termios_baud_code_ >> 16));
    setting.push_back(static_cast<uint8_t>(termios_baud_code_ >> 8));
    setting.push_back(static_cast<uint8_t>(termios_baud_code_));
    setting.push_back(RECEIVE_MSG_LEN_);
    try {
	    std::cout <<"register_comm_base, going to try" << std::endl;
      comm_base_ = std::make_unique<CommSerial>(
          device, [this](std::vector<uint8_t> c) { unpack_comm_response(c); },
          setting);
    } catch (int i) {
	    std::cout << "register comm base, catch" << std::endl;
      throw(i);
    }

  } else {  // not supported device
	  std::cout << "register_comm_base, no supported device." << std::endl;
    throw(-2);
  }
}

void ProProtocolObject::send_command(int sleeptime,
                                     std::vector<uint32_t> datalist)
{
  while (true) {
    for (int x : datalist) {
      if (comm_type_ == "serial") {
        robotstatus_mutex_.lock();
        std::vector<unsigned char> write_buffer = {
            (unsigned char)startbyte_,
            (unsigned char)int(motors_speeds_[LEFT_MOTOR]),
            (unsigned char)int(motors_speeds_[RIGHT_MOTOR]),
            (unsigned char)int(motors_speeds_[FLIPPER_MOTOR]),
            (unsigned char)requestbyte_,
            (unsigned char)x};

        write_buffer.push_back(
            (char)255 - ((unsigned char)int(motors_speeds_[LEFT_MOTOR]) +
                         (unsigned char)int(motors_speeds_[RIGHT_MOTOR]) +
                         (unsigned char)int(motors_speeds_[FLIPPER_MOTOR]) +
                         requestbyte_ + x) %
                            255);
        comm_base_->write_to_device(write_buffer);
        robotstatus_mutex_.unlock();
      } else if (comm_type_ == "can") {
        return;  //* no CAN for rover pro
      } else {   //! How did you get here?
        return;  // TODO: Return error ?
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(sleeptime));
    }
  }
}

void ProProtocolObject::use_external_fb(const bool use_ext_fb)
{
  std::cout << "ProProtocolObject::use_external_fb called , use_ext_fb = " << use_ext_fb << std::endl;
  use_ext_fb_ = use_ext_fb;
}

}  // namespace RoverRobotics
