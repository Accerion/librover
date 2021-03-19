#pragma once

#include "protocol_base.hpp"

namespace RoverRobotics {
class Pro2ProtocolObject;
}
class RoverRobotics::Pro2ProtocolObject
    : public RoverRobotics::BaseProtocolObject {
   public:
    Pro2ProtocolObject(const char* device, std::string new_comm_type,
                       bool closed_loop, PidGains pid);
    /*
   * @brief Trim Robot Velocity
   * Modify robot velocity differential (between the left side/right side) with
   * the input parameter. Useful to compensate if the robot tends to drift
   * either left or right while commanded to drive straight.
   * @param double of velocity offset
   */
    void update_drivetrim(double) override;
    /*
   * @brief Handle Estop Event
   * Send an estop event to the robot
   * @param bool accept a estop state
   */
    void send_estop(bool) override;
    /*
   * @brief Request Robot Status
   * @return structure of statusData
   */
    robotData status_request() override;
    /*
   * @brief Request Robot Unique Infomation
   * @return structure of statusData
   */
    robotData info_request() override;
    /*
   * @brief Set Robot velocity
   * Set Robot velocity: IF closed_loop_ TRUE, this function will attempt a
   * speed PID loop which uses all the available sensor data (wheels, IMUs, etc)
   * from the robot to produce the commanded velocity as best as possible. IF
   * closed_loop_ FALSE, this function simply translates the commanded
   * velocities into motor duty cycles and there is no expectation that the
   * commanded velocities will be realized by the robot. In closed_loop_ FALSE
   * mode, motor power is roughly proportional to commanded velocity.
   * @param controllarray an double array of control in m/s
   */
    void set_robot_velocity(double* controllarray) override;
    /*
   * @brief Unpack bytes from the robot
   * This is meant to use as a callback function when there are bytes available
   * to process
   * @param std::vector<uin32_t> Bytes stream from the robot
   * @return structure of statusData
   */
    void unpack_comm_response(std::vector<uint32_t>) override;
    /*
   * @brief Check if Communication still exist
   * @return bool
   */
    bool is_connected() override;
    /*
   * @brief Attempt to make connection to robot via device
   * @param device is the address of the device (ttyUSB0 , can0, ttyACM0)
   */
    void register_comm_base(const char* device) override;

   private:
    /*
   * @brief Thread Driven function that will send commands to the robot at set
   * interval
   * @param sleeptime sleep time between each cycle
   * @param datalist list of data to request
   */
    void sendCommand(int sleeptime, std::vector<uint32_t> datalist);
    /*
   * @brief Thread Driven function update the robot motors using pid
   * @param sleeptime sleep time between each cycle
   */
    void motors_control_loop(int sleeptime);
    motors_id_[0] = 1;
    motors_id_[1] = 2;
    motors_id_[2] = 3;
    motors_id_[3] = 4;
    const float MOTOR_RPM_TO_MPS_RATIO_ = 13749 / 1.26 / 0.72;
    const int MOTOR_NEUTRAL_ = 0;
    const int MOTOR_MAX_ = 5;
    const int MOTOR_MIN_ = -5;

    const unsigned char startbyte_ = 253;
    const int requestbyte_ = 10;
    const int termios_baud_code_ = 4097;  // THIS = baudrate of 57600
    const int RECEIVE_MSG_LEN_ = 5;
    const double odom_angular_coef_ = 2.3;
    const double odom_traction_factor_ = 0.7;
    const double CONTROL_LOOP_TIMEOUT_MS_ = 200;
    std::unique_ptr<CommBase> comm_base_;
    std::string comm_type_;

    std::mutex robotstatus_mutex_;
    robotData robotstatus_;
    double motors_speeds_[4];
    double trimvalue_;
    std::thread fast_data_write_thread_;
    std::thread slow_data_write_thread_;
    std::thread motor_commands_update_thread_;
    bool estop_;
    // Motor PID variables
    OdomControl motor1_control_;
    OdomControl motor2_control_;
    OdomControl motor3_control_;
    OdomControl motor4_control_;

    bool closed_loop_;
    PidGains pid_;

    const int termios_baud_code_ = 4098;
    const int RECEIVE_MSG_LEN_ = 8;

    enum robot_motors { FRONT_LEFT_MOTOR = 1,
                        FRONT_RIGHT_MOTOR = 2,
                        BACK_LEFT_MOTOR = 3,
                        BACK_RIGHT_MOTOR = 4 };

    enum uart_param {
        REG_PWR_TOTAL_CURRENT = 0,
        REG_MOTOR_FB_RPM_LEFT = 2,
        REG_MOTOR_FB_RPM_RIGHT = 4,
        REG_FLIPPER_FB_POSITION_POT1 = 6,
        REG_FLIPPER_FB_POSITION_POT2 = 8,
        REG_MOTOR_FB_CURRENT_LEFT = 10,
        REG_MOTOR_FB_CURRENT_RIGHT = 12,
        REG_MOTOR_ENCODER_COUNT_LEFT = 14,
        REG_MOTOR_ENCODER_COUNT_RIGHT = 16,
        REG_MOTOR_FAULT_FLAG_LEFT = 18,
        REG_MOTOR_TEMP_LEFT = 20,
        REG_MOTOR_TEMP_RIGHT = 22,
        REG_PWR_BAT_VOLTAGE_A = 24,
        REG_PWR_BAT_VOLTAGE_B = 26,
        EncoderInterval_0 = 28,
        EncoderInterval_1 = 30,
        EncoderInterval_2 = 32,
        REG_ROBOT_REL_SOC_A = 34,
        REG_ROBOT_REL_SOC_B = 36,
        REG_MOTOR_CHARGER_STATE = 38,
        BuildNO = 40,
        REG_PWR_A_CURRENT = 42,
        REG_PWR_B_CURRENT = 44,
        REG_MOTOR_FLIPPER_ANGLE = 46,
        to_computer_REG_MOTOR_SIDE_FAN_SPEED = 48,
        to_computer_REG_MOTOR_SLOW_SPEED = 50,
        BATTERY_STATUS_A = 52,
        BATTERY_STATUS_B = 54,
        BATTERY_MODE_A = 56,
        BATTERY_MODE_B = 58,
        BATTERY_TEMP_A = 60,
        BATTERY_TEMP_B = 62,
        BATTERY_VOLTAGE_A = 64,
        BATTERY_VOLTAGE_B = 66,
        BATTERY_CURRENT_A = 68,
        BATTERY_CURRENT_B = 70
    };
};
