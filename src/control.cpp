#include "control.hpp"

#include <math.h>

#include <limits>

namespace Control {
/* functions */

motor_data computeSkidSteerWheelSpeeds(robot_velocities target_velocities,
                                       robot_geometry robot_geometry) {
  /* see documentation for math */
  /* radius of robot's stance */
  float rs = sqrt(pow(0.5 * robot_geometry.wheel_base, 2) +
                  pow(0.5 * robot_geometry.intra_axle_distance, 2));

  /* travel rate(m/s) */
  float left_travel_rate = target_velocities.linear_velocity -
                           (target_velocities.angular_velocity * rs);
  float right_travel_rate = target_velocities.linear_velocity +
                            (target_velocities.angular_velocity * rs);

  /* convert (m/s) -> rpm */
  float left_wheel_speed =
      (left_travel_rate / robot_geometry.wheel_radius) / RPM_TO_RADS_SEC;
  float right_wheel_speed =
      (right_travel_rate / robot_geometry.wheel_radius) / RPM_TO_RADS_SEC;

  motor_data returnstruct = {left_wheel_speed, right_wheel_speed,
                             left_wheel_speed, right_wheel_speed};
  return returnstruct;
}

robot_velocities computeVelocitiesFromWheelspeeds(
    motor_data wheel_speeds, robot_geometry robot_geometry) {
  /* see documentation for math */
  /* radius of robot's stance */
  float rs = sqrt(pow(0.5 * robot_geometry.wheel_base, 2) +
                  pow(0.5 * robot_geometry.intra_axle_distance, 2));

  /* circumference of robot's stance (meters) */
  float cs = 2 * M_PI * rs;

  float left_magnitude = (wheel_speeds.fl + wheel_speeds.rl) / 2;
  float right_magnitude = (wheel_speeds.fr + wheel_speeds.rr) / 2;

  float left_travel_rate =
      left_magnitude * RPM_TO_RADS_SEC * robot_geometry.wheel_radius;
  float right_travel_rate =
      right_magnitude * RPM_TO_RADS_SEC * robot_geometry.wheel_radius;

  /* difference between left and right travel rates */
  float travel_differential = right_travel_rate - left_travel_rate;

  /* compute velocities */
  float linear_velocity = (right_travel_rate + left_travel_rate) / 2;
  float angular_velocity =
      travel_differential / (2 * rs);  // possibly add traction factor here

  robot_velocities returnstruct;
  returnstruct.linear_velocity = linear_velocity;
  returnstruct.angular_velocity = angular_velocity;
  return returnstruct;
}

robot_velocities limitAcceleration(robot_velocities target_velocities,
                                   robot_velocities measured_velocities,
                                   robot_velocities delta_v_limits, float dt) {
  /* compute proposed acceleration */
  float linear_acceleration = (target_velocities.linear_velocity -
                               measured_velocities.linear_velocity) /
                              dt;
  float angular_acceleration = (target_velocities.angular_velocity -
                                measured_velocities.angular_velocity) /
                               dt;

  /* clip the proposed acceleration into an acceptable acceleration */
  if (std::abs(linear_acceleration) >
      std::abs(delta_v_limits.linear_velocity)) {
    std::signbit(linear_acceleration)
        ? linear_acceleration = -delta_v_limits.linear_velocity
        : linear_acceleration = delta_v_limits.linear_velocity;
  }

  /* TODO: fix this */
  // if (std::abs(angular_acceleration) >
  //     std::abs(delta_v_limits.angular_velocity))
  // {
  //   std::signbit(linear_acceleration)
  //       ? angular_acceleration = -delta_v_limits.angular_velocity
  //       : angular_acceleration = delta_v_limits.angular_velocity;
  // }

  /* calculate new velocities */
  robot_velocities return_velocities;
  return_velocities.linear_velocity =
      measured_velocities.linear_velocity + linear_acceleration * dt;
  // return_velocities.angular_velocity =
  //     measured_velocities.angular_velocity + angular_acceleration * dt;
  return_velocities.angular_velocity = target_velocities.angular_velocity;
#ifdef DEBUG
  std::cerr << "target " << target_velocities.linear_velocity << std::endl;
  std::cerr << "measured " << measured_velocities.linear_velocity << std::endl;
  std::cerr << "return " << return_velocities.linear_velocity << std::endl;
  std::cerr << "linear acc " << linear_acceleration << std::endl;
#endif
  return return_velocities;
}

/* classes */
PidController::PidController(struct pid_gains pid_gains, std::string name)
    : /* defaults */
      integral_error_(0),
      previous_error_(0),
      integral_error_limit_(std::numeric_limits<float>::max()),
      pos_max_output_(std::numeric_limits<float>::max()),
      neg_max_output_(std::numeric_limits<float>::lowest()),
      time_last_(std::chrono::steady_clock::now()),
      time_origin_(std::chrono::steady_clock::now()) {
  name_ = name;
  kp_ = pid_gains.kp;
  kd_ = pid_gains.kd;
  ki_ = pid_gains.ki;
};

PidController::PidController(struct pid_gains pid_gains,
                             pid_output_limits pid_output_limits,
                             std::string name)
    : /* defaults */
      integral_error_(0),
      previous_error_(0),
      integral_error_limit_(std::numeric_limits<float>::max()),
      time_last_(std::chrono::steady_clock::now()),
      time_origin_(std::chrono::steady_clock::now()) {
  name_ = name;
  kp_ = pid_gains.kp;
  kd_ = pid_gains.kd;
  ki_ = pid_gains.ki;
  pos_max_output_ = pid_output_limits.posmax;
  neg_max_output_ = pid_output_limits.negmax;
};

void PidController::setGains(struct pid_gains pid_gains) {
  kp_ = pid_gains.kp;
  kd_ = pid_gains.kd;
  ki_ = pid_gains.ki;
};

pid_gains PidController::getGains() {
  pid_gains pid_gains;
  pid_gains.kp = kp_;
  pid_gains.ki = ki_;
  pid_gains.kd = kd_;
  return pid_gains;
}

void PidController::setOutputLimits(pid_output_limits pid_output_limits) {
  pos_max_output_ = pid_output_limits.posmax;
  neg_max_output_ = pid_output_limits.negmax;
}

pid_output_limits PidController::getOutputLimits() {
  pid_output_limits returnstruct;
  returnstruct.posmax = pos_max_output_;
  returnstruct.negmax = neg_max_output_;
  return returnstruct;
}

void PidController::setIntegralErrorLimit(float error_limit) {
  integral_error_limit_ = error_limit;
}

float PidController::getIntegralErrorLimit() { return integral_error_limit_; }

void PidController::writePidDataToCsv(std::ofstream &log_file,
                                      pid_outputs data) {
  log_file << "pid," << data.name << "," << data.time << ","
           << data.target_value << "," << data.measured_value << ","
           << data.pid_output << "," << data.error << "," << data.integral_error
           << "," << data.delta_error << "," << data.kp << "," << data.ki << ","
           << data.kd << "," << std::endl;
  log_file.flush();
}

pid_outputs PidController::runControl(float target, float measured) {
  /* current time */
  std::chrono::steady_clock::time_point time_now =
      std::chrono::steady_clock::now();

  /* delta time (S) */
  float delta_time =
      std::chrono::duration<float>(time_now - time_last_).count();

  /* update time bookkeeping */
  time_last_ = time_now;

  /* error */
  float error = target - measured;

  /* integrate */
  integral_error_ += error * delta_time;

  /* differentiate */
  float delta_error = error - previous_error_;

  /* clip integral error */
  integral_error_ = std::clamp(integral_error_, -integral_error_limit_,
                               integral_error_limit_);

  /* P I D terms */
  float p = kp_ * error;
  float i = ki_ * integral_error_;
  float d = kd_ * (delta_error / delta_time);

  /* compute output */
  float output = p + i + d;

  /* clip output */
  output = std::clamp(output, neg_max_output_, pos_max_output_);

  pid_outputs returnstruct;
  returnstruct.pid_output = output;
  returnstruct.name = name_;
  returnstruct.dt = delta_time;
  returnstruct.time =
      std::chrono::duration<double>(time_now - time_origin_).count();
  returnstruct.error = error;
  returnstruct.integral_error = integral_error_;
  returnstruct.delta_error = (delta_error / delta_time);
  returnstruct.target_value = target;
  returnstruct.measured_value = measured;
  returnstruct.kp = kp_;
  returnstruct.ki = ki_;
  returnstruct.kd = kd_;

  previous_error_ = error;
  return returnstruct;
}

SkidRobotMotionController::SkidRobotMotionController(float max_motor_duty,
                                                     float min_motor_duty,
                                                     float left_trim,
                                                     float right_trim)
    : log_folder_path_("~/Documents/"),
      operating_mode_(OPEN_LOOP),
      duty_cycles_({0}),
      measured_velocities_({0}),
      time_last_(std::chrono::steady_clock::now()),
      time_origin_(std::chrono::steady_clock::now()) {
  min_motor_duty_ = min_motor_duty;
  max_motor_duty_ = max_motor_duty;
  left_trim_value_ = left_trim;
  right_trim_value_ = right_trim;
#ifdef DEBUG
  /*open a log file to store control data*/
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
  auto filename = oss.str();

  log_file_.open("/home/rover/Documents/" + filename + ".csv");
  log_file_ << "type,"
            << "name,"
            << "time,"
            << "col0,"
            << "col1,"
            << "col2,"
            << "col3,"
            << "col4,"
            << "col5,"
            << "col6,"
            << "col7,"
            << "col8,"
            << "col9,"
            << "col10,"
            << "col11," << std::endl;
  log_file_.flush();
#endif
}

SkidRobotMotionController::SkidRobotMotionController(
    robot_motion_mode_t operating_mode, robot_geometry robot_geometry,
    pid_gains pid_gains, float max_motor_duty, float min_motor_duty,
    float left_trim, float right_trim, float geometric_decay)
    : log_folder_path_("~/Documents/"),
      duty_cycles_({0}),
      measured_velocities_({0}),
      max_linear_acceleration_(std::numeric_limits<float>::max()),
      max_angular_acceleration_(std::numeric_limits<float>::max()),
      time_last_(std::chrono::steady_clock::now()),
      time_origin_(std::chrono::steady_clock::now()) {
#ifdef DEBUG
  /*open a log file to store control data*/
  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  std::ostringstream oss;
  oss << std::put_time(&tm, "%d-%m-%Y-%H-%M-%S");
  auto filename = oss.str();
  std::cerr << "log file name " << filename + ".csv" << std::endl;
  log_file_.open("/home/rover/Documents/" + filename + ".csv");
  log_file_ << "type,"
            << "name,"
            << "time,"
            << "col0,"
            << "col1,"
            << "col2,"
            << "col3,"
            << "col4,"
            << "col5,"
            << "col6,"
            << "col7,"
            << "col8,"
            << "col9,"
            << "col10,"
            << "col11," << std::endl;
  log_file_.flush();
#endif

  operating_mode_ = operating_mode;
  robot_geometry_ = robot_geometry;
  pid_gains_ = pid_gains;
  max_motor_duty_ = max_motor_duty;
  min_motor_duty_ = min_motor_duty;
  left_trim_value_ = left_trim;
  right_trim_value_ = right_trim;
  geometric_decay_ = geometric_decay;
  switch (operating_mode) {
    case OPEN_LOOP:
      break;
    case INDEPENDENT_WHEEL:
      /* one pid per wheel */
      pid_controller_fl_ =
          std::make_unique<PidController>(pid_gains_, "pid_front_left");
      pid_controller_fr_ =
          std::make_unique<PidController>(pid_gains_, "pid_front_right");
      pid_controller_rl_ =
          std::make_unique<PidController>(pid_gains_, "pid_rear_left");
      pid_controller_rr_ =
          std::make_unique<PidController>(pid_gains_, "pid_rear_right");
      break;
    case TRACTION_CONTROL:
      /* one pid per side */
      pid_controller_left_ =
          std::make_unique<PidController>(pid_gains_, "pid_left");
      pid_controller_right_ =
          std::make_unique<PidController>(pid_gains_, "pid_right");
      break;
    default:
      /* probably throw exception here */
      break;
  }
}

void SkidRobotMotionController::setAccelerationLimits(robot_velocities limits) {
  max_linear_acceleration_ = limits.linear_velocity;
  max_angular_acceleration_ = limits.angular_velocity;
}

robot_velocities SkidRobotMotionController::getAccelerationLimits() {
  robot_velocities returnstruct;
  returnstruct.angular_velocity = max_angular_acceleration_;
  returnstruct.linear_velocity = max_linear_acceleration_;
  return returnstruct;
}

void SkidRobotMotionController::setOperatingMode(
    robot_motion_mode_t operating_mode) {
  operating_mode_ = operating_mode;
}

robot_motion_mode_t SkidRobotMotionController::getOperatingMode() {
  return operating_mode_;
}

void SkidRobotMotionController::setRobotGeometry(
    robot_geometry robot_geometry) {
  robot_geometry_ = robot_geometry;
}

robot_geometry SkidRobotMotionController::getRobotGeometry() {
  return robot_geometry_;
}

void SkidRobotMotionController::setPidGains(pid_gains pid_gains) {
  pid_gains_ = pid_gains;
}

pid_gains SkidRobotMotionController::getPidGains() { return pid_gains_; }

void SkidRobotMotionController::setMotorMaxDuty(float max_motor_duty) {
  max_motor_duty_ = max_motor_duty;
}
float SkidRobotMotionController::getMotorMaxDuty() { return max_motor_duty_; }

void SkidRobotMotionController::setOutputDecay(float geometric_decay){
  geometric_decay_ = geometric_decay;
}
float SkidRobotMotionController::getOutputDecay(){
  return geometric_decay_;
}

motor_data SkidRobotMotionController::computeMotorCommandsTc_(
    motor_data target_wheel_speeds, motor_data current_motor_speeds) {
  float left_magnitude =
      (current_motor_speeds.fl + current_motor_speeds.rl) / 2;
  float right_magnitude =
      (current_motor_speeds.fr + current_motor_speeds.rr) / 2;

  /* run pid, 1 per side */
  pid_outputs l_pid_output =
      pid_controller_left_->runControl(target_wheel_speeds.fl, left_magnitude);

  pid_outputs r_pid_output = pid_controller_right_->runControl(
      target_wheel_speeds.fr, right_magnitude);

#ifdef DEBUG
  pid_controller_left_->writePidDataToCsv(log_file_, l_pid_output);
  pid_controller_right_->writePidDataToCsv(log_file_, r_pid_output);
#endif

  /* math to split the torque distribution */
  motor_data power_proposals = {
      l_pid_output.pid_output, r_pid_output.pid_output, l_pid_output.pid_output,
      r_pid_output.pid_output};

  isnan(power_proposals.fr) ? power_proposals.fr = 0
                            : power_proposals.fr = power_proposals.fr;
  isnan(power_proposals.fl) ? power_proposals.fl = 0
                            : power_proposals.fl = power_proposals.fl;
  isnan(power_proposals.rr) ? power_proposals.rr = 0
                            : power_proposals.rr = power_proposals.rr;
  isnan(power_proposals.rl) ? power_proposals.rl = 0
                            : power_proposals.rl = power_proposals.rl;

  return power_proposals;
}

motor_data SkidRobotMotionController::clipDutyCycles_(
    motor_data proposed_duties) {
  proposed_duties.fr =
      std::clamp(proposed_duties.fr, -max_motor_duty_, max_motor_duty_);
  proposed_duties.fl =
      std::clamp(proposed_duties.fl, -max_motor_duty_, max_motor_duty_);
  proposed_duties.rr =
      std::clamp(proposed_duties.rr, -max_motor_duty_, max_motor_duty_);
  proposed_duties.rl =
      std::clamp(proposed_duties.rl, -max_motor_duty_, max_motor_duty_);
  return proposed_duties;
}

motor_data SkidRobotMotionController::computeTorqueDistribution_(
    motor_data current_motor_speeds, motor_data power_proposals) {
  /* right side */
  /* if both wheels are moving then ... */

  /* if front wheel is spinning faster ... */
  if (std::abs(current_motor_speeds.fr) >= std::abs(current_motor_speeds.rr)) {
    /* scale down FRONT RIGHT power */
    power_proposals.fr *=
        (isnan(std::abs(current_motor_speeds.rr / current_motor_speeds.fr))
             ? 1.0
             : std::abs(current_motor_speeds.rr / current_motor_speeds.fr));
  } else {
    /* scale down REAR RIGHT power */
    power_proposals.rr *=
        (isnan(std::abs(current_motor_speeds.fr / current_motor_speeds.rr))
             ? 1.0
             : std::abs(current_motor_speeds.fr / current_motor_speeds.rr));
  }

  /* left side */
  if (std::abs(current_motor_speeds.fl) >= std::abs(current_motor_speeds.rl)) {
    /* scale down FRONT LEFT power */
    power_proposals.fl *=
        (isnan(std::abs(current_motor_speeds.rl / current_motor_speeds.fl))
             ? 1.0
             : std::abs(current_motor_speeds.rl / current_motor_speeds.fl));
  } else {
    /* scale down REAR LEFT power */
    power_proposals.rl *=
        (isnan(std::abs(current_motor_speeds.fl / current_motor_speeds.rl))
             ? 1.0
             : std::abs(current_motor_speeds.fl / current_motor_speeds.rl));
  }

  return power_proposals;
}

robot_velocities SkidRobotMotionController::getMeasuredVelocities(
    motor_data current_motor_speeds) {
  return computeVelocitiesFromWheelspeeds(current_motor_speeds,
                                          robot_geometry_);
}

motor_data SkidRobotMotionController::runMotionControl(
    robot_velocities velocity_targets, motor_data current_duty_cycles,
    motor_data current_motor_speeds) {
  /* take the time*/
  std::chrono::steady_clock::time_point time_now =
      std::chrono::steady_clock::now();

  /* delta time (S) */
  float delta_time =
      std::chrono::duration<float>(time_now - time_last_).count();

  float accumulated_time =
      std::chrono::duration<float>(time_now - time_origin_).count();

  time_last_ = time_now;

  /* get estimated robot velocities */
  measured_velocities_ =
      computeVelocitiesFromWheelspeeds(current_motor_speeds, robot_geometry_);

  /* limit acceleration */
  robot_velocities velocity_commands;
  robot_velocities acceleration_limits = {max_linear_acceleration_,
                                          max_angular_acceleration_};

  velocity_commands = limitAcceleration(velocity_targets, measured_velocities_,
                                        acceleration_limits, delta_time);
  // velocity_commands = velocity_targets;

  /* get target wheelspeeds from velocities */
  motor_data target_wheel_speeds =
      computeSkidSteerWheelSpeeds(velocity_commands, robot_geometry_);

  /* apply trim value to targets */
  target_wheel_speeds.fl *= left_trim_value_;
  target_wheel_speeds.rl *= left_trim_value_;
  target_wheel_speeds.fr *= right_trim_value_;
  target_wheel_speeds.rr *= right_trim_value_;

  /* do control */
  motor_data motor_duties_add;
  motor_data modified_duties;
  switch (operating_mode_) {
    case OPEN_LOOP:
      std::cerr << "control type not yet implemented.. commanding 0 motion"
                << std::endl;
      duty_cycles_ = {0, 0, 0, 0};
      break;
    case INDEPENDENT_WHEEL:
      std::cerr << "control type not yet implemented.. commanding 0 motion"
                << std::endl;
      duty_cycles_ = {0, 0, 0, 0};
      break;

    case TRACTION_CONTROL:
      /* determine how much change is needed to the duty cycles */
      motor_duties_add =
          computeMotorCommandsTc_(target_wheel_speeds, current_motor_speeds);

      /* add the change to the duty cycles */
      duty_cycles_.fl += motor_duties_add.fl;
      duty_cycles_.fr += motor_duties_add.fr;
      duty_cycles_.rr += motor_duties_add.rr;
      duty_cycles_.rl += motor_duties_add.rl;

      /* add a geometric decay to the duty cycles */
      duty_cycles_.fl *= geometric_decay_;
      duty_cycles_.fr *= geometric_decay_;
      duty_cycles_.rr *= geometric_decay_;
      duty_cycles_.rl *= geometric_decay_;

      /* run traction control */
      modified_duties =
          computeTorqueDistribution_(current_motor_speeds, duty_cycles_);

      /* don't allow duties higher than the limits */
      modified_duties = clipDutyCycles_(modified_duties);

      /* enforce minimum duty cycles */
      if (std::abs(modified_duties.fl) < min_motor_duty_)
        modified_duties.fl = 0;
      if (std::abs(modified_duties.fr) < min_motor_duty_)
        modified_duties.fr = 0;
      if (std::abs(modified_duties.rl) < min_motor_duty_)
        modified_duties.rl = 0;
      if (std::abs(modified_duties.rr) < min_motor_duty_)
        modified_duties.rr = 0;

      break;
    default:
      std::cerr << "invalid motion control type.. commanding 0 motion"
                << std::endl;
      duty_cycles_ = {0, 0, 0, 0};
      break;
  }

#ifdef DEBUG
  log_file_ << "motion,"
            << "skid," << accumulated_time << ","
            << velocity_commands.linear_velocity << ","
            << velocity_commands.angular_velocity << ","
            << measured_velocities_.linear_velocity << ","
            << measured_velocities_.angular_velocity << ","
            << current_motor_speeds.fl << "," << current_motor_speeds.fr << ","
            << current_motor_speeds.rl << "," << current_motor_speeds.rr << ","
            << duty_cycles_.fl << "," << duty_cycles_.fr << ","
            << duty_cycles_.rr << "," << duty_cycles_.rl << "," << std::endl;
  log_file_.flush();
#endif

  return modified_duties;
}
}  // namespace Control