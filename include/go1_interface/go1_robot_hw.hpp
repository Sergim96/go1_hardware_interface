/*
 * Copyright (C) 2022 Francesco Roscia
 * Author: Francesco Roscia
 * email:  francesco.roscia@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
 */

#ifndef GO1_ROBOT_HW_H
#define GO1_ROBOT_HW_H

#include <base_hardware_interface/base_robot_hw.h>
#include <go1_hal/go1_hal.h>
#include <legged_common/hardware_interface/HybridJointInterface.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>

#include <srdfdom/model.h>
#include <srdfdom/srdf_writer.h>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>

typedef Eigen::Matrix<double, 6, 1> butterFilterParams;

namespace go12ros {

class Go1RobotHw : public hardware_interface::RobotHW {
 public:
  Go1RobotHw();
  virtual ~Go1RobotHw();

  void init();
  void read();
  void write();
  const std::string CLASS_NAME = "Go1RobotHw";
  std::string getRobotName() { return robot_name_; }

 protected:
  std::string robot_name_;

  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::ImuSensorInterface imu_sensor_interface_;
  hardware_interface::EffortJointInterface joint_effort_interface_;
  legged::HybridJointInterface hybridJointInterface_;

  unsigned int n_dof_;
  std::vector<std::string> joint_names_;
  std::vector<int> joint_types_;
  std::vector<double> joint_effort_limits_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;

  hardware_interface::ImuSensorHandle::Data imu_data_;
  std::vector<double> imu_orientation_;
  std::vector<double> imu_ang_vel_;
  std::vector<double> imu_lin_acc_;

  std::vector<double> joint_effort_command_;  //?????

 private:
  /** @brief Map Go1 internal joint indices to WoLF joints order */
  std::array<unsigned int, 12> go1_motor_idxs_{{
      go1hal::FL_0, go1hal::FL_1, go1hal::FL_2,  // LF
      go1hal::RL_0, go1hal::RL_1, go1hal::RL_2,  // LH
      go1hal::FR_0, go1hal::FR_1, go1hal::FR_2,  // RF
      go1hal::RR_0, go1hal::RR_1, go1hal::RR_2,  // RH
  }};

  /** @brief Go1-HAL */
  go1hal::LowLevelInterface go1_interface_;
  go1hal::LowState go1_state_ = {0};
  go1hal::LowCmd go1_lowcmd_ = {0};

  /** @brief Sends a zero command to the robot */
  void send_zero_command();

  /** @brief Executes the robot's startup routine */
  void startup_routine();

  /** @brief IMU realtime publisher */
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odom_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::Vector3>> imu_acc_pub_;
  std::vector<butterFilterParams> velocityFilterBuffer;
  void filt(const double raw, butterFilterParams &filt);

  //*****
  void initializeJointsInterface(const std::vector<std::string> &joint_names);
  void initializeImuInterface(const std::string &imu_link_name);
  std::vector<std::string> loadJointNamesFromSRDF();
  std::string loadImuLinkNameFromSRDF();
  bool parseSRDF(srdf::Model &srdf_model);
};

}  // namespace go12ros

#endif
