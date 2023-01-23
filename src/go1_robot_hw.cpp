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

#include "go1_interface/go1_robot_hw.hpp"

namespace go12ros {

using namespace hardware_interface;

int64_t utime_now() {
  struct timeval timeofday;
  gettimeofday(&timeofday, NULL);
  if (timeofday.tv_sec < 0 || timeofday.tv_sec > UINT_MAX)
    throw std::runtime_error("Timeofday is out of dual signed 32-bit range");
  uint32_t sec = timeofday.tv_sec;
  uint32_t nsec = timeofday.tv_usec * 1000;

  return (int64_t)(((uint64_t)sec) * 1000000 + ((uint64_t)nsec) / 1000);
}

Go1RobotHw::Go1RobotHw() { robot_name_ = "go1"; }

Go1RobotHw::~Go1RobotHw() {}

void Go1RobotHw::init() {
  // Hardware interfaces: Joints
  auto joint_names = loadJointNamesFromSRDF();
  if (joint_names.size() > 0) {
    initializeJointsInterface(joint_names);
    registerInterface(&joint_state_interface_);
    // registerInterface(&joint_effort_interface_);
    registerInterface(&hybridJointInterface_);
    velocityFilterBuffer.resize(joint_names.size());
    for (unsigned int i = 0; i < joint_names.size(); i++) {
      // init velocity filters
      velocityFilterBuffer[i][0] = 0.0;
      velocityFilterBuffer[i][1] = 0.0;
      velocityFilterBuffer[i][2] = 0.0;
      velocityFilterBuffer[i][3] = 0.0;
      velocityFilterBuffer[i][4] = 0.0;
      velocityFilterBuffer[i][5] = 0.0;
    }
  } else {
    ROS_ERROR_NAMED(CLASS_NAME, "Failed to register joint interface.");
    return;
  }

  // Hardware interfaces: IMU
  auto imu_name = loadImuLinkNameFromSRDF();
  if (!imu_name.empty()) {
    initializeImuInterface(imu_name);
    registerInterface(&imu_sensor_interface_);
  } else {
    ROS_ERROR_NAMED(CLASS_NAME, "Failed to register imu interface.");
    return;
  }

  go1_interface_.InitCmdData(go1_lowcmd_);
  startup_routine();

  ros::NodeHandle root_nh;
  odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(root_nh, "/go1/ground_truth", 1));
  imu_acc_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Vector3>(root_nh, "/go1/trunk_imu", 1));
}

void Go1RobotHw::filt(const double raw, butterFilterParams &buffer) {
  // notch filter cut-off 50 Hz; BW 20 Hz.
  // double a[3] = {1.0000000e+00,  -1.7633 ,  0.8541};
  // double b[3] = {0.9270 ,  -1.7633 ,  0.9270};
  // notch filter cut-off 110 Hz; BW 220 Hz.
  // double a[3] = {1.0000000e+00,  -0.8433 ,  0.0945};
  // double b[3] = {0.5473 ,  -0.8433 ,  0.5473};
  // 2nd order butterworth cut 150 for fs = 1000 Hz
  // double a[3] = {1.0000000e+00,  -0.7478 ,  0.2722};
  // double b[3] = {0.1311 ,  0.2622 ,  0.1311};
  // 2nd order butterworth cut 150 for fs = 2000 Hz
  // double a[3] = {1.0000 ,  -1.349 ,  0.5140};
  // double b[3] = {0.0413 ,  0.0825 ,  0.0413};
  // 2nd order butterworth cut 150 for fs = 800 Hz
  // double a[3] = {1.0000 ,  -0.4629 ,  0.2097};
  // double b[3] = {0.1867 ,   0.3734 ,  0.1867};
  // 2nd order butterworth cut 250 for fs = 1000 Hz
  //    double a[3] = { 1.0000, 0.0000, 0.1716 };
  //    double b[3] = { 0.2929, 0.5858, 0.2929 };
  // 2nd order butterworth cut 40
  // double a[3] = {1.0000000e+00,  -3.6952738e-01 ,  1.9581571e-01};
  // double b[3] = {2.0657208e-01 ,  4.1314417e-01 ,  2.0657208e-01};
  // 2nd order butterworth 30
  double a[3] = {1.0000000e+00, -7.4778918e-01, 2.7221494e-01};
  double b[3] = {1.3110644e-01, 2.6221288e-01, 1.3110644e-01};
  // 2nd order butterworth 15
  // double a[3] = {1.0000000e+00 , -1.3489677e+00 ,  5.1398189e-01 };
  // double b[3] = {4.1253537e-02 ,  8.2507074e-02,   4.1253537e-02};
  // 2nd order butterworth 8
  // double a[3] = {1.0000000e+00 , -1.6474600e+00 ,  7.0089678e-01 };
  // double b[3] = {1.3359200e-02 ,  2.6718400e-02 ,  1.3359200e-02  };
  // first 3 elements are y0 y1 y2 second 3 x0 x1 x2
  int input = 3;
  buffer[input + 0] = raw;
  buffer[0] = -a[1] * buffer[1] - a[2] * buffer[2] + b[0] * buffer[input + 0] + b[1] * buffer[input + 1] +
              b[2] * buffer[input + 2];

  buffer[input + 2] = buffer[input + 1];
  buffer[input + 1] = buffer[input + 0];
  buffer[input] = buffer[2];
  buffer[2] = buffer[1];
  buffer[1] = buffer[0];
}

void Go1RobotHw::read() {
  // Get robot data
  go1_state_ = go1_interface_.ReceiveObservation();

  // ------
  // Joints
  // ------
  for (unsigned int jj = 0; jj < n_dof_; ++jj) {
    joint_position_[jj] = static_cast<double>(go1_state_.motorState[go1_motor_idxs_[jj]].q);

    // filt(static_cast<double>(go1_state_.motorState[go1_motor_idxs_[jj]].dq) ,
    // velocityFilterBuffer[jj]);
    joint_velocity_[jj] =
        static_cast<double>(go1_state_.motorState[go1_motor_idxs_[jj]].dq);  // velocityFilterBuffer[jj][0];

    joint_effort_[jj] = static_cast<double>(go1_state_.motorState[go1_motor_idxs_[jj]].tauEst);
  }

  // ---
  // IMU
  // ---
  imu_orientation_[0] = static_cast<double>(go1_state_.imu.quaternion[0]);  // w
  imu_orientation_[1] = static_cast<double>(go1_state_.imu.quaternion[1]);  // x
  imu_orientation_[2] = static_cast<double>(go1_state_.imu.quaternion[2]);  // y
  imu_orientation_[3] = static_cast<double>(go1_state_.imu.quaternion[3]);  // z

  imu_ang_vel_[0] = static_cast<double>(go1_state_.imu.gyroscope[0]);
  imu_ang_vel_[1] = static_cast<double>(go1_state_.imu.gyroscope[1]);
  imu_ang_vel_[2] = static_cast<double>(go1_state_.imu.gyroscope[2]);

  imu_lin_acc_[0] = static_cast<double>(go1_state_.imu.accelerometer[0]);
  imu_lin_acc_[1] = static_cast<double>(go1_state_.imu.accelerometer[1]);
  imu_lin_acc_[2] = static_cast<double>(go1_state_.imu.accelerometer[2]);

  // Publish the IMU data NOTE: missing covariances
  if (odom_pub_.get() && odom_pub_->trylock()) {
    odom_pub_->msg_.pose.pose.orientation.w = imu_orientation_[0];
    odom_pub_->msg_.pose.pose.orientation.x = imu_orientation_[1];
    odom_pub_->msg_.pose.pose.orientation.y = imu_orientation_[2];
    odom_pub_->msg_.pose.pose.orientation.z = imu_orientation_[3];
    odom_pub_->msg_.twist.twist.angular.x = imu_ang_vel_[0];
    odom_pub_->msg_.twist.twist.angular.y = imu_ang_vel_[1];
    odom_pub_->msg_.twist.twist.angular.z = imu_ang_vel_[2];

    odom_pub_->msg_.header.stamp = ros::Time::now();
    odom_pub_->unlockAndPublish();
  }

  if (imu_acc_pub_.get() && imu_acc_pub_->trylock()) {
    imu_acc_pub_->msg_.x = imu_lin_acc_[0];
    imu_acc_pub_->msg_.y = imu_lin_acc_[1];
    imu_acc_pub_->msg_.z = imu_lin_acc_[2];

    imu_acc_pub_->unlockAndPublish();
  }
}

void Go1RobotHw::write() {
  std::cout << "write" << std::endl;
  for (unsigned int jj = 0; jj < n_dof_; ++jj) {
    go1_lowcmd_.motorCmd[go1_motor_idxs_[jj]].mode = 0x0A;  // motor switch to servo (PMSM) mode
    go1_lowcmd_.motorCmd[go1_motor_idxs_[jj]].tau = static_cast<float>(joint_hybrid_[jj].ff_);
    go1_lowcmd_.motorCmd[go1_motor_idxs_[jj]].q = static_cast<float>(joint_hybrid_[jj].posDes_);
    go1_lowcmd_.motorCmd[go1_motor_idxs_[jj]].dq = static_cast<float>(joint_hybrid_[jj].velDes_);
    go1_lowcmd_.motorCmd[go1_motor_idxs_[jj]].Kp = static_cast<float>(joint_hybrid_[jj].kp_);
    go1_lowcmd_.motorCmd[go1_motor_idxs_[jj]].Kd = static_cast<float>(joint_hybrid_[jj].kd_);
  }
  go1_lowcmd_.head[0] = 0xFE;
  go1_lowcmd_.head[1] = 0xEF;
  go1_lowcmd_.levelFlag = unitree::LOWLEVEL;

  go1_interface_.SendLowCmd(go1_lowcmd_);
}

void Go1RobotHw::send_zero_command() {
  std::array<float, 60> zero_command = {0};
  // go1_interface_->SendCommand(zero_command);
  // IMPORTANT! this ensures all the Kp Kd gains are set to zero
  go1_interface_.SendCommand(zero_command);
}

void Go1RobotHw::startup_routine() {
  send_zero_command();
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void Go1RobotHw::initializeJointsInterface(const std::vector<std::string> &joint_names) {
  // Resize vectors to our DOF
  n_dof_ = static_cast<unsigned int>(joint_names.size());
  joint_names_.resize(n_dof_);
  joint_types_.resize(n_dof_);
  joint_effort_limits_.resize(n_dof_);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_hybrid_.resize(n_dof_);

  for (unsigned int j = 0; j < n_dof_; j++) {
    ROS_DEBUG_STREAM_NAMED(CLASS_NAME, "Loading joint: " << joint_names[j]);

    joint_names_[j] = joint_names[j];
    joint_position_[j] = 1.0;
    joint_velocity_[j] = 0.0;
    joint_effort_[j] = 0.0;  // N/m for continuous joints

    joint_effort_command_[j] = 0.0;

    joint_hybrid_[j].posDes_ = 0;
    joint_hybrid_[j].velDes_ = 0;
    joint_hybrid_[j].kp_ = 0;
    joint_hybrid_[j].kd_ = 0;
    joint_hybrid_[j].ff_ = 0;
    // Create joint state interface for all joints
    hardware_interface::JointStateHandle state_handle(
        joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]);
    joint_state_interface_.registerHandle(state_handle);

    // joint_effort_interface_.registerHandle(
    //     JointHandle(joint_state_interface_.getHandle(joint_names_[j]), &joint_effort_command_[j]));

    hybridJointInterface_.registerHandle(
        legged::HybridJointHandle(state_handle, &joint_hybrid_[j].posDes_, &joint_hybrid_[j].velDes_,
        &joint_hybrid_[j].kp_, &joint_hybrid_[j].kd_, &joint_hybrid_[j].ff_));
  }
}

void Go1RobotHw::initializeImuInterface(const std::string &imu_link_name) {
  imu_orientation_.resize(4);
  imu_ang_vel_.resize(3);
  imu_lin_acc_.resize(3);

  imu_data_.name = "imu";
  imu_data_.frame_id = imu_link_name;
  imu_data_.orientation = &imu_orientation_[0];
  imu_data_.angular_velocity = &imu_ang_vel_[0];
  imu_data_.linear_acceleration = &imu_lin_acc_[0];
  imu_sensor_interface_.registerHandle(hardware_interface::ImuSensorHandle(imu_data_));
}

std::vector<std::string> Go1RobotHw::loadJointNamesFromSRDF() {
  std::vector<std::string> joint_names;
  srdf::Model srdf_model;
  if (parseSRDF(srdf_model)) {
    auto group_states = srdf_model.getGroupStates();
    for (unsigned int i = 0; i < group_states.size(); i++)
      if (group_states[i].name_ == "standup")  // Look for the standup group state and get the names of
                                               // the joints in there
        for (auto &tmp : group_states[i].joint_values_) joint_names.push_back(tmp.first);
  }
  return joint_names;
}

std::string Go1RobotHw::loadImuLinkNameFromSRDF() {
  std::string imu_name;
  srdf::Model srdf_model;

  if (parseSRDF(srdf_model)) {
    auto groups = srdf_model.getGroups();
    for (unsigned int i = 0; i < groups.size(); i++) {
      const auto &links = groups[i].links_;
      if (groups[i].name_.find("imu") != std::string::npos) {
        if (links.size() == 1)
          imu_name = links[0];
        else
          throw std::runtime_error("There can be only one imu_sensor defined in the SRDF file!");
      }
    }
  }
  return imu_name;
}

bool Go1RobotHw::parseSRDF(srdf::Model &srdf_model) {
  ros::NodeHandle nh;
  std::string srdf, urdf;
  if (!nh.getParam("/robot_description", urdf)) {
    ROS_ERROR_NAMED(CLASS_NAME, "robot_description not available in the ros param server");
    return false;
  }
  if (!nh.getParam("/robot_semantic_description", srdf)) {
    ROS_ERROR_NAMED(CLASS_NAME, "robot_semantic_description not available in the ros param server");
    return false;
  }

  urdf::ModelInterfaceSharedPtr u = urdf::parseURDF(urdf);
  if (!srdf_model.initString(*u, srdf)) {
    ROS_ERROR_NAMED(CLASS_NAME, "Can not initialize SRDF model from XML string!");
    return false;
  }

  return true;
}

}  // namespace go12ros
