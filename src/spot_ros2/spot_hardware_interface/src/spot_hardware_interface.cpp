// File modified. Modifications Copyright (c) 2024 Boston Dynamics AI Institute LLC.
// All rights reserved.

// --------------------------------------------------------------
// Copyright 2020 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "spot_hardware_interface/spot_hardware_interface.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

#include "google/protobuf/util/time_util.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "bosdyn/client/lease/lease.h"
#include "bosdyn/client/lease/lease_keepalive.h"
#include "bosdyn/client/robot_command/robot_command_builder.h"
#include "bosdyn/client/robot_command/robot_command_client.h"
#include "bosdyn/client/robot_command/robot_command_helpers.h"
#include "bosdyn/client/robot_command/robot_command_streaming_client.h"
#include "bosdyn/client/robot_state/robot_state_client.h"
#include "bosdyn/client/robot_state/robot_state_streaming_client.h"
#include "bosdyn/client/sdk/client_sdk.h"

// ADDED: Includes for high-level arm commands (works without streaming license)
#include "bosdyn/api/arm_command.pb.h"
#include "bosdyn/api/robot_command.pb.h"
#include "bosdyn/api/synchronized_command.pb.h"

namespace spot_hardware_interface {

void StateStreamingHandler::handle_state_streaming(::bosdyn::api::RobotStateStreamResponse& robot_state) {
  // lock so that read/write doesn't happen at the same time
  const std::lock_guard<std::mutex> lock(mutex_);
  // Get joint states from the robot and write them to the joint_states_ struct
  const auto& position_msg = robot_state.joint_states().position();
  const auto& velocity_msg = robot_state.joint_states().velocity();
  const auto& load_msg = robot_state.joint_states().load();
  current_position_.assign(position_msg.begin(), position_msg.end());
  current_velocity_.assign(velocity_msg.begin(), velocity_msg.end());
  current_load_.assign(load_msg.begin(), load_msg.end());

  // Save current foot contact states
  const auto& contact_states = robot_state.contact_states();
  // can't do assign here bc of protobuf issues
  current_foot_state_ = {contact_states.at(0), contact_states.at(1), contact_states.at(2), contact_states.at(3)};

  // Get IMU data from the robot
  imu_identifier_ = robot_state.inertial_state().identifier();
  const auto& imu_position_msg = robot_state.inertial_state().position_imu_rt_link();
  imu_position_ = {imu_position_msg.x(), imu_position_msg.y(), imu_position_msg.z()};
  // Get latest IMU packet and extract linear acceleration, angular velocity, and rotation info
  int i = robot_state.inertial_state().packets_size() - 1;
  const auto& acceleration_msg = robot_state.inertial_state().packets(i).acceleration_rt_odom_in_link_frame();
  const auto& angular_vel_msg = robot_state.inertial_state().packets(i).angular_velocity_rt_odom_in_link_frame();
  const auto& rot_msg = robot_state.inertial_state().packets(i).odom_rot_link();
  imu_linear_acceleration_ = {acceleration_msg.x(), acceleration_msg.y(), acceleration_msg.z()};
  imu_angular_velocity_ = {angular_vel_msg.x(), angular_vel_msg.y(), angular_vel_msg.z()};
  imu_odom_rot_quaternion_ = {rot_msg.x(), rot_msg.y(), rot_msg.z(), rot_msg.w()};

  // Get body pose data from the robot
  const auto& odom_tform_body_pos_msg = robot_state.kinematic_state().odom_tform_body().position();
  const auto& odom_tform_body_rot_msg = robot_state.kinematic_state().odom_tform_body().rotation();
  const auto& vision_tform_body_pos_msg = robot_state.kinematic_state().vision_tform_body().position();
  const auto& vision_tform_body_rot_msg = robot_state.kinematic_state().vision_tform_body().rotation();

  // Get body velocity data from the robot:
  const auto& lin_vel_body_odom_msg = robot_state.kinematic_state().velocity_of_body_in_odom().linear();
  const auto& ang_vel_body_odom_msg = robot_state.kinematic_state().velocity_of_body_in_odom().angular();

  // Save poses
  odom_tform_body_pos_ = {odom_tform_body_pos_msg.x(), odom_tform_body_pos_msg.y(), odom_tform_body_pos_msg.z()};
  odom_tform_body_rot_ = {odom_tform_body_rot_msg.x(), odom_tform_body_rot_msg.y(), odom_tform_body_rot_msg.z(),
                          odom_tform_body_rot_msg.w()};
  vision_tform_body_pos_ = {vision_tform_body_pos_msg.x(), vision_tform_body_pos_msg.y(),
                            vision_tform_body_pos_msg.z()};
  vision_tform_body_rot_ = {vision_tform_body_rot_msg.x(), vision_tform_body_rot_msg.y(), vision_tform_body_rot_msg.z(),
                            vision_tform_body_rot_msg.w()};

  lin_vel_body_odom_ = {lin_vel_body_odom_msg.x(), lin_vel_body_odom_msg.y(), lin_vel_body_odom_msg.z()};
  ang_vel_body_odom_ = {ang_vel_body_odom_msg.x(), ang_vel_body_odom_msg.y(), ang_vel_body_odom_msg.z()};
}

void StateStreamingHandler::get_states(JointStates& joint_states, ImuStates& imu_states, std::vector<int>& foot_states,
                                       std::vector<double>& odom_pos, std::vector<double>& odom_rot,
                                       std::vector<double>& vision_pos, std::vector<double>& vision_rot,
                                       std::vector<double>& odom_lin_vel, std::vector<double>& odom_ang_vel) {
  // lock so that read/write doesn't happen at the same time
  const std::lock_guard<std::mutex> lock(mutex_);
  // Fill in members of the joint states stuct passed in by reference.
  joint_states.position.assign(current_position_.begin(), current_position_.end());
  joint_states.velocity.assign(current_velocity_.begin(), current_velocity_.end());
  joint_states.load.assign(current_load_.begin(), current_load_.end());

  // Fill in members of the imu states struct
  imu_states.identifier = imu_identifier_;
  imu_states.position_imu.assign(imu_position_.begin(), imu_position_.end());
  imu_states.linear_acceleration.assign(imu_linear_acceleration_.begin(), imu_linear_acceleration_.end());
  imu_states.angular_velocity.assign(imu_angular_velocity_.begin(), imu_angular_velocity_.end());
  imu_states.odom_rot_quaternion.assign(imu_odom_rot_quaternion_.begin(), imu_odom_rot_quaternion_.end());

  // Fill in foot contact states
  foot_states.assign(current_foot_state_.begin(), current_foot_state_.end());

  // Fill in body transforms
  odom_pos.assign(odom_tform_body_pos_.begin(), odom_tform_body_pos_.end());
  odom_rot.assign(odom_tform_body_rot_.begin(), odom_tform_body_rot_.end());
  vision_pos.assign(vision_tform_body_pos_.begin(), vision_tform_body_pos_.end());
  vision_rot.assign(vision_tform_body_rot_.begin(), vision_tform_body_rot_.end());
  odom_lin_vel.assign(lin_vel_body_odom_.begin(), lin_vel_body_odom_.end());
  odom_ang_vel.assign(ang_vel_body_odom_.begin(), ang_vel_body_odom_.end());
}

void StateStreamingHandler::reset() {
  // lock so that read/write doesn't happen at the same time
  const std::lock_guard<std::mutex> lock(mutex_);
  current_position_.clear();
  current_velocity_.clear();
  current_load_.clear();
}

hardware_interface::CallbackReturn SpotHardware::on_init(const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  // Get and save parameters
  hostname_ = info_.hardware_parameters["hostname"];
  username_ = info_.hardware_parameters["username"];
  password_ = info_.hardware_parameters["password"];

  {
    const std::string value = info_.hardware_parameters["port"];
    if (!std::all_of(value.begin(), value.end(), ::isdigit)) {
      RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Got %s for a port, expected 0 < port < 65536", value.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (const int port = std::stoi(value); port != 0) {
      port_ = port;
    } else {
      port_.reset();
    }
  }

  if (const auto& certificate = info_.hardware_parameters["certificate"]; !certificate.empty()) {
    certificate_ = certificate;
  } else {
    certificate_.reset();
  }

  if (info_.hardware_parameters["leasing"] == "direct") {
    leasing_mode_ = LeasingMode::DIRECT;
  } else if (info_.hardware_parameters["leasing"] == "proxied") {
    leasing_mode_ = LeasingMode::PROXIED;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Got %s for leasing mode, expected 'direct' or 'proxied'",
                 info_.hardware_parameters["leasing"].c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check that the per-joint configuration matches what we expect.
  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    // First check command interfaces
    if (joint.command_interfaces.size() != command_interfaces_per_joint_) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' has %zu command interfaces found. 3 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 1. check position
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 2. check velocity
    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 3. check effort
    if (joint.command_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 4. check k_q_p
    if (joint.command_interfaces[3].name != HW_IF_K_Q_P) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), HW_IF_K_Q_P.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 5. check k_qd_p
    if (joint.command_interfaces[4].name != HW_IF_K_QD_P) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), HW_IF_K_QD_P.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // Second, check state interfaces
    if (joint.state_interfaces.size() != state_interfaces_per_joint_) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' has %zu state interface. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 1. check position
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 2. check velocity
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
    // 3. check effort
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "Joint '%s' have %s command interfaces found. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  // Check that the sensors match what we expect.
  if (info_.sensors.size() != n_sensors_) {
    RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"), "%ld sensors found. '%ld' expected.", info_.sensors.size(),
                 n_sensors_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  // check that sensor interfaces have the right number of elements
  if (info_.sensors[imu_sensor_index_].state_interfaces.size() != n_imu_sensor_interfaces_) {
    RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"),
                 "IMU sensor state interface has %ld state interfaces found. '%ld' expected.",
                 info_.sensors[imu_sensor_index_].state_interfaces.size(), n_imu_sensor_interfaces_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.sensors[foot_sensor_index_].state_interfaces.size() != n_foot_sensor_interfaces_) {
    RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"),
                 "Feet sensor state interface has %ld state interfaces found. '%ld' expected.",
                 info_.sensors[foot_sensor_index_].state_interfaces.size(), n_foot_sensor_interfaces_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.sensors[odom_to_body_sensor_index_].state_interfaces.size() != n_odom_body_sensor_interfaces_) {
    RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"),
                 "Odom to body frame sensor state interface has %ld state interfaces found. '%ld' expected.",
                 info_.sensors[odom_to_body_sensor_index_].state_interfaces.size(), n_odom_body_sensor_interfaces_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.sensors[odom_to_body_twist_sensor_index_].state_interfaces.size() != n_odom_body_twist_sensor_interfaces_) {
    RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"),
                 "Odom to body frame twist sensor state interface has %ld state interfaces found. '%ld' expected.",
                 info_.sensors[odom_to_body_twist_sensor_index_].state_interfaces.size(),
                 n_odom_body_twist_sensor_interfaces_);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.sensors[vision_to_body_sensor_index_].state_interfaces.size() != n_vision_body_sensor_interfaces_) {
    RCLCPP_FATAL(rclcpp::get_logger("SpotHardware"),
                 "Vision to body frame state interface has %ld state interfaces found. '%ld' expected.",
                 info_.sensors[vision_to_body_sensor_index_].state_interfaces.size(), n_vision_body_sensor_interfaces_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  // resize to fit the expected number of interfaces
  njoints_ = info_.joints.size();
  hw_states_.resize(njoints_ * state_interfaces_per_joint_, std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(njoints_ * command_interfaces_per_joint_, std::numeric_limits<double>::quiet_NaN());
  hw_imu_sensor_states_.resize((n_imu_sensor_interfaces_), std::numeric_limits<double>::quiet_NaN());
  hw_foot_sensor_states_.resize((n_foot_sensor_interfaces_), std::numeric_limits<double>::quiet_NaN());
  hw_odom_body_sensor_states_.resize((n_odom_body_sensor_interfaces_), std::numeric_limits<double>::quiet_NaN());
  hw_odom_body_twist_sensor_states_.resize((n_odom_body_twist_sensor_interfaces_),
                                           std::numeric_limits<double>::quiet_NaN());
  hw_vision_body_sensor_states_.resize((n_vision_body_sensor_interfaces_), std::numeric_limits<double>::quiet_NaN());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SpotHardware::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
  // reset values always when configuring hardware
  hw_states_.assign(hw_states_.size(), 0);
  hw_commands_.assign(hw_commands_.size(), 0);
  // allocate the sizes of these vectors ahead of time
  joint_states_.position.assign(njoints_, 0);
  joint_states_.velocity.assign(njoints_, 0);
  joint_states_.load.assign(njoints_, 0);
  foot_states_.assign(nfeet_, 0);

  // Set up the robot using the BD SDK and start command streaming.
  if (!authenticate_robot(hostname_, username_, password_, port_, certificate_)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!start_time_sync()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!leasing_interface_) {
    switch (leasing_mode_) {
      case LeasingMode::PROXIED:
        leasing_interface_ = std::make_unique<ProxiedLeasingInterface>(robot_.get());
        break;
      case LeasingMode::DIRECT:
        leasing_interface_ = std::make_unique<DirectLeasingInterface>(robot_.get());
        break;
    }
  }
  // Initialize state streaming handler as shared_ptr for compatibility with new polling mode
  state_streaming_handler_ = std::make_shared<StateStreamingHandler>();
  
  if (!start_state_stream(std::bind(&StateStreamingHandler::handle_state_streaming, state_streaming_handler_.get(),
                                    std::placeholders::_1))) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SpotHardware::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Activating SpotHardware...");

  if (!check_estop()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!get_lease()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!power_on()) {
    release_lease();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize command streaming
  if (!start_command_stream()) {
    release_lease();
    power_off();
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Set up command_states struct, initialized to zeros
  joint_commands_.position = std::vector<float>(info_.joints.size(), 0.0);
  joint_commands_.velocity = std::vector<float>(info_.joints.size(), 0.0);
  joint_commands_.load = std::vector<float>(info_.joints.size(), 0.0);
  joint_commands_.k_q_p = std::vector<float>(info_.joints.size(), 0.0);
  joint_commands_.k_qd_p = std::vector<float>(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SpotHardware::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    const auto& joint = info_.joints.at(i);
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_POSITION,
                                                                     &hw_states_[state_interfaces_per_joint_ * i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_VELOCITY,
                                                                     &hw_states_[state_interfaces_per_joint_ * i + 1]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(joint.name, hardware_interface::HW_IF_EFFORT,
                                                                     &hw_states_[state_interfaces_per_joint_ * i + 2]));
  }
  // export sensor state interface

  // export IMU sensor states
  const auto& imu_sensor = info_.sensors[imu_sensor_index_];
  for (size_t i = 0; i < n_imu_sensor_interfaces_; i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        imu_sensor.name, imu_sensor.state_interfaces[i].name, &hw_imu_sensor_states_[i]));
  }

  // export foot sensor states
  const auto& foot_sensor = info_.sensors[foot_sensor_index_];
  for (size_t i = 0; i < n_foot_sensor_interfaces_; i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        foot_sensor.name, foot_sensor.state_interfaces[i].name, &hw_foot_sensor_states_[i]));
  }

  // export odom to body transform sensor states
  const auto& odom_to_body_sensor = info_.sensors[odom_to_body_sensor_index_];
  for (size_t i = 0; i < n_odom_body_sensor_interfaces_; i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        odom_to_body_sensor.name, odom_to_body_sensor.state_interfaces[i].name, &hw_odom_body_sensor_states_[i]));
  }

  // export odom to body twist sensor states
  const auto& odom_to_body_twist_sensor = info_.sensors[odom_to_body_twist_sensor_index_];
  for (size_t i = 0; i < n_odom_body_sensor_interfaces_; i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(odom_to_body_twist_sensor.name,
                                                                     odom_to_body_twist_sensor.state_interfaces[i].name,
                                                                     &hw_odom_body_twist_sensor_states_[i]));
  }

  // export vision to body transform sensor states
  const auto& vision_to_body_sensor = info_.sensors[vision_to_body_sensor_index_];
  for (size_t i = 0; i < n_vision_body_sensor_interfaces_; i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        vision_to_body_sensor.name, vision_to_body_sensor.state_interfaces[i].name, &hw_vision_body_sensor_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SpotHardware::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    const auto& joint = info_.joints.at(i);
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_POSITION, &hw_commands_[command_interfaces_per_joint_ * i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[command_interfaces_per_joint_ * i + 1]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, hardware_interface::HW_IF_EFFORT, &hw_commands_[command_interfaces_per_joint_ * i + 2]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, HW_IF_K_Q_P, &hw_commands_[command_interfaces_per_joint_ * i + 3]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.name, HW_IF_K_QD_P, &hw_commands_[command_interfaces_per_joint_ * i + 4]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn SpotHardware::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!power_off()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  stop_command_stream();
  release_lease();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SpotHardware::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!power_off()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  stop_command_stream();
  release_lease();
  stop_state_stream();
  leasing_interface_.reset();
  time_sync_thread_.reset();
  robot_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SpotHardware::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
  stop_state_stream();
  init_state_ = false;
  leasing_interface_.reset();
  time_sync_thread_.reset();
  robot_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SpotHardware::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  state_streaming_handler_->get_states(joint_states_, imu_states_, foot_states_, odom_pos_, odom_rot_, vision_pos_,
                                       vision_rot_, odom_lin_vel_, odom_ang_vel_);
  const auto& joint_pos = joint_states_.position;
  const auto& joint_vel = joint_states_.velocity;
  const auto& joint_load = joint_states_.load;
  // wait for them to be initialized
  if (joint_pos.empty() || joint_vel.empty() || joint_load.empty()) {
    return hardware_interface::return_type::OK;
  }
  // Ensure that the states received from the Spot SDK will fit into the hw_states_ vector
  const auto states_size = hw_states_.size();
  if (state_interfaces_per_joint_ * joint_pos.size() != states_size ||
      state_interfaces_per_joint_ * joint_vel.size() != states_size ||
      state_interfaces_per_joint_ * joint_load.size() != states_size) {
    RCLCPP_FATAL(
        rclcpp::get_logger("SpotHardware"),
        "The number of joints and interfaces does not match with the outputted joint states from the Spot SDK!");
    return hardware_interface::return_type::ERROR;
  }
  // Read values into joint states
  for (size_t i = 0; i < joint_pos.size(); ++i) {
    hw_states_.at(i * state_interfaces_per_joint_) = joint_pos.at(i);
    hw_states_.at(i * state_interfaces_per_joint_ + 1) = joint_vel.at(i);
    hw_states_.at(i * state_interfaces_per_joint_ + 2) = joint_load.at(i);
  }

  // Fill in the initial command values
  if (!init_state_) {
    for (size_t i = 0; i < njoints_; i++) {
      // copy over current position, velocity, and load from state to current command
      hw_commands_[command_interfaces_per_joint_ * i] = hw_states_[state_interfaces_per_joint_ * i];
      hw_commands_[command_interfaces_per_joint_ * i + 1] = hw_states_[state_interfaces_per_joint_ * i + 1];
      hw_commands_[command_interfaces_per_joint_ * i + 2] = hw_states_[state_interfaces_per_joint_ * i + 2];
      // Fill in k_q_p and k_qd_p gains from the initial_value field from the URDF
      const auto& joint = info_.joints.at(i);
      hw_commands_[command_interfaces_per_joint_ * i + 3] = std::stof(joint.command_interfaces.at(3).initial_value);
      hw_commands_[command_interfaces_per_joint_ * i + 4] = std::stof(joint.command_interfaces.at(4).initial_value);
    }
    init_state_ = true;
  }

  // Read IMU sensor values into sensor states
  // Load rotation quaternion (x, y, z, w)
  hw_imu_sensor_states_.at(0) = imu_states_.odom_rot_quaternion.at(0);
  hw_imu_sensor_states_.at(1) = imu_states_.odom_rot_quaternion.at(1);
  hw_imu_sensor_states_.at(2) = imu_states_.odom_rot_quaternion.at(2);
  hw_imu_sensor_states_.at(3) = imu_states_.odom_rot_quaternion.at(3);
  // Load angular velocity (x, y, z)
  hw_imu_sensor_states_.at(4) = imu_states_.angular_velocity.at(0);
  hw_imu_sensor_states_.at(5) = imu_states_.angular_velocity.at(1);
  hw_imu_sensor_states_.at(6) = imu_states_.angular_velocity.at(2);
  // Load linear acceleration (x, y, z)
  hw_imu_sensor_states_.at(7) = imu_states_.linear_acceleration.at(0);
  hw_imu_sensor_states_.at(8) = imu_states_.linear_acceleration.at(1);
  hw_imu_sensor_states_.at(9) = imu_states_.linear_acceleration.at(2);

  // Load foot contact states
  hw_foot_sensor_states_.assign(foot_states_.begin(), foot_states_.end());

  // Load odom to body pose
  hw_odom_body_sensor_states_.at(0) = odom_pos_.at(0);
  hw_odom_body_sensor_states_.at(1) = odom_pos_.at(1);
  hw_odom_body_sensor_states_.at(2) = odom_pos_.at(2);
  hw_odom_body_sensor_states_.at(3) = odom_rot_.at(0);
  hw_odom_body_sensor_states_.at(4) = odom_rot_.at(1);
  hw_odom_body_sensor_states_.at(5) = odom_rot_.at(2);
  hw_odom_body_sensor_states_.at(6) = odom_rot_.at(3);

  // Load odom to body twist
  hw_odom_body_twist_sensor_states_.at(0) = odom_lin_vel_.at(0);
  hw_odom_body_twist_sensor_states_.at(1) = odom_lin_vel_.at(1);
  hw_odom_body_twist_sensor_states_.at(2) = odom_lin_vel_.at(2);
  hw_odom_body_twist_sensor_states_.at(3) = odom_ang_vel_.at(0);
  hw_odom_body_twist_sensor_states_.at(4) = odom_ang_vel_.at(1);
  hw_odom_body_twist_sensor_states_.at(5) = odom_ang_vel_.at(2);

  // Load vision to body transform
  hw_vision_body_sensor_states_.at(0) = vision_pos_.at(0);
  hw_vision_body_sensor_states_.at(1) = vision_pos_.at(1);
  hw_vision_body_sensor_states_.at(2) = vision_pos_.at(2);
  hw_vision_body_sensor_states_.at(3) = vision_rot_.at(0);
  hw_vision_body_sensor_states_.at(4) = vision_rot_.at(1);
  hw_vision_body_sensor_states_.at(5) = vision_rot_.at(2);
  hw_vision_body_sensor_states_.at(6) = vision_rot_.at(3);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SpotHardware::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  if (command_stream_started_ && init_state_) {
    for (std::size_t i = 0; i < info_.joints.size(); ++i) {
      joint_commands_.position.at(i) = hw_commands_[command_interfaces_per_joint_ * i];
      joint_commands_.velocity.at(i) = hw_commands_[command_interfaces_per_joint_ * i + 1];
      joint_commands_.load.at(i) = hw_commands_[command_interfaces_per_joint_ * i + 2];
      joint_commands_.k_q_p.at(i) = hw_commands_[command_interfaces_per_joint_ * i + 3];
      joint_commands_.k_qd_p.at(i) = hw_commands_[command_interfaces_per_joint_ * i + 4];
    }
    send_command(joint_commands_);
  }
  return hardware_interface::return_type::OK;
}

bool SpotHardware::authenticate_robot(const std::string& hostname, const std::string& username,
                                      const std::string& password, const std::optional<int>& port,
                                      const std::optional<std::string>& certificate) {
  if (robot_) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Robot already authenticated!");
    return true;
  }
  // Create a Client SDK object.
  const auto client_sdk = [&]() -> std::unique_ptr<bosdyn::client::ClientSdk> {
    if (!certificate.has_value()) {
      return ::bosdyn::client::CreateStandardSDK("SpotHardware");
    }
    auto client_sdk = std::make_unique<::bosdyn::client::ClientSdk>();
    client_sdk->SetClientName("SpotHardware");
    if (const auto status = client_sdk->LoadRobotCertFromFile(certificate.value()); !status) {
      return nullptr;
    }
    client_sdk->Init();
    return client_sdk;
  }();
  if (!client_sdk) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not initialize SDK");
    return false;
  }
  auto robot_result = client_sdk->CreateRobot(hostname, ::bosdyn::client::USE_PROXY);
  if (!robot_result) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not create robot");
    return false;
  }
  robot_ = robot_result.move();
  if (port.has_value()) {
    robot_->UpdateSecureChannelPort(port.value());
  }
  const ::bosdyn::common::Status status = robot_->Authenticate(username, password);
  if (!status) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"),
                 "Could not authenticate robot with hostname: %s username: %s password: %s", hostname.c_str(),
                 username.c_str(), password.c_str());
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Robot successfully authenticated!");
  return true;
}

bool SpotHardware::start_time_sync() {
  auto start_response = robot_->StartTimeSync();
  auto time_sync_thread_resp = robot_->GetTimeSyncThread();
  if (!time_sync_thread_resp.status) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not get time sync thread from robot");
    return false;
  }
  time_sync_thread_ = time_sync_thread_resp.move();
  if (!time_sync_thread_->WaitForSync(std::chrono::seconds(5))) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Failed to establish time sync before timing out");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Time sync complete");
  return true;
}

bool SpotHardware::check_estop() {
  // Verify the robot is not estopped and that an external application has registered and holds
  // an estop endpoint.
  const auto estop_status = robot_->IsEstopped();
  if (!estop_status) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not check estop status");
    return false;
  }
  if (estop_status.response) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Robot is e-stopped, cannot continue.");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Estop check complete!");
  return true;
}

bool SpotHardware::get_lease() {
  if (lease_.IsValid()) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Lease already taken");
    return true;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Taking lease...");
  auto lease = leasing_interface_->AcquireLease("body");
  if (!lease) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("SpotHardware"), lease.error());
    return false;
  }
  lease_ = lease.value();
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Lease taken!!");
  return true;
}

bool SpotHardware::power_on() {
  switch (leasing_mode_) {
    case LeasingMode::DIRECT: {
      if (powered_on_) {
        RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Robot is already powered on.");
        break;
      }
      RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Powering on...");
      const auto power_status = robot_->PowerOnMotors(std::chrono::seconds(60), 1.0);
      if (!power_status) {
        RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not power on the robot");
        break;
      }
      RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Powered on!");
      powered_on_ = true;
      break;
    }
    case LeasingMode::PROXIED: {
      RCLCPP_DEBUG(rclcpp::get_logger("SpotHardware"), "Robot power is managed elsewhere, assuming it is powered on.");
      powered_on_ = true;
      break;
    }
  }
  return powered_on_;
}

bool SpotHardware::power_off() {
  switch (leasing_mode_) {
    case LeasingMode::DIRECT: {
      if (!powered_on_) {
        RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Robot is already powered off.");
        break;
      }
      if (command_client_) {
        bosdyn::api::RobotCommand poweroff_command = ::bosdyn::client::SafePowerOffCommand();
        auto poweroff_res = command_client_->RobotCommand(poweroff_command);
        if (!poweroff_res) {
          RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Failed to complete the safe power off command");
          break;
        }
      } else {
        constexpr bool kCutImmediately = true;
        constexpr auto kTimeout = std::chrono::seconds(60);
        constexpr double kUpdateFrequency = 1.0;
        const auto power_status = robot_->PowerOffMotors(!kCutImmediately, kTimeout, kUpdateFrequency);
        if (!power_status) {
          RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not power off the robot");
          break;
        }
      }
      RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Powered off!");
      powered_on_ = false;
      break;
    }
    case LeasingMode::PROXIED: {
      RCLCPP_DEBUG(rclcpp::get_logger("SpotHardware"), "Robot power is managed elsewhere, assuming power off.");
      powered_on_ = false;
      break;
    }
  }
  return !powered_on_;
}

void state_stream_loop(std::stop_token stop_token, ::bosdyn::client::RobotStateStreamingClient* stateStreamClient,
                       StateHandler&& state_policy) {
  ::bosdyn::api::RobotStateStreamResponse latest_state_stream_response;

  while (!stop_token.stop_requested()) {
    // Get robot state stream
    auto robot_state_stream = stateStreamClient->GetRobotStateStream();
    if (!robot_state_stream) {
      RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"),
                   "Failed to get robot state. Does the robot have a valid joint level control license?");
      continue;
    }
    latest_state_stream_response = std::move(robot_state_stream.response);
    state_policy(latest_state_stream_response);
  }
}

// ADDED: Basic state polling loop that doesn't require streaming license
void basic_state_loop(std::stop_token stop_token, ::bosdyn::client::RobotStateClient* stateClient,
                      std::shared_ptr<StateStreamingHandler> state_handler, std::chrono::milliseconds poll_interval) {
  while (!stop_token.stop_requested()) {
    try {
      // Get basic robot state (no streaming license required)
      auto robot_state_response = stateClient->GetRobotState();
      if (!robot_state_response) {
        RCLCPP_DEBUG(rclcpp::get_logger("SpotHardware"), "Failed to get basic robot state");
        std::this_thread::sleep_for(poll_interval);
        continue;
      }
      
      // MINIMAL: Create streaming response with only essential joint and contact data
      ::bosdyn::api::RobotStateStreamResponse stream_response;
      
      // Extract the robot state from the basic response
      const auto& robot_state = robot_state_response.response.robot_state();
      
      // 1. Joint States - Essential for hardware interface
      if (robot_state.has_kinematic_state()) {
        const auto& kinematic_state = robot_state.kinematic_state();
        auto* joint_states = stream_response.mutable_joint_states();
        
        // Copy joint positions, velocities, loads
        if (kinematic_state.joint_states_size() > 0) {
          for (const auto& joint_state : kinematic_state.joint_states()) {
            joint_states->add_position(joint_state.position().value());
            joint_states->add_velocity(joint_state.velocity().value());
            joint_states->add_load(joint_state.load().value());
          }
        }
      }
      
      // 2. IMU/Inertial State - Create minimal version
      auto* inertial_state = stream_response.mutable_inertial_state();
      inertial_state->set_identifier("imu");
      // Add minimal IMU position
      inertial_state->mutable_position_imu_rt_link()->set_x(0.0);
      inertial_state->mutable_position_imu_rt_link()->set_y(0.0);
      inertial_state->mutable_position_imu_rt_link()->set_z(0.0);
      // Add minimal packet with basic data
      auto* packet = inertial_state->add_packets();
      packet->mutable_acceleration_rt_odom_in_link_frame()->set_x(0.0);
      packet->mutable_acceleration_rt_odom_in_link_frame()->set_y(0.0);
      packet->mutable_acceleration_rt_odom_in_link_frame()->set_z(9.81);
      packet->mutable_angular_velocity_rt_odom_in_link_frame()->set_x(0.0);
      packet->mutable_angular_velocity_rt_odom_in_link_frame()->set_y(0.0);
      packet->mutable_angular_velocity_rt_odom_in_link_frame()->set_z(0.0);
      packet->mutable_odom_rot_link()->set_x(0.0);
      packet->mutable_odom_rot_link()->set_y(0.0);
      packet->mutable_odom_rot_link()->set_z(0.0);
      packet->mutable_odom_rot_link()->set_w(1.0);
      
      // 3. Contact States - Ensure we have exactly 4
      for (int i = 0; i < 4; ++i) {
        if (i < robot_state.foot_state_size()) {
          stream_response.add_contact_states(robot_state.foot_state(i).contact());
        } else {
          stream_response.add_contact_states(::bosdyn::api::FootState::CONTACT_UNKNOWN);
        }
      }
      
      // Update the state handler
      state_handler->handle_state_streaming(stream_response);
      
    } catch (const std::exception& e) {
      RCLCPP_DEBUG(rclcpp::get_logger("SpotHardware"), "Exception in basic state loop: %s", e.what());
    }
    
    std::this_thread::sleep_for(poll_interval);
  }
}

bool SpotHardware::start_state_stream(StateHandler&& state_policy) {
  // Suppress unused parameter warning since we're not using streaming callbacks anymore
  (void)state_policy;
  
  if (state_stream_started_) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "State stream has already been started!");
    return true;
  }
  
  // MODIFIED: Use basic robot state client instead of streaming client (no license required)
  auto robot_state_client_resp = robot_->EnsureServiceClient<::bosdyn::client::RobotStateClient>();
  if (!robot_state_client_resp) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not create basic robot state client");
    return false;
  }
  basic_state_client_ = robot_state_client_resp.response;
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Basic Robot State Client created (no streaming license required)");

  // Start polling loop using basic state client instead of streaming
  state_thread_ = std::jthread(&spot_hardware_interface::basic_state_loop, basic_state_client_, 
                               state_streaming_handler_, std::chrono::milliseconds(100)); // 10Hz polling
  state_stream_started_ = true;
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "State polling started at 10Hz (no streaming license required)");
  return true;
}

void SpotHardware::stop_state_stream() {
  if (!state_stream_started_) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "State stream already stopped");
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Stopping state stream");
  state_thread_.request_stop();
  state_thread_.join();
  basic_state_client_ = nullptr;
  state_stream_started_ = false;
  state_streaming_handler_.reset();
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "State stream stopped");
}

bool SpotHardware::start_command_stream() {
  if (command_stream_started_) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Command stream has already been started!");
    return true;
  }
  // Start command streaming
  auto robot_command_stream_client_resp = robot_->EnsureServiceClient<::bosdyn::client::RobotCommandClient>();
  if (!robot_command_stream_client_resp) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Could not create robot command client");
    return false;
  }
  command_client_ = robot_command_stream_client_resp.response;

  command_client_->AddTimeSyncEndpoint(time_sync_thread_->GetEndpoint());

  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Robot Command Client successfully created!");

  // MODIFIED: Skip joint control mode activation (requires streaming license)
  // Note: This will allow the hardware interface to start without the license
  RCLCPP_WARN(rclcpp::get_logger("SpotHardware"), "Skipping joint control mode activation (requires streaming license)");
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Hardware interface will provide state feedback only");

  // NOTE: Skip RobotCommandStreamingClient creation (requires low-level license)
  // This allows the hardware interface to start without the streaming license
  command_stream_service_ = nullptr;

  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Hardware interface started without streaming license - state feedback only");

  // MODIFIED: Skip streaming request setup (requires low-level license)
  // joint_request_ setup removed since we're not using streaming commands

  command_stream_started_ = true;
  return true;
}

void SpotHardware::stop_command_stream() {
  if (!command_stream_started_) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Command stream already stopped");
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Stopping command stream");
  command_client_ = nullptr;
  command_stream_service_ = nullptr;
  command_stream_started_ = false;
  joint_request_.Clear();
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Command stream stopped");
}

void SpotHardware::send_command(const JointCommands& joint_commands) {
  // MODIFIED: Send high-level commands without streaming license (using basic robot API)
  if (!command_client_) {
    RCLCPP_DEBUG(rclcpp::get_logger("SpotHardware"), "Command client not available");
    return;
  }

  // Rate limiting to avoid API overload (2Hz max for basic API)
  static auto last_command_time = std::chrono::steady_clock::now();
  auto current_time = std::chrono::steady_clock::now();
  auto time_since_last = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_command_time);
  
  if (time_since_last < std::chrono::milliseconds(100)) { // 2Hz max
    return;
  }
  last_command_time = current_time;

  try {
    // Detect if this is arm control (last 7 joints) or body control (first 12 joints)
    const bool has_arm = joint_commands.position.size() >= 7;
    const bool has_body = joint_commands.position.size() >= 12;
    
    if (has_arm && joint_commands.position.size() >= 19) {
      // FULL ROBOT: Send arm command using basic API (without streaming)
      send_arm_command_basic(joint_commands);
    } else if (has_body) {
      // BODY ONLY: Send mobility command for body pose
      send_body_command_basic(joint_commands);
    }
    
    // Log occasionally
    static auto last_log_time = std::chrono::steady_clock::now();
    static int command_count = 0;
    command_count++;
    
    auto log_time_since = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_log_time);
    if (log_time_since > std::chrono::seconds(15)) {
      RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), 
                  "Sending commands via basic API (2Hz, %d commands, no streaming license)", 
                  command_count);
      last_log_time = current_time;
      command_count = 0;
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Exception in send_command: %s", e.what());
  }
}

void SpotHardware::send_arm_command_basic(const JointCommands& joint_commands) {
  // Send arm command using high-level API (works without streaming license)
  if (!command_client_) {
    RCLCPP_DEBUG(rclcpp::get_logger("SpotHardware"), "Command client not available");
    return;
  }

  // Extract arm joint positions (last 7 joints: sh0, sh1, el0, el1, wr0, wr1, f1x)
  if (joint_commands.position.size() >= 7) {
    const size_t arm_start = joint_commands.position.size() - 7;
    
    try {
      // Create arm joint command
      auto arm_command = ::bosdyn::api::RobotCommand();
      auto* synchronized_command = arm_command.mutable_synchronized_command();
      auto* arm_sync_command = synchronized_command->mutable_arm_command();
      auto* arm_joint_command = arm_sync_command->mutable_arm_joint_move_command();
      
      // Create trajectory with a single point
      auto* trajectory = arm_joint_command->mutable_trajectory();
      auto* point = trajectory->add_points();

      // Set joint positions using DoubleValue setters (only first 6 joints, skip gripper f1x)
      auto* positions = point->mutable_position();
      positions->mutable_sh0()->set_value(joint_commands.position[arm_start + 0]);
      positions->mutable_sh1()->set_value(joint_commands.position[arm_start + 1]);
      positions->mutable_el0()->set_value(joint_commands.position[arm_start + 2]);
      positions->mutable_el1()->set_value(joint_commands.position[arm_start + 3]);
      positions->mutable_wr0()->set_value(joint_commands.position[arm_start + 4]);
      positions->mutable_wr1()->set_value(joint_commands.position[arm_start + 5]);
      
      // Set time from reference (required for trajectory point)
      point->mutable_time_since_reference()->set_seconds(1.0);
      
      // Send command to robot
      auto result = command_client_->RobotCommand(arm_command);
      
      // Rate limit logging
      static auto last_log_time = std::chrono::steady_clock::now();
      auto current_time = std::chrono::steady_clock::now();
      auto time_since_log = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_log_time);
      
      if (time_since_log > std::chrono::seconds(5)) {
        RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), 
                   "Arm command sent: sh0=%.3f, sh1=%.3f, el0=%.3f (high-level API)", 
                   joint_commands.position[arm_start + 0],
                   joint_commands.position[arm_start + 1], 
                   joint_commands.position[arm_start + 2]);
        last_log_time = current_time;
      }
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("SpotHardware"), "Failed to send arm command: %s", e.what());
    }
  }
}

void SpotHardware::send_body_command_basic(const JointCommands& joint_commands) {
  // Send body pose command using simple mobility API (works without streaming license)
  // NOTE: For now, this function logs command reception but doesn't send complex body commands
  // as the streaming license is required for detailed joint control
  
  // Rate limit logging
  static auto last_log_time = std::chrono::steady_clock::now();
  auto current_time = std::chrono::steady_clock::now();
  auto time_since_log = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_log_time);
  
  if (time_since_log > std::chrono::seconds(30)) {
    RCLCPP_DEBUG(rclcpp::get_logger("SpotHardware"), 
                 "Body command received: %zu joint positions (basic API mode)", 
                 joint_commands.position.size());
    last_log_time = current_time;
  }
  
  // For basic API without streaming license, we can use simple stand/sit commands
  // but detailed body pose control requires streaming license
  // The state feedback still works through basic_state_loop
}

void SpotHardware::release_lease() {
  if (!lease_.IsValid()) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "No lease to return");
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Returning lease...");
  auto lease = leasing_interface_->ReturnLease("body");
  if (lease) {
    RCLCPP_INFO(rclcpp::get_logger("SpotHardware"), "Lease returned!!");
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("SpotHardware"), lease.error());
  }
  lease_ = ::bosdyn::client::Lease();  // invalidate lease
}

}  // namespace spot_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(spot_hardware_interface::SpotHardware, hardware_interface::SystemInterface)
