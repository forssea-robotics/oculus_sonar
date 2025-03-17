//
// @file - oculus_republisher.cpp
// @author - Ian MCELROY, Forssea Robotics (ian@forssea-robotics.fr)
// @brief - Oculus sonar data republisher
// @version - 0.1
// @date - 2025-03-11
//
// Copyright (c) 2025 Forssea Robotics. All rights reserved.
//
/**
 * BSD 3-Clause License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <rclcpp/executors.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "oculus_ros2/oculus_republisher_node.hpp"
#include <oculus_driver/Oculus.h>
#include <oculus_driver/OculusMessage.h>
#include <oculus_driver/Recorder.h>
#include <functional>

OculusRepublisherNode::OculusRepublisherNode()
 : 
 Node("oculus_republisher"),
 sonar_viewer_(static_cast<rclcpp::Node*>(this)) {
    if (!this->has_parameter("data_filename")) {
        this->declare_parameter<std::string>("data_filename", filename_);
    }
    if (!this->has_parameter("publish_rate_ms")) {
        this->declare_parameter<int>("publish_rate_ms", rate_ms_);
    }
    this->get_parameter("data_filename", filename_);
    this->get_parameter("publish_rate_ms", rate_ms_);

    this->status_publisher_ = this->create_publisher<oculus_interfaces::msg::OculusStatus>("/oculus_sonar/status", 1);
    this->ping_publisher_ = this->create_publisher<oculus_interfaces::msg::Ping>("/oculus_sonar/ping", 1);
    this->temperature_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("/oculus_sonar/temperature", 1);
    this->pressure_publisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>("/oculus_sonar/pressure", 1);

    std::cout << "Opening file : " << filename_ << std::endl;

    file_ = std::make_shared<oculus::FileReader>(filename_);

    const double rate_seconds = rate_ms_ / 1000.0;
    timer_ = rclcpp::create_timer(
    this, this->get_clock(), rclcpp::Duration::from_seconds(rate_seconds), std::bind(&OculusRepublisherNode::republishData, this));
}

OculusRepublisherNode::~OculusRepublisherNode() {}

void OculusRepublisherNode::republishData() {
    oculus::Message::ConstPtr msg = file_->read_next_message();
    while (rclcpp::ok() && msg != nullptr) {
        if (msg->is_ping_message()) {
            publishPing(oculus::PingMessage::Create(msg));
            std::this_thread::sleep_for(std::chrono::milliseconds(rate_ms_));
        } else if (msg->is_status_message()) {
            publishStatus(oculus::StatusMessage::Create(msg));
            std::this_thread::sleep_for(std::chrono::milliseconds(rate_ms_));
        }
        msg = file_->read_next_message();
    }
}

void OculusRepublisherNode::publishPing(const oculus::PingMessage::ConstPtr& ping) {
  static oculus_interfaces::msg::Ping msg;
  msg.header.frame_id = frame_id_;
  oculus::toMsg(msg, ping);
  ping_publisher_->publish(msg);

  sensor_msgs::msg::Temperature temperature_ros_msg;
  temperature_ros_msg.header = msg.header;
  temperature_ros_msg.temperature = msg.temperature;  // Measurement of the Temperature in Degrees Celsius
  temperature_ros_msg.variance = 0;  // 0 is interpreted as variance unknown
  temperature_publisher_->publish(temperature_ros_msg);

  sensor_msgs::msg::FluidPressure pressure_ros_msg;
  pressure_ros_msg.header = msg.header;
  pressure_ros_msg.fluid_pressure = msg.pressure;  // Absolute pressure reading in Pascals.
  pressure_ros_msg.variance = 0;  // 0 is interpreted as variance unknown
  pressure_publisher_->publish(pressure_ros_msg);

  sonar_viewer_.publishFan(ping, frame_id_);
}

void OculusRepublisherNode::publishStatus(const std::shared_ptr<const oculus::StatusMessage>& status) {
  auto oculus_status = status->status();
  if (oculus_status.partNumber != oculus::OculusPartNumberType::M1200d) {
    RCLCPP_ERROR_STREAM(get_logger(),
        "The sonar version seems to be different than M1200d."
        " This driver is not suppose to work with your sonar.");
  }

  static oculus_interfaces::msg::OculusStatus msg;
  oculus::toMsg(msg, oculus_status);
  this->status_publisher_->publish(msg);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    try {
        std::shared_ptr<OculusRepublisherNode> republisher = std::make_shared<OculusRepublisherNode>();
        executor.add_node(republisher->get_node_base_interface());
        executor.spin();
    } catch (const std::exception &e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    rclcpp::shutdown();
    return 0;
}