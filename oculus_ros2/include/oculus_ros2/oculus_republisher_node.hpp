//
// @file - oculus_republisher_node.hpp
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
 #pragma once

#include <oculus_driver/AsyncService.h>

#include <oculus_interfaces/msg/oculus_status.hpp>
#include <oculus_interfaces/msg/ping.hpp>
#include <oculus_ros2/conversions.hpp>
#include <oculus_ros2/sonar_viewer.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include "oculus_driver/OculusMessage.h"

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

// #include "oculus_ros2/mock_sonar.hpp"

class OculusRepublisherNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Oculus Republisher object
     * 
     */
    OculusRepublisherNode();

    /**
    * @brief Destroy the Oculus Republisher object
    * 
    */
    ~OculusRepublisherNode();

    /**
     * @brief Republishing data from a .oculus file.
     * 
     */
    void republishData();

    /**
     * @brief Publish ping message
     * 
     * @param ping 
     */
    void publishPing(const oculus::PingMessage::ConstPtr& ping);

    /**
     * @brief Publish sonar status
     * 
     * @param status Sonar status
     */
    void publishStatus(const std::shared_ptr<const oculus::StatusMessage>& status);

private:
    // sonar viewer
    SonarViewer sonar_viewer_;
    // sonar frame id
    const std::string frame_id_ = "sonar";
    // .oculus filename
    std::string filename_ = "";
    // .oculus file reader
    std::shared_ptr<oculus::FileReader> file_;
    // republish rate (40hz)
    int rate_ms_ = 25;
    // republish data timer
    rclcpp::TimerBase::SharedPtr timer_;
    // sonar status publisher
    rclcpp::Publisher<oculus_interfaces::msg::OculusStatus>::SharedPtr status_publisher_{nullptr};
    // sonar ping publisher
    rclcpp::Publisher<oculus_interfaces::msg::Ping>::SharedPtr ping_publisher_{nullptr};
    // sonar temperature publisher
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_{nullptr};
    // sonar pressure publisher
    rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_publisher_{nullptr};
};