/*
 * Copyright 2024 Construction Engineering Research Laboratory (CERL)
 * Engineer Reseach and Development Center (ERDC)
 * U.S. Army Corps of Engineers
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * Modified by Noah Reeder on 2026-05-07
 * Modifications Copyright 2026 Noah Reeder, University of Manitoba Robotics Team.
 */

#ifndef ROS2_J1939_BABBLER__INTERNAL__BRIDGE_CORE_
#define ROS2_J1939_BABBLER__INTERNAL__BRIDGE_CORE_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "can_msgs/msg/frame.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "can_dbc_parser/Dbc.hpp"
#include "can_dbc_parser/DbcBuilder.hpp"
#include "can_dbc_parser/DbcMessage.hpp"

#include <regex>
#include <sstream>
#include <utility>

constexpr inline uint32_t PGN_MASK = 0x03FFFF00u;
constexpr inline uint32_t PF_MASK = 0x00FF0000u;
constexpr inline uint32_t PS_MASK = 0x0000FF00u;
constexpr inline uint32_t SOURCE_ADDR_MASK = 0x000000FFu;
constexpr inline uint32_t MAX_CAN_ID = 0x1FFFFFFFu; // 29-bits
constexpr inline uint32_t PDU2_PF_BOUNDARY = 0xF0;  // Messages with PF >= this are PDU2

namespace ros2_j1939_babbler {
    /**
     * @brief Shared code between bridge implementations.
     *
     * Uses CRTP pattern to perform compile-time polymorphism.
     *
     * @tparam T type of bridge
     */
    template<typename T>
    class BridgeCore {
    public:
        /**
         * Initialise structures for the bridge.
         */
        explicit BridgeCore(rclcpp::Node* node) : node_{ node } {
            RCLCPP_INFO(node_->get_logger(), "Starting Generic Can Driver...");

            dbw_dbc_file_ = node_->declare_parameter<std::string>("dbw_dbc_file", "");
            frame_id_ = node_->declare_parameter<std::string>("frame_id", "");
            sensor_name_ = node_->declare_parameter<std::string>("sensor_name", "");
            device_ID_ = node_->declare_parameter<uint8_t>("device_ID", 0);
            can_sub_topic_ = node_->declare_parameter<std::string>("can_sub_topic", "/from_can_bus");
            can_pub_topic_ = node_->declare_parameter<std::string>("can_pub_topic", "/to_can_bus");
            msg_topic_prefix_ = node_->declare_parameter<std::string>("msg_topic_prefix", "");
            auto msg_filter_range = rcl_interfaces::msg::ParameterDescriptor{};
            msg_filter_range.integer_range = {
                rcl_interfaces::msg::IntegerRange().set__from_value(0).set__to_value(MAX_CAN_ID)
            };
            msg_filter_ids_ = node_->declare_parameter<std::vector<int64_t>>("msg_filter_ids", { 0 }, msg_filter_range);
            msg_filter_masks_ = node_->declare_parameter<std::vector<int64_t>>("msg_filter_masks", { 0 }, msg_filter_range);

            if (msg_filter_ids_.size() != msg_filter_masks_.size()) {
                throw std::invalid_argument((std::ostringstream{}
                                             << "Message filter must have same number of IDs and masks, found "
                                             << msg_filter_ids_.size() << " ids and " << msg_filter_masks_.size()
                                             << " masks")
                                                    .str());
            }

            device_ID_str_ = std::to_string(device_ID_);

            RCLCPP_INFO(node_->get_logger(), "dbw_dbc_file: %s", dbw_dbc_file_.c_str());
            RCLCPP_INFO(node_->get_logger(), "frame_id: %s", frame_id_.c_str());
            RCLCPP_INFO(node_->get_logger(), "sensor_name: %s", sensor_name_.c_str());
            RCLCPP_INFO(node_->get_logger(), "device_id: %d", device_ID_);
            RCLCPP_INFO(node_->get_logger(), "sub_topic_can: %s", can_sub_topic_.c_str());
            RCLCPP_INFO(node_->get_logger(), "pub_topic_can: %s", can_pub_topic_.c_str());

            this->sub_can_ = node_->create_subscription<can_msgs::msg::Frame>(
                    this->can_sub_topic_, 500,
                    [this](std::unique_ptr<can_msgs::msg::Frame> message) { on_can_to_ros(std::move(message)); }
            );
            this->pub_can_ =
                    node_->create_publisher<can_msgs::msg::Frame>(this->can_pub_topic_, 500, rclcpp::PublisherOptions{});

            RCLCPP_DEBUG(node_->get_logger(), "Generic Can Driver Core configured!");
        }

        /**
         * @brief Release resources.
         */
        ~BridgeCore() = default;

    protected:
        /**
         * @brief Handle an incoming CAN frame.
         *
         * Uses the derived implementation's message handler to handle the message.
         *
         * @param MSG CAN message to handle
         */
        void on_can_to_ros(std::unique_ptr<can_msgs::msg::Frame> message) {
            static_cast<T*>(this)->on_can_to_ros(std::move(message));
        }

        /**
        * @brief check if an incoming message passes the message filters
        */
        [[nodiscard]] bool filter(const uint32_t id) const {
            bool pass = false;

            uint8_t pf = id & PF_MASK;
            bool is_pdu2 = pf >= PDU2_PF_BOUNDARY;
            bool addressed_to_us = is_pdu2 || (id & PF_MASK) == device_ID_;
            if (!addressed_to_us) { return false; }

            for (std::size_t i = 0; i < msg_filter_ids_.size() && !pass; ++i) {
                pass = (id & msg_filter_masks_[i]) == (msg_filter_ids_[i] & msg_filter_masks_[i]);
            }

            return pass;
        }

        rclcpp::Node* node_; // Node to use for interactions with ROS
        // Safe because public class always outlives implementation. Don't love, but can't use shared_from_this because we need to
        //   instantiate Impl from the public class' constructor and trying to delay initialisation would be a nightmare

        std::string dbw_dbc_file_; // The messages definition (such as J1939 standard)
        std::string frame_id_;     // Used on the published messages - usually just the location of the sensor on your robot
        std::string sensor_name_;  // Name of your sensor / device (e.g. engine ECU)
        uint8_t device_ID_; // J1939 source address of your device, in decimal format (last 2 hex numbers of the CANID)
        std::string device_ID_str_;             // String representation of device_ID_
        std::string can_sub_topic_;             // Topic to listen for ros2_socketcan messages on
        std::string can_pub_topic_;             // Topic to send outgoing ros2_socketcan messages on
        std::string msg_topic_prefix_;          // Prefix to add to the topic name for each CAN message
        std::vector<int64_t> msg_filter_ids_;   // List of ids messages must match (within a mask) to be processed
        std::vector<int64_t> msg_filter_masks_; // Parallel list of masks to apply to the filter ids

        std::shared_ptr<rclcpp::Subscription<can_msgs::msg::Frame>> sub_can_; // ROS subscriber to sub_topic_can_
        std::shared_ptr<rclcpp::Publisher<can_msgs::msg::Frame>> pub_can_;    // ROS publisher to pub_topic_can_
    };
} // namespace ros2_j1939_babbler

#endif // ROS2_J1939_BABBLER__INTERNAL__BRIDGE_CORE_
