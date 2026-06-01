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

#include "internal/static_bridge_impl.hpp"

namespace {
    constexpr uint32_t PGN_MASK = 0x03FFFF00;
}

namespace ros2_j1939_babbler {

    StaticBridge::Impl::Impl(rclcpp::Node* node) : BridgeCore(node) {
        // automatically configure publishers
        this->configurePublishers(msg_topic_prefix_);
        RCLCPP_INFO(node_->get_logger(), "Setup publishers!");
    }


    StaticBridge::Impl::~Impl() = default;

    void StaticBridge::Impl::rxFrame(const can_msgs::msg::Frame::SharedPtr& MSG) {
        RCLCPP_DEBUG(
                node_->get_logger(), "New message; is_rtr:%d is_error:%d id:%d, sa:%d", MSG->is_rtr, MSG->is_error, MSG->id,
                MSG->id & 0x000000FFu
        );
        // if message is not a request, error, and matches device ID
        if (!MSG->is_rtr && !MSG->is_error && (device_ID_ == (MSG->id & 0x000000FFu) && filter(MSG->id))) {
            // local const to store incoming message
            const can_msgs::msg::Frame::SharedPtr incoming_MSG = MSG;
            RCLCPP_DEBUG(
                    node_->get_logger(), "Filtering message; sa:%d, count:%zu", MSG->id & 0x00FFFF00u,
                    dbc_id_msg_map_.count(MSG->id & 0x00FFFF00u)
            );
            // if the message type / PGN is found in the dbc
            if (dbc_id_msg_map_.count(MSG->id & 0x00FFFF00u)) {

                // translate the message data
                NewEagle::DbcMessage message = dbc_id_msg_map_[incoming_MSG->id & PFPS_MASK];
                message.SetFrame(incoming_MSG);

                // Transform the fields map
                std::unordered_map<std::string, double> fields;
                for (const auto& [key, value] : *message.GetSignals()) { fields[key] = value.GetResult(); }

                // populate the local ros2 message header, frame, and message name
                std_msgs::msg::Header header;
                header.stamp = node_->now();
                header.frame_id = sensor_name_;

                // publish finalized message
                publisher_dispatch_table_->runtime_dispatch(
                        incoming_MSG->id & PGN_MASK, fields, std::move(header),
                        static_cast<uint8_t>(incoming_MSG->id & SOURCE_ADDR_MASK)
                );
            }
        }
    }

    // BEGIN MANAGEMENT FUNCTIONS //

    void StaticBridge::Impl::configurePublishers(const std::string& msg_topic_prefix) {
        // Create the runtime dispatch table
        this->publisher_dispatch_table_ = std::make_unique<ros2_j1939_babbler_msgs::DispatchTable>(
                node_,
                (std::ostringstream{} << msg_topic_prefix
                                      << (!msg_topic_prefix.empty() && msg_topic_prefix.back() == '/' ? "" : "/")
                                      << sensor_name_).str(),
                10
        );
    }

    // END MANAGEMENT FUNCTIONS //

} // namespace ros2_j1939_babbler