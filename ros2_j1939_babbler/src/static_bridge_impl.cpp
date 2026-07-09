/*
 * Copyright 2026 University of Manitoba Robotics Team
 * Noah Reeder
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

#include "internal/static_bridge_impl.hpp"

namespace ros2_j1939_babbler {
    StaticBridge::Impl::Impl(rclcpp::Node* node) : BridgeCore(node) {
        this->configurePublishers(msg_topic_prefix_, can_pub_topic_);
        RCLCPP_INFO(node_->get_logger(), "Setup publishers!");
    }

    StaticBridge::Impl::~Impl() = default;

    void StaticBridge::Impl::on_can_to_ros(std::unique_ptr<can_msgs::msg::Frame> message) {
        RCLCPP_DEBUG(
                node_->get_logger(), "New message; is_rtr:%d is_error:%d id:%d, sa:%d", message->is_rtr, message->is_error,
                message->id, message->id & SOURCE_ADDR_MASK
        );
        // If message is a request frame or error frame, we ignore it
        // Note that in intersection with the parameter-specified filters, we also filter by messages in the DBC, so if the
        //    message isn't in there we ignore and let others endpoints handle
        if (!message->is_rtr && !message->is_error && filter(message->id)) {
            const uint32_t pgn = message->id & PGN_MASK;
            RCLCPP_DEBUG(node_->get_logger(), "Message passed filter");

            publisher_dispatch_table_->runtime_dispatch(
                    pgn, std::move(message), static_cast<uint8_t>(message->id & SOURCE_ADDR_MASK)
            );
        }
    }

    void StaticBridge::Impl::configurePublishers(const std::string& msg_topic_prefix, const std::string& transmitter_topic) {
        this->publisher_dispatch_table_ = std::make_unique<ros2_j1939_babbler_msgs::DispatchTable>(
                node_,
                device_id_,
                (std::ostringstream{} << msg_topic_prefix
                                      << (!msg_topic_prefix.empty() && msg_topic_prefix.back() == '/' ? "" : "/")
                                      << sensor_name_)
                        .str(),
                transmitter_topic, 10
        );
    }
} // namespace ros2_j1939_babbler
