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

#ifndef ROS2_J1939_BABBLER__INTERNAL__STATIC_BRIDGE_IMPL_
#define ROS2_J1939_BABBLER__INTERNAL__STATIC_BRIDGE_IMPL_

#include "bridge_core.hpp"
#include "ros2_j1939_babbler/static_bridge.hpp"

#include <rclcpp/rclcpp.hpp>

#include <ros2_j1939_babbler_msgs/type_conversion.hpp>

namespace ros2_j1939_babbler {
    /**
     * @brief Implementation of the compile-time bridge, hidden from users through PIMPL pattern.
     */
    class StaticBridge::Impl : public BridgeCore<StaticBridge::Impl> {
    public:
        /**
         * @brief Initialise the node implementation.
         * @param node ROS node to interact with the ROS system through
         */
        explicit Impl(rclcpp::Node *node);

        /**
         * @brief Release resources.
         */
        ~Impl();

        /**
         * @brief Handle an incoming CAN frame.
         *
         * If the message is known (i.e. was present in the DBC at compile time), populates a ROS message using the CAN data
         * and dispatches it to the appropriate publisher.
         *
         * @param MSG CAN message to handle
         */
        void on_can_to_ros(std::unique_ptr<can_msgs::msg::Frame> MSG);

    private:
        // Structure holding publishers and handling dispatching messages to them
        std::shared_ptr<ros2_j1939_babbler_msgs::DispatchTable> publisher_dispatch_table_;

        /**
         * @brief Creates a publisher for each message type.
         *
         * Topics follow the pattern `msg_topic_prefix/sensor_name/key_message`.
         *
         * @param msg_topic_prefix prefix to apply before message topics
         * @param transmitter_topic topic to send outgoing CAN messages to
         */
        void configurePublishers(const std::string &msg_topic_prefix, const std::string &transmitter_topic);
    };
} // namespace ros2_j1939_babbler

#endif // ROS2_J1939_BABBLER__INTERNAL__STATIC_BRIDGE_IMPL_
