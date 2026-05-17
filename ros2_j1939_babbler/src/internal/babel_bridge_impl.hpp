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

#ifndef ROS2_J1939_BABBLER__INTERNAL__BABEL_BRIDGE_IMPL_
#define ROS2_J1939_BABBLER__INTERNAL__BABEL_BRIDGE_IMPL_

#include "bridge_core.hpp"
#include "ros2_j1939_babbler/babel_bridge.hpp"

#include <rclcpp/rclcpp.hpp>

#include <ros_babel_fish/babel_fish.hpp>


namespace ros2_j1939_babbler {
    /**
     * @brief Implementation of the runtime bridge, hidden from users through PIMPL pattern.
     */
    class BabelBridge::Impl : public BridgeCore<BabelBridge::Impl> {
    public:
        /**
         * @brief Initialise the node implementation.
         * @param node ROS node to interact with the ROS system through
         */
        explicit Impl(rclcpp::Node* node);

        /**
         * @brief Release resources.
         */
        ~Impl();

        /**
         * @brief Handle an incoming CAN frame.
         *
         * If the message is present in the DBC file, determines what the corresponding ROS message would be, and if it
         * exists populates a message and dispatches it to the appropriate publisher.
         *
         * @param MSG CAN message to handle
         */
        void rxFrame(const can_msgs::msg::Frame::SharedPtr& MSG);

        /**
         * @brief Checks the messages in the DBC and creates a publisher for each one.
         *
         * For every message in the DBC which has an associated ROS type, a publisher is created with a topic following the
         * pattern `msg_topic_prefix/sensor_name/key_message`.
         *
         * @param msg_topic_prefix prefix to apply before message topics
         */
        void configurePublishers(const std::string& msg_topic_prefix);

    private:
        std::string msg_package_; // ROS2 package containing ROS msg definitions for CAN messages described in DBC file
        ros_babel_fish::BabelFish::UniquePtr fish_; // Babelfish instance for loading/populating message definitions
        std::map<std::string, ros_babel_fish::BabelFishPublisher::SharedPtr> publishers_; // Lookup publisher from ROS message name
    };
} // namespace ros2_j1939_babbler

#endif // ROS2_J1939_BABBLER__INTERNAL__BABEL_BRIDGE_IMPL_
