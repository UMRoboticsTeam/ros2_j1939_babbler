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

#ifndef ROS2_J1939_BABBLER__BABEL_BRIDGE_
#define ROS2_J1939_BABBLER__BABEL_BRIDGE_

#include <rclcpp/rclcpp.hpp>

namespace ros2_j1939_babbler
{
    class BabelBridge : public rclcpp::Node
    {
    public:
        explicit BabelBridge(const rclcpp::NodeOptions& OPTIONS);

        ~BabelBridge() override;

    private:
        // Forward-declare implementation
        class Impl;

        std::shared_ptr<Impl> impl_;
    };
} // namespace ros2_j1939_babbler

#endif  // ROS2_J1939_BABBLER__BABEL_BRIDGE_
