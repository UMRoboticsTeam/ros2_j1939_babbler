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

#include "ros2_j1939_babbler/babel_bridge.hpp"
#include "internal/babel_bridge_impl.hpp"

namespace ros2_j1939_babbler {
    BabelBridge::BabelBridge(const rclcpp::NodeOptions& OPTIONS) : rclcpp::Node("babel_bridge", OPTIONS) {
        impl_ = std::make_shared<BabelBridge::Impl>(this);
    }

    BabelBridge::~BabelBridge() = default;
} // namespace ros2_j1939_babbler

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_j1939_babbler::BabelBridge)