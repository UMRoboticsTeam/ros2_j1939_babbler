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

#include "generic_can_driver/generic_can_driver.hpp"
#include "internal/generic_can_driver.hpp"

namespace ros2_j1939 {
    GenericCanDriver::GenericCanDriver(const rclcpp::NodeOptions & OPTIONS) : rclcpp::Node("generic_can_driver", OPTIONS) {
        impl_ = std::make_shared<GenericCanDriver::Impl>(this);
    }

    GenericCanDriver::~GenericCanDriver()  = default;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_j1939::GenericCanDriver)
