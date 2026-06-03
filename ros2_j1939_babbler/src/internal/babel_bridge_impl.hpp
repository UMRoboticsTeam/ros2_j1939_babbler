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

#include "dbc/dbc_parser.h"

#include "dbc/dbc_parser.h"

namespace ros2_j1939_babbler {
    class DbcParser {
        std::unordered_map<uint32_t, std::vector<std::string>> signals;

        friend void tag_invoke(
        can::def_sg_cpo, DbcParser &this_,
        uint32_t msg_id, std::optional<unsigned> sg_mux_switch_val, std::string sg_name,
        unsigned sg_start_bit, unsigned sg_size, char sg_byte_order, char sg_sign,
        double sg_factor, double sg_offset, double sg_min, double sg_max,
        std::string sg_unit, std::vector<size_t> receiver_ords
        ) {
            this_.signals[msg_id].push_back(sg_name); // add the signal name to the vector for the message id
        }
    };

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
         * @brief Handle an outgoing CAN frame.
         *
         * If the message is present in the DBC file, determines converts it to a CAN message and sends it to the bus.
         *
         * @param MSG ROS message to handle
         */
        void txFrame(ros_babel_fish::CompoundMessage::UniquePtr MSG);

        /**
         * @brief Checks the messages in the DBC and creates a publisher for each one.
         *
         * For every message in the DBC which has an associated ROS type, a publisher is created with a topic following the
         * pattern `msg_topic_prefix/sensor_name/key_message`.
         *
         * @param msg_topic_prefix prefix to apply before message topics
         */
        void configurePublishers(const std::string& msg_topic_prefix);

        /**
         * @brief Checks the messages in the DBC and creates a subscriber for each one.
         *
         * For every message in the DBC which has an associated ROS type, a subscriber is created with a topic following the
         * pattern `msg_topic_prefix/sensor_name/key_message/tx`.
         *
         * @param msg_topic_prefix prefix to apply before message topics
         */
        void configureSubscribers(const std::string& msg_topic_prefix);

    private:
        std::string msg_package_; // ROS2 package containing ROS msg definitions for CAN messages described in DBC file
        ros_babel_fish::BabelFish::UniquePtr fish_; // Babelfish instance for loading/populating message definitions
        std::unordered_map<std::string, ros_babel_fish::BabelFishPublisher::SharedPtr> publishers_; // Lookup publisher from ROS message name TODO: true?
        std::unordered_map<std::string, ros_babel_fish::BabelFishSubscription::SharedPtr> subscribers_; // Lookup subscriber from ROS message name
        std::unordered_map<std::string, std::uint32_t> ros_msg_to_ids_; // Lookup CAN message ID from ROS message type
        std::unordered_map<std::string, std::string> dbc_ros_message_name_mappings_;
        std::unordered_map<std::string, std::string> dbc_ros_signal_name_mappings_;
        DbcParser dbc_parser_;

        /**
            * @brief Strips characters other than [A-Za-z0-9].
            * @return the modified string
            */
        static std::string dbc_message_name_to_ros(const std::string& dbc_message_name) {
         std::string result;
         result.reserve(dbc_message_name.size());
         std::copy_if(
                 dbc_message_name.begin(), dbc_message_name.end(), std::back_inserter(result),
                 [](const unsigned char& c) { return std::isalnum(c); }
         );
         return result;
        }

        /**
         * @brief Brings characters to lowercase and strips other than [a-z0-9_]
         * @return
         */
        static std::string dbc_signal_name_to_ros(const std::string& dbc_signal_name) {
         // Sincce basically the same requirements as dbc_message_name_to_ros, use that and then change all uppercase to lowercase
         std::string result;
         result.reserve(dbc_signal_name.size());
         std::copy_if(
                 dbc_signal_name.begin(), dbc_signal_name.end(), std::back_inserter(result),
                 [](const unsigned char& c) { return std::isalnum(c) || c == '_'; }
         );
         std::transform(result.begin(), result.end(), result.begin(), [](const unsigned char& c) {
             return std::tolower(c);
         });
         return result;
        }
    };
} // namespace ros2_j1939_babbler

#endif // ROS2_J1939_BABBLER__INTERNAL__BABEL_BRIDGE_IMPL_
