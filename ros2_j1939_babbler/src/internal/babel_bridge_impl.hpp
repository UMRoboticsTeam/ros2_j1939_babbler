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

#include "ros2_j1939_babbler/babel_bridge.hpp"
#include "bridge_core.hpp"
#include "v2c/v2c_transcoder.h"

#include <ros_babel_fish/babel_fish.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cassert>
#include <string>

namespace ros2_j1939_babbler {
    struct PhysicalValue {
        can::sig_codec codec;
        can::tr_signal signal;
        double factor;
        double value_offset;
        unsigned size;
        double min;
        double max;
        bool is_signed;
    };

    struct MessageDefinition {
        std::unordered_map<std::string, PhysicalValue> signals; // Maps signal name to signal info
        std::string name;
        uint8_t dlc;
    };

    struct DbcDatabase {
        std::unordered_map<uint32_t, MessageDefinition> messages; // Maps PGN to message definition
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
        void receive_frame(std::unique_ptr<can_msgs::msg::Frame> message);

        /**
         * @brief Handle an outgoing CAN frame.
         *
         * If the message is present in the DBC file, determines converts it to a CAN message and sends it to the bus.
         *
         * @param message ROS message to handle
         */
        void transmit_frame(std::unique_ptr<ros_babel_fish::CompoundMessage> message);

    private:
        std::string msg_package_; // ROS2 package containing ROS msg definitions for CAN messages described in DBC file
        std::unique_ptr<ros_babel_fish::BabelFish> fish_; // Babelfish instance for loading/populating message definitions
        std::unordered_map<uint32_t, std::shared_ptr<ros_babel_fish::BabelFishPublisher>> publishers_; // Lookup publisher from PGN
        std::unordered_map<uint32_t, std::shared_ptr<ros_babel_fish::BabelFishSubscription>> subscribers_; // Lookup subscriber from ROS message name
        std::unordered_map<std::string, std::uint32_t> ros_name_pgn_mappings_; // Lookup CAN message ID from ROS message type
        std::unordered_map<uint32_t, std::string> pgn_ros_name_mappings_;
        DbcDatabase dbc_parser_;

        /**
         * @brief Sets up ROS publishers and subscribers for every message in the DBC.
         *
         * For every message in the DBC which has an associated ROS type, a publisher (CAN -> ROS) is created with a
         * topic following the pattern `msg_topic_prefix/sensor_name/key_message`, and a subscriber (ROS -> CAN) with
         * the pattern `msg_topic_prefix/sensor_name/key_message/tx`.
         *
         * @param msg_topic_prefix prefix to apply before message topics
         */
        void configure_publishers_subscribers(const std::string& msg_topic_prefix);

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

    void tag_invoke(
        can::def_bo_cpo, DbcDatabase &this_,
        uint32_t msg_id, std::string msg_name, size_t msg_size, size_t transmitter_ord
    );

    void tag_invoke(
        can::def_sg_cpo, DbcDatabase &this_,
        uint32_t message_id, std::optional<unsigned> sg_mux_switch_val, std::string sg_name,
        unsigned sg_start_bit, unsigned sg_size, char sg_byte_order, char sg_sign,
        double sg_factor, double sg_offset, double sg_min, double sg_max,
        std::string /*sg_unit*/, std::vector<size_t> /*rec_ords*/
    );

    void tag_invoke(
        can::def_sig_valtype_cpo, DbcDatabase &this_,
        unsigned message_id, std::string sg_name, unsigned sg_ext_val_type
    );

} // namespace ros2_j1939_babbler

#endif // ROS2_J1939_BABBLER__INTERNAL__BABEL_BRIDGE_IMPL_
