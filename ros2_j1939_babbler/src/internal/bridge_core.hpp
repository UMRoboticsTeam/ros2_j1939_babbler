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
#include <utility>
#include <sstream>

constexpr inline uint32_t PFPS_MASK = 0x00FFFF00u; // Note doesn't include data page, NewEagle limitation?
constexpr inline uint32_t SOURCE_ADDR_MASK = 0x000000FFu;
constexpr inline uint32_t MAX_CAN_ID = 0x1FFFFFFFu; // 29-bits

using namespace std::chrono_literals;

namespace ros2_j1939_babbler
{
    template <typename T>
    class BridgeCore
    {
    public:
        explicit BridgeCore(rclcpp::Node* node) : node_{node}
        {
            RCLCPP_INFO(node_->get_logger(), "Starting Generic Can Driver...");

            dbw_dbc_file_ = node_->declare_parameter<std::string>("dbw_dbc_file", "");
            frame_id_ = node_->declare_parameter<std::string>("frame_id", "");
            sensor_name_ = node_->declare_parameter<std::string>("sensor_name", "");
            device_ID_ = node_->declare_parameter<uint8_t>("device_ID", 0);
            sub_topic_can_ = node_->declare_parameter<std::string>("can_sub_topic", "");
            //pub_topic_can_ = node_->declare_parameter<std::string>("pub_topic_can", ""); TODO: Implement ROS-to-J1939
            msg_topic_prefix_ = node_->declare_parameter<std::string>("msg_topic_prefix", "");
            auto msg_filter_range = rcl_interfaces::msg::ParameterDescriptor{};
            msg_filter_range.integer_range = {
                rcl_interfaces::msg::IntegerRange().set__from_value(0).set__to_value(MAX_CAN_ID)
            };
            msg_filter_ids_ = node_->declare_parameter<std::vector<int64_t>>("msg_filter_ids", {0}, msg_filter_range);
            // TODO: Explain somewhere that default is all-pass
            msg_filter_masks_ = node_->declare_parameter<std::vector<int64_t>>(
                "msg_filter_masks", {0}, msg_filter_range);

            if (msg_filter_ids_.size() != msg_filter_masks_.size())
            {
                throw std::invalid_argument((
                    std::ostringstream{} << "Message filter must have same number of IDs and masks, found " <<
                    msg_filter_ids_.size()
                    << " ids and " << msg_filter_masks_.size() << " masks").str());
            }

            device_ID_str_ = std::to_string(device_ID_);

            // printing to user
            RCLCPP_INFO(node_->get_logger(), "dbw_dbc_file: %s", dbw_dbc_file_.c_str());
            RCLCPP_INFO(node_->get_logger(), "frame_id: %s", frame_id_.c_str());
            RCLCPP_INFO(node_->get_logger(), "sensor_name: %s", sensor_name_.c_str());
            RCLCPP_INFO(node_->get_logger(), "device_id: %d", device_ID_);
            RCLCPP_INFO(node_->get_logger(), "sub_topic_can: %s", sub_topic_can_.c_str());
            //RCLCPP_INFO(node_->get_logger(), "pub_topic_can: %s", pub_topic_can_.c_str()); TODO: Implement ROS-to-J1939

            // setup dbc database - brings in j1939 standard
            this->setupDatabase();
            RCLCPP_INFO(node_->get_logger(), "Setup DBC database!");

            // setup subscriber, bind rxFrame
            this->sub_can_ = node_->create_subscription<can_msgs::msg::Frame>(
                this->sub_topic_can_, 500, [this](const can_msgs::msg::Frame::SharedPtr msg)
                {
                    rxFrame(std::forward<decltype(msg)>(msg));
                });

            RCLCPP_DEBUG(node_->get_logger(), "Generic Can Driver Core configured!");
        }

        ~BridgeCore() = default;

    protected:
        /**
         * Different lengths of integers which are available for use in ROS messages.
         */
        enum class IntegerLengths
        {
            b8,
            b16,
            b32,
            b64
        };

        /**
         * Strips characters other than [A-Za-z0-9].
         * @return the modified string
         */
        static std::string dbc_message_name_to_ros(const std::string& dbc_message_name)
        {
            std::string result;
            result.reserve(dbc_message_name.size());
            std::copy_if(dbc_message_name.begin(), dbc_message_name.end(), std::back_inserter(result),
                         [](const unsigned char& c) { return std::isalnum(c); });
            return result;
        }

        /**
         * Brings characters to lowercase and strips other than [a-z0-9_]
         * @return
         */
        static std::string dbc_signal_name_to_ros(const std::string& dbc_signal_name)
        {
            // Sincce basically the same requirements as dbc_message_name_to_ros, use that and then change all uppercase to lowercase
            std::string result;
            result.reserve(dbc_signal_name.size());
            std::copy_if(dbc_signal_name.begin(), dbc_signal_name.end(), std::back_inserter(result),
                         [](const unsigned char& c) { return std::isalnum(c) || c == '_'; });
            std::transform(result.begin(), result.end(), result.begin(),
                           [](const unsigned char& c) { return std::tolower(c); });
            return result;
        }

        static IntegerLengths ceil_bits(const uint8_t bit_length)
        {
            if (bit_length <= 8) { return IntegerLengths::b8; }
            if (bit_length <= 16) { return IntegerLengths::b16; }
            if (bit_length <= 32) { return IntegerLengths::b32; }
            if (bit_length <= 64) { return IntegerLengths::b64; }
            throw std::invalid_argument("Signals with length greater than 64 bits are not supported");
        }

        /**
         * @brief Parses incoming CAN frames.
         *
         * 1. Checks if incoming frame is valid and has a matching device ID (as set in params)
         *
         * 2. Passes can frame to a local constant
         *
         * 3. Checks if the message exists in the dbc
         *
         * 4. Stuffs a CanData value-key message
         *
         * 5. Publishes that message on the message topic
         */
        void rxFrame(const can_msgs::msg::Frame::SharedPtr& MSG)
        {
            static_cast<T*>(this)->rxFrame(MSG);
        }

        // DATABASE MANAGEMENT FUNCTIONS //
        /**
         * @brief Instantiates a NewEagle dbc database object using the dbc file specified in params.
         * Strips the priority and source address id from the message ID in the DBC.
         * These stripped IDs are stored as keys in a map with the NewEagle DBC messages as values.
         */
        void setupDatabase()
        {
            // build the new eagle dbc database
            this->dbw_dbc_db_ = NewEagle::DbcBuilder().NewDbc(dbw_dbc_file_);
            this->dbc_name_msg_map_ = *this->dbw_dbc_db_.GetMessages();
            // for every message in the database, leave only PGN
            for (auto [key, value] : dbc_name_msg_map_)
            {
                // strip id of priority and source address info
                uint32_t stripped_id = value.GetId() & 0x00FFFF00u;
                dbc_id_msg_map_[stripped_id] = value;
                RCLCPP_DEBUG(node_->get_logger(), "Accepting messages with stripped ID:%d from raw ID:%d", stripped_id,
                             value.GetId());
            }
        }

        /**
         * @brief functions that takes list of full addresses (such as 0x0CEEFFA1) as defined in params
         * yaml and attempts and address claim attack. It adopts the lowest value name and publishes it on
         * that address, forcing the target device on that address to either stop publishing or move to a
         * different address, depending on its internal logic.
        */
        void generateAddressClaimAttackMsg(
            can_msgs::msg::Frame::SharedPtr MSG, const std::vector<uint32_t> source_addresses
        )
        {
            // we go through each address given in the list
            for (uint32_t address : source_addresses)
            {
                // we add the source address (target of the claim attack) to a 'name declaration' message
                address += 0x18EEFF00;
                // by sending this message with only 0s, our name takes priority,
                // and the competing device stops publishing
                std::array<uint8_t, 8UL> claim_data = {
                    0x00u, 0x00u, 0x00u, 0x00u, 0x00, 0x00, 0x00, 0x00u
                };

                // then we just stuff the can frame with all our data
                MSG->header.stamp = node_->now();
                MSG->header.frame_id = "can";
                MSG->id = address;
                MSG->is_rtr = false;
                MSG->is_extended = true;
                MSG->is_error = false;
                MSG->dlc = 8;
                MSG->data = claim_data;
            }
        }

        /**
         * @brief formats data nicely for use in CAN frames
         */
        void createDataArray(
            const std::vector<uint16_t> data_in,
            const std::vector<uint16_t> data_lengths,
            std::array<uint8_t, 8UL>& data_out
        )
        {
            uint64_t data_concatenated = 0;
            uint64_t data_mask = 0x00000000000000FF;
            int size = data_in.size();
            for (int i = 0; i < size; i++)
            {
                data_concatenated = data_concatenated << data_lengths[size - 1 - i];
                data_concatenated += data_in[size - 1 - i];
            }
            for (int i = 0; i < 8; i++)
            {
                data_out[i] = (data_mask & data_concatenated >> 8 * i);
            }
        }

        /**
        * @brief check if an incoming message passes the message filters
        */
        [[nodiscard]] bool filter(const uint32_t id) const
        {
            bool pass = false;
            for (std::size_t i = 0; i < msg_filter_ids_.size() && !pass; ++i)
            {
                pass = (id & msg_filter_masks_[i]) == (msg_filter_ids_[i] & msg_filter_masks_[i]);
            }
            return pass;
        }

        rclcpp::Node* node_; // Node to use for interactions with ROS
        // Safe because public class always outlives implementation. Don't love, but can't use shared_from_this because we need to
        //   instantiate Impl from the public class' constructor and trying to delay initialisation would be a nightmare

        // params
        std::string dbw_dbc_file_; // the messages definition (such as J1939 standard)
        std::string frame_id_; // used on the published messages - usually just the location of the sensor on your robot
        std::string sensor_name_; // name of your sensor / device (e.g. engine ECU)
        uint8_t device_ID_; // j1939 source address of your device, in decimal format (last 2 hex numbers of the CANID)
        std::string device_ID_str_;
        // [device_ID_str_] just turning the device_id into a string. used to populate message headers
        std::string sub_topic_can_; // subscribe to the topic socket_can is publishing from the CAN line
        //std::string pub_topic_can_; // publish to the topic socket_can is sending to the CAN line TODO: Implement ROS-to-J1939
        std::string msg_topic_prefix_; // prefix to add to the topic name for each CAN message
        std::vector<int64_t> msg_filter_ids_; // list of ids messages must match (within a mask) to be processed
        std::vector<int64_t> msg_filter_masks_; // parallel list of masks to apply to the filter ids

        NewEagle::Dbc dbw_dbc_db_; // new eagle dbc database
        std::map<uint32_t, NewEagle::DbcMessage> dbc_id_msg_map_;
        std::map<std::string, NewEagle::DbcMessage> dbc_name_msg_map_;
        rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
    };
} // namespace ros2_j1939_babbler

#endif  // ROS2_J1939_BABBLER__INTERNAL__BRIDGE_CORE_
