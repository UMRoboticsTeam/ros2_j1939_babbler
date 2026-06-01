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

#include "internal/babel_bridge_impl.hpp"

namespace ros2_j1939_babbler {

    BabelBridge::Impl::Impl(rclcpp::Node* node) : BridgeCore(node) {
        msg_package_ = node_->declare_parameter<std::string>("msg_package", "");

        fish_ = ros_babel_fish::BabelFish::make_unique();

        // automatically configure publishers
        this->configurePublishers(msg_topic_prefix_);
        RCLCPP_INFO(node_->get_logger(), "Setup publishers!");
    }


    BabelBridge::Impl::~Impl() = default;

    void BabelBridge::Impl::rxFrame(const can_msgs::msg::Frame::SharedPtr& MSG) {
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

                // then create a local ros2 message
                // not really sure why we create the shared_ptr and reference the object instead of stack-allocating, but this is what all the examples do
                ros_babel_fish::CompoundMessage::SharedPtr can_data_ptr =
                        fish_->create_message_shared(msg_package_ + "/msg/" + dbc_message_name_to_ros(message.GetName()));
                ros_babel_fish::CompoundMessage& can_data = *can_data_ptr;

                // populate the local ros2 message header, frame, and message name
                can_data["header"]["stamp"] = node_->now();
                can_data["header"]["frame_id"] = sensor_name_;
                can_data["src_addr"] = static_cast<uint8_t>(incoming_MSG->id & SOURCE_ADDR_MASK);

                // get the signals (e.g. x, y, z) within the message (e.g. acceleration)
                std::map<std::string, NewEagle::DbcSignal> signals_map = *message.GetSignals();
                for (auto [key_signal, value_signal] : signals_map) {
                    // get the data for the current signal, figure out type, and insert into message accordingly
                    switch (value_signal.GetDataType()) {
                        case NewEagle::INT:
                            RCLCPP_DEBUG(
                                    node_->get_logger(),
                                    "Processing integer field: signed:%s, raw:%f, scale:%f, length:%u, offset:%f, result:%f",
                                    value_signal.GetSign() == NewEagle::SIGNED ? "Y" : "N", value_signal.GetInitialValue(),
                                    value_signal.GetGain(), value_signal.GetDlc(), value_signal.GetOffset(),
                                    value_signal.GetResult()
                            );
                            // This is unbelievably ugly, but was only way I could get the compiler to not promote to int/uint and cause a Babel fish warning
                            switch (ceil_bits(value_signal.GetDlc())) {
                                case IntegerLengths::b8:
                                    if (value_signal.GetSign() == NewEagle::SIGNED) {
                                        can_data[dbc_signal_name_to_ros(key_signal)] =
                                                static_cast<int8_t>(value_signal.GetResult());
                                    } else {
                                        can_data[dbc_signal_name_to_ros(key_signal)] =
                                                static_cast<uint8_t>(value_signal.GetResult());
                                    }
                                    break;
                                case IntegerLengths::b16:
                                    if (value_signal.GetSign() == NewEagle::SIGNED) {
                                        can_data[dbc_signal_name_to_ros(key_signal)] =
                                                static_cast<int16_t>(value_signal.GetResult());
                                    } else {
                                        can_data[dbc_signal_name_to_ros(key_signal)] =
                                                static_cast<uint16_t>(value_signal.GetResult());
                                    }
                                    break;
                                case IntegerLengths::b32:
                                    if (value_signal.GetSign() == NewEagle::SIGNED) {
                                        can_data[dbc_signal_name_to_ros(key_signal)] =
                                                static_cast<int32_t>(value_signal.GetResult());
                                    } else {
                                        can_data[dbc_signal_name_to_ros(key_signal)] =
                                                static_cast<uint32_t>(value_signal.GetResult());
                                    }
                                    break;
                                case IntegerLengths::b64:
                                    if (value_signal.GetSign() == NewEagle::SIGNED) {
                                        can_data[dbc_signal_name_to_ros(key_signal)] =
                                                static_cast<int64_t>(value_signal.GetResult());
                                    } else {
                                        can_data[dbc_signal_name_to_ros(key_signal)] =
                                                static_cast<uint64_t>(value_signal.GetResult());
                                    }
                                    break;
                            }
                            break;
                        case NewEagle::FLOAT:
                            // This is dumb, but can_dbc_parser is handling this awkwardly
                            // Babel fish won't let us put a double in a float32 field, but can_dbc_parser always returns double even if underlying data is supposed to be float32
                            // So we cast the result from can_dbc_parser to float before we insert into the field, and call it a day
                            can_data[dbc_signal_name_to_ros(key_signal)] = static_cast<float>(value_signal.GetResult());
                            RCLCPP_DEBUG(
                                    node_->get_logger(), "Processing float field: raw:%f, scale:%f, offset:%f, result:%f",
                                    value_signal.GetInitialValue(), value_signal.GetGain(), value_signal.GetOffset(),
                                    value_signal.GetResult()
                            );
                            break;
                        case NewEagle::DOUBLE:
                            can_data[dbc_signal_name_to_ros(key_signal)] = value_signal.GetResult();
                            RCLCPP_DEBUG(
                                    node_->get_logger(), "Processing double field: raw:%f, scale:%f, offset:%f, result:%f",
                                    value_signal.GetInitialValue(), value_signal.GetGain(), value_signal.GetOffset(),
                                    value_signal.GetResult()
                            );
                            break;
                    }
                }

                // Note that in intersection with the parameter-specified filters, we also filter by messages in the DBC, so if the
                //    message isn't in there we ignore and let others endpoints handle

                // publish finalized message
                publishers_[dbc_message_name_to_ros(message.GetName())]->publish(can_data);
            }
        }
    }

    // BEGIN MANAGEMENT FUNCTIONS //

    void BabelBridge::Impl::configurePublishers(const std::string& msg_topic_prefix) {
        // iterate over the dbc to spawn an equal amount of publishers
        for (const auto& [key_message, value_message] : dbc_name_msg_map_) {
            std::string msg_name = (std::ostringstream{} << msg_package_ << "/msg/"
                                    << dbc_message_name_to_ros(key_message)).str();
            RCLCPP_DEBUG(node_->get_logger(), "Configuring Publishers - found key_message: %s", key_message.c_str());
            RCLCPP_DEBUG(node_->get_logger(), "Attempting to load '%s'", msg_name.c_str());
            try {
                // There is a missing @throws marker in the documentation for ros_babel_fish:::BabelFish::create_publisher, but it
                //      raises BabbleFishException if the type cannot be found
                std::string topic_name = (std::ostringstream{} << msg_topic_prefix
                                          << (!msg_topic_prefix.empty() && msg_topic_prefix.back() == '/' ? "" : "/")
                                          << sensor_name_ << '/' << key_message).str();
                publishers_[key_message] =
                        this->fish_->create_publisher(*node_, topic_name, msg_name, 20, rclcpp::PublisherOptions{});
            } catch (class_loader::LibraryLoadException& e) {
                RCLCPP_FATAL_STREAM(
                        node_->get_logger(), "Failed to load library containing message type '" << msg_name << "'\n"
                                                                                                << e.what()
                );
                throw;
            } catch (ros_babel_fish::BabelFishException& e) {
                RCLCPP_WARN_STREAM(
                        node_->get_logger(), "Could not find message type for message '" << msg_name << "'\n"
                                                                                         << e.what()
                );
            }
        }
    }

    // END MANAGEMENT FUNCTIONS //

} // namespace ros2_j1939_babbler