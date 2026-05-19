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

namespace {
    enum class IntegerLengths;
    IntegerLengths ceil_bits(const uint8_t bit_length);
    void putSignalInRosMessage(
            ros_babel_fish::CompoundMessage& ros_msg, NewEagle::DbcSignal& can_signal, rclcpp::Logger&& logger,
            const std::string& ros_signal_name
    );
    void putSignalInCanMessage(
            const ros_babel_fish::CompoundMessage& ros_msg, NewEagle::DbcSignal& can_signal, rclcpp::Logger&& logger,
            const std::string& ros_signal_name
    );
} // namespace

namespace ros2_j1939_babbler {

    BabelBridge::Impl::Impl(rclcpp::Node* node) : BridgeCore(node) {
        msg_package_ = node_->declare_parameter<std::string>("msg_package", "");

        fish_ = ros_babel_fish::BabelFish::make_unique();

        // automatically configure publishers
        this->configurePublishers(msg_topic_prefix_);
        RCLCPP_INFO(node_->get_logger(), "Setup publishers!");
        this->configureSubscribers(msg_topic_prefix_);
        RCLCPP_INFO(node_->get_logger(), "Setup subscribers!");
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
                        fish_->create_message_shared(dbc_ros_message_name_mappings_.at(message.GetName()));
                ros_babel_fish::CompoundMessage& can_data = *can_data_ptr;

                // populate the local ros2 message header, frame, and message name
                can_data["header"]["stamp"] = node_->now();
                can_data["header"]["frame_id"] = sensor_name_;
                can_data["src_addr"] = static_cast<uint8_t>(incoming_MSG->id & SOURCE_ADDR_MASK);

                // get the signals (e.g. x, y, z) within the message (e.g. acceleration)
                std::map<std::string, NewEagle::DbcSignal> signals_map = *message.GetSignals();
                for (auto [key_signal, value_signal] : signals_map) {
                    // get the data for the current signal, figure out type, and insert into message accordingly
                    putSignalInRosMessage(
                            can_data, value_signal, node_->get_logger(), dbc_ros_signal_name_mappings_.at(key_signal)
                    );
                }

                // Note that in intersection with the parameter-specified filters, we also filter by messages in the DBC, so if the
                //    message isn't in there we ignore and let others endpoints handle

                // publish finalized message
                publishers_[dbc_ros_message_name_mappings_.at(message.GetName())]->publish(can_data);
            }
        }
    }

    void BabelBridge::Impl::txFrame(ros_babel_fish::CompoundMessage::UniquePtr MSG) {
        auto it = ros_msg_to_ids_.find(MSG->name());

        if (it == ros_msg_to_ids_.end()) {
            RCLCPP_ERROR(node_->get_logger(), "Could not map ROS message '%s' to message ID in DBC", MSG->name().c_str());
            return;
        }

        NewEagle::DbcMessage* message_type = dbw_dbc_db_.GetMessageById(it->second);
        if (message_type == nullptr) {
            RCLCPP_ERROR(
                    node_->get_logger(), "Could not map ROS message '%s' with CAN ID '%d' to a type", MSG->name().c_str(),
                    it->second
            );
            return;
        }

        for (auto& [signal_name, signal] : *message_type->GetSignals()) {
            putSignalInCanMessage(*MSG, signal, node_->get_logger(), dbc_ros_signal_name_mappings_.at(signal_name));
        }

        pub_can_->publish(message_type->GetFrame());
    }

    // BEGIN MANAGEMENT FUNCTIONS //

    void BabelBridge::Impl::configurePublishers(const std::string& msg_topic_prefix) {
        // iterate over the dbc to spawn an equal amount of publishers
        for (auto& [key_message, value_message] : dbc_name_msg_map_) { // GetSignals is not const-qualified...
            std::string msg_name =
                    (std::ostringstream{} << msg_package_ << "/msg/" << dbc_message_name_to_ros(key_message)).str();
            RCLCPP_DEBUG(node_->get_logger(), "Configuring Publishers - found key_message: %s", key_message.c_str());
            RCLCPP_DEBUG(node_->get_logger(), "Attempting to load '%s'", msg_name.c_str());
            try {
                // There is a missing @throws marker in the documentation for ros_babel_fish:::BabelFish::create_publisher, but it
                //      raises BabbleFishException if the type cannot be found
                std::string topic_name =
                        (std::ostringstream{} << msg_topic_prefix
                                              << (!msg_topic_prefix.empty() && msg_topic_prefix.back() == '/' ? "" : "/")
                                              << sensor_name_ << '/' << key_message)
                                .str();
                publishers_[msg_name] =
                        this->fish_->create_publisher(*node_, topic_name, msg_name, 20, rclcpp::PublisherOptions{});

                this->dbc_ros_message_name_mappings_.emplace(key_message, std::move(msg_name));
                for (const auto& [signal_name, signal] : *value_message.GetSignals()) {
                    dbc_ros_signal_name_mappings_.emplace(signal_name, std::move(dbc_signal_name_to_ros(signal_name)));
                }
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

    void BabelBridge::Impl::configureSubscribers(const std::string& msg_topic_prefix) {
        // iterate over the dbc to spawn an equal amount of publishers
        for (auto [key_message, value_message] : dbc_name_msg_map_) {
            std::string msg_name =
                    (std::ostringstream{} << msg_package_ << "/msg/" << dbc_message_name_to_ros(key_message)).str();
            RCLCPP_DEBUG(node_->get_logger(), "Configuring Publishers - found key_message: %s", key_message.c_str());
            RCLCPP_DEBUG(node_->get_logger(), "Attempting to load '%s'", msg_name.c_str());
            try {
                // There is a missing @throws marker in the documentation for ros_babel_fish:::BabelFish::create_publisher, but it
                //      raises BabbleFishException if the type cannot be found
                std::string topic_name =
                        (std::ostringstream{} << msg_topic_prefix
                                              << (!msg_topic_prefix.empty() && msg_topic_prefix.back() == '/' ? "" : "/")
                                              << sensor_name_ << '/' << key_message << "/tx")
                                .str();
                this->subscribers_[dbc_message_name_to_ros(key_message)] = this->fish_->create_subscription(
                        *node_, topic_name, msg_name, 20,
                        [this](ros_babel_fish::CompoundMessage::UniquePtr MSG) { txFrame(std::move(MSG)); }
                );
                this->ros_msg_to_ids_[msg_name] = value_message.GetId();
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

    // TODO:Arturo - Look through this and make sure it's the standard way of renaming CAN devices
    // also, this could just be its own .log file or something idk

    // void GenericCanDriver::Impl::txRename(
    //   const std::array<uint8_t, 8UL> name, const uint8_t new_source_address)
    // {
    //   std::array<uint8_t, 8UL> BAM_data_out = {0x20u, 0x09u, 0x00u, 0x02u, 0xFFu, 0xD8u, 0xFEu, 0x00u};
    //   can_msgs::msg::Frame BAM_frame_out;
    //   uint32_t j1939_id = 0x1CECFF00u;
    //   BAM_frame_out.header.stamp = node_->now();
    //   BAM_frame_out.header.frame_id = "ROS2_command";
    //   BAM_frame_out.id = j1939_id;
    //   BAM_frame_out.is_rtr = false;
    //   BAM_frame_out.is_extended = true;
    //   BAM_frame_out.is_error = false;
    //   BAM_frame_out.dlc = 8;
    //   BAM_frame_out.data = BAM_data_out;

    //   std::array<uint8_t, 8UL> name_data_out_1 = {0x01u, name[0], name[1], name[2],
    //     name[3], name[4], name[5], name[6]};
    //   can_msgs::msg::Frame name_frame_out_1;
    //   j1939_id = 0x1CEBFF00u;
    //   name_frame_out_1.header.stamp = node_->now();
    //   name_frame_out_1.header.frame_id = "ROS2_command";
    //   name_frame_out_1.id = j1939_id;
    //   name_frame_out_1.is_rtr = false;
    //   name_frame_out_1.is_extended = true;
    //   name_frame_out_1.is_error = false;
    //   name_frame_out_1.dlc = 8;
    //   name_frame_out_1.data = name_data_out_1;

    //   std::array<uint8_t, 8UL> name_data_out_2 = {0x02u, name[7], new_source_address, 0xFF, 0xFF, 0xFF,
    //     0xFF, 0xFF};
    //   can_msgs::msg::Frame name_frame_out_2;
    //   j1939_id = 0x1CEBFF00u;
    //   name_frame_out_2.header.stamp = node_->now();
    //   name_frame_out_2.header.frame_id = "ROS2_command";
    //   name_frame_out_2.id = j1939_id;
    //   name_frame_out_2.is_rtr = false;
    //   name_frame_out_2.is_extended = true;
    //   name_frame_out_2.is_error = false;
    //   name_frame_out_2.dlc = 8;
    //   name_frame_out_2.data = name_data_out_2;

    //   pub_can_->publish(BAM_frame_out);
    //   rclcpp::sleep_for(std::chrono::milliseconds(100));
    //   pub_can_->publish(name_frame_out_1);
    //   rclcpp::sleep_for(std::chrono::milliseconds(100));
    //   pub_can_->publish(name_frame_out_2);
    //   RCLCPP_INFO(node_->get_logger(), "Published renaming thing!!!!!!!! %d", new_source_address);
    // }

} // namespace ros2_j1939_babbler

namespace {
    /**
     * @brief Different lengths of integers which are available for use in ROS messages.
     */
    enum class IntegerLengths { b8, b16, b32, b64 };

    void putSignalInRosMessage(
            ros_babel_fish::CompoundMessage& ros_msg, NewEagle::DbcSignal& can_signal, rclcpp::Logger&& logger,
            const std::string& ros_signal_name
    ) {
        // can_signal can't be const because GetInitialValue isn't const qualified
        switch (can_signal.GetDataType()) {
            case NewEagle::INT:
                RCLCPP_DEBUG(
                        logger, "Processing integer field: signed:%s, raw:%f, scale:%f, length:%u, offset:%f, result:%f",
                        can_signal.GetSign() == NewEagle::SIGNED ? "Y" : "N", can_signal.GetInitialValue(),
                        can_signal.GetGain(), can_signal.GetDlc(), can_signal.GetOffset(), can_signal.GetResult()
                );
                // This is unbelievably ugly, but was only way I could get the compiler to not promote to int/uint and cause a Babel fish warning
                switch (ceil_bits(can_signal.GetDlc())) {
                    case IntegerLengths::b8:
                        if (can_signal.GetSign() == NewEagle::SIGNED) {
                            ros_msg[ros_signal_name] = static_cast<int8_t>(can_signal.GetResult());
                        } else {
                            ros_msg[ros_signal_name] = static_cast<uint8_t>(can_signal.GetResult());
                        }
                        break;
                    case IntegerLengths::b16:
                        if (can_signal.GetSign() == NewEagle::SIGNED) {
                            ros_msg[ros_signal_name] = static_cast<int16_t>(can_signal.GetResult());
                        } else {
                            ros_msg[ros_signal_name] = static_cast<uint16_t>(can_signal.GetResult());
                        }
                        break;
                    case IntegerLengths::b32:
                        if (can_signal.GetSign() == NewEagle::SIGNED) {
                            ros_msg[ros_signal_name] = static_cast<int32_t>(can_signal.GetResult());
                        } else {
                            ros_msg[ros_signal_name] = static_cast<uint32_t>(can_signal.GetResult());
                        }
                        break;
                    case IntegerLengths::b64:
                        if (can_signal.GetSign() == NewEagle::SIGNED) {
                            ros_msg[ros_signal_name] = static_cast<int64_t>(can_signal.GetResult());
                        } else {
                            ros_msg[ros_signal_name] = static_cast<uint64_t>(can_signal.GetResult());
                        }
                        break;
                }
                break;
            case NewEagle::FLOAT:
                // This is dumb, but can_dbc_parser is handling this awkwardly
                // Babel fish won't let us put a double in a float32 field, but can_dbc_parser always returns double even if underlying data is supposed to be float32
                // So we cast the result from can_dbc_parser to float before we insert into the field, and call it a day
                ros_msg[ros_signal_name] = static_cast<float>(can_signal.GetResult());
                RCLCPP_DEBUG(
                        logger, "Processing float field: raw:%f, scale:%f, offset:%f, result:%f",
                        can_signal.GetInitialValue(), can_signal.GetGain(), can_signal.GetOffset(), can_signal.GetResult()
                );
                break;
            case NewEagle::DOUBLE:
                ros_msg[ros_signal_name] = can_signal.GetResult();
                RCLCPP_DEBUG(
                        logger, "Processing double field: raw:%f, scale:%f, offset:%f, result:%f",
                        can_signal.GetInitialValue(), can_signal.GetGain(), can_signal.GetOffset(), can_signal.GetResult()
                );
                break;
        }
    }

    void putSignalInCanMessage(
            const ros_babel_fish::CompoundMessage& ros_msg, NewEagle::DbcSignal& can_signal, rclcpp::Logger&& logger,
            const std::string& ros_signal_name
    ) {
        switch (can_signal.GetDataType()) {
            case NewEagle::INT:
                // This is unbelievably ugly, but was only way I could get the compiler to not promote to int/uint and cause a Babel fish warning
                switch (ceil_bits(can_signal.GetDlc())) {
                    case IntegerLengths::b8:
                        if (can_signal.GetSign() == NewEagle::SIGNED) {
                            can_signal.SetResult(
                                    ros_msg[ros_signal_name].as<ros_babel_fish::ValueMessage<int8_t>>().getValue()
                            );
                        } else {
                            can_signal.SetResult(
                                    ros_msg[ros_signal_name].as<ros_babel_fish::ValueMessage<uint8_t>>().getValue()
                            );
                        }
                        break;
                    case IntegerLengths::b16:
                        if (can_signal.GetSign() == NewEagle::SIGNED) {
                            can_signal.SetResult(
                                    ros_msg[ros_signal_name].as<ros_babel_fish::ValueMessage<int16_t>>().getValue()
                            );
                        } else {
                            can_signal.SetResult(
                                    ros_msg[ros_signal_name].as<ros_babel_fish::ValueMessage<uint16_t>>().getValue()
                            );
                        }
                        break;
                    case IntegerLengths::b32:
                        if (can_signal.GetSign() == NewEagle::SIGNED) {
                            can_signal.SetResult(
                                    ros_msg[ros_signal_name].as<ros_babel_fish::ValueMessage<int32_t>>().getValue()
                            );
                        } else {
                            can_signal.SetResult(
                                    ros_msg[ros_signal_name].as<ros_babel_fish::ValueMessage<uint32_t>>().getValue()
                            );
                        }
                        break;
                    case IntegerLengths::b64:
                        if (can_signal.GetSign() == NewEagle::SIGNED) {
                            can_signal.SetResult(
                                    ros_msg[ros_signal_name].as<ros_babel_fish::ValueMessage<int64_t>>().getValue()
                            );
                        } else {
                            can_signal.SetResult(
                                    ros_msg[ros_signal_name].as<ros_babel_fish::ValueMessage<uint64_t>>().getValue()
                            );
                        }
                        break;
                }
                can_signal.SetInitialValue((can_signal.GetResult() - can_signal.GetOffset()) / can_signal.GetGain());
                RCLCPP_DEBUG(
                        logger, "Pushed integer field: signed:%s, scale:%f, length:%u, offset:%f, result:%f",
                        can_signal.GetSign() == NewEagle::SIGNED ? "Y" : "N", can_signal.GetGain(), can_signal.GetDlc(),
                        can_signal.GetOffset(), can_signal.GetResult()
                );
                break;
            case NewEagle::FLOAT:
                can_signal.SetResult(ros_msg[ros_signal_name].as<ros_babel_fish::ValueMessage<float>>().getValue());
                RCLCPP_DEBUG(
                        logger, "Pushed float field: scale:%f, offset:%f, result:%f", can_signal.GetGain(),
                        can_signal.GetOffset(), can_signal.GetResult()
                );
                break;
            case NewEagle::DOUBLE:
                can_signal.SetResult(ros_msg[ros_signal_name].as<ros_babel_fish::ValueMessage<double>>().getValue());
                RCLCPP_DEBUG(
                        logger, "Pushed double field: scale:%f, offset:%f, result:%f", can_signal.GetGain(),
                        can_signal.GetOffset(), can_signal.GetResult()
                );
                break;
        }
    }

    /**
     * @brief Determines the ROS integer type needed to hold an integer of a certain bit length.
     * @param bit_length the number of bits the integer to store is composed of
     */
    IntegerLengths ceil_bits(const uint8_t bit_length) {
        if (bit_length <= 8) { return IntegerLengths::b8; }
        if (bit_length <= 16) { return IntegerLengths::b16; }
        if (bit_length <= 32) { return IntegerLengths::b32; }
        if (bit_length <= 64) { return IntegerLengths::b64; }
        throw std::invalid_argument("Signals with length greater than 64 bits are not supported");
    }
} // namespace