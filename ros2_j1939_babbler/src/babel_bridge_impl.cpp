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

#include "can/can_codec.h"

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
    template<typename T>
    void decodeAndPut(ros_babel_fish::Message &ros_signal, uint64_t raw_value,
                      const ros2_j1939_babbler::PhysicalValue &can_signal, rclcpp::Logger &&logger);
    template<typename T>
    void encodeAndPut(const ros_babel_fish::Message &ros_signal,
                      const ros2_j1939_babbler::PhysicalValue &can_signal, std::array<uint8_t, 8> can_data, rclcpp::Logger &&logger);
    std::string read_file(const std::string& dbc_path);
} // namespace

namespace ros2_j1939_babbler {

    BabelBridge::Impl::Impl(rclcpp::Node* node) : BridgeCore(node) {
        msg_package_ = node_->declare_parameter<std::string>("msg_package", "");

        fish_ = ros_babel_fish::BabelFish::make_unique();
        can::parse_dbc(read_file(dbw_dbc_file_), std::ref(dbc_parser_));

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
                                        dbc_parser_.find_message()                                                 << e.what()
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
            ros_babel_fish::Message& ros_signal, std::array<uint8_t, 8>& can_data, ros2_j1939_babbler::PhysicalValue& can_signal, rclcpp::Logger&& logger
    ) {
        uint64_t raw = 0;
        std::memcpy(&raw, &can_data[0], sizeof(raw));

        // can_signal can't be const because GetInitialValue isn't const qualified
        switch (can_signal.signal.value_type()) {
            case can::i64:
                RCLCPP_DEBUG_STREAM(
                    logger,
                    "Processing signed integer signal '" << can_signal.signal.name() << "': raw=" << raw << ", scale="
                    << can_signal.factor << ", offset=" << can_signal.value_offset << ", length=" << can_signal.dlc
                );
                switch (ceil_bits(can_signal.dlc)) {
                    case IntegerLengths::b8:
                        decodeAndPut<int8_t>(ros_signal, raw, can_signal, logger);
                        break;
                    case IntegerLengths::b16:
                        decodeAndPut<int16_t>(ros_signal, raw, can_signal, logger);
                        break;
                    case IntegerLengths::b32:
                        decodeAndPut<int32_t>(ros_signal, raw, can_signal, logger);
                        break;
                    case IntegerLengths::b64:
                        decodeAndPut<int64_t>(ros_signal, raw, can_signal, logger);
                        break;
                }
                break;
            case can::u64:
                RCLCPP_DEBUG_STREAM(
                    logger,
                    "Processing unsigned integer signal '" << can_signal.signal.name() << "': raw=" << raw << ", scale="
                    << can_signal.factor << ", offset=" << can_signal.value_offset << ", length=" << can_signal.dlc
                );
                // This is unbelievably ugly, but was only way I could get the compiler to not promote to int/uint and cause a Babel fish warning
                switch (ceil_bits(can_signal.dlc)) {
                    case IntegerLengths::b8:
                        decodeAndPut<uint8_t>(ros_signal, raw, can_signal, logger);
                        break;
                    case IntegerLengths::b16:
                        decodeAndPut<uint16_t>(ros_signal, raw, can_signal, logger);
                        break;
                    case IntegerLengths::b32:
                        decodeAndPut<uint32_t>(ros_signal, raw, can_signal, logger);
                        break;
                    case IntegerLengths::b64:
                        decodeAndPut<uint64_t>(ros_signal, raw, can_signal, logger);
                        break;
                }
                break;
            case can::f32:
                RCLCPP_DEBUG_STREAM(
                    logger,
                    "Processing float signal '" << can_signal.signal.name() << "': raw=" << raw << ", scale="
                    << can_signal.factor << ", offset=" << can_signal.value_offset << ", length=" << can_signal.dlc
                );
                decodeAndPut<float>(ros_signal, raw, can_signal, logger);
                break;
            case can::f64:
                RCLCPP_DEBUG_STREAM(
                     logger,
                     "Processing double signal '" << can_signal.signal.name() << "': raw=" << raw << ", scale="
                     << can_signal.factor << ", offset=" << can_signal.value_offset << ", length=" << can_signal.dlc
                 );
                decodeAndPut<double>(ros_signal, raw, can_signal, logger);
                break;
        }
    }

    void putSignalInCanMessage(
            const ros_babel_fish::Message& ros_signal, std::array<uint8_t, 8>& can_data, const ros2_j1939_babbler::PhysicalValue& can_signal, rclcpp::Logger&& logger
    ) {
        switch (can_signal.signal.value_type()) {
            case can::i64:
                switch (ceil_bits(can_signal.dlc)) {
                    case IntegerLengths::b8:
                        encodeAndPut<int8_t>(ros_signal, can_signal, can_data, logger);
                        break;
                    case IntegerLengths::b16:
                        encodeAndPut<int16_t>(ros_signal, can_signal, can_data, logger);
                        break;
                    case IntegerLengths::b32:
                        encodeAndPut<int32_t>(ros_signal, can_signal, can_data, logger);
                        break;
                    case IntegerLengths::b64:
                        encodeAndPut<int64_t>(ros_signal, can_signal, can_data, logger);
                        break;
                }
                break;
            case can::u64:
                switch (ceil_bits(can_signal.dlc)) {
                    case IntegerLengths::b8:
                        encodeAndPut<uint8_t>(ros_signal, can_signal, can_data, logger);
                        break;
                    case IntegerLengths::b16:
                        encodeAndPut<uint16_t>(ros_signal, can_signal, can_data, logger);
                        break;
                    case IntegerLengths::b32:
                        encodeAndPut<uint32_t>(ros_signal, can_signal, can_data, logger);
                        break;
                    case IntegerLengths::b64:
                        encodeAndPut<uint64_t>(ros_signal, can_signal, can_data, logger);
                        break;
                }
                RCLCPP_DEBUG_STREAM(
                    logger,
                    "Pushed unsigned integer signal '" << can_signal.signal.name() << "': scale="
                    << can_signal.factor << ", offset=" << can_signal.value_offset << ", length=" << can_signal.dlc
                );
                break;
            case can::f32:
                encodeAndPut<float>(ros_signal, can_signal, can_data, logger);
                RCLCPP_DEBUG_STREAM(
                    logger,
                    "Pushed float signal '" << can_signal.signal.name() << "': scale="
                    << can_signal.factor << ", offset=" << can_signal.value_offset << ", length=" << can_signal.dlc
                );
                break;
            case can::f64:
                encodeAndPut<double>(ros_signal, can_signal, can_data, logger);
                RCLCPP_DEBUG_STREAM(
                    logger,
                    "Pushed double signal '" << can_signal.signal.name() << "': scale="
                    << can_signal.factor << ", offset=" << can_signal.value_offset << ", length=" << can_signal.dlc
                );
                break;
        }
    }

    template<typename T>
    void decodeAndPut(ros_babel_fish::Message &ros_signal, uint64_t raw_value,
                      const ros2_j1939_babbler::PhysicalValue &can_signal, rclcpp::Logger &&logger) {
        T data = static_cast<T>(static_cast<double>(raw_value) * can_signal.factor + can_signal.value_offset);
        if (data < can_signal.min || data > can_signal.max) {
            RCLCPP_DEBUG_STREAM(
                logger,
                "Signal '" << can_signal.signal.name() << "': value=" << data << " is out of range; min=" << can_signal.
                min << ", max=" << can_signal.max
            );
        }
        ros_signal = data;
    }

    template<typename T>
    void encodeAndPut(const ros_babel_fish::Message &ros_signal,
                      const ros2_j1939_babbler::PhysicalValue &can_signal, std::array<uint8_t, 8> can_data, rclcpp::Logger &&logger) {
        T value = ros_signal.as<ros_babel_fish::ValueMessage<T>>().getValue();
        if (value < can_signal.min || value > can_signal.max) {
            RCLCPP_DEBUG_STREAM(
                logger,
                "Signal '" << can_signal.signal.name() << "': value=" << value << " is out of range; min=" << can_signal.
                min << ", max=" << can_signal.max
            );
        }
        T raw_value = (ros_signal.as<ros_babel_fish::ValueMessage<T>>().getValue() - can_signal.value_offset) / can_signal.factor;
        can_signal.codec(*reinterpret_cast<uint64_t*>(&raw_value), can_data.data());
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

    std::string read_file(const std::string& dbc_path) {
        std::ifstream dbc_content(dbc_path);
        std::ostringstream ss;
        ss << dbc_content.rdbuf();
        return ss.str();
    }
} // namespace