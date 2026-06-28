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
        ros_babel_fish::Message &ros_signal, std::array<uint8_t, 8> &can_data,
        const ros2_j1939_babbler::PhysicalValue &can_signal, const rclcpp::Logger &logger
    );

    void putSignalInCanMessage(
        const ros_babel_fish::Message &ros_signal, std::array<uint8_t, 8> &can_data,
        const ros2_j1939_babbler::PhysicalValue &can_signal, const rclcpp::Logger &logger
    );

    template<typename T>
    void decodeAndPut(ros_babel_fish::Message &ros_signal, uint64_t raw_value,
                      const ros2_j1939_babbler::PhysicalValue &can_signal, const rclcpp::Logger &logger);

    template<typename T>
    void encodeAndPut(const ros_babel_fish::Message &ros_signal,
                      const ros2_j1939_babbler::PhysicalValue &can_signal, std::array<uint8_t, 8> can_data,
                      const rclcpp::Logger &logger);

    std::string read_file(const std::string &dbc_path);
} // namespace

namespace ros2_j1939_babbler {
    BabelBridge::Impl::Impl(rclcpp::Node *node) : BridgeCore(node) {
        msg_package_ = node_->declare_parameter<std::string>("msg_package", "");

        fish_ = ros_babel_fish::BabelFish::make_unique();
        bool successfully_parsed = can::parse_dbc(read_file(dbw_dbc_file_), std::ref(dbc_parser_));
        if (!successfully_parsed) {
            RCLCPP_ERROR_STREAM(node_->get_logger(), "DBC parsing failed for file '" << dbw_dbc_file_ << "'");
        }


        // automatically configure publishers
        this->configure_publishers_subscribers(msg_topic_prefix_);
        RCLCPP_INFO(node_->get_logger(), "Setup ROS topics!");
    }


    BabelBridge::Impl::~Impl() = default;

    void BabelBridge::Impl::receive_frame(std::unique_ptr<can_msgs::msg::Frame> message) {
        RCLCPP_DEBUG(
            node_->get_logger(), "New message; is_rtr:%d is_error:%d id:%d, sa:%d", message->is_rtr, message->is_error,
            message->id,
            message->id & 0x000000FFu
        );
        // If message is a request frame or error frame, we ignore it
        // Note that in intersection with the parameter-specified filters, we also filter by messages in the DBC, so if the
        //    message isn't in there we ignore and let others endpoints handle
        if (!message->is_rtr && !message->is_error && filter(message->id)) {
            const uint32_t pgn = message->id & PGN_MASK;
            const auto it = dbc_parser_.messages.find(pgn);
            const bool found = it != dbc_parser_.messages.end();
            RCLCPP_DEBUG(
                node_->get_logger(), "Message passed filter; found in map: %s", found ? "true" : "false"
            );

            if (found) {
                // Not really sure why we create the shared_ptr and reference the object instead of stack-allocating, but this is what all the examples do
                std::shared_ptr<ros_babel_fish::CompoundMessage> can_data_ptr =
                        fish_->create_message_shared(pgn_ros_name_mappings_.at(pgn));
                ros_babel_fish::CompoundMessage &can_data = *can_data_ptr; // Shorthand

                can_data["header"]["stamp"] = node_->now();
                can_data["header"]["frame_id"] = sensor_name_;
                can_data["src_addr"] = static_cast<uint8_t>(message->id & SOURCE_ADDR_MASK);

                const std::unordered_map<std::string, PhysicalValue> &signals_map = it->second.signals;
                for (const auto &signal_info: signals_map | std::views::values) {
                    putSignalInRosMessage(
                        can_data[dbc_signal_name_to_ros(signal_info.signal.name())], message->data, signal_info, node_->get_logger()
                    );
                }

                publishers_.at(pgn)->publish(can_data);
            }
        }
    }

    void BabelBridge::Impl::transmit_frame(std::unique_ptr<ros_babel_fish::CompoundMessage> message) {
        const auto pgn_lookup_it = ros_name_pgn_mappings_.find(message->name());
        const bool pgn_found = pgn_lookup_it != ros_name_pgn_mappings_.end();
        if (!pgn_found) {
            RCLCPP_ERROR(node_->get_logger(), "Could not map ROS message '%s' to message ID in DBC",
                         message->name().c_str());
            return;
        }

        const uint32_t pgn = pgn_lookup_it->second;
        const auto message_lookup_it = dbc_parser_.messages.find(pgn);
        const bool message_found = message_lookup_it != dbc_parser_.messages.end();
        if (!message_found) {
            RCLCPP_ERROR(
                node_->get_logger(), "Could not map ROS message '%s' with PGN '%d' to a type", message->name().c_str(),
                pgn
            );
            return;
        }
        MessageDefinition message_definition = message_lookup_it->second;

        auto can_message = std::make_unique<can_msgs::msg::Frame>();
        can_message->header.stamp = node_->now();
        can_message->header.frame_id = sensor_name_;
        can_message->id = pgn | device_ID_;
        can_message->dlc =  message_definition.dlc;

        for (const auto &signal_info: message_definition.signals | std::views::values) {
            putSignalInCanMessage((*message)[dbc_signal_name_to_ros(signal_info.signal.name())], can_message->data, signal_info, node_->get_logger());
        }

        pub_can_->publish(std::move(can_message));
    }

    // BEGIN MANAGEMENT FUNCTIONS //

    void BabelBridge::Impl::configure_publishers_subscribers(const std::string &msg_topic_prefix) {
        RCLCPP_INFO(node_->get_logger(), "AAA");
        for (const auto &[pgn, message_definition]: dbc_parser_.messages) {
            RCLCPP_INFO(node_->get_logger(), "AAAB");
            const std::string &unprefixed_ros_message_name = dbc_message_name_to_ros(message_definition.name);
            std::string message_name =
                    (std::ostringstream{} << msg_package_ << "/msg/" << unprefixed_ros_message_name).str();
            RCLCPP_DEBUG_STREAM(node_->get_logger(), "Configuring Publishers - found PGN: " << pgn);
            RCLCPP_DEBUG_STREAM(node_->get_logger(), "Attempting to load '" << message_name << "'");
            try {
                // There is a missing @throws marker in the documentation for ros_babel_fish:::BabelFish::create_publisher, but it
                //      raises BabbleFishException if the type cannot be found
                std::string topic_name =
                        (std::ostringstream{} << msg_topic_prefix
                         << (!msg_topic_prefix.empty() && msg_topic_prefix.back() == '/' ? "" : "/")
                         << sensor_name_ << '/' << unprefixed_ros_message_name)
                        .str();
                publishers_[pgn] =
                        fish_->create_publisher(*node_, topic_name, message_name, 20, rclcpp::PublisherOptions{});
                subscribers_[pgn] = fish_->create_subscription(
                    *node_, topic_name + "/tx", message_name, 20,
                    [this](std::unique_ptr<ros_babel_fish::CompoundMessage> message) { transmit_frame(std::move(message)); }
                );

                pgn_ros_name_mappings_.emplace(pgn, message_name);
                ros_name_pgn_mappings_.emplace(std::move(message_name), pgn);
            } catch (class_loader::LibraryLoadException &e) {
                RCLCPP_FATAL_STREAM(node_->get_logger(),
                                    "Failed to load library containing message type '" << message_name << "'\n"
                                    << e.what()
                );
                throw;
            } catch (ros_babel_fish::BabelFishException &e) {
                RCLCPP_WARN_STREAM(node_->get_logger(),
                                   "Could not find message type for message '" << message_name << "'\n"
                                   << e.what()
                );
            }
        }
    }

} // namespace ros2_j1939_babbler

namespace {
    /**
     * @brief Different lengths of integers which are available for use in ROS messages.
     */
    enum class IntegerLengths { b8, b16, b32, b64 };

    void putSignalInRosMessage(
        ros_babel_fish::Message &ros_signal, std::array<uint8_t, 8> &can_data,
        const ros2_j1939_babbler::PhysicalValue &can_signal, const rclcpp::Logger &logger
    ) {
        uint64_t raw = 0;
        std::memcpy(&raw, &can_data[0], sizeof(raw));

        // can_signal can't be const because GetInitialValue isn't const qualified
        switch (can_signal.signal.value_type()) {
            case can::i64:
                RCLCPP_DEBUG_STREAM(
                    logger,
                    "Processing signed integer signal '" << can_signal.signal.name() << "': raw=" << raw << ", scale="
                    << can_signal.factor << ", offset=" << can_signal.value_offset << ", length=" << can_signal.size
                );
                switch (ceil_bits(can_signal.size)) {
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
                    << can_signal.factor << ", offset=" << can_signal.value_offset << ", length=" << can_signal.size
                );
                // This is unbelievably ugly, but was only way I could get the compiler to not promote to int/uint and cause a Babel fish warning
                switch (ceil_bits(can_signal.size)) {
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
                    << can_signal.factor << ", offset=" << can_signal.value_offset << ", length=" << can_signal.size
                );
                decodeAndPut<float>(ros_signal, raw, can_signal, logger);
                break;
            case can::f64:
                RCLCPP_DEBUG_STREAM(
                    logger,
                    "Processing double signal '" << can_signal.signal.name() << "': raw=" << raw << ", scale="
                    << can_signal.factor << ", offset=" << can_signal.value_offset << ", length=" << can_signal.size
                );
                decodeAndPut<double>(ros_signal, raw, can_signal, logger);
                break;
        }
    }

    void putSignalInCanMessage(
        const ros_babel_fish::Message &ros_signal, std::array<uint8_t, 8> &can_data,
        const ros2_j1939_babbler::PhysicalValue &can_signal, const rclcpp::Logger &logger
    ) {
        switch (can_signal.signal.value_type()) {
            case can::i64:
                switch (ceil_bits(can_signal.size)) {
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
                switch (ceil_bits(can_signal.size)) {
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
                    << can_signal.factor << ", offset=" << can_signal.value_offset << ", length=" << can_signal.size
                );
                break;
            case can::f32:
                encodeAndPut<float>(ros_signal, can_signal, can_data, logger);
                RCLCPP_DEBUG_STREAM(
                    logger,
                    "Pushed float signal '" << can_signal.signal.name() << "': scale="
                    << can_signal.factor << ", offset=" << can_signal.value_offset << ", length=" << can_signal.size
                );
                break;
            case can::f64:
                encodeAndPut<double>(ros_signal, can_signal, can_data, logger);
                RCLCPP_DEBUG_STREAM(
                    logger,
                    "Pushed double signal '" << can_signal.signal.name() << "': scale="
                    << can_signal.factor << ", offset=" << can_signal.value_offset << ", length=" << can_signal.size
                );
                break;
        }
    }

    template<typename T>
    void decodeAndPut(ros_babel_fish::Message &ros_signal, uint64_t raw_value,
                      const ros2_j1939_babbler::PhysicalValue &can_signal, const rclcpp::Logger &logger) {
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
                      const ros2_j1939_babbler::PhysicalValue &can_signal, std::array<uint8_t, 8> can_data,
                      const rclcpp::Logger &logger) {
        T value = ros_signal.as<ros_babel_fish::ValueMessage<T> >().getValue();
        if (value < can_signal.min || value > can_signal.max) {
            RCLCPP_DEBUG_STREAM(
                logger,
                "Signal '" << can_signal.signal.name() << "': value=" << value << " is out of range; min=" << can_signal
                .
                min << ", max=" << can_signal.max
            );
        }
        T raw_value = (ros_signal.as<ros_babel_fish::ValueMessage<T> >().getValue() - can_signal.value_offset) /
                      can_signal.factor;
        can_signal.codec(*reinterpret_cast<uint64_t *>(&raw_value), can_data.data());
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

    std::string read_file(const std::string &dbc_path) {
        std::ifstream dbc_content(dbc_path);
        std::ostringstream ss;
        ss << dbc_content.rdbuf();
        return ss.str();
    }
} // namespace
