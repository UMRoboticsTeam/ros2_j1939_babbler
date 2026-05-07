import cantools
import os
import sys
import re
import typing

# There are more rules than these, but these cover almost all realistic cases
# Additional restrictions I'm aware of:
#   - Field name can't start or end with underscore, can't have consecutive underscores
#   - Field names must start with lowercase letter
#   - Message name must start with uppercase letter
MESSAGE_NAME_MASK = re.compile('[^A-Za-z0-9]')
FIELD_NAME_MASK = re.compile('[^a-z0-9_]')
PGN_MASK = 0x03FFFF00

PASCAL_TO_SNAKE_CASE_CONVERTER = re.compile(r'(?<!^)(?=[A-Z])')


def convert_files(dbc_file: str, msg_output_dir: str, type_conversion_output: str):
    print(f"Loading DBC file '{dbc_file}'")
    db = cantools.database.load_file(dbc_file)

    print(f"Exporting msg files to '{msg_output_dir}'")
    os.makedirs(msg_output_dir, exist_ok=True)  # Make the output directory if it doesn't exist

    # Loop through all the messages in the DBC and create ROS2 msg files for them
    for message in db.messages:
        print(f"Found message '{message.name}'")

        with open(f"{msg_output_dir}/{MESSAGE_NAME_MASK.sub('', message.name)}.msg", "w") as f:
            # We always want messages to include a header (timestamp etc.) and source address
            f.write("std_msgs/Header header\n")
            f.write("uint8 src_addr\n\n")

            # Loop through each signal and add it into the ROS message type
            for signal in message.signals:
                dtype = compute_dtype(signal)
                if signal.choices:  # This is an enum, special processing
                    # Each option is added as a constant prefixed with the signal name
                    # Not sure if nested named values are legal, but if they are we won't support them
                    for value, choice in signal.choices.items():
                        f.write(f"{dtype} {f'{signal.name}_{choice}'.upper()}={value}\n")

                f.write(f"{dtype} {FIELD_NAME_MASK.sub('', signal.name.lower())}\n\n")

    generate_type_conversions(db.messages, type_conversion_output)


def compute_dtype(signal: cantools.database.Signal) -> str:
    dtype = None
    if signal.choices:  # This is an enum
        # Represent as the smallest uint type which can fit all the options
        # Each option is added as a constant prefixed with the signal name
        # Not sure if nested named values are legal, but if they are we won't support them
        dtype = f"uint{ceil_bits(len(signal.choices))}"
    elif signal.is_float:
        # Use the length to decide if we need a float32 or float64, but note float8/float16 doesn't exist
        dtype = f"float{ceil_bits(max(32, signal.length))}"
    elif signal.is_signed:  # By default signal is either int/uint
        dtype = f"int{ceil_bits(signal.length)}"
    else:  # Finally, if it isn't an enum, float, or signed int, it must be a uint
        dtype = f"uint{ceil_bits(signal.length)}"
    return dtype


def generate_type_conversions(messages: list[cantools.database.Message], type_conversion_output: str):
    with open(type_conversion_output, "w") as f:
        f.write("""
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

/*
 * The below code is automatically generated from a utility made by Noah Reeder, University of Manitoba Robotics Team.
 * No ownership is claimed on contents between "BEGIN AUTO-GENERATED SPECIALISATION" and "END AUTO-GENERATED SPECIALISATION". 
 */

#ifndef ROS2_J1939_BABBLER_MSGS__AUTO_GENERATED_TYPE_CONVERSIONS
#define ROS2_J1939_BABBLER_MSGS__AUTO_GENERATED_TYPE_CONVERSIONS

#include <unordered_map>
#include <string>
#include <tuple>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>

// ===== Message type includes =====
        """)
        for message in messages:
            msg_type_name = MESSAGE_NAME_MASK.sub('', message.name)
            f.write(f"""
#include "msg/{PASCAL_TO_SNAKE_CASE_CONVERTER.sub('_', msg_type_name).lower()}.hpp" """)

        f.write("""
        
namespace ros2_j1939_babbler_msgs {
    // Need this to prevent compiler from automatically instantiating default buildAndPublish and hitting the static_assert
    template<typename T>
    struct dependent_false : std::false_type {};
    
    // Default type conversion, throws a compile-time error because we are expecting template to be specialised for known messages
    template<typename MSG_TYPE>
    void buildAndPublish(rclcpp::Publisher<MSG_TYPE>& publisher, const std::unordered_map<std::string, double>& fields, std_msgs::msg::Header header, const uint8_t src_addr)
    {
        static_assert(false, "No type conversion was generated for the provided message type");
    }
    
    // ===== Type conversion functions =====
        """)
        for message in messages:
            msg_type_name = MESSAGE_NAME_MASK.sub('', message.name)
            f.write("""
    template<>""")
            f.write(f"""
    inline void buildAndPublish<msg::{msg_type_name}>(rclcpp::Publisher<msg::{msg_type_name}>& publisher, const std::unordered_map<std::string, double>& fields, std_msgs::msg::Header header, const uint8_t src_addr)""")
            f.write("""
    {""")
            f.write(f"""
        msg::{msg_type_name} msg;
        msg.header = std::move(header);
        msg.src_addr = src_addr;
        // BEGIN AUTO-GENERATED SPECIALISATION
            """)
            for signal in message.signals:
                dtype = compute_dtype(signal)
                # Map ROS2 message field type to C++ type
                if dtype == "float32": dtype = "float"
                elif dtype == "float64": dtype = "double"
                else: dtype = f"std::{dtype}_t"
                f.write(f"""
        msg.{FIELD_NAME_MASK.sub('', signal.name.lower())} = static_cast<{dtype}>(fields.at("{signal.name}"));
                """)
            f.write("""
        // END AUTO-GENERATED SPECIALISATION
        publisher.publish(msg);
    }
            """)
        f.write("""
    
    // ===== Dispatch table =====
    
    template<uint32_t PGN> struct pgn_message_type_map;""")
        for message in messages:
            msg_type_name = MESSAGE_NAME_MASK.sub('', message.name)
            f.write(f"""
    template<> struct pgn_message_type_map<{message.frame_id & PGN_MASK}> {{ using type = msg::{msg_type_name}; }};""")

        f.write("""
        
    template<typename MSG_TYPE> struct message_type_name_map;""")
        for message in messages:
            msg_type_name = MESSAGE_NAME_MASK.sub('', message.name)
            f.write(f"""
    template<> struct message_type_name_map<msg::{msg_type_name}> {{ static constexpr char name[] = "{msg_type_name}"; }};""")

        f.write("""
        
    // Use loop, evaluated at compile time, to find index in storage tuple of a given PGN 
    template<uint32_t PGN, uint32_t... PGNs>
    constexpr std::size_t index_of() {
        constexpr uint32_t arr[] = {PGNs...};
        for (std::size_t i = 0; i < sizeof...(PGNs); ++i)
        {
            if (arr[i] == PGN) { return i; }
        }
        static_assert(sizeof...(PGNs) > 0, "PGN not found in set of known PGNs");
        return 0;
    }
    
    template<uint32_t... PGNs>
    class DispatchTable_T {
    public:
        DispatchTable_T(rclcpp::Node* node, const std::string& topic_prefix, const size_t qos_history_depth) 
            : publishers_(createPublishers(node, topic_prefix, qos_history_depth, std::integral_constant<uint32_t, PGNs>{}...))
            {}
    
        void runtime_dispatch(const uint32_t pgn, const std::unordered_map<std::string, double>& fields, std_msgs::msg::Header header, const uint8_t src_addr)
        {
            bool handled = ((pgn == PGNs ? (dispatch<PGNs>(fields, std::move(header), src_addr), true) : false) || ...); // Fold expression will expand into a giant switch
            assert(handled);
        }
        
        template<uint32_t PGN>
        void dispatch(const std::unordered_map<std::string, double>& fields, std_msgs::msg::Header header, const uint8_t src_addr) {
            constexpr std::size_t index = index_of<PGN, PGNs...>();
            auto& publisher = std::get<index>(publishers_);
            buildAndPublish<typename pgn_message_type_map<PGN>::type>(*publisher, fields, std::move(header), src_addr);
        }
        
    private:
        using storage_t = std::tuple<std::shared_ptr<rclcpp::Publisher<typename pgn_message_type_map<PGNs>::type>>...>;
        storage_t publishers_;
        
        template<uint32_t PGN>
        static std::shared_ptr<rclcpp::Publisher<typename pgn_message_type_map<PGN>::type>> makePublisher(rclcpp::Node* node, const std::string& topic_prefix, const size_t qos_history_depth)
        {
            using MSG_TYPE = typename pgn_message_type_map<PGN>::type;
            return node->create_publisher<MSG_TYPE>((std::ostringstream() << topic_prefix << '/' << message_type_name_map<MSG_TYPE>::name).str(), qos_history_depth);
        }
        
        template<uint32_t... PGNs_L>
        static storage_t createPublishers(rclcpp::Node* node, const std::string& topic_prefix, const size_t qos_history_depth, std::integral_constant<uint32_t, PGNs_L>... /*unused*/)
        {
            return std::make_tuple(makePublisher<PGNs_L>(node, topic_prefix, qos_history_depth)...);
        }   
    }; 
        """)
        f.write(f"""
    using DispatchTable = DispatchTable_T<{", ".join(f"{msg.frame_id & PGN_MASK}" for msg in messages)}>;
        """)
        f.write("""
} // namespace ros2_j1939_babbler_msgs
#endif
        """)

### Alternatively could use table for runtime dispatch ###
#using func_t = void(*)(DispatchTable_T&)
#static constexpr func_t table[] = {
#    [](DispatchTable& self) { self.template dispatch<PGNs>(); }...
#};

def ceil_bits(bit_length):
    if bit_length <= 8: return 8
    if bit_length <= 16: return 16
    if bit_length <= 32: return 32
    if bit_length <= 64: return 64
    raise Exception("Signals with length greater than 64 bits are not supported")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        raise RuntimeError("Script must be called with a path to a DBC file, a path to export msg files to, and a path to export the type conversion header")
    convert_files(sys.argv[1], sys.argv[2], sys.argv[3])