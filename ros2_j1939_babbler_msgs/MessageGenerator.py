# Copyright 2026 University of Manitoba Robotics Team
# Noah Reeder
#
# # Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# #     http://www.apache.org/licenses/LICENSE-2.0
# # Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


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
        f.write("""/*
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

#ifndef ROS2_J1939_BABBLER_MSGS__AUTO_GENERATED_TYPE_CONVERSIONS_
#define ROS2_J1939_BABBLER_MSGS__AUTO_GENERATED_TYPE_CONVERSIONS_

#include "type_conversion_helpers.hpp"

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
    
    
    // ===== Type conversion functions =====

    // BEGIN AUTO-GENERATED SPECIALISATIONS""")
        for message in messages:
            msg_type_name = MESSAGE_NAME_MASK.sub('', message.name)
            f.write(f"""
    template<>
    inline void populate<msg::{msg_type_name}>(msg::{msg_type_name}& msg, const std::unordered_map<std::string, double>& fields)""")
            f.write("""
    {""")
            for signal in message.signals:
                dtype = compute_dtype(signal)
                # Map ROS2 message field type to C++ type
                if dtype == "float32": dtype = "float"
                elif dtype == "float64": dtype = "double"
                else: dtype = f"std::{dtype}_t"
                f.write(f"""
        msg.{FIELD_NAME_MASK.sub('', signal.name.lower())} = static_cast<{dtype}>(fields.at("{signal.name}"));""")
            f.write("""
    }""")
        f.write("""
    // END AUTO-GENERATED SPECIALISATIONS
    
    
    // ===== Map converting PGN to ROS message type =====
    
    // BEGIN AUTO-GENERATED SPECIALISATIONS""")
        for message in messages:
            msg_type_name = MESSAGE_NAME_MASK.sub('', message.name)
            f.write(f"""
    template<> struct pgn_message_type_map<{message.frame_id & PGN_MASK}> {{ using type = msg::{msg_type_name}; }};""")
        f.write("""
    // END AUTO-GENERATED SPECIALISATIONS
    
    
    // ===== Map converting ROS message type to the ROS message type as a string =====    
    
    // BEGIN AUTO-GENERATED SPECIALISATIONS""")
        for message in messages:
            msg_type_name = MESSAGE_NAME_MASK.sub('', message.name)
            f.write(f"""
    template<> struct message_type_name_map<msg::{msg_type_name}> {{ static constexpr char name[] = "{msg_type_name}"; }};""")
        f.write("""
    // END AUTO-GENERATED SPECIALISATIONS
    
    
    // ===== Create alias for dispatch table containing list of PGNs for all generated ROS messages =====
        """)
        f.write(f"""
    using DispatchTable = DispatchTable_T<{", ".join(f"{msg.frame_id & PGN_MASK}" for msg in messages)}>;
        """)
        f.write("""
} // namespace ros2_j1939_babbler_msgs

#endif //ROS2_J1939_BABBLER_MSGS__AUTO_GENERATED_TYPE_CONVERSIONS_
        """)

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