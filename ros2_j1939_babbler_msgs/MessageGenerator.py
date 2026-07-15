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

from cantools.database.can.c_source import camel_to_snake_case, CodeGenMessage, CodeGenSignal

C_TYPE_DEFINITION_FILE_BASENAME = "can_message_types"

# There are more rules than these, but these cover almost all realistic cases
# Additional restrictions I'm aware of:
#   - Field name can't start or end with underscore, can't have consecutive underscores
#   - Field names must start with lowercase letter
#   - Message name must start with uppercase letter
MESSAGE_NAME_MASK = re.compile('[^A-Za-z0-9]')
FIELD_NAME_MASK = re.compile('[^a-z0-9_]')
PGN_MASK = 0x03FFFF00


def ros_header_name_formatter(message_name: str):
    """
    Re-implementation of how ROS generates the name:
    https://github.com/ros2/rosidl/blob/humble/rosidl_cmake/cmake/string_camel_case_to_lower_case_underscore.cmake
    """
    # Insert an underscore before any uppercase letter
    # which is followed by lowercase letters.
    header_name = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", message_name)

    # Insert an underscore before any uppercase letter
    # which is preceded by a lowercase letter or digit.
    header_name = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", header_name)

    return header_name.lower()
    
def convert_files(db: cantools.database.Database, msg_output_dir: str, type_conversion_output: str, database_name: str):
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

    generate_type_conversions(db.messages, type_conversion_output, database_name)


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
        
    # Kind of sketchy, but if scale is less than 1 then we always use a double to represent
    if signal.conversion.scale < 1:
        dtype = f"float{ceil_bits(max(32, signal.length))}"
        
    return dtype

HEADER_TEMPLATE = """/*
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

{include_statements}

namespace ros2_j1939_babbler_msgs {{
    
    // C header for CAN messages put in its own namespace
    namespace can {{
        extern "C" {{
            #include "{path_to_can_msgs_header}"
        }}
    }} // namespace can
    
    
    // ===== Type conversion functions =====
    
    // BEGIN AUTO-GENERATED SPECIALISATIONS
    {type_conversion_functions}
    // END AUTO-GENERATED SPECIALISATIONS
    
    
    // ===== Maps converting PGN to ROS and CAN message types =====
    
    // BEGIN AUTO-GENERATED SPECIALISATIONS
    {message_type_mappings}
    // END AUTO-GENERATED SPECIALISATIONS
    
    
    // ===== Map converting ROS message type to the ROS message type as a string =====
    
    // BEGIN AUTO-GENERATED SPECIALISATIONS
    {message_name_mappings}
    // END AUTO-GENERATED SPECIALISATIONS
    
    
    // ===== Maps C CAN message types to their packing function =====
    
    // BEGIN AUTO-GENERATED SPECIALISATIONS
    {can_packing_functions}
    // END AUTO-GENERATED SPECIALISATIONS
    
    // ===== Maps C CAN message types to their unpacking function =====
    
    // BEGIN AUTO-GENERATED SPECIALISATIONS
    {can_unpacking_functions}
    // END AUTO-GENERATED SPECIALISATIONS


    // ===== Map providing DLC for each known PGN =====
    
    // BEGIN AUTO-GENERATED SPECIALISATIONS
    {pgn_dlc_mappings}
    // END AUTO-GENERATED SPECIALISATIONS

    // ===== Create alias for dispatch table containing list of PGNs for all generated messages =====
    
    {dispatch_table_alias}
    
}} // namespace ros2_j1939_babbler_msgs

#endif //ROS2_J1939_BABBLER_MSGS__AUTO_GENERATED_TYPE_CONVERSIONS_
"""

INCLUDE_ROS_MESSAGE_HEADER_TEMPLATE = \
"""
#include "msg/{header_formatted_name}.hpp" """

POPULATE_ROS_TEMPLATE = \
"""
    template<>
    inline void populateRos<msg::{ros_msg_type_name}>(msg::{ros_msg_type_name}& ros_msg, const can::{can_msg_type_name}& can_msg)
    {{\
        {signal_mappings}
    }}"""

POPULATE_ROS_SIGNAL_MAPPING_TEMPLATE = \
"""
        ros_msg.{ros_field_name} = (static_cast<decltype(ros_msg.{ros_field_name})>(can_msg.{can_signal_name}) * {scale}) + {offset};"""

POPULATE_CAN_TEMPLATE = \
"""
    template<>
    inline void populateCan(can::{can_msg_type_name}& can_msg, const msg::{ros_msg_type_name}& ros_msg)
    {{\
        {signal_mappings}
    }}"""

POPULATE_CAN_SIGNAL_MAPPING_TEMPLATE = \
"""
        can_msg.{can_signal_name} = static_cast<decltype(can_msg.{can_signal_name})>((ros_msg.{ros_field_name} - {offset}) / {scale});"""

MESSAGE_TYPE_MAPPING_TEMPLATE = \
"""
    template<> struct pgn_ros_message_type_map<{pgn}> {{ using type = msg::{ros_msg_type_name}; }};
    template<> struct pgn_can_message_type_map<{pgn}> {{ using type = can::{can_msg_type_name}; }};"""

MESSAGE_NAME_MAPPING_TEMPLATE = \
"""
    template<> struct message_type_name_map<msg::{ros_msg_type_name}> {{ static constexpr char name[] = "{ros_msg_type_name}"; }};"""

CAN_PACK_FUNCTION_MAPPING_TEMPLATE = \
"""
    template<> inline int pack_can_message<can::{can_msg_type_name}>(uint8_t* dst_p, const struct can::{can_msg_type_name}* src_p, size_t size) {{ return {pack_function_name}(dst_p, src_p, size); }};"""

CAN_UNPACK_FUNCTION_MAPPING_TEMPLATE = \
    """
        template<> inline int unpack_can_message<can::{can_msg_type_name}>(struct can::{can_msg_type_name}* dst_p, const uint8_t* src_p, size_t size) {{ return {unpack_function_name}(dst_p, src_p, size); }};"""

PGN_DLC_MAPPING_TEMPLATE = \
"""
    template<> struct pgn_dlc_map<{pgn}> {{ static constexpr uint8_t value = {dlc}; }};"""

DISPATCH_TABLE_ALIAS_TEMPLATE = \
"""\
using DispatchTable = DispatchTable_T<{dispatch_table_pgns_string}>;"""

# TODO: Run generate_c_code program, include in namespace, and generate calls to that in populateCan functions
# TODO: Future improvement: Do same thing for CAN-to-J1939, replace fields parameter with n-tuple
# We take advantage of knowing what struct/function names are generated by cantools.database.c_source.py
# https://github.com/cantools/cantools/blob/master/src/cantools/database/can/c_source.py
def generate_type_conversions(messages: list[cantools.database.Message], type_conversion_output: str, database_name: str):
    with open(type_conversion_output, "w") as f:

        include_statements = ""
        type_conversion_functions = ""
        message_type_mappings = ""
        message_name_mappings = ""
        can_packing_functions = ""
        can_unpacking_functions = ""
        pgn_dlc_mappings = ""
        pgns = []

        for message in messages:
            ros_msg_type_name = MESSAGE_NAME_MASK.sub('', message.name)
            can_msg_type_name = f"{database_name}_{CodeGenMessage(message).snake_name}_t"  # Distilled from c_source.py
            pack_function_name = f"{database_name}_{CodeGenMessage(message).snake_name}_pack"  # Also distilled
            unpack_function_name = f"{database_name}_{CodeGenMessage(message).snake_name}_unpack"  # Also distilled
            pgn = message.frame_id & PGN_MASK

            include_statements += INCLUDE_ROS_MESSAGE_HEADER_TEMPLATE.format(
                header_formatted_name=ros_header_name_formatter(ros_msg_type_name))

            ros_to_can_signal_mappings = ""
            can_to_ros_signal_mappings = ""
            for signal in message.signals:
                dtype = compute_dtype(signal)
                # Map ROS2 message field type to C++ type
                if dtype == "float32": dtype = "float"
                elif dtype == "float64": dtype = "double"
                else: dtype = f"std::{dtype}_t"

                ros_field_name = FIELD_NAME_MASK.sub('', signal.name.lower())
                signal_info = CodeGenSignal(signal)
                can_signal_name = signal_info.snake_name  # Distilled from c_source.py

                can_to_ros_signal_mappings += POPULATE_ROS_SIGNAL_MAPPING_TEMPLATE.format(ros_field_name = ros_field_name,
                                                                                          dtype = dtype,
                                                                                          can_signal_name = can_signal_name,
                                                                                          scale = signal_info.signal.conversion.scale,
                                                                                          offset = signal_info.signal.conversion.offset)
                ros_to_can_signal_mappings += POPULATE_CAN_SIGNAL_MAPPING_TEMPLATE.format(ros_field_name = ros_field_name,
                                                                                          can_signal_name = can_signal_name,
                                                                                          scale = signal_info.signal.conversion.scale,
                                                                                          offset = signal_info.signal.conversion.offset)

            type_conversion_functions += POPULATE_ROS_TEMPLATE.format(can_msg_type_name = can_msg_type_name,
                                                                      ros_msg_type_name = ros_msg_type_name,
                                                                      signal_mappings = can_to_ros_signal_mappings)
            type_conversion_functions += "\n"
            type_conversion_functions += POPULATE_CAN_TEMPLATE.format(can_msg_type_name = can_msg_type_name,
                                                                      ros_msg_type_name = ros_msg_type_name,
                                                                      signal_mappings = ros_to_can_signal_mappings)
            type_conversion_functions += "\n\n"

            message_type_mappings += MESSAGE_TYPE_MAPPING_TEMPLATE.format(pgn = pgn,
                                                                          ros_msg_type_name = ros_msg_type_name,
                                                                          can_msg_type_name = can_msg_type_name)
            message_type_mappings += "\n"

            message_name_mappings += MESSAGE_NAME_MAPPING_TEMPLATE.format(pgn = pgn,
                                                                          ros_msg_type_name = ros_msg_type_name)

            can_packing_functions += CAN_PACK_FUNCTION_MAPPING_TEMPLATE.format(can_msg_type_name = can_msg_type_name,
                                                                               pack_function_name =pack_function_name)
            can_unpacking_functions += CAN_UNPACK_FUNCTION_MAPPING_TEMPLATE.format(can_msg_type_name = can_msg_type_name,
                                                                                 unpack_function_name = unpack_function_name)

            pgn_dlc_mappings += PGN_DLC_MAPPING_TEMPLATE.format(pgn = pgn,
                                                                dlc = message.length)

            pgns.append(pgn)

        dispatch_table_pgns_string = ", ".join(f"{pgn}" for pgn in pgns)
        dispatch_table_alias = DISPATCH_TABLE_ALIAS_TEMPLATE.format(dispatch_table_pgns_string = dispatch_table_pgns_string)

        f.write(HEADER_TEMPLATE.format(
            path_to_can_msgs_header = f"{C_TYPE_DEFINITION_FILE_BASENAME}.h",
            include_statements = include_statements,
            type_conversion_functions = type_conversion_functions,
            message_type_mappings= message_type_mappings,
            message_name_mappings = message_name_mappings,
            can_packing_functions = can_packing_functions,
            can_unpacking_functions = can_unpacking_functions,
            pgn_dlc_mappings = pgn_dlc_mappings,
            dispatch_table_alias = dispatch_table_alias
        ))

def ceil_bits(bit_length):
    if bit_length <= 8: return 8
    if bit_length <= 16: return 16
    if bit_length <= 32: return 32
    if bit_length <= 64: return 64
    raise Exception("Signals with length greater than 64 bits are not supported")


if __name__ == "__main__":
    if len(sys.argv) != 6:
        raise RuntimeError(f"Script must be called with a path to a DBC file, a folder to export msg files to, a path to " 
                           "export the type conversion header, a folder to export the C message definition header, and a " 
                           f"folder to export the C message definition source file. Received: {sys.argv}")
    dbc_path = sys.argv[1]
    msg_export_path = sys.argv[2]
    type_conversion_header_export_path = sys.argv[3]
    c_type_def_header_export_folder = sys.argv[4]
    c_type_def_source_export_folder = sys.argv[5]
    c_type_def_header_file_path = f"{c_type_def_header_export_folder}/{C_TYPE_DEFINITION_FILE_BASENAME}.h"
    c_type_def_source_file_path = f"{c_type_def_source_export_folder}/{C_TYPE_DEFINITION_FILE_BASENAME}.c"

    print("Ensuring output folders exist")
    os.makedirs(os.path.dirname(type_conversion_header_export_path), exist_ok=True)
    os.makedirs(c_type_def_header_export_folder, exist_ok=True)
    os.makedirs(c_type_def_source_export_folder, exist_ok=True)

    print(f"Loading DBC file '{dbc_path}'")
    db = cantools.database.load_file(dbc_path)
    convert_files(db, msg_export_path, type_conversion_header_export_path, C_TYPE_DEFINITION_FILE_BASENAME)
    header, source, _, _ = cantools.database.can.c_source.generate(
        db,
        C_TYPE_DEFINITION_FILE_BASENAME,
        c_type_def_header_file_path,
        c_type_def_source_file_path,
        "",
        True,
        False,
        False,
        None,
        False)
    with open(c_type_def_header_file_path, "w") as header_file:
        header_file.write(header)
    with open(c_type_def_source_file_path, "w") as source_file:
        source_file.write(source)