# ROS2 J1939 Babbler

ros2_j1939_babbler is a J1939-to-ROS message bridge based off the [work of Arturo Saucedo and Isaac Blankenau](https://github.com/psaucedoa/generic_can_driver)
which was [presented at ROSCON 2024](https://vimeo.com/1026028313). It expands on this by automatically generating ROS
message definitions from a DBC file and automatically converting incoming J1939 messages to their respective ROS messages.

There are two variations of the bridge. The first, `babel_bridge`, uses [ros2_babel_fish](https://github.com/LOEWE-emergenCITY/ros_babel_fish)
to load and map message definitions at runtime. This allows you to use ros2_j1939_babbler without needing to recompile 
the bridge every time your message definitions change: as long as you start it with a current DBC file and have message
a ROS package with message definitions available, it will work. The second variation, `static_bridge` bakes message
translations in at compile time for better and more reliable performance. In nearly all applications, the J1939 messages
which will be on the bus are known at compile time, so for those willing to recompile the bridge whenever the DBC changes,
the static bridge bypasses all the type introspection for a much simpler node.

The backbone of ros2_j1939_babbler is `ros2_j1939_babbler_msgs/MessageGenerator.py`, which parses the DBC file and
generates ROS message definitions. As well, it generates a C++ header containing the template specialisations the static
bridge needs for populating the ROS message from the J1939 message and for generating the compile-time dispatch table.
An example of the generated header is shown below. The templates to complete the conversions and dispatchng are contained in
`ros2_j1939_babbler_msgs/include/ros2_j1939_babbler_msgs/type_conversion_helpers.hpp`.

As templates are extensively used as part of the compile-time implementation, the PIMPL technique is used to prevent
implementation details from leaking into public headers.

## Non-ROS Build Dependencies
- Python and cantools package
- If compiling the Babel Bridge, You must have the Boost Fusion and Spirit headers available
  - These are header only libraries, so are not required at runtime


## How to Use
1. Replace `ros2_J1939_babbler_msgs/Messages.dbc` with your own DBC file, or alternatively set the CMake variable `DBC_PATH`
   to your DBC file
2. Compile and install the ros2_j1939_babbler_msgs package, or your own fork
3. If using the babel_bridge, simply run the babel_bridge node.
   - e.g. `ros2 run ros2_j1939_babbler babel_bridge --ros-args -p dbw_dbc_file:=./ros2_j1939_babbler_msgs/Messages.dbc -p msg_package:=ros2_j1939_babbler_msgs ...and so on`
4. If you wish to use the static_bridge, recompile and install the ros2_j1939_babbler package, and run the static_bridge node
   - e.g. `ros2 run ros2_j1939_babbler static_bridge --ros-args -p dbw_dbc_file:=./ros2_j1939_babbler_msgs/Messages.dbc -p msg_package:=ros2_j1939_babbler_msgs ...and so on`

Both nodes are offered as components if you desire to run them as part of a composable node container.

## ROS Parameters
| Parameter        | Type        | Description                                                                                                | Default | babel_bridge | static_bridge |
|------------------|-------------|------------------------------------------------------------------------------------------------------------|---------|--------------|---------------|
| dbw_dbc_file     | string      | Path to the DBC file to use for decoding                                                                   | empty   | &check;      | &cross;       |
| msg_package      | string      | ROS package to load message definitions from                                                               | empty   | &check;      | &cross;       |
| frame_id         | string      | TF2 frame designator                                                                                       | empty   | &check;      | &check;       |
| sensor_name      | string      | Name of the ECU, to prefix topics with                                                                     | empty   | &check;      | &check;       |
| device_ID        | uint8       | Source address of this ECU, used to filter PDU1 messages                                                   | 0       | &check;      | &check;       |
| can_sub_topic    | string      | [ros2_socketcan](https://github.com/autowarefoundation/ros2_socketcan) topic to listen for CAN messages on | empty   | &check;      | &check;       |
| msg_topic_prefix | string      | Name of the ECU, to prefix topics with                                                                     | empty   | &check;      | &check;       |
| msg_filter_ids   | int64 array | List of message IDs to match before processing                                                             | {0}     | &check;      | &check;       |
| msg_filter_masks | int64 array | List of ID masks to control matching, each associated with the ID at the same index in `msg_filter_ids`    | {0}     | &check;      | &check;       |
| promiscuous      | boolean     | Make the bridge process messages regardless of if they are addressed to it                                 | false   | &check;      | &check;       |

Note that the default ID/mask pair functions as an all-pass filter.


## Future Work
- Eliminate/clarify overlap between `sensor_name` and `msg_topic_prefix` parameters
- Replace `can_dbc_parser` with a compile-time mapping in static bridge
  - Ideally supports enums as well
- Extend to support plain CAN messages
- Extend to support multiplexed messages
- Fix address claim sequence
- J1939 TP message support
- Clean up remaining ros2_j1939 code
- Fix launch file
- Provide better example commands, some example screenshots
- Unit tests


## Example of auto-generated `ros2_j1939_babbler_msgs/type_conversion.hpp` for the curious:
In addition to the ROS2 message interfaces generated by the `ros2_j1939_babbler_msgs` package, a C++ header providing
utilities to convert J1939 messages to the associated ROS message is also generated. These are in the form of template
specialisations, which are then used by `ros2_j1939_babbler_msgs/type_conversion_helpers.hpp` to produce more 
sophisticated type conversion functions. This culminates in a compile-time-generated dispatch table that can be used to
automatically select the correct function for building up and publishing the ROS2 version of a received J1939 message.

Below is an example of what this type conversion header may contain. This was generated from a DBC file containing only 
the messages "EngineData" (PGN 0x00001 / CAN ID 0x00000100) and "WheelData" (PGN 0x00002 / CAN ID 0x00000200). 
```c++
#ifndef ROS2_J1939_BABBLER_MSGS__AUTO_GENERATED_TYPE_CONVERSIONS_
#define ROS2_J1939_BABBLER_MSGS__AUTO_GENERATED_TYPE_CONVERSIONS_

#include "type_conversion_helpers.hpp"

#include <unordered_map>
#include <string>
#include <tuple>
#include <cstdint>

#include <rclcpp/rclcpp.hpp>

// ===== Message type includes =====
        
#include "msg/engine_data.hpp" 
#include "msg/wheel_data.hpp" 
        
namespace ros2_j1939_babbler_msgs {
    
    
    // ===== Type conversion functions =====

    // BEGIN AUTO-GENERATED SPECIALISATIONS
    template<>
    inline void populate<msg::EngineData>(msg::EngineData& msg, const std::unordered_map<std::string, double>& fields)
    {
        msg.engine_rpm = static_cast<float>(fields.at("Engine_RPM"));
        msg.engine_temp = static_cast<std::uint8_t>(fields.at("Engine_Temp"));
    }
    template<>
    inline void populate<msg::WheelData>(msg::WheelData& msg, const std::unordered_map<std::string, double>& fields)
    {
        msg.wheel_speed_fl = static_cast<float>(fields.at("Wheel_Speed_FL"));
        msg.wheel_speed_fr = static_cast<float>(fields.at("Wheel_Speed_FR"));
    }
    // END AUTO-GENERATED SPECIALISATIONS
    
    
    // ===== Map converting PGN to ROS message type =====
    
    // BEGIN AUTO-GENERATED SPECIALISATIONS
    template<> struct pgn_message_type_map<256> { using type = msg::EngineData; };
    template<> struct pgn_message_type_map<512> { using type = msg::WheelData; };
    // END AUTO-GENERATED SPECIALISATIONS
    
    
    // ===== Map converting ROS message type to the ROS message type as a string =====    
    
    // BEGIN AUTO-GENERATED SPECIALISATIONS
    template<> struct message_type_name_map<msg::EngineData> { static constexpr char name[] = "EngineData"; };
    template<> struct message_type_name_map<msg::WheelData> { static constexpr char name[] = "WheelData"; };
    // END AUTO-GENERATED SPECIALISATIONS
    
    
    // ===== Create alias for dispatch table containing list of PGNs for all generated ROS messages =====
        
    using DispatchTable = DispatchTable_T<256, 512>;
        
} // namespace ros2_j1939_babbler_msgs

#endif //ROS2_J1939_BABBLER_MSGS__AUTO_GENERATED_TYPE_CONVERSIONS_
```