 # ROS2 J1939 Babbler Messages

 This is a ROS2 interface package template intended to be used with [ros2_j1939_babbler](https://github.com/umroboticsteam/ros2_j1939_babbler) on ROS2 Humble.
 It consists of some package boilerplate and a Python script which parses CAN message descriptions from a DBC file, and generates a ROS message for each of the CAN messages.
 The resulting ROS messages can then be used by `ros2_j1939_babbler` to convert between the CAN messages and ROS messages.

 ## How to Use

1. Download/fork this repository
2. Replace the information in `package.xml` with the information for your project (package name, version, description, maintainer, etc.)
3. Replace `Messages.dbc` with your DBC file
   - If you do not wish to name your file `Messages.dbc`, change the `DBC_PATH` constant in `CMakeLists.txt` to point to your DBC file
4. Build and install the package!

Whenever you make changes to the DBC file, rebuild the package and the ROS messages will be regenerated to match.
