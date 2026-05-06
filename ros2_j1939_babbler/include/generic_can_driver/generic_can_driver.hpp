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

#ifndef GENERIC_CAN_DRIVER__GENERIC_CAN_DRIVER_HPP_
#define GENERIC_CAN_DRIVER__GENERIC_CAN_DRIVER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "can_msgs/msg/frame.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/temperature.hpp"

#include "can_dbc_parser/Dbc.hpp"
#include "can_dbc_parser/DbcBuilder.hpp"
#include "can_dbc_parser/DbcMessage.hpp"
#include "can_dbc_parser/DbcSignal.hpp"

#include <ros_babel_fish/babel_fish.hpp>

using namespace std::chrono_literals;

namespace ros2_j1939
{

    class GenericCanDriver : public rclcpp::Node
{
public:
  explicit GenericCanDriver(const rclcpp::NodeOptions & OPTIONS);

  ~GenericCanDriver();

private:

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
  void rxFrame(const can_msgs::msg::Frame::SharedPtr& MSG);

  // DATABASE MANAGEMENT FUNCTIONS //
  /**
   * @brief Instantiates a NewEagle dbc database object using the dbc file specified in params.
   * Strips the priority and source address id from the message ID in the DBC.
   * These stripped IDs are stored as keys in a map with the NewEagle DBC messages as values.
   */
  void setupDatabase();

  /**
   * @brief Checks the messages in the DBC and creates a publisher for each one
   * 
   * The publishers are of type "j1939_msgs::msg::CanData" with a topic name folling a
   * "sensor_name/key_message" pattern TODO: Docs
   *
   * @param msg_topic_prefix prefix to apply before message topics
   */
  void configurePublishers(const std::string& msg_topic_prefix);

  /**
   * @brief functions that takes list of full addresses (such as 0x0CEEFFA1) as defined in params
   * yaml and attempts and address claim attack. It adopts the lowest value name and publishes it on
   * that address, forcing the target device on that address to either stop publishing or move to a
   * different address, depending on its internal logic.
  */
  void generateAddressClaimAttackMsg(
    can_msgs::msg::Frame::SharedPtr MSG, const std::vector<uint32_t> source_addresses
  );

  /**
   * @brief formats data nicely for use in CAN frames
   */
  void createDataArray(
    const std::vector<uint16_t> data_in, 
    const std::vector<uint16_t> data_lengths, 
    std::array<uint8_t, 8UL> &data_out
  );

  /**
  * @brief check if an incoming message passes the message filters
  */
  bool filter(const uint32_t id) const;

  // params
  std::string dbw_dbc_file_;  // the messages definition (such as J1939 standard)
  std::string msg_package_;   // ROS2 package containing ROS msg definitions for CAN messages described in DBC file
  std::string frame_id_;      // used on the published messages - usually just the location of the sensor on your robot
  std::string sensor_name_;   // name of your sensor / device (e.g. engine ECU)
  uint8_t device_ID_;         // j1939 source address of your device, in decimal format (last 2 hex numbers of the CANID)
  std::string device_ID_str_; // [device_ID_str_] just turning the device_id into a string. used to populate message headers
  std::string can_interface_; // the name of the CAN line the device is on (e.g. can0)
                              // if true, will set up publishers for EVERY message in the provided dbc
  std::string sub_topic_can_; // subscribe to the topic socket_can is publishing from the CAN line
  std::string pub_topic_can_; // publish to the topic socket_can is sending to the CAN line
  std::vector<int64_t> msg_filter_ids_; // list of ids messages must match (within a mask) to be processed
  std::vector<int64_t> msg_filter_masks_; // parallel list of masks to apply to the filter ids

  NewEagle::Dbc dbw_dbc_db_;   // new eagle dbc database
  std::map<uint32_t , NewEagle::DbcMessage> dbc_id_msg_map_;
  std::map<std::string , NewEagle::DbcMessage> dbc_name_msg_map_;
  ros_babel_fish::BabelFish::UniquePtr fish_;
  std::map<std::string, ros_babel_fish::BabelFishPublisher::SharedPtr> publishers_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr sub_can_;
};

}  // namespace generic_can_driver

#endif  // GENERIC_CAN_DRIVER__GENERIC_CAN_DRIVER_HPP_
