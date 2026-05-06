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

#include "generic_can_driver/generic_can_driver.hpp"

#include <memory>
#include <string>
#include <regex>
#include <variant>
#include <sstream>

constexpr inline uint32_t SOURCE_ADDR_MASK = 0x000000FFu;
constexpr inline uint32_t PFPS_MASK = 0x00FFFF00u; // Note doesn't include data page, NewEagle limitation?
constexpr inline uint32_t MAX_CAN_ID = 0x1FFFFFFFu; // 29-bits

namespace {
    enum class IntegerLengths {
        b8,
        b16,
        b32,
        b64
    };
    std::string dbc_message_name_to_ros(const std::string&);
    std::string dbc_signal_name_to_ros(const std::string&);
    IntegerLengths ceil_bits(const uint8_t bit_length);
}

namespace ros2_j1939
{

GenericCanDriver::GenericCanDriver(const rclcpp::NodeOptions & OPTIONS)
: rclcpp::Node("generic_can_driver", OPTIONS)
{
    RCLCPP_INFO(this->get_logger(), "Starting Generic Can Driver...");

    dbw_dbc_file_ = this->declare_parameter<std::string>("dbw_dbc_file", "");
    msg_package_ = this->declare_parameter<std::string>("msg_package", "");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "");
    sensor_name_ = this->declare_parameter<std::string>("sensor_name", "");
    device_ID_ = this->declare_parameter<uint8_t>("device_ID", 0);
    can_interface_ = this->declare_parameter<std::string>("can_interface", "can0");
    sub_topic_can_ = this->declare_parameter<std::string>("can_sub_topic", "");
    pub_topic_can_ = this->declare_parameter<std::string>("pub_topic_can", "");
    auto msg_filter_range = rcl_interfaces::msg::ParameterDescriptor{};
    msg_filter_range.integer_range = {rcl_interfaces::msg::IntegerRange().set__from_value(0).set__to_value(MAX_CAN_ID)};
    msg_filter_ids_ = this->declare_parameter<std::vector<int64_t>>("msg_filter_ids", {0}, msg_filter_range); // TODO: Explain somewhere that default is all-pass
    msg_filter_masks_ = this->declare_parameter<std::vector<int64_t>>("msg_filter_masks", {0}, msg_filter_range);

    auto msg_topic_prefix = this->declare_parameter<std::string>("msg_topic_prefix", "");

    if (msg_filter_ids_.size() != msg_filter_masks_.size()) {
        throw std::invalid_argument((
            std::ostringstream{} << "Message filter must have same number of IDs and masks, found " << msg_filter_ids_.size()
            << " ids and " << msg_filter_masks_.size() << " masks").str());
    }

    device_ID_str_ = std::to_string(device_ID_);
    fish_ = ros_babel_fish::BabelFish::make_unique();

    // printing to user
    RCLCPP_INFO(this->get_logger(), "dbw_dbc_file: %s", dbw_dbc_file_.c_str());
    RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
    RCLCPP_INFO(this->get_logger(), "sensor_name: %s", sensor_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "device_id: %d", device_ID_);
    RCLCPP_INFO(this->get_logger(), "sub_topic_can: %s", sub_topic_can_.c_str());
    RCLCPP_INFO(this->get_logger(), "pub_topic_can: %s", pub_topic_can_.c_str());

    // setup dbc database - brings in j1939 standard
    this->setupDatabase();
    RCLCPP_INFO(this->get_logger(), "Setup DBC database!");

    // automatically configure publishers
    this->configurePublishers(msg_topic_prefix);
    RCLCPP_INFO(this->get_logger(), "Setup publishers!");

    // setup subscriber, bind rxFrame
    this->sub_can_ = this->create_subscription<can_msgs::msg::Frame>(
            this->sub_topic_can_, 500, [this](const can_msgs::msg::Frame::SharedPtr msg) { rxFrame(std::forward<decltype(msg)>(msg)); });

    RCLCPP_DEBUG(this->get_logger(), "Generic Can Driver configured!");
}

GenericCanDriver::~GenericCanDriver() {}

// BGN CANUSB COMMS FUNCTIONS //
void GenericCanDriver::rxFrame(const can_msgs::msg::Frame::SharedPtr& MSG)
{
  RCLCPP_DEBUG(this->get_logger(), "New message; is_rtr:%d is_error:%d id:%d, sa:%d", MSG->is_rtr, MSG->is_error, MSG->id, MSG->id & 0x000000FFu);
  // if message is not a request, error, and matches device ID
  if(!MSG->is_rtr && !MSG->is_error && (device_ID_ == (MSG->id & 0x000000FFu) && filter(MSG->id)))
  {
    // local const to store incoming message
    const can_msgs::msg::Frame::SharedPtr incoming_MSG = MSG;
      RCLCPP_DEBUG(this->get_logger(), "Filtering message; sa:%d, count:%zu",MSG->id & 0x00FFFF00u, dbc_id_msg_map_.count(MSG->id & 0x00FFFF00u));
    // if the message type / PGN is found in the dbc
    if(dbc_id_msg_map_.count(MSG->id & 0x00FFFF00u) )
    {
      // RCLCPP_INFO(this->get_logger(), "Key: %s", msg_name.c_str());

      // translate the message data
      NewEagle::DbcMessage message = dbc_id_msg_map_[incoming_MSG->id & PFPS_MASK];
      message.SetFrame(incoming_MSG);

      // then create a local ros2 message
      // not really sure why we create the shared_ptr and reference the object instead of stack-allocating, but this is what all the examples do
      ros_babel_fish::CompoundMessage::SharedPtr can_data_ptr  = fish_->create_message_shared(msg_package_ + "/msg/" + dbc_message_name_to_ros(message.GetName()));
      ros_babel_fish::CompoundMessage& can_data = *can_data_ptr;

      // populate the local ros2 message header, frame, and message name
      can_data["header"]["stamp"] = this->now();
      can_data["header"]["frame_id"] = sensor_name_;
      can_data["src_addr"] = static_cast<uint8_t>(incoming_MSG->id & SOURCE_ADDR_MASK);

      // get the signals (e.g. x, y, z) within the message (e.g. acceleration)
      std::map<std::string, NewEagle::DbcSignal> signals_map = *message.GetSignals();
      for (auto [key_signal, value_signal] : signals_map)
      {
        // get the data for the current signal, figure out type, and insert into message accordingly
        switch (value_signal.GetDataType()) {
            case NewEagle::INT:
                RCLCPP_DEBUG(this->get_logger(), "Processing integer field: signed:%s, raw:%f, scale:%f, length:%u, offset:%f, result:%f", value_signal.GetSign() == NewEagle::SIGNED ? "Y" : "N", value_signal.GetInitialValue(), value_signal.GetGain(), value_signal.GetDlc(), value_signal.GetOffset(), value_signal.GetResult());
                // This is unbelievably ugly, but was only way I could get the compiler to not promote to int/uint and cause a Babel fish warning
                switch (ceil_bits(value_signal.GetDlc())) {
                    case IntegerLengths::b8:
                      if (value_signal.GetSign() == NewEagle::SIGNED) { can_data[dbc_signal_name_to_ros(key_signal)] = static_cast<int8_t>(value_signal.GetResult()); }
                      else { can_data[dbc_signal_name_to_ros(key_signal)] = static_cast<uint8_t>(value_signal.GetResult());}
                      break;
                    case IntegerLengths::b16:
                        if (value_signal.GetSign() == NewEagle::SIGNED) { can_data[dbc_signal_name_to_ros(key_signal)] = static_cast<int16_t>(value_signal.GetResult()); }
                        else { can_data[dbc_signal_name_to_ros(key_signal)] = static_cast<uint16_t>(value_signal.GetResult());}
                    break;
                    case IntegerLengths::b32:
                        if (value_signal.GetSign() == NewEagle::SIGNED) { can_data[dbc_signal_name_to_ros(key_signal)] = static_cast<int32_t>(value_signal.GetResult()); }
                        else { can_data[dbc_signal_name_to_ros(key_signal)] = static_cast<uint32_t>(value_signal.GetResult());}
                    break;
                    case IntegerLengths::b64:
                        if (value_signal.GetSign() == NewEagle::SIGNED) { can_data[dbc_signal_name_to_ros(key_signal)] = static_cast<int64_t>(value_signal.GetResult()); }
                        else { can_data[dbc_signal_name_to_ros(key_signal)] = static_cast<uint64_t>(value_signal.GetResult());}
                    break;
                }
                break;
            case NewEagle::FLOAT:
               // This is dumb, but can_dbc_parser is handling this awkwardly
               // Babel fish won't let us put a double in a float32 field, but can_dbc_parser always returns double even if underlying data is supposed to be float32
               // So we cast the result from can_dbc_parser to float before we insert into the field, and call it a day
               can_data[dbc_signal_name_to_ros(key_signal)] = static_cast<float>(value_signal.GetResult());
               RCLCPP_DEBUG(this->get_logger(), "Processing float field: raw:%f, scale:%f, offset:%f, result:%f", value_signal.GetInitialValue(), value_signal.GetGain(), value_signal.GetOffset(), value_signal.GetResult());
               break;
            case NewEagle::DOUBLE:
               can_data[dbc_signal_name_to_ros(key_signal)] = value_signal.GetResult();
               RCLCPP_DEBUG(this->get_logger(), "Processing double field: raw:%f, scale:%f, offset:%f, result:%f", value_signal.GetInitialValue(), value_signal.GetGain(), value_signal.GetOffset(), value_signal.GetResult());
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
// END CANUSB COMMS FUNCTIONS //

// BEGIN MANAGEMENT FUNCTIONS //
void GenericCanDriver::setupDatabase()
{
  // build the new eagle dbc database
  this->dbw_dbc_db_ = NewEagle::DbcBuilder().NewDbc(dbw_dbc_file_);
  this->dbc_name_msg_map_ = * this->dbw_dbc_db_.GetMessages();

  // for every message in the database, leave only PGN
  for (auto [key, value] : dbc_name_msg_map_)
  {
    // strip id of priority and source address info
    uint32_t stripped_id = value.GetId() & 0x00FFFF00u;
    dbc_id_msg_map_[stripped_id] = value;
    RCLCPP_DEBUG(this->get_logger(), "Accepting messages with stripped ID:%d from raw ID:%d", stripped_id, value.GetId());
  }
}

void GenericCanDriver::configurePublishers(const std::string& msg_topic_prefix)
{
  // iterate over the dbc to spawn an equal amount of publishers
  for (auto [key_message, value_message] : dbc_name_msg_map_)
  {
      std::string msg_name = (std::ostringstream{} << msg_package_ << "/msg/" << dbc_message_name_to_ros(key_message)).str();
      RCLCPP_DEBUG(this->get_logger(), "Configuring Publishers - found key_message: %s", key_message.c_str());
      RCLCPP_DEBUG(this->get_logger(), "Attempting to load '%s'", msg_name.c_str());
      try {
        // There is a missing @throws marker in the documentation for ros_babel_fish:::BabelFish::create_publisher, but it
        //      raises BabbleFishException if the type cannot be found
        std::string topic_name =  (std::ostringstream{} << msg_topic_prefix << (!msg_topic_prefix.empty() && msg_topic_prefix.back() == '/' ? "" : "/") << sensor_name_  << key_message).str();
        publishers_[key_message] = this->fish_->create_publisher(*this, topic_name, msg_name, 20, rclcpp::PublisherOptions{});
    }
    catch (class_loader::LibraryLoadException& e) {
        RCLCPP_FATAL_STREAM(this->get_logger(), "Failed to load library containing message type '" << msg_name << "'\n" << e.what());
        throw;
    }
    catch (ros_babel_fish::BabelFishException& e){
        RCLCPP_WARN_STREAM(this->get_logger(), "Could not find message type for message '" << msg_name << "'\n" << e.what());
    }
  }
}

void GenericCanDriver::createDataArray(
  const std::vector<uint16_t> data_in, const std::vector<uint16_t> data_lengths,
  std::array<uint8_t, 8UL> &data_out)
{
  uint64_t data_concatenated = 0;
  uint64_t data_mask = 0x00000000000000FF;
  int size = data_in.size();

  for (int i = 0; i < size; i++)
  {
    data_concatenated = data_concatenated << data_lengths[size - 1 - i];
    data_concatenated += data_in[size - 1 - i];
  }

  for (int i = 0; i < 8; i++)
  {
    data_out[i] = (data_mask & data_concatenated >> 8*i);
  }
}

void GenericCanDriver::generateAddressClaimAttackMsg(
  can_msgs::msg::Frame::SharedPtr MSG, const std::vector<uint32_t> source_addresses)
{
  // we go through each address given in the list
  for(uint32_t address : source_addresses)
  {
    // we add the source address (target of the claim attack) to a 'name declaration' message
    address += 0x18EEFF00;
    // by sending this message with only 0s, our name takes priority,
    // and the competing device stops publishing
    std::array<uint8_t, 8UL> claim_data = {
      0x00u, 0x00u, 0x00u, 0x00u, 0x00, 0x00, 0x00, 0x00u
      };

    // then we just stuff the can frame with all our data
    MSG->header.stamp = this->now();
    MSG->header.frame_id = "can";
    MSG->id = address;
    MSG->is_rtr = false;
    MSG->is_extended = true;
    MSG->is_error = false;
    MSG->dlc = 8;
    MSG->data = claim_data;
  }
}

bool GenericCanDriver::filter(const uint32_t id) const {
    bool pass = false;
    for (std::size_t i = 0; i < msg_filter_ids_.size() && !pass; ++i) {
        pass = (id & msg_filter_masks_[i]) == (msg_filter_ids_[i] & msg_filter_masks_[i]);
    }
    return pass;
}


// END MANAGEMENT FUNCTIONS //

// TODO:Arturo - Look through this and make sure it's the standard way of renaming CAN devices
// also, this could just be its own .log file or something idk

// void GenericCanDriver::txRename(
//   const std::array<uint8_t, 8UL> name, const uint8_t new_source_address)
// {
//   std::array<uint8_t, 8UL> BAM_data_out = {0x20u, 0x09u, 0x00u, 0x02u, 0xFFu, 0xD8u, 0xFEu, 0x00u};
//   can_msgs::msg::Frame BAM_frame_out;
//   uint32_t j1939_id = 0x1CECFF00u;
//   BAM_frame_out.header.stamp = this->now();
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
//   name_frame_out_1.header.stamp = this->now();
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
//   name_frame_out_2.header.stamp = this->now();
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
//   RCLCPP_INFO(this->get_logger(), "Published renaming thing!!!!!!!! %d", new_source_address);
// }


}  // namespace generic_can_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_j1939::GenericCanDriver)

namespace {
    /**
     * Strips characters other than [A-Za-z0-9].
     * @return the modified string
     */
    std::string dbc_message_name_to_ros(const std::string& dbc_message_name) {
        std::string result;
        result.reserve(dbc_message_name.size());
        std::copy_if(dbc_message_name.begin(), dbc_message_name.end(), std::back_inserter(result), [](const unsigned char& c){ return std::isalnum(c); });
        return result;
    }

    /**
     * Brings characters to lowercase and strips other than [a-z0-9_]
     * @return
     */
    std::string dbc_signal_name_to_ros(const std::string& dbc_signal_name) {
        // Sincce basically the same requirements as dbc_message_name_to_ros, use that and then change all uppercase to lowercase
        std::string result;
        result.reserve(dbc_signal_name.size());
        std::copy_if(dbc_signal_name.begin(), dbc_signal_name.end(), std::back_inserter(result), [](const unsigned char& c){ return std::isalnum(c)||c=='_'; });
        std::transform(result.begin(), result.end(), result.begin(),
                       [](const unsigned char& c){ return std::tolower(c); });
        return result;
    }

    IntegerLengths ceil_bits(const uint8_t bit_length) {
        if (bit_length <= 8) { return IntegerLengths::b8; }
        if (bit_length <= 16) { return IntegerLengths::b16; }
        if (bit_length <= 32) { return IntegerLengths::b32; }
        if (bit_length <= 64) { return IntegerLengths::b64; }
        throw std::invalid_argument("Signals with length greater than 64 bits are not supported");
    }
}
