// Copyright 2023 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Maciej Bednarczyk (macbednarczyk@gmail.com)

#include <numeric>

#include "ethercat_generic_plugins/generic_ec_cia402_drive.hpp"

namespace ethercat_generic_plugins {

  EcCiA402Drive::EcCiA402Drive() : GenericEcSlave() {
  }
  EcCiA402Drive::~EcCiA402Drive() {
  }

  bool EcCiA402Drive::initialized() const {
    // Check if every drive is initialized
    return std::all_of(
        initialized_.begin(), initialized_.end(),
        [](const std::pair<std::string, bool> &pair) {
          return pair.second;
        }
    );
    // return initialized_;
  }

  void EcCiA402Drive::processData(size_t index, uint8_t *domain_address) {
    // Special case: ControlWord
    ethercat_interface::EcPdoChannelManager channel = 
        pdo_channels_info_[domain_map_[index]];
    std::string for_name = channel.for_name;
    
    if (channel.index == CiA402D_RPDO_CONTROLWORD + channel.pdo_offset) {
      if (is_operational_) {
        if (fault_reset_command_interface_index_[for_name] >= 0) {
          if (joint_command_interfaces_[for_name]->at(fault_reset_command_interface_index_[for_name]
              ) == 0) {
            last_fault_reset_command_[for_name] = false;
          }
          if (last_fault_reset_command_[for_name] == false &&
              joint_command_interfaces_[for_name]->at(fault_reset_command_interface_index_[for_name]
              ) != 0 &&
              !std::isnan(joint_command_interfaces_[for_name]->at(
                  fault_reset_command_interface_index_[for_name]
              ))) {
            std::cout << "Triggering fault reset :" << std::endl;
            last_fault_reset_command_[for_name] = true;
            fault_reset_[for_name] = true;
          }
        }

        if (auto_state_transitions_) {
          channel.default_value = transition(
              state_[for_name], channel.ec_read(domain_address), for_name
          );
          if (mode_of_operation_[for_name] == ModeOfOperation::MODE_PROFILED_POSITION) {
            // Send New Target triggers if target position changes
            if (state_[for_name] == STATE_OPERATION_ENABLED &&
                position_command_interface_index_[for_name] >= 0) {
              uint16_t control_word = channel.default_value;
              double target_position =
                  joint_command_interfaces_[for_name]->at(position_command_interface_index_[for_name]);
              if (!std::isnan(target_position) &&
                  target_position == previous_target_[for_name] &&
                  (control_word & 0b00010000) == 0b00000000) {
                channel.default_value = transition(
                    STATE_NEW_TARGET,
                    channel.ec_read(domain_address), for_name
                );
              } else if ((!std::isnan(target_position) ||
                          !std::isnan(previous_target_[for_name])) &&
                         previous_target_[for_name] != target_position) {
                previous_target_[for_name] = target_position;
                channel.default_value = transition(
                    STATE_NEW_TARGET_RESET,
                    channel.ec_read(domain_address), for_name
                );
              }
            }
          }
        }

        if (mode_of_operation_display_[for_name] == ModeOfOperation::MODE_HOMING) {
          // Also check if start_homing is triggerd
          if (start_homing_command_interface_index_[for_name] >= 0) {
            if (joint_command_interfaces_[for_name]->at(start_homing_command_interface_index_[for_name]
                ) != 0) {
              std::cerr << "EcCiA402Drive: Homing triggerd " << std::endl;
              // Start homing is triggerd. We will only remove this when we
              // actualy trigger the homing.
              if (state_[for_name] == STATE_OPERATION_ENABLED) {
                // Get if already running, if so reset to 0
                // check if bit 10 is set, set = not running, not set = running
                bool set =
                    (status_word_[for_name] & 0b0000010000000000) == 0b0000010000000000;
                if (set) {
                  std::cerr
                      << "EcCiA402Drive: Transitioning to new homing target "
                      << std::endl;
                  channel.default_value = transition(
                      STATE_NEW_TARGET,
                      channel.ec_read(domain_address), for_name
                  );
                  joint_command_interfaces_[for_name]->at(
                      start_homing_command_interface_index_[for_name]
                  ) = 0;
                } else {
                  std::cerr << "EcCiA402Drive: Homing already running "
                            << std::endl;
                  channel.default_value = transition(
                      STATE_NEW_TARGET_RESET,
                      channel.ec_read(domain_address), for_name
                  );
                }

                last_position_[for_name] = 0; // Set command interface to 0
                std::cerr
                    << "EcCiA402Drive: Setting last_position_ tot 0 for DRIVE"
                    << for_name << std::endl;
                for (auto &chan : pdo_channels_info_) {
                  if (chan.for_name != for_name) {
                    continue; // Only reset channels for this drive
                  }
                  if (chan.index == CiA402D_RPDO_POSITION + chan.pdo_offset) {
                    chan.last_value = 0;
                    chan.default_value = 0;
                    std::cerr << "EcCiA402Drive: Setting last value tot NAN "
                                 "for DRIVE "
                              << for_name << std::endl;
                  } else if (chan.index == CiA402D_TPDO_POSITION + chan.pdo_offset) {
                    std::cerr << "EcCiA402Drive: Current position "
                              << chan.last_value << std::endl;
                    joint_command_interfaces_[for_name]->at(position_command_interface_index_[for_name]
                    ) = 0;
                  }
                }
              }
              // } else {
              //   pdo_channels_info_[domain_map_[index]].default_value = transition(
              //       STATE_NEW_TARGET_RESET,
              //       pdo_channels_info_[domain_map_[index]].ec_read(domain_address)
              //   );
              // }
            }
          }
        }
      }

      // if homing, reset cmd interface to nan
      // if (mode_of_operation_display_ == ModeOfOperation::MODE_HOMING) {
      //   last_position_ = 0; // Set command interface to 0
      //   std::cerr << "EcCiA402Drive: Setting last_position_ tot 0 for DRIVE "
      //             << for_name_ << std::endl;
      //   for (auto &channel : pdo_channels_info_) {
      //     if (channel.index == CiA402D_RPDO_POSITION) {
      //       channel.last_value = 0;
      //       channel.default_value = 0;
      //       std::cerr << "EcCiA402Drive: Setting last value tot NAN for DRIVE
      //       "
      //                 << for_name_ << std::endl;
      //     } else if (channel.index == CiA402D_TPDO_POSITION) {
      //       std::cerr << "EcCiA402Drive: Current position "
      //                 << channel.last_value << std::endl;
      //       command_interface_ptr_->at(position_command_interface_index_) =
      //       0;
      //     }
      //   }
      // }
    }

    // setup current position as default position
    if (channel.index == CiA402D_RPDO_POSITION + channel.pdo_offset) {
      if (mode_of_operation_display_[for_name] != ModeOfOperation::MODE_NO_MODE &&
          !std::isnan(last_position_[for_name])) {
        channel.default_value =
            channel.factor * last_position_[for_name] +
            channel.offset;
      }
      channel.override_command =
          (mode_of_operation_display_[for_name] !=
               ModeOfOperation::MODE_CYCLIC_SYNC_POSITION &&
           mode_of_operation_display_[for_name] != ModeOfOperation::MODE_PROFILED_POSITION
          )
          ? true
          : false;
    }

    // setup mode of operation
    if (channel.index == CiA402D_RPDO_MODE_OF_OPERATION + channel.pdo_offset) {
      if (mode_of_operation_[for_name] >= 0 && mode_of_operation_[for_name] <= 10) {
        channel.default_value = mode_of_operation_[for_name];
      }
    }

    
    channel.ec_update(domain_address);

    // get mode_of_operation_display_
    if (channel.index ==
        CiA402D_TPDO_MODE_OF_OPERATION_DISPLAY + channel.pdo_offset) {
      mode_of_operation_display_[for_name] = channel.last_value;
    }

    if (channel.index == CiA402D_TPDO_POSITION + channel.pdo_offset) {
      last_position_[for_name] = channel.last_value;
    }

    // Special case: StatusWord
    if (channel.index == CiA402D_TPDO_STATUSWORD + channel.pdo_offset) {
      status_word_[for_name] = channel.last_value;
    // }

    // CHECK FOR STATE CHANGE
    // need to check if the index is the last one for this for_name. Not so easy.. so we check when index is status_word 
    // if (index == all_channels_.size() - 1) { // if last entry  in domain
      if (status_word_[for_name] != last_status_word_[for_name]) {
        state_[for_name] = deviceState(status_word_[for_name]);
        if (state_[for_name] != last_state_[for_name]) {
          std::cout << "STATE[" << alias << " - " << for_name <<"]: (" << channel.pdo_offset << ") " << DEVICE_STATE_STR.at(state_[for_name])
                    << " with status word :" << status_word_[for_name] << std::endl;
        }
      }
      initialized_[for_name] = ((state_[for_name] == STATE_OPERATION_ENABLED) &&
                      (last_state_[for_name] == STATE_OPERATION_ENABLED))
          ? true
          : false;

      last_status_word_[for_name] = status_word_[for_name];
      last_state_[for_name] = state_[for_name];
      counter_[for_name]++;
    }
  }

  bool EcCiA402Drive::setupSlave(
      std::unordered_map<std::string, std::string> slave_paramters,
      std::unordered_map<std::string, std::vector<double>*> joint_state_interfaces,
      std::unordered_map<std::string, std::vector<double>*> joint_command_interfaces
  ) {
    // state_interface_ptr_ = state_interface;
    // command_interface_ptr_ = command_interface;
    joint_state_interfaces_ = joint_state_interfaces;
    joint_command_interfaces_ = joint_command_interfaces;
    paramters_ = slave_paramters;

    if (paramters_.find("slave_config") != paramters_.end()) {
      if (!setup_from_config_file(paramters_["slave_config"])) {
        return false;
      }
    } else {
      std::cerr << "EcCiA402Drive: failed to find 'slave_config' tag in URDF."
                << std::endl;
      return false;
    }
    setup_interface_mapping();
    setup_syncs();

    std::vector<std::string> joint_names;
    for (const auto &joint : joint_command_interfaces) {
      joint_names.push_back(joint.first);
    }
    for (const auto &joint : joint_state_interfaces) {
      if (std::find(joint_names.begin(), joint_names.end(), joint.first) ==
          joint_names.end()) {
          joint_names.push_back(joint.first);
      }
    }
    for (auto &joint : joint_names) {
      counter_[joint] = 0;
      last_status_word_[joint] = -1;
      status_word_[joint] = 0;
      control_word_[joint] = 0;
      last_state_[joint] = STATE_START;
      state_[joint] = STATE_START;
      initialized_[joint] = false;
      last_fault_reset_command_[joint] = false;
      fault_reset_timer_[joint] = 0; // 0 is not a valid timer value
      fault_reset_[joint] = fault_reset_on_start_up; // reset on start up
      previous_target_[joint] = -1; // -1 is not a valid target position
      last_position_[joint] = std::numeric_limits<double>::quiet_NaN(); // NAN is not a valid position
      mode_of_operation_display_[joint] = -1; // default to no mode
      if (paramters_.find("mode_of_operation") != paramters_.end()) {
        mode_of_operation_[joint] = std::stod(paramters_["mode_of_operation"]);
        mode_of_operation_display_[joint] = mode_of_operation_[joint];
      } else {
        mode_of_operation_[joint] = -1; // default to no mode
      }

      if (paramters_.find("command_interface/" + joint + "/reset_fault") != paramters_.end()) {
        fault_reset_command_interface_index_[joint] =
            std::stoi(paramters_["command_interface/" + joint + "/reset_fault"]);
      } else {
        fault_reset_command_interface_index_[joint] = -1;
      }
      
      std::string start_homing_command = "command_interface/" + joint + "/start_homing";
      if (paramters_.find(start_homing_command) != paramters_.end()) {
        start_homing_command_interface_index_[joint] =
            std::stoi(paramters_[start_homing_command]);
        std::cerr << "EcCiA402Drive: Setup start homing index." << std::endl;
      } else {
        start_homing_command_interface_index_[joint] = -1;
      }

      if (paramters_.find("command_interface/" + joint + "/position") != paramters_.end()) {
        position_command_interface_index_[joint] =
            std::stoi(paramters_["command_interface/" + joint + "/position"]);
      } else {
        position_command_interface_index_[joint] = -1;
      }
    }

    if (paramters_.find("alias") != paramters_.end()) {
      alias = std::stod(paramters_["alias"]);
    }

    std::cout << paramters_["name"] << " was setup with cia402" << std::endl;

    return true;
  }

  bool EcCiA402Drive::setup_from_config(YAML::Node drive_config) {
    if (!GenericEcSlave::setup_from_config(drive_config)) {
      return false;
    }
    // additional configuration parameters for CiA402 Drives
    if (drive_config["auto_fault_reset"]) {
      auto_fault_reset_ = drive_config["auto_fault_reset"].as<bool>();
    }
    if (drive_config["fault_reset_on_start_up"]) {
      fault_reset_on_start_up = drive_config["fault_reset_on_start_up"].as<bool>();
    }
    if (drive_config["auto_state_transitions"]) {
      auto_state_transitions_ =
          drive_config["auto_state_transitions"].as<bool>();
    }
    return true;
  }

  bool EcCiA402Drive::setup_from_config_file(std::string config_file) {
    // Read drive configuration from YAML file
    try {
      slave_config_ = YAML::LoadFile(config_file);
    } catch (const YAML::ParserException &ex) {
      std::cerr << "EcCiA402Drive: failed to load drive configuration: "
                << ex.what() << std::endl;
      return false;
    } catch (const YAML::BadFile &ex) {
      std::cerr << "EcCiA402Drive: failed to load drive configuration: "
                << ex.what() << std::endl;
      return false;
    }
    if (!setup_from_config(slave_config_)) {
      return false;
    }
    return true;
  }

  /** returns device state based upon the status_word */
  DeviceState EcCiA402Drive::deviceState(uint16_t status_word) {
    if ((status_word & 0b01001111) == 0b00000000) {
      return STATE_NOT_READY_TO_SWITCH_ON;
    } else if ((status_word & 0b01001111) == 0b01000000) {
      return STATE_SWITCH_ON_DISABLED;
    } else if ((status_word & 0b01101111) == 0b00100001) {
      return STATE_READY_TO_SWITCH_ON;
    } else if ((status_word & 0b01101111) == 0b00100011) {
      return STATE_SWITCH_ON;
    } else if ((status_word & 0b01101111) == 0b00100111) {
      return STATE_OPERATION_ENABLED;
    } else if ((status_word & 0b01101111) == 0b00000111) {
      return STATE_QUICK_STOP_ACTIVE;
    } else if ((status_word & 0b01001111) == 0b00001111) {
      return STATE_FAULT_REACTION_ACTIVE;
    } else if ((status_word & 0b01001111) == 0b00001000) {
      return STATE_FAULT;
    }
    return STATE_UNDEFINED;
  }

  /** returns the control word that will take device from state to next desired
   * state */
  uint16_t EcCiA402Drive::transition(DeviceState state, uint16_t control_word, std::string for_name) {
    switch (state) {
    case STATE_START: // -> STATE_NOT_READY_TO_SWITCH_ON (automatic)
      return control_word;
    case STATE_NOT_READY_TO_SWITCH_ON: // -> STATE_SWITCH_ON_DISABLED
                                       // (automatic)
      return control_word;
    case STATE_SWITCH_ON_DISABLED: // -> STATE_READY_TO_SWITCH_ON
      return (control_word & 0b01111110) | 0b00000110;
    case STATE_READY_TO_SWITCH_ON: // -> STATE_SWITCH_ON
      return (control_word & 0b01110111) | 0b00000111;
    case STATE_SWITCH_ON: // -> STATE_OPERATION_ENABLED
      return (control_word & 0b01111111) | 0b00001111;
    case STATE_NEW_TARGET:
      return control_word | 0b00010000;
    case STATE_NEW_TARGET_RESET:
      return control_word & 0b11101111;
    case STATE_OPERATION_ENABLED: // -> GOOD
      if (fault_reset_[for_name]) {
        fault_reset_[for_name] = false;
        fault_reset_timer_[for_name] = 0;
      }
      return control_word;
    case STATE_QUICK_STOP_ACTIVE: // -> STATE_OPERATION_ENABLED
      return (control_word & 0b01111111) | 0b00001111;
    case STATE_FAULT_REACTION_ACTIVE: // -> STATE_FAULT (automatic)
      return control_word;
    case STATE_FAULT: // -> STATE_SWITCH_ON_DISABLED
      if (auto_fault_reset_ || fault_reset_[for_name] || fault_reset_timer_[for_name] > 0) {
        fault_reset_[for_name] = false;
        auto current_command =
            joint_command_interfaces_[for_name]->at(position_command_interface_index_[for_name]);
        // command_interface_ptr_->at(position_command_interface_index_) =
        //     std::numeric_limits<double>::quiet_NaN(); // Clear command
        //     interface

        last_position_[for_name] =
            std::numeric_limits<double>::quiet_NaN(); // Clear command interface
        std::cerr << "EcCiA402Drive: Setting last_position_ tot NAN for DRIVE " << std::endl;
        for (auto &channel : pdo_channels_info_) {
          if (channel.for_name != for_name) {
            continue; // Only reset channels for this drive
          }
          if (channel.index == CiA402D_RPDO_POSITION + channel.pdo_offset) {
            channel.last_value = std::numeric_limits<double>::quiet_NaN();
            channel.default_value = std::numeric_limits<double>::quiet_NaN();
            std::cerr << "EcCiA402Drive: Setting last value tot NAN for DRIVE " << std::endl;
          } else if (channel.index == CiA402D_TPDO_POSITION + channel.pdo_offset) {
            std::cerr << "EcCiA402Drive: Current position "
                      << channel.last_value << std::endl;
            joint_command_interfaces_[for_name]->at(position_command_interface_index_[for_name]) =
                channel.last_value;
          }

          std::cerr
            << "EcCiA402Drive: Setting command_interface_ptr_ tot current "
               "for DRIVE " << " Previous: " << current_command << std::endl;
          std::cerr << "Now: "
                    << joint_command_interfaces_[for_name]->at(
                          position_command_interface_index_[for_name]
                      )
                    << std::endl;
        }
        std::cerr << "EcCiA402Drive: RESETTING DRIVE " << std::endl;
      
        fault_reset_timer_[for_name] += 1;
        if (fault_reset_timer_[for_name] > 100) {
          fault_reset_timer_[for_name] = 0;
          return (control_word & 0b01111111); // set automatic reset to 0
        }
        return (control_word & 0b11111111) | 0b10000000; // automatic reset
      } else {
        return control_word;
      }
    default:
      break;
    }
    return control_word;
  }

} // namespace ethercat_generic_plugins

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    ethercat_generic_plugins::EcCiA402Drive, ethercat_interface::EcSlave
)
