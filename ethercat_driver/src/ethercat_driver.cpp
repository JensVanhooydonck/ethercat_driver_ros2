// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#include "ethercat_driver/ethercat_driver.hpp"

#include <tinyxml2.h>
#include <string>
#include <regex>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ethercat_driver {
  CallbackReturn
  EthercatDriver::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) !=
        CallbackReturn::SUCCESS) {
      return CallbackReturn::ERROR;
    }
    const std::lock_guard<std::mutex> lock(ec_mutex_);
    activated_ = false;

    hw_joint_states_.resize(info_.joints.size());
    for (uint j = 0; j < info_.joints.size(); j++) {
      hw_joint_states_[j].resize(
          info_.joints[j].state_interfaces.size(),
          std::numeric_limits<double>::quiet_NaN()
      );
    }
    hw_sensor_states_.resize(info_.sensors.size());
    for (uint s = 0; s < info_.sensors.size(); s++) {
      hw_sensor_states_[s].resize(
          info_.sensors[s].state_interfaces.size(),
          std::numeric_limits<double>::quiet_NaN()
      );
    }
    hw_gpio_states_.resize(info_.gpios.size());
    for (uint g = 0; g < info_.gpios.size(); g++) {
      hw_gpio_states_[g].resize(
          info_.gpios[g].state_interfaces.size(),
          std::numeric_limits<double>::quiet_NaN()
      );
    }
    hw_joint_commands_.resize(info_.joints.size());
    for (uint j = 0; j < info_.joints.size(); j++) {
      hw_joint_commands_[j].resize(
          info_.joints[j].command_interfaces.size(),
          std::numeric_limits<double>::quiet_NaN()
      );
    }
    hw_sensor_commands_.resize(info_.sensors.size());
    for (uint s = 0; s < info_.sensors.size(); s++) {
      hw_sensor_commands_[s].resize(
          info_.sensors[s].command_interfaces.size(),
          std::numeric_limits<double>::quiet_NaN()
      );
    }
    hw_gpio_commands_.resize(info_.gpios.size());
    for (uint g = 0; g < info_.gpios.size(); g++) {
      hw_gpio_commands_[g].resize(
          info_.gpios[g].command_interfaces.size(),
          std::numeric_limits<double>::quiet_NaN()
      );
    }

    std::unordered_map<std::string, std::vector<std::string>> module_joints = {};
    std::unordered_map<std::string, std::vector<std::string>> module_gpio = {};
    std::unordered_map<std::string, std::vector<std::string>> module_sensors = {};
    // std::unordered_map<std::string, std::vector<std::unordered_map<std::string, std::string>>> = joint_module_params = {};

    for (uint j = 0; j < info_.joints.size(); j++) {
      RCLCPP_INFO(
          rclcpp::get_logger("EthercatDriver"), "joint: %s",
          info_.joints[j].name.c_str()
      );
      // check all joints for EC modules and load into ec_modules_
      auto module_params =
          getEcModuleParam(info_.original_xml, info_.joints[j].name, "joint");

      for (auto i = 0ul; i < module_params.size(); i++) {
        // for (auto k = 0ul; k < info_.joints[j].state_interfaces.size(); k++) {
        //   module_params[i]
        //   joint_module_params[info_.joints[j].name]
        //                ["state_interface/" + info_.joints[j].name + "/" +
        //                 info_.joints[j].state_interfaces[k].name] =
        //                    std::to_string(k);
        // }
        // for (auto k = 0ul; k < info_.joints[j].command_interfaces.size(); k++) {
        //   joint_module_params[info_.joints[j].name]
        //                ["command_interface/" + info_.joints[j].name + "/" +
        //                 info_.joints[j].command_interfaces[k].name] =
        //                    std::to_string(k);
        // }
        if (module_joints.find(module_params[i]["name"]) == module_joints.end()) {
          module_joints[module_params[i]["name"]] = {};
        } 
        module_joints[module_params[i]["name"]].push_back(
            info_.joints[j].name
        );
      }
    }
    for (uint g = 0; g < info_.gpios.size(); g++) {
      RCLCPP_INFO(
          rclcpp::get_logger("EthercatDriver"), "gpio: %s",
          info_.gpios[g].name.c_str()
      );
      // check all gpios for EC modules and load into ec_modules_
      auto module_params =
          getEcModuleParam(info_.original_xml, info_.gpios[g].name, "gpio");
      for (auto i = 0ul; i < module_params.size(); i++) {
        if (module_gpio.find(module_params[i]["name"]) == module_gpio.end()) {
          module_gpio[module_params[i]["name"]] = {};
        } 
        module_gpio[module_params[i]["name"]].push_back(
            info_.gpios[g].name
        );
      }
    }

    for (uint s = 0; s < info_.sensors.size(); s++) {
      RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "sensors");
      // check all sensors for EC modules and load into ec_modules_
      auto module_params =
          getEcModuleParam(info_.original_xml, info_.sensors[s].name, "sensor");
      for (auto i = 0ul; i < module_params.size(); i++) {
        if (module_sensors.find(module_params[i]["name"]) == module_sensors.end()) {
          module_sensors[module_params[i]["name"]] = {};
        } 
        module_sensors[module_params[i]["name"]].push_back(
            info_.sensors[s].name
        );
      }
    }

    for (uint i = 0; i < ec_module_parameters_.size(); i++) {
      auto joint_names = module_joints[ec_module_parameters_[i]["name"]];
      std::unordered_map<std::string, std::vector<double>*> hw_joint_states = {};
      std::unordered_map<std::string, std::vector<double>*> hw_joint_commands = {};
      for (auto j = 0ul; j < info_.joints.size(); j++) {
        auto joint_name = info_.joints[j].name;
        if (std::find(joint_names.begin(), joint_names.end(), joint_name) !=
            joint_names.end()) {
              hw_joint_states[joint_name] = &hw_joint_states_[j];
              hw_joint_commands[joint_name] = &hw_joint_commands_[j];
              for (auto k = 0ul; k < info_.joints[j].state_interfaces.size(); k++) {
                    ec_module_parameters_[i]
                       ["state_interface/" + info_.joints[j].name + "/" +
                        info_.joints[j].state_interfaces[k].name] =
                           std::to_string(k);
              }
              for (auto k = 0ul; k < info_.joints[j].command_interfaces.size(); k++) {
                ec_module_parameters_[i]
                            ["command_interface/" + info_.joints[j].name + "/" +
                              info_.joints[j].command_interfaces[k].name] =
                                std::to_string(k);
              }
          }
      }

      auto gpio_names = module_gpio[ec_module_parameters_[i]["name"]];
      for (auto j = 0ul; j < info_.gpios.size(); j++) {
        auto gpio_name = info_.gpios[j].name;
        if (std::find(gpio_names.begin(), gpio_names.end(), gpio_name) !=
            gpio_names.end()) {
              hw_joint_states[gpio_name] = &hw_gpio_states_[j];
              hw_joint_commands[gpio_name] = &hw_gpio_commands_[j];
              for (auto k = 0ul; k < info_.gpios[j].state_interfaces.size(); k++) {
                    ec_module_parameters_[i]
                       ["state_interface/" + info_.gpios[j].name + "/" +
                        info_.gpios[j].state_interfaces[k].name] =
                           std::to_string(k);
              }
              for (auto k = 0ul; k < info_.gpios[j].command_interfaces.size(); k++) {
                ec_module_parameters_[i]
                            ["command_interface/" + info_.gpios[j].name + "/" +
                              info_.gpios[j].command_interfaces[k].name] =
                                std::to_string(k);
              }
          }
      }

      auto sensors_names = module_sensors[ec_module_parameters_[i]["name"]];
      for (auto j = 0ul; j < info_.sensors.size(); j++) {
        auto sensors_name = info_.sensors[j].name;
        if (std::find(sensors_names.begin(), sensors_names.end(), sensors_name) !=
            sensors_names.end()) {
              hw_joint_states[sensors_name] = &hw_sensor_states_[j];
              hw_joint_commands[sensors_name] = &hw_sensor_commands_[j];
              for (auto k = 0ul; k < info_.sensors[j].state_interfaces.size(); k++) {
                    ec_module_parameters_[i]
                       ["state_interface/" + info_.sensors[j].name + "/" +
                        info_.sensors[j].state_interfaces[k].name] =
                           std::to_string(k);
              }
              for (auto k = 0ul; k < info_.sensors[j].command_interfaces.size(); k++) {
                ec_module_parameters_[i]
                            ["command_interface/" + info_.sensors[j].name + "/" +
                              info_.sensors[j].command_interfaces[k].name] =
                                std::to_string(k);
              }
          }
      }

      try {
        RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"), "Creating module %s for %s",
          ec_module_parameters_[i].at("plugin").c_str(), ec_module_parameters_[i]["name"].c_str()
      );
      if ( ec_module_parameters_[i]["name"] == "454_weight") {
        for (auto &param : ec_module_parameters_[i]) {
          RCLCPP_INFO(
            rclcpp::get_logger("EthercatDriver"),
            "Param: %s : %s", param.first.c_str(), param.second.c_str()
          );
        }
      }
        auto module =
            ec_loader_.createSharedInstance(ec_module_parameters_[i].at("plugin"));
        if (!module->setupSlave(
                ec_module_parameters_[i], hw_joint_states,
                hw_joint_commands
            )) {
          RCLCPP_FATAL(
              rclcpp::get_logger("EthercatDriver"),
              "Setup of Joint module %i FAILED.", i + 1
          );
          return CallbackReturn::ERROR;
        }
        RCLCPP_INFO(
            rclcpp::get_logger("EthercatDriver"), "Adding ECSlave %i %s",
            i,
            ec_module_parameters_[i]["name"].c_str()
        );
        ec_modules_.push_back(module);
      } catch (pluginlib::PluginlibException &ex) {
        RCLCPP_FATAL(
            rclcpp::get_logger("EthercatDriver"),
            "The plugin of %s failed to load for some reason. Error: %s\n",
            ec_module_parameters_[i]["name"].c_str(), ex.what()
        );
      }
    }
    
    // for (uint g = 0; g < info_.gpios.size(); g++) {
    //   RCLCPP_INFO(
    //       rclcpp::get_logger("EthercatDriver"), "gpio: %s",
    //       info_.gpios[g].name.c_str()
    //   );
    //   // check all gpios for EC modules and load into ec_modules_
    //   auto module_params =
    //       getEcModuleParam(info_.original_xml, info_.gpios[g].name, "gpio");
    //   std::unordered_map<std::string, std::vector<double>*> hw_joint_states = {};
    //   std::unordered_map<std::string, std::vector<double>*> hw_joint_commands = {};
    //   hw_joint_states[info_.gpios[g].name] = &hw_gpio_states_[g];
    //   hw_joint_commands[info_.gpios[g].name] = &hw_gpio_commands_[g];
    //   // ec_module_parameters_.insert(
    //   //     ec_module_parameters_.end(), module_params.begin(),
    //   //     module_params.end()
    //   // );
    //   for (auto i = 0ul; i < module_params.size(); i++) {
    //     for (auto k = 0ul; k < info_.gpios[g].state_interfaces.size(); k++) {
    //       module_params[i]
    //                    ["state_interface/" +
    //                     info_.gpios[g].state_interfaces[k].name] =
    //                        std::to_string(k);
    //     }
    //     for (auto k = 0ul; k < info_.gpios[g].command_interfaces.size(); k++) {
    //       module_params[i]
    //                    ["command_interface/" +
    //                     info_.gpios[g].command_interfaces[k].name] =
    //                        std::to_string(k);
    //     }
    //     try {
    //       auto module =
    //           ec_loader_.createSharedInstance(module_params[i].at("plugin"));
    //       if (!module->setupSlave(
    //               module_params[i], hw_joint_states, hw_joint_commands
    //           )) {
    //         RCLCPP_FATAL(
    //             rclcpp::get_logger("EthercatDriver"),
    //             "Setup of GPIO module %li FAILED.", i + 1
    //         );
    //         return CallbackReturn::ERROR;
    //       }
    //       ec_modules_.push_back(module);
    //     } catch (pluginlib::PluginlibException &ex) {
    //       RCLCPP_FATAL(
    //           rclcpp::get_logger("EthercatDriver"),
    //           "The plugin of %s failed to load for some reason. Error: %s\n",
    //           info_.gpios[g].name.c_str(), ex.what()
    //       );
    //     }
    //   }
    // }
    // for (uint s = 0; s < info_.sensors.size(); s++) {
    //   RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "sensors");
    //   // check all sensors for EC modules and load into ec_modules_
    //   auto module_params =
    //       getEcModuleParam(info_.original_xml, info_.sensors[s].name, "sensor");
    //   // ec_module_parameters_.insert(
    //   //     ec_module_parameters_.end(), module_params.begin(),
    //   //     module_params.end()
    //   // );
    //   std::unordered_map<std::string, std::vector<double>*> hw_joint_states = {};
    //   std::unordered_map<std::string, std::vector<double>*> hw_joint_commands = {};
    //   hw_joint_states[info_.sensors[s].name] = &hw_sensor_states_[s];
    //   hw_joint_commands[info_.sensors[s].name] = &hw_sensor_commands_[s];
    //   for (auto i = 0ul; i < module_params.size(); i++) {
    //     for (auto k = 0ul; k < info_.sensors[s].state_interfaces.size(); k++) {
    //       module_params[i]
    //                    ["state_interface/" +
    //                     info_.sensors[s].state_interfaces[k].name] =
    //                        std::to_string(k);
    //     }
    //     for (auto k = 0ul; k < info_.sensors[s].command_interfaces.size();
    //          k++) {
    //       module_params[i]
    //                    ["command_interface/" +
    //                     info_.sensors[s].command_interfaces[k].name] =
    //                        std::to_string(k);
    //     }
    //     try {
    //       auto module =
    //           ec_loader_.createSharedInstance(module_params[i].at("plugin"));
    //       if (!module->setupSlave(
    //               module_params[i], hw_joint_states,
    //               hw_joint_commands
    //           )) {
    //         RCLCPP_FATAL(
    //             rclcpp::get_logger("EthercatDriver"),
    //             "Setup of Sensor module %li FAILED.", i + 1
    //         );
    //         return CallbackReturn::ERROR;
    //       }
    //       ec_modules_.push_back(module);
    //     } catch (pluginlib::PluginlibException &ex) {
    //       RCLCPP_FATAL(
    //           rclcpp::get_logger("EthercatDriver"),
    //           "The plugin of %s failed to load for some reason. Error: %s\n",
    //           info_.sensors[s].name.c_str(), ex.what()
    //       );
    //     }
    //   }
    // }

    RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"), "Got %li modules",
        ec_modules_.size()
    );

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn EthercatDriver::on_configure(const rclcpp_lifecycle::State
                                                  & /*previous_state*/) {
    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  EthercatDriver::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    // export joint state interface
    for (uint j = 0; j < info_.joints.size(); j++) {
      for (uint i = 0; i < info_.joints[j].state_interfaces.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[j].name, info_.joints[j].state_interfaces[i].name,
            &hw_joint_states_[j][i]
        ));
      }
    }
    // export sensor state interface
    for (uint s = 0; s < info_.sensors.size(); s++) {
      for (uint i = 0; i < info_.sensors[s].state_interfaces.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.sensors[s].name, info_.sensors[s].state_interfaces[i].name,
            &hw_sensor_states_[s][i]
        ));
      }
    }
    // export gpio state interface
    for (uint g = 0; g < info_.gpios.size(); g++) {
      for (uint i = 0; i < info_.gpios[g].state_interfaces.size(); i++) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.gpios[g].name, info_.gpios[g].state_interfaces[i].name,
            &hw_gpio_states_[g][i]
        ));
      }
    }
    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  EthercatDriver::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    // export joint command interface
    std::vector<double> test;
    for (uint j = 0; j < info_.joints.size(); j++) {
      for (uint i = 0; i < info_.joints[j].command_interfaces.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[j].name, info_.joints[j].command_interfaces[i].name,
            &hw_joint_commands_[j][i]
        ));
      }
    }
    // export sensor command interface
    for (uint s = 0; s < info_.sensors.size(); s++) {
      for (uint i = 0; i < info_.sensors[s].command_interfaces.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.sensors[s].name, info_.sensors[s].command_interfaces[i].name,
            &hw_sensor_commands_[s][i]
        ));
      }
    }
    // export gpio command interface
    for (uint g = 0; g < info_.gpios.size(); g++) {
      for (uint i = 0; i < info_.gpios[g].command_interfaces.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.gpios[g].name, info_.gpios[g].command_interfaces[i].name,
            &hw_gpio_commands_[g][i]
        ));
      }
    }
    return command_interfaces;
  }

  CallbackReturn EthercatDriver::on_activate(const rclcpp_lifecycle::State
                                                 & /*previous_state*/) {
    const std::lock_guard<std::mutex> lock(ec_mutex_);
    if (activated_) {
      RCLCPP_FATAL(rclcpp::get_logger("EthercatDriver"), "Double on_activate()");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"), "Starting ...please wait..."
    );
    if (info_.hardware_parameters.find("control_frequency") ==
        info_.hardware_parameters.end()) {
      control_frequency_ = 100;
    } else {
      control_frequency_ =
          std::stod(info_.hardware_parameters["control_frequency"]);
    }

    if (control_frequency_ < 0) {
      RCLCPP_FATAL(
          rclcpp::get_logger("EthercatDriver"), "Invalid control frequency!"
      );
      return CallbackReturn::ERROR;
    }

    // start EC and wait until state operative

    master_.setCtrlFrequency(control_frequency_);

    for (auto i = 0ul; i < ec_modules_.size(); i++) {
      RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"), "Module %li ", 
        i
    );
        RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"), "Adding slave %s at position %s", 
        ec_module_parameters_[i]["alias"].c_str(),
        ec_module_parameters_[i]["position"].c_str()
    );
      master_.addSlave(
          std::stod(ec_module_parameters_[i]["alias"]),
          std::stod(ec_module_parameters_[i]["position"]), ec_modules_[i].get()
      );

      RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"), "Added slave %s at position %s", 
        ec_module_parameters_[i]["alias"].c_str(),
        ec_module_parameters_[i]["position"].c_str()
    );
    }
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Configure SDO");

    // configure SDO
    for (auto i = 0ul; i < ec_modules_.size(); i++) {
      for (auto &sdo : ec_modules_[i]->sdo_config) {
        uint32_t abort_code;
        int ret = master_.configSlaveSdo(
            std::stod(ec_module_parameters_[i]["alias"]),
            std::stod(ec_module_parameters_[i]["position"]), sdo, &abort_code
        );
        if (ret) {
          RCLCPP_INFO(
              rclcpp::get_logger("EthercatDriver"),
              "Failed to download config SDO for module at position %s with "
              "Error: %d",
              ec_module_parameters_[i]["position"].c_str(), abort_code
          );
        }
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Activating _----");
    if (!master_.activate()) {
      RCLCPP_ERROR(rclcpp::get_logger("EthercatDriver"), "Activate EcMaster failed");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Activated EcMaster!");

    // start after one second
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    t.tv_sec++;

    bool running = true;
    while (running) {
      // wait until next shot
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
      // update EtherCAT bus
      RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Before updated!");
      master_.update();
      RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "updated!");

      // check if operational
      bool isAllInit = true;
      for (auto &module : ec_modules_) {
        isAllInit = isAllInit && module->initialized();
      }
      if (isAllInit) {
        running = false;
      }
      // calculate next shot. carry over nanoseconds into microseconds.
      t.tv_nsec += master_.getInterval();
      while (t.tv_nsec >= 1000000000) {
        t.tv_nsec -= 1000000000;
        t.tv_sec++;
      }
    }

    RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"), "System Successfully started!"
    );

    activated_ = true;

    return CallbackReturn::SUCCESS;
  }

  CallbackReturn EthercatDriver::on_deactivate(const rclcpp_lifecycle::State
                                                   & /*previous_state*/) {
    const std::lock_guard<std::mutex> lock(ec_mutex_);
    activated_ = false;
    RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"), "Stopping ...please wait..."
    );

    // stop EC and disconnect
    master_.stop();

    RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"), "System successfully stopped!"
    );

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type EthercatDriver::
      read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
    // try to lock so we can avoid blocking the read/write loop on the lock.
    const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
    if (lock.owns_lock() && activated_) {
      master_.readData();
    }
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type EthercatDriver::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
  ) {
    // try to lock so we can avoid blocking the read/write loop on the lock.
    const std::unique_lock<std::mutex> lock(ec_mutex_, std::try_to_lock);
    if (lock.owns_lock() && activated_) {
      master_.writeData();
    }
    return hardware_interface::return_type::OK;
  }

  std::vector<std::unordered_map<std::string, std::string>>
  EthercatDriver::getEcModuleParam(
      std::string &urdf, std::string component_name, std::string component_type
  ) {
    // Check if everything OK with URDF string
    if (urdf.empty()) {
      throw std::runtime_error("empty URDF passed to robot");
    }
    tinyxml2::XMLDocument doc;
    if (!doc.Parse(urdf.c_str()) && doc.Error()) {
      throw std::runtime_error("invalid URDF passed in to robot parser");
    }
    if (doc.Error()) {
      throw std::runtime_error("invalid URDF passed in to robot parser");
    }

    tinyxml2::XMLElement *robot_it = doc.RootElement();
    if (std::string("robot").compare(robot_it->Name())) {
      throw std::runtime_error("the robot tag is not root element in URDF");
    }

    const tinyxml2::XMLElement *ros2_control_it =
        robot_it->FirstChildElement("ros2_control");
    if (!ros2_control_it) {
      throw std::runtime_error("no ros2_control tag");
    }

    std::vector<std::unordered_map<std::string, std::string>> module_params;
    std::unordered_map<std::string, std::string> module_param;

    while (ros2_control_it) {
      const auto *ros2_control_child_it =
          ros2_control_it->FirstChildElement(component_type.c_str());
      while (ros2_control_child_it) {
        if (!component_name.compare(ros2_control_child_it->Attribute("name"))) {

          const auto *ec_module_it =
              ros2_control_child_it->FirstChildElement("ec_module");
          while (ec_module_it) {
            module_param.clear();
            std::string name = ec_module_it->Attribute("name");
            std::cout << "ec_module name: " << name.c_str() << std::endl;
            // Check if this ec_module is already registered. We doe not want the same name (with same parameter file)
            auto existing = std::find_if(
                ec_module_parameters_.begin(), ec_module_parameters_.end(),
                [&name](const auto &param) { return param.at("name") == name; }
            );
            std::cout << "Search for it" << std::endl;
            module_param["name"] = name;
            const auto *plugin_it = ec_module_it->FirstChildElement("plugin");
            if (NULL != plugin_it) {
              module_param["plugin"] = plugin_it->GetText();
            }
            const auto *param_it = ec_module_it->FirstChildElement("param");
            bool skip = false;
            while (param_it) {
              std::string param_name = param_it->Attribute("name");
              module_param[param_name] = param_it->GetText();
               if (!param_name.compare("slave_config") && existing != ec_module_parameters_.end()) {
                std::cout << "Found slave_config" << std::endl;
                if (existing->count("slave_config") > 0 && // Check if the key exists
                  existing->at("slave_config") != module_param["slave_config"]) {
                RCLCPP_WARN(
                    rclcpp::get_logger("EthercatDriver"),
                    "Duplicate parameter %s in module %s",
                    param_name.c_str(), name.c_str()
                );
                RCLCPP_WARN(
                    rclcpp::get_logger("EthercatDriver"),
                    "Combining parameter %s in module %s",
                    param_name.c_str(), name.c_str()
                );
                  } else {
                skip = true;
                }
              }
              param_it = param_it->NextSiblingElement("param");
            }
            if (skip) {
              module_params.push_back(*existing);
            } else {
              module_params.push_back(module_param);
              ec_module_parameters_.push_back(module_param);
              RCLCPP_INFO(
                rclcpp::get_logger("EthercatDriver"), "Adding to ec_module_parameters %s",
                component_name.c_str()
            );
            }
            ec_module_it = ec_module_it->NextSiblingElement("ec_module");
          }
        }
        ros2_control_child_it =
            ros2_control_child_it->NextSiblingElement(component_type.c_str());
      }
      ros2_control_it = ros2_control_it->NextSiblingElement("ros2_control");
    }

    return module_params;
  }

} // namespace ethercat_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    ethercat_driver::EthercatDriver, hardware_interface::SystemInterface
)
