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

#ifndef ETHERCAT_INTERFACE__EC_SLAVE_HPP_
#define ETHERCAT_INTERFACE__EC_SLAVE_HPP_

#include <ecrt.h>
#include <map>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <cmath>
#include <string>

#include "ethercat_interface/ec_sdo_manager.hpp"
#include "ethercat_interface/ec_slave_base.hpp"

namespace ethercat_interface
{

/** Multi-joint slave plugin model.
 *
 * One EcSlave instance serves every joint/gpio/sensor whose <ec_module> has the
 * same name: setupSlave() receives the interface vectors of all of them, keyed
 * by component name, and the slave routes each PDO channel by its `for:` key.
 * The plugin provides the IgH sync/PDO/domain layout itself (syncs(),
 * channels(), domains()) and processData() receives the position of the entry
 * in its domain map.
 *
 * Plugins declare base_class_type="ethercat_interface::EcSlave" and are loaded
 * by the EthercatDriver through their own pluginlib loader; the master sees
 * them as an EcSlaveBase (bus/slave diagnostics, exception isolation, SDO
 * config, alias/position). */
class EcSlave : public EcSlaveBase
{
public:
  EcSlave(uint32_t vendor_id, uint32_t product_id)
  : sdo_config(sdo_config_)
  {
    vendor_id_ = vendor_id;
    product_id_ = product_id;
  }
  EcSlave(const EcSlave &) = delete;
  EcSlave & operator=(const EcSlave &) = delete;
  virtual ~EcSlave() {}

  /** read or write data to the domain; index = position in the domain map */
  virtual void processData(size_t /*index*/, uint8_t * /*domain_address*/) {}
  void process_data(int index, uint8_t * domain_address) override
  {
    processData(static_cast<size_t>(index), domain_address);
  }

  /** a pointer to syncs. return &syncs[0] */
  virtual const ec_sync_info_t * syncs() {return NULL;}
  bool initialized() override {return _initialized;}
  void set_state_is_operational(bool value) override
  {
    is_operational_ = value;
    if (value && !_initialized) {
      _initialized = value;
    }
  }
  /** Assign activate DC synchronization. return activate word*/
  int assign_activate_dc_sync() override {return 0x00;}
  /** number of elements in the syncs array. */
  virtual size_t syncSize() {return 0;}
  /** a pointer to all PDO entries */
  virtual const ec_pdo_entry_info_t * channels() {return NULL;}
  /** a map from domain index to pdo indices in that domain.
  *  map<domain index, vector<channels_ indices> > */
  typedef std::map<unsigned int, std::vector<unsigned int>> DomainMap;
  virtual void domains(DomainMap & /*domains*/) const {}
  virtual bool setupSlave(
    std::unordered_map<std::string, std::string> slave_paramters,
    std::unordered_map<std::string, std::vector<double> *> joint_state_interfaces,
    std::unordered_map<std::string, std::vector<double> *> joint_command_interfaces)
  {
    joint_state_interfaces_ = joint_state_interfaces;
    joint_command_interfaces_ = joint_command_interfaces;
    paramters_ = slave_paramters;
    return true;
  }

  using EcSlaveBase::vendor_id_;
  using EcSlaveBase::product_id_;

  /** config SDOs to download at startup (same vector as EcSlaveBase::sdo_config_) */
  std::vector<SdoConfigEntry> & sdo_config;

protected:
  std::unordered_map<std::string, std::vector<double> *> joint_state_interfaces_;
  std::unordered_map<std::string, std::vector<double> *> joint_command_interfaces_;
  std::unordered_map<std::string, std::string> paramters_;
  bool _initialized = false;
};
}  // namespace ethercat_interface
#endif  // ETHERCAT_INTERFACE__EC_SLAVE_HPP_
