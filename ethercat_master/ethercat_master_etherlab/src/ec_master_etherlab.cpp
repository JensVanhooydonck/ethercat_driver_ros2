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

#include <unistd.h>
#include <sys/resource.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <time.h>
#include <sys/mman.h>
#include <string.h>
#include <iostream>
#include <sstream>
#include <cmath>

#include "ethercat_master/ec_master_etherlab.hpp"
#include "ethercat_interface/ec_master_base.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ethercat_master
{

DomainInfo::DomainInfo(ec_master_t *master)
{
  domain = ecrt_master_create_domain(master);
  if (domain == NULL) {
    EtherlabMaster::printWarning("Failed to create domain");
    return;
  }

  const ec_pdo_entry_reg_t empty = {0, 0, 0, 0, 0, 0, nullptr, nullptr};
  domain_regs.push_back(empty);
}

DomainInfo::~DomainInfo()
{
  for (Entry & entry : entries) {
    delete[] entry.offset;
    delete[] entry.bit_position;
  }
}

EtherlabMaster::EtherlabMaster()
{
  interval_ = 0;
}

EtherlabMaster::~EtherlabMaster()
{
  for (auto & domain : domain_info_) {
    if (domain.second != NULL) {
      delete domain.second;
    }
  }

  for (auto i = 0ul; i < slave_info_.size(); i++) {
    if (slave_info_[i].slave != nullptr) {
      slave_info_[i].slave.reset();
    }
  }

  // Release the EtherCAT master so the kernel module can be reused — without this the
  // master stays locked after the process exits, preventing re-initialization on restart
  // without unloading the kernel module (critical in multi-master setups).
  if (master_) {
    ecrt_release_master(master_);
  }
}

bool EtherlabMaster::init(std::string master_interface)
{
  master_ = ecrt_request_master(std::stoul(master_interface));
  if (master_ == NULL) {
    RCLCPP_FATAL(
          rclcpp::get_logger("EtherlabMaster"),
          "Failed to obtain master.");
    return false;
  }
  interval_ = 0;
  RCLCPP_INFO(
        rclcpp::get_logger("EtherlabMaster"),
        "master %lu ready.", std::stoul(master_interface));
  return true;
}

bool EtherlabMaster::checkSlaveIdentity(
  std::shared_ptr<ethercat_interface::EcSlaveBase> slave) const
{
  // Verify the drive actually on the bus matches what the slave_config declares (vendor +
  // product). The master would otherwise leave a mismatched slave unconfigured and it would
  // silently never reach OPERATIONAL; surface it loudly and refuse to configure the drive
  // instead. ecrt_master_get_slave() addresses by ABSOLUTE ring position, which equals
  // slave->get_position() only for alias 0 (for a non-zero alias, position is relative to
  // that alias), so the identity check is enforced for alias-0 slaves and skipped for
  // aliased ones.
  if (slave->get_alias() == 0) {
    ec_slave_info_t info{};
    if (ecrt_master_get_slave(master_, slave->get_position(), &info) != 0) {
      std::ostringstream msg;
      msg << "Add slave. No slave found at ring position " << slave->get_position() << std::hex
          << " (slave_config expects vendor=0x" << slave->get_vendor_id()
          << ", product=0x" << slave->get_product_id()
          << "). Refusing to configure this drive."
          << " Is the drive powered and connected on the bus?";
      printError(msg.str());
      return false;
    }
    if (info.vendor_id != slave->get_vendor_id() || info.product_code != slave->get_product_id()) {
      std::ostringstream msg;
      msg << "Add slave. Identity mismatch at ring position " << slave->get_position()
          << std::hex << ": the drive on the bus is vendor=0x" << info.vendor_id
          << ", product=0x" << info.product_code << " (revision 0x" << info.revision_number
          << ", \"" << info.name << "\"), but the slave_config expects vendor=0x"
          << slave->get_vendor_id() << ", product=0x" << slave->get_product_id()
          << ". Refusing to configure this drive — check the slave_config matches your hardware.";
      printError(msg.str());
      return false;
    }
  } else {
    std::ostringstream msg;
    msg << "Add slave at alias " << slave->get_alias() << " position " << slave->get_position()
        << ": skipping the vendor/product identity check (only enforced for alias-0 slaves "
      "addressed by absolute ring position).";
    printWarning(msg.str());
  }
  return true;
}

int EtherlabMaster::resolveAbsolutePosition(uint16_t alias, uint16_t position) const
{
  if (alias == 0) {
    return position;
  }
  ec_slave_info_t info;
  for (uint16_t ring = 0; ecrt_master_get_slave(master_, ring, &info) == 0; ++ring) {
    if (info.alias == alias) {
      return ring + position;
    }
  }
  return -1;
}

bool EtherlabMaster::checkSlaveSdoChecks(
  std::shared_ptr<ethercat_interface::EcSlaveBase> slave) const
{
  const int position = resolveAbsolutePosition(slave->get_alias(), slave->get_position());
  if (position < 0) {
    RCLCPP_ERROR(
          rclcpp::get_logger("EtherlabMaster"),
          "sdo_check: no slave found for alias %u on the bus.", slave->get_alias());
    return false;
  }
  for (auto & check : slave->get_sdo_check_config()) {
    uint8_t buffer[8] = {0};
    size_t result_size = 0;
    uint32_t abort_code = 0;
    const int ret = ecrt_master_sdo_upload(
          master_,
          position,
          check.index,
          check.sub_index,
          buffer,
          sizeof(buffer),
          &result_size,
          &abort_code);
    if (ret) {
      RCLCPP_ERROR(
            rclcpp::get_logger("EtherlabMaster"),
            "Failed to read check SDO 0x%04X:%u (%s) for module at position %i with "
            "Error: %d",
            check.index, check.sub_index,
            check.description.empty() ? "sdo_check" : check.description.c_str(),
            slave->get_position(),
            abort_code);
      return false;
    }
    if (result_size != check.data_size()) {
      RCLCPP_ERROR(
            rclcpp::get_logger("EtherlabMaster"),
            "Check SDO 0x%04X:%u (%s) for module at position %i returned %zu byte(s), "
            "expected %zu.",
            check.index, check.sub_index,
            check.description.empty() ? "sdo_check" : check.description.c_str(),
            slave->get_position(),
            result_size, check.data_size());
      return false;
    }
    if (!check.matches(buffer)) {
      std::ostringstream allowed;
      for (std::size_t v = 0; v < check.allowed_values.size(); ++v) {
        if (v) {allowed << ", ";}
        allowed << check.allowed_values[v];
      }
      RCLCPP_ERROR(
            rclcpp::get_logger("EtherlabMaster"),
            "Check SDO 0x%04X:%u (%s) for module at position %i read %ld, expected one "
            "of [%s]. The drive is not commissioned as this slave_config requires.",
            check.index, check.sub_index,
            check.description.empty() ? "sdo_check" : check.description.c_str(),
            slave->get_position(),
            check.decode(buffer), allowed.str().c_str());
      return false;
    }
  }
  return true;
}

bool EtherlabMaster::check_slave(std::shared_ptr<ethercat_interface::EcSlaveBase> slave)
{
  if (false == slave->isAliasAndPositionSet()) {
    std::string error_message = "Alias and position not set for slave (vendor id=" +
      std::to_string(slave->get_vendor_id()) + ",product_code=" +
      std::to_string(slave->get_product_id()) + ").";
    throw std::runtime_error(error_message);
  }
  return checkSlaveIdentity(slave) && checkSlaveSdoChecks(slave);
}

bool EtherlabMaster::add_slave(std::shared_ptr<ethercat_interface::EcSlaveBase> slave)
{
  if (false == slave->isAliasAndPositionSet()) {
    std::string error_message = "Alias and position not set for slave (vendor id=" +
      std::to_string(slave->get_vendor_id()) + ",product_code=" +
      std::to_string(slave->get_product_id()) + ").";
    throw std::runtime_error(error_message);
  }

  if (!checkSlaveIdentity(slave)) {
    return false;
  }

  // configure slave in master
  SlaveInfo slave_info;

  // slave_info_.emplace_back();

  slave_info.slave = std::make_shared<EtherlabSlave>(slave);
  slave_info.config = ecrt_master_slave_config(
        master_,
        slave->get_alias(),
        slave->get_position(),
        slave->get_vendor_id(),
        slave->get_product_id());
  if (slave_info.config == NULL) {
    printError("Add slave. Failed to get slave configuration.");
    return false;
  }

    // check and setup dc

  if (slave_info.slave->assign_activate_dc_sync()) {
    // Same shift for every slave: IgH starts SYNC0 in phase with dc_ref_time + shift,
    // so all slaves share one SYNC0 phase. write_process_data() keeps the send
    // phase half a cycle away from it (lockSendPhase()).
    has_dc_slaves_ = true;
    sync0_shift_ns_ = interval_ / 2;
    ecrt_slave_config_dc(
          slave_info.config,
          slave_info.slave->assign_activate_dc_sync(),
          interval_,
          sync0_shift_ns_,
          0,
          0);
  }

  slave_info_.push_back(slave_info);

    // check if slave has pdos
  size_t num_syncs = slave_info.slave->sync_size();
  const ec_sync_info_t *syncs = slave_info.slave->syncs();
  if (num_syncs > 0) {
      // configure pdos in slave
    int pdos_status = ecrt_slave_config_pdos(slave_info.config, num_syncs, syncs);
    if (pdos_status) {
      printError("Add slave. Failed to configure PDOs");
      return false;
    }
  } else {
    printWarning(
          "Add slave. Sync size is zero for " +
          std::to_string(slave->get_alias()) + ":" +
          std::to_string(slave->get_position()));
  }

    // check if slave registered any pdos for the domain
  EtherlabSlave::DomainMap domain_map;
  slave_info.slave->domains(domain_map);
  for (auto & iter : domain_map) {
      // get the domain info, create if necessary
    uint32_t domain_index = iter.first;
    DomainInfo *domain = NULL;
    if (domain_info_.count(domain_index)) {
      domain = domain_info_.at(domain_index);
    }
    if (domain == NULL) {
      domain = new DomainInfo(master_);
      domain_info_[domain_index] = domain;
    }
    registerPDOInDomain(
          iter.second, domain,
          slave_info.slave);
  }

  return true;
}

bool EtherlabMaster::configure_slaves()
{
  for (auto i = 0ul; i < slave_info_.size(); i++) {
    for (auto & sdo : slave_info_[i].slave->get_slave()->get_sdo_config()) {
      uint8_t buffer[8];
      sdo.buffer_write(buffer);
      int ret = ecrt_slave_config_sdo(
            slave_info_[i].config,
            sdo.index,
            sdo.sub_index,
            buffer,
            sdo.data_size());

      if (ret) {
        RCLCPP_FATAL(
              rclcpp::get_logger("EtherlabMaster"),
              "Failed to download config SDO for module at position %i with Error: %d",
              slave_info_[i].slave->get_slave()->get_position(),
              ret);
        return false;
      }
    }
  }

  return true;
}

int EtherlabMaster::upload_slave_sdo(
  uint16_t slave_position, uint16_t index, uint8_t sub_index,
  uint8_t * target, size_t target_size, size_t * result_size, uint32_t * abort_code,
  uint16_t alias)
{
  if (activated_) {
    printError(
      "Upload slave SDO. Refusing: this is a blocking mailbox round-trip and must only be "
      "called during the configure phase (after configure_slaves(), before start()), never "
      "while the master is activated — it would stall the real-time cycle.");
    return -1;
  }
  const int position = resolveAbsolutePosition(alias, slave_position);
  if (position < 0) {
    printError("Upload slave SDO. No slave found for alias " + std::to_string(alias) + ".");
    return -1;
  }
  return ecrt_master_sdo_upload(
    master_, position, index, sub_index, target, target_size, result_size, abort_code);
}

ethercat_interface::EcMasterStateInfo EtherlabMaster::get_master_state() const
{
  ethercat_interface::EcMasterStateInfo info;
  info.link_up = master_state_.link_up;
  info.slaves_responding = master_state_.slaves_responding;
  info.al_states = static_cast<uint8_t>(master_state_.al_states);
  return info;
}

ethercat_interface::EcDomainStateInfo EtherlabMaster::get_domain_state(uint32_t domain) const
{
  ethercat_interface::EcDomainStateInfo info;
  const DomainInfo * domain_info = domain_info_.at(domain);
  info.working_counter = domain_info->domain_state.working_counter;
  info.wc_state = static_cast<uint8_t>(domain_info->domain_state.wc_state);
  return info;
}

std::vector<ethercat_interface::EcSlaveStateInfo> EtherlabMaster::get_slave_states() const
{
  std::vector<ethercat_interface::EcSlaveStateInfo> out;
  out.reserve(slave_info_.size());
  for (const SlaveInfo & s : slave_info_) {
    ethercat_interface::EcSlaveStateInfo info;
    info.alias = s.slave->get_slave()->get_alias();
    info.position = s.slave->get_slave()->get_position();
    info.al_state = s.config_state.al_state;
    info.online = s.config_state.online;
    info.operational = s.config_state.operational;
    out.push_back(info);
  }
  return out;
}

  /*int EtherlabMaster::configure_slave(uint16_t slave_position, ethercat_interface::SdoConfigEntry sdo_config, uint32_t *abort_code)
  {
    uint8_t buffer[8];
    sdo_config.buffer_write(buffer);
    int ret = ecrt_master_sdo_download(
      master_,
      slave_position,
      sdo_config.index,
      sdo_config.sub_index,
      buffer,
      sdo_config.data_size(),
      abort_code
    );
    return ret;
  }*/

void EtherlabMaster::registerPDOInDomain(
  std::vector<uint32_t> & channel_indices,
  DomainInfo *domain_info,
  std::shared_ptr<EtherlabSlave> slave)
{
    // expand the size of the domain
  uint32_t num_pdo_regs = channel_indices.size();
  size_t start_index = domain_info->domain_regs.size() - 1;   // empty element at end
  domain_info->domain_regs.resize(domain_info->domain_regs.size() + num_pdo_regs);

    // create a new entry in the domain
  DomainInfo::Entry domain_entry;
  domain_entry.slave = slave;
  domain_entry.num_pdos = num_pdo_regs;
  domain_entry.offset = new uint32_t[num_pdo_regs];
  domain_entry.bit_position = new uint32_t[num_pdo_regs];
  domain_info->entries.push_back(domain_entry);

    // EtherlabSlave::DomainMap domain_map;
    // slave->domains(domain_map);

    // add to array of pdos registrations
  const ec_pdo_entry_info_t *pdo_regs = slave->channels();
  for (size_t i = 0; i < num_pdo_regs; ++i) {
      // create pdo entry in the domain
    ec_pdo_entry_reg_t & pdo_reg = domain_info->domain_regs[start_index + i];
    pdo_reg.alias = slave->get_slave()->get_alias();
    pdo_reg.position = slave->get_slave()->get_position();
    pdo_reg.vendor_id = slave->get_slave()->get_vendor_id();
    pdo_reg.product_code = slave->get_slave()->get_product_id();
    pdo_reg.index = pdo_regs[channel_indices[i]].index;
    pdo_reg.subindex = pdo_regs[channel_indices[i]].subindex;
    pdo_reg.offset = &(domain_entry.offset[i]);
    pdo_reg.bit_position = &(domain_entry.bit_position[i]);

      // print the domain pdo entry
    RCLCPP_INFO(
          rclcpp::get_logger("EthercatDriver"),
          "{ %d, %d, 0x%x, 0x%x, 0x%x, 0x%x }",
          pdo_reg.alias,
          pdo_reg.position,
          pdo_reg.vendor_id,
          pdo_reg.product_code,
          pdo_reg.index,
          static_cast<int>(pdo_reg.subindex));
  }

    // set the last element to null
  ec_pdo_entry_reg_t empty = {0, 0, 0, 0, 0, 0, nullptr, nullptr};
  domain_info->domain_regs.back() = empty;
}

bool EtherlabMaster::start()
{
    // register domain
  for (auto & iter : domain_info_) {
    DomainInfo * domain_info = iter.second;
    if (domain_info == NULL) {
      throw std::runtime_error("Null domain info: " + std::to_string(iter.first));
    }
    bool domain_status = ecrt_domain_reg_pdo_entry_list(
        domain_info->domain,
        &(domain_info->domain_regs[0]));
    if (domain_status) {
      printWarning("Start. Failed to register domain PDO entries.");
      return false;
    }
  }
  //  set application time
  //  struct timespec t;
  //  clock_gettime(CLOCK_MONOTONIC, &t);
  //  ecrt_master_application_time(master_, EC_NEWTIMEVAL2NANO(t));

  // activate master
  bool activate_status = ecrt_master_activate(master_);
  if (activate_status) {
    printWarning("Start. Failed to activate ecat master.");
    return false;
  }
  // From here on the master is cyclically exchanging process data: blocking mailbox SDO
  // calls (upload_slave_sdo) are no longer safe, regardless of whether the rest of start()
  // below succeeds.
  activated_ = true;

  // retrieve domain data
  for (auto & iter : domain_info_) {
    DomainInfo * domain_info = iter.second;
    if (domain_info == NULL) {
      throw std::runtime_error("Null domain info: " + std::to_string(iter.first));
    }

    domain_info->domain_pd = ecrt_domain_data(domain_info->domain);
    if (domain_info->domain_pd == NULL) {
      printWarning("Activate. Failed to retrieve domain process data.");
      return false;
    }
  }
  return true;
}


  /** stop the control loop.
   */
bool EtherlabMaster::stop()
{
  activated_ = false;
  return true;
}

bool EtherlabMaster::deactivate()
{
  if (master_ == NULL) {
    printError("Deactivate. Master not obtained.");
    return false;
  }

  int ret = ecrt_master_deactivate(master_);
  if (ret != 0) {
    printWarning("Deactivate. ecrt_master_deactivate() failed with code " + std::to_string(ret));
    return false;
  }
  resetDcPhase();

  // ecrt_master_deactivate() frees everything created by ecrt_master_create_domain() /
  // ecrt_master_slave_config() / ecrt_domain_data(); drop everything that referenced those
  // objects so a subsequent add_slave()/registerTransferInDomain() cycle starts clean instead
  // of appending onto or dereferencing stale entries.
  for (auto & domain : domain_info_) {
    delete domain.second;
  }
  domain_info_.clear();
  slave_info_.clear();
  transfers_.clear();

  return true;
}

bool EtherlabMaster::read_process_data()
{
  uint32_t domain = 0;

  // receive process data
  ecrt_master_receive(master_);

  // Margin of the frame sent by the previous write_process_data(): the reference
  // clock's DC time when that frame passed it (lower 32 bit, widened around the
  // application time sent with it) -> time until the next SYNC0.
  uint32_t ref_time32;
  if (has_dc_slaves_ && has_dc_ref_time_ &&
    ecrt_master_reference_clock_time(master_, &ref_time32) == 0)
  {
    const int32_t diff = static_cast<int32_t>(
      ref_time32 - static_cast<uint32_t>(last_app_time_));
    const uint64_t phase = sync0Phase(last_app_time_ + diff);
    const int64_t margin = phase ? interval_ - phase : 0;
    if (margin_samples_ == 0 || margin < margin_min_ns_) {margin_min_ns_ = margin;}
    if (margin_samples_ == 0 || margin > margin_max_ns_) {margin_max_ns_ = margin;}
    margin_sum_ns_ += margin;
    ++margin_samples_;
  }

  DomainInfo * domain_info = domain_info_.at(domain);
  if (domain_info == NULL) {
    throw std::runtime_error("Null domain info: " + std::to_string(domain));
  }

  ecrt_domain_process(domain_info->domain);

  // Transfer data if configured
  // TODO(@yguel) make transfer per domain ? Quid of transfers across domains ?
  transferAll();

  // check process data state (optional)
  checkDomainState(domain);

  // check for master and slave state change
  if (update_counter_ % check_state_frequency_ == 0) {
    checkMasterState();
    checkSlaveStates();
  }

  // read and write process data
  EtherlabSlave::DomainMap domain_map;
  std::vector<unsigned int> domain_map_;
  for (DomainInfo::Entry & entry : domain_info->entries) {
    std::shared_ptr<ethercat_interface::EcSlaveBase> slave = entry.slave->get_slave();
    entry.slave->domains(domain_map);
    domain_map_ = domain_map.at(0);

    // processDataSafe()/updateStateSafe() isolate a slave plugin's exception so it can't abort
    // the bus cycle for the other slaves; see EcSlaveBase.
    bool ok = true;
    for (auto i = 0; ok && i < entry.num_pdos; ++i) {
      auto index = domain_map_[i];
      ok = slave->processDataSafe(index, domain_info->domain_pd + entry.offset[i]);
    }
    if (ok) {
      ok = slave->updateStateSafe();
    }
    if (ok) {
      slave->clearProcessDataFault();
    }
  }

  ++update_counter_;
  return true;
}
bool EtherlabMaster::write_process_data()
{
  uint32_t domain = 0;
  DomainInfo * domain_info = domain_info_.at(domain);
  if (domain_info == NULL) {
    throw std::runtime_error("Null domain info: " + std::to_string(domain));
  }

  // read and write process data
  EtherlabSlave::DomainMap domain_map;
  std::vector<unsigned int> domain_map_;
  for (DomainInfo::Entry & entry : domain_info->entries) {
    std::shared_ptr<ethercat_interface::EcSlaveBase> slave = entry.slave->get_slave();
    entry.slave->domains(domain_map);
    domain_map_ = domain_map.at(0);

    // Same isolation as read_process_data(): one slave's exception must not stop the rest of
    // the bus from getting its commands written.
    bool ok = true;
    for (auto i = 0; ok && i < entry.num_pdos; ++i) {
      auto index = domain_map_[i];
      ok = slave->processDataSafe(index, domain_info->domain_pd + entry.offset[i]);
    }
    if (ok) {
      slave->clearProcessDataFault();
    }
  }

  const uint64_t now = monotonicNs();
  if (has_dc_slaves_) {
    lockSendPhase(now);
  }
  setApplicationTime(now);
  ecrt_master_sync_reference_clock(master_);
  ecrt_master_sync_slave_clocks(master_);

  // send process data
  ecrt_domain_queue(domain_info->domain);
  ecrt_master_send(master_);

  reportSync0Margin();
  return true;
}

uint64_t EtherlabMaster::monotonicNs()
{
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return static_cast<uint64_t>(t.tv_sec) * 1000000000ULL + t.tv_nsec;
}

void EtherlabMaster::setApplicationTime(uint64_t now)
{
  const uint64_t app_time = now + app_time_offset_ns_;
  ecrt_master_application_time(master_, app_time);
  if (!has_dc_ref_time_) {
    dc_ref_time_ = app_time;  // IgH takes the first application time as dc_ref_time
    has_dc_ref_time_ = true;
  }
  last_app_time_ = app_time;
}

uint64_t EtherlabMaster::sync0Phase(uint64_t app_time) const
{
  if (interval_ == 0) {return 0;}
  int64_t r = static_cast<int64_t>(app_time - dc_ref_time_ - sync0_shift_ns_) %
    static_cast<int64_t>(interval_);
  if (r < 0) {r += interval_;}
  return static_cast<uint64_t>(r);
}

void EtherlabMaster::lockSendPhase(uint64_t now)
{
  // The slaves latch the outputs at SYNC0. If the frame arrives close to SYNC0,
  // send-time jitter makes them alternately latch a stale and a fresh setpoint
  // (CSP motion becomes choppy). SYNC0 is fixed relative to dc_ref_time, but the
  // phase at which the caller sends is not: ros2_control_node runs its loop on
  // its own CLOCK_MONOTONIC period grid (fixed phase, overruns skip whole periods)
  // that starts at an arbitrary instant, and the settle loop before it has yet
  // another phase. So: track the send phase and, when it has been coherent but
  // more than a quarter cycle off the ideal (SYNC0 half a cycle after the send)
  // for kPhaseSamples cycles, step the application time once to re-centre it.
  // The DC clocks follow the application time and slew to the step in hardware;
  // the step is at most interval/2, in the direction that moves SYNC0 away from
  // the send. A drifting send phase (e.g. a sleep_for() loop) is not coherent and
  // never triggers a step.
  if (!has_dc_ref_time_ || interval_ == 0) {
    return;
  }
  const int64_t half = interval_ / 2;
  int64_t err = static_cast<int64_t>(sync0Phase(now + app_time_offset_ns_)) - half;
  if (err <= -half) {err += interval_;}  // (-half, half], 0 = SYNC0 half a cycle after send
  const double angle = 2.0 * M_PI * static_cast<double>(err) / interval_;
  const double alpha = 1.0 / kPhaseSamples;
  if (phase_samples_ == 0) {
    phase_mean_cos_ = std::cos(angle);
    phase_mean_sin_ = std::sin(angle);
  } else {
    phase_mean_cos_ += alpha * (std::cos(angle) - phase_mean_cos_);
    phase_mean_sin_ += alpha * (std::sin(angle) - phase_mean_sin_);
  }
  if (phase_samples_ < kPhaseSamples) {
    ++phase_samples_;
    return;
  }
  const double coherence = std::hypot(phase_mean_cos_, phase_mean_sin_);
  const int64_t mean_err = static_cast<int64_t>(std::llround(
      std::atan2(phase_mean_sin_, phase_mean_cos_) / (2.0 * M_PI) * interval_));
  if (coherence < 0.5 || std::llabs(mean_err) <= half / 2) {
    phase_off_target_ = 0;
    return;
  }
  if (++phase_off_target_ < kPhaseSamples) {
    return;
  }
  app_time_offset_ns_ -= mean_err;
  last_step_ns_ = -mean_err;
  ++phase_steps_;
  phase_samples_ = 0;
  phase_off_target_ = 0;
}

void EtherlabMaster::reportSync0Margin()
{
  // Where the command frames land relative to SYNC0: ~interval/2 is ideal; close
  // to 0 or to interval, the slaves alternately latch a stale and a fresh setpoint.
  if (!has_dc_slaves_ || interval_ == 0 ||
    ++margin_report_cycles_ < 10000000000ULL / interval_)
  {
    return;
  }
  margin_report_cycles_ = 0;
  if (margin_samples_ == 0) {
    return;
  }
  const int64_t mean = margin_sum_ns_ / margin_samples_;
  const int64_t guard = 500000;  // 0.5 ms
  const bool bad = margin_min_ns_ < guard || margin_max_ns_ > interval_ - guard;
  const bool startup = margin_reports_++ < 6;
  if (bad || startup) {
    RCLCPP_WARN(
      rclcpp::get_logger("EthercatDriver"),
      "SYNC0 margin mean %.3f ms [%.3f .. %.3f] of %.3f ms cycle (%u samples, "
      "%u phase step(s), last %.3f ms)%s",
      mean / 1e6, margin_min_ns_ / 1e6, margin_max_ns_ / 1e6, interval_ / 1e6,
      margin_samples_, phase_steps_, last_step_ns_ / 1e6,
      bad ? " -- frames arrive close to SYNC0, drives may move choppy" : "");
  } else {
    RCLCPP_INFO(
      rclcpp::get_logger("EthercatDriver"),
      "SYNC0 margin mean %.3f ms [%.3f .. %.3f] of %.3f ms cycle (%u samples)",
      mean / 1e6, margin_min_ns_ / 1e6, margin_max_ns_ / 1e6, interval_ / 1e6,
      margin_samples_);
  }
  margin_samples_ = 0;
  margin_sum_ns_ = 0;
}

void EtherlabMaster::resetDcPhase()
{
  has_dc_ref_time_ = false;
  has_dc_slaves_ = false;
  app_time_offset_ns_ = 0;
  phase_samples_ = 0;
  phase_off_target_ = 0;
  margin_samples_ = 0;
  margin_sum_ns_ = 0;
}

bool EtherlabMaster::reset()
{
  return true;
}

void EtherlabMaster::checkDomainState(uint32_t domain)
{
  DomainInfo * domain_info = domain_info_.at(domain);
  if (domain_info == NULL) {
    throw std::runtime_error("Null domain info: " + std::to_string(domain));
  }

  ec_domain_state_t ds;
  ecrt_domain_state(domain_info->domain, &ds);

  if (ds.working_counter != domain_info->domain_state.working_counter) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Domain: WC %d.", ds.working_counter);
  }
  if (ds.wc_state != domain_info->domain_state.wc_state) {
    RCLCPP_INFO(
        rclcpp::get_logger("EthercatDriver"),
        "Domain: State %s.",
        ds.wc_state == EC_WC_ZERO ? "ZERO" :
      (
        (ds.wc_state == EC_WC_INCOMPLETE) ? "INCOMPLETE" :
        (ds.wc_state == EC_WC_COMPLETE) ? "COMPLETE" : "UNKNOWN"
      )
    );
  }
  domain_info->domain_state = ds;
}

void EtherlabMaster::checkMasterState()
{
  ec_master_state_t ms;
  ecrt_master_state(master_, &ms);

  if (ms.slaves_responding != master_state_.slaves_responding) {
    RCLCPP_WARN(rclcpp::get_logger("EthercatDriver"), "%d slave(s).", ms.slaves_responding);
  }
  if (ms.al_states != master_state_.al_states) {
    RCLCPP_WARN(rclcpp::get_logger("EthercatDriver"), "Master AL states: 0x%02X.", ms.al_states);
  }
  if (ms.link_up != master_state_.link_up) {
    RCLCPP_WARN(rclcpp::get_logger("EthercatDriver"), "Link is %s.", ms.link_up ? "up" : "down");
  }
  master_state_ = ms;
}

void EtherlabMaster::checkSlaveStates()
{
  for (SlaveInfo & slave : slave_info_) {
    ec_slave_config_state_t s;
    ecrt_slave_config_state(slave.config, &s);

    if (s.al_state != slave.config_state.al_state) {
        // this spams the terminal at initialization.
      RCLCPP_WARN(rclcpp::get_logger("EthercatDriver"), "Slave: State 0x%02X.", s.al_state);
    }
    if (s.online != slave.config_state.online) {
      RCLCPP_WARN(
          rclcpp::get_logger(
            "EthercatDriver"), "Slave: %s.", s.online ? "online" : "offline");
    }
    if (s.operational != slave.config_state.operational) {
      RCLCPP_WARN(
          rclcpp::get_logger("EthercatDriver"),
          "Slave: (alias: %d, pos: %d, vendor_id: %d, prod_id: %d) --> %soperational.",
          slave.slave->get_slave()->get_alias(),
          slave.slave->get_slave()->get_position(),
          slave.slave->get_slave()->get_vendor_id(),
          slave.slave->get_slave()->get_product_id(),
          s.operational ? "" : "NOT ");
      slave.slave->get_slave()->set_state_is_operational(s.operational ? true : false);
    }
    slave.config_state = s;
  }
}

  /*void EtherlabMaster::checkDomainInfoValidity(
    const DomainInfo & domain_info,
    const ec_pdo_entry_reg_t & pdo_entry_reg)
  {
    if (nullptr == domain_info.domain_pd) {
      throw std::runtime_error("Domain process data pointer not set.");
    }
    if (nullptr == pdo_entry_reg.offset) {
      throw std::runtime_error("Offset not set in pdo_entry_reg.");
    }
  }*/

void EtherlabMaster::registerTransferInDomain(
  const std::vector<ethercat_interface::EcTransferNet> & transfer_nets)
{
  // Fill in the EcTransferInfo structures

  // For each transfer of each net,
  for (auto & net : transfer_nets) {
    for (auto & transfer : net.transfers) {
      ethercat_interface::EcTransferInfo transfer_info;
      transfer_info.size = transfer.size;
      RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "Transfer size: %ld", transfer.size);
        /**
         * For the input and the output of the transfer find
         *   1. the process domain data pointer
         *   2. the offset in the process domain data
         * By iterating over the existing DomainInfo and domain_regs vector
         * to find the ec_pdo_entry_reg_t whose alias, position, index and subindex
         * match the transfer input and output memory entries
         * */
      for (const auto & key_val : domain_info_) {
        const DomainInfo & domain = *(key_val.second);
        for (auto & domain_reg : domain.domain_regs) {
            // Find match for input
          if (domain_reg.alias == transfer.input.alias &&
            domain_reg.position == transfer.input.position &&
            domain_reg.index == transfer.input.index &&
            domain_reg.subindex == transfer.input.subindex)
          {
            transfer_info.input_domain = reinterpret_cast<const void *>(&domain);
              // 3. Compute the pointer arithmetic and store the result in the EcTransferInfo object
            transfer_info.in_ptr = domain.domain_pd + *(domain_reg.offset);
            RCLCPP_INFO(
                rclcpp::get_logger("EthercatDriver"),
                "Transfer input:  esclave position: %d / index: 0x%x / in offset:  %d",
                domain_reg.position,
                domain_reg.index,
                *(domain_reg.offset)
            );
          }
            // Find match for output
          if (domain_reg.alias == transfer.output.alias &&
            domain_reg.position == transfer.output.position &&
            domain_reg.index == transfer.output.index &&
            domain_reg.subindex == transfer.output.subindex)
          {
            transfer_info.output_domain = reinterpret_cast<const void *>(&domain);
              // 3. Compute the pointer arithmetic and store the result in the EcTransferInfo object
            transfer_info.out_ptr = domain.domain_pd + *(domain_reg.offset);
            RCLCPP_INFO(
                rclcpp::get_logger("EthercatDriver"),
                "Transfer output: slave position: %d / index: 0x%x / out offset: %d",
                domain_reg.position,
                domain_reg.index,
                *(domain_reg.offset)
            );
          }
        }
      }

        // Record the transfer
      transfers_.push_back(transfer_info);
    }
  }
}

void EtherlabMaster::transferAll()
{
    // Proceed to the transfer of all the data declared in transfers_.
  for (auto & transfer : transfers_) {
      // Copy the data from the input to the output
    memcpy(transfer.out_ptr, transfer.in_ptr, transfer.size);
  }
}

}  // namespace ethercat_master
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(ethercat_master::EtherlabMaster, ethercat_interface::EcMasterBase)
