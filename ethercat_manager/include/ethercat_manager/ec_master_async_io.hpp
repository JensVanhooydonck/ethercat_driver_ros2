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

#ifndef ETHERCAT_MANAGER__EC_MASTER_ASYNC_IO_HPP_
#define ETHERCAT_MANAGER__EC_MASTER_ASYNC_IO_HPP_

#define EC_IOCTL_TYPE 0xa4
#define EC_RATE_COUNT 3
#define EC_MAX_NUM_DEVICES 60
#define EC_MAX_SYNC_MANAGERS 16
#define EC_SYNC_SIGNAL_COUNT 2
#define EC_IO(nr)  _IO(EC_IOCTL_TYPE, nr)
#define EC_IOR(nr, type)  _IOR(EC_IOCTL_TYPE, nr, type)
#define EC_IOW(nr, type)  _IOW(EC_IOCTL_TYPE, nr, type)
#define EC_IOWR(nr, type)  _IOWR(EC_IOCTL_TYPE, nr, type)
#define EC_IOCTL_VERSION_MAGIC 37
#define EC_IOCTL_MODULE  EC_IOR(0x00, ec_ioctl_module_t)
#define EC_IOCTL_MASTER  EC_IOR(0x01, ec_ioctl_master_t)    
#define EC_IOCTL_CONFIG               EC_IOWR(0x19, ec_ioctl_config_t)
#define EC_IOCTL_SLAVE_SDO_UPLOAD  EC_IOWR(0x0e, ec_ioctl_slave_sdo_upload_t)
#define EC_IOCTL_SLAVE_SDO_DOWNLOAD  EC_IOWR(0x0f, ec_ioctl_slave_sdo_download_t)

typedef struct
{
  uint32_t ioctl_version_magic;
  uint32_t master_count;
} ec_ioctl_module_t;

typedef struct{
    uint32_t slave_count;
    uint32_t scan_index;
    uint32_t config_count;
    uint32_t domain_count;
    uint32_t eoe_handler_count;
    uint8_t phase;
    uint8_t active;
    uint8_t scan_busy;
    struct ec_ioctl_device {
        uint8_t address[6];
        uint8_t attached;
        uint8_t link_state;
        uint64_t tx_count;
        uint64_t rx_count;
        uint64_t tx_bytes;
        uint64_t rx_bytes;
        uint64_t tx_errors;
        int32_t tx_frame_rates[EC_RATE_COUNT];
        int32_t rx_frame_rates[EC_RATE_COUNT];
        int32_t tx_byte_rates[EC_RATE_COUNT];
        int32_t rx_byte_rates[EC_RATE_COUNT];
    } devices[EC_MAX_NUM_DEVICES];
    uint32_t num_devices;
    uint64_t tx_count;
    uint64_t rx_count;
    uint64_t tx_bytes;
    uint64_t rx_bytes;
    int32_t tx_frame_rates[EC_RATE_COUNT];
    int32_t rx_frame_rates[EC_RATE_COUNT];
    int32_t tx_byte_rates[EC_RATE_COUNT];
    int32_t rx_byte_rates[EC_RATE_COUNT];
    int32_t loss_rates[EC_RATE_COUNT];
    uint64_t app_time;
    uint64_t dc_ref_time;
    uint16_t ref_clock;
} ec_ioctl_master_t;

typedef struct
{
  // inputs
  uint16_t slave_position;
  uint16_t sdo_index;
  uint8_t sdo_entry_subindex;
  size_t target_size;
  uint8_t * target;

  // outputs
  size_t data_size;
  uint32_t abort_code;
} ec_ioctl_slave_sdo_upload_t;

typedef struct
{
  // inputs
  uint16_t slave_position;
  uint16_t sdo_index;
  uint8_t sdo_entry_subindex;
  uint8_t complete_access;
  size_t data_size;
  uint8_t * data;

  // outputs
  uint32_t abort_code;
} ec_ioctl_slave_sdo_download_t;

typedef struct {
    uint32_t cycle_time; /**< Cycle time [ns]. */
    int32_t shift_time; /**< Shift time [ns]. */
} ec_sync_signal_t;

typedef struct {
    // inputs
    uint32_t config_index;

    // outputs
    uint16_t alias;
    uint16_t position;
    uint32_t vendor_id;
    uint32_t product_code;
    struct {
        ec_direction_t dir;
        ec_watchdog_mode_t watchdog_mode;
        uint32_t pdo_count;
        uint8_t config_this;
    } syncs[EC_MAX_SYNC_MANAGERS];
    uint16_t watchdog_divider;
    uint16_t watchdog_intervals;
    uint32_t sdo_count;
    uint32_t idn_count;
    uint32_t flag_count;
    int32_t slave_position;
    uint16_t dc_assign_activate;
    ec_sync_signal_t dc_sync[EC_SYNC_SIGNAL_COUNT];
} ec_ioctl_config_t;

#endif  // ETHERCAT_MANAGER__EC_MASTER_ASYNC_IO_HPP_
