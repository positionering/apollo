/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file gear_98.h
 * @brief the class of Gear98 (for twizy vehicle)
 */

#ifndef MODULES_CANBUS_VEHICLE_TWIZY_PROTOCOL_GEAR_98_H_
#define MODULES_CANBUS_VEHICLE_TWIZY_PROTOCOL_GEAR_98_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

/**
 * @namespace apollo::canbus::twizy
 * @brief apollo::canbus::twizy
 */
namespace apollo {
namespace canbus {
namespace twizy {

/**
 * @class Gear98
 *
 * @brief one of the protocol data of twizy vehicle
 */
class Gear98 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;

  /**
   * @brief get the data period
   * @return the value of data period
   */
  virtual uint32_t GetPeriod() const;

  /**
   * @brief update the data
   * @param data a pointer to the data to be updated
   */
  virtual void UpdateData(uint8_t *data);

  /**
   * @brief reset the private variables
   */
  virtual void Reset();

  /**
   * @brief set gear to none
   * @return a this pointer to the instance itself
   */
  Gear98 *set_gear_none();

  /**
   * @brief set gear to reverse
   * @return a this pointer to the instance itself
   */
  Gear98 *set_gear_reverse();

  /**
   * @brief set gear to neutral
   * @return a this pointer to the instance itself
   */
  Gear98 *set_gear_neutral();

  /**
   * @brief set gear to drive
   * @return a this pointer to the instance itself
   */
  Gear98 *set_gear_drive();

  Gear98 *set_brake_pedalstatus(bool pedal);

 private:
  /**
   * @brief set gear position
   * config detail: {'name': 'gcmd', 'offset': 0.0, 'precision': 1.0, 'len': 3,
   * 'f_type': 'value', 'is_signed_var': False, 'physical_range': '[0|0]',
   * 'bit': 0, 'type': 'int', 'order': 'intel', 'physical_unit': '""'}
   */
  void set_gear_p(uint8_t *data, uint8_t gear);

  void set_brake_pedalstatus_p(uint8_t *data, uint8_t pedal);
  
 private:
  uint8_t gear_ = 0;
  uint8_t pedal_ = 0;
  bool update_ = false;
};

}  // namespace twizy 
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICLE_TWIZY_PROTOCOL_GEAR_98_H_
