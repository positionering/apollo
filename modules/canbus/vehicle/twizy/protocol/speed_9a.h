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
 * @file speed_9a.h
 * @brief the class of Speed9A (for twizy vehicle)
 */

#ifndef MODULES_CANBUS_VEHICL_TWIZY_PROTOCOL_SPEED_9A_H_
#define MODULES_CANBUS_VEHICL_TWIZY_PROTOCOL_SPEED_9A_H_

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
 * @class Speed9A
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Speed9A : public ::apollo::drivers::canbus::ProtocolData<
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

  Speed9A *set_ref_speed(double speed);
 
  void set_ref_speed_p(uint8_t *data, double speed);

 private:
  double speed_ = 0.0;

  };

}  // namespace twizy 
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_TWIZY_PROTOCOL_SPEED_9A_H_
