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
 * @file steering_96.h
 * @brief the class of Steering96 (for twizy vehicle)
 */

#ifndef MODULES_CANBUS_VEHICL_TWIZY_PROTOCOL_STEERING_96_H_
#define MODULES_CANBUS_VEHICL_TWIZY_PROTOCOL_STEERING_96_H_

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
 * @class Steering96
 *
 * @brief one of the protocol data of lincoln vehicle
 */
class Steering96 : public ::apollo::drivers::canbus::ProtocolData<
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
   * @brief set steering angle
   * @return a this pointer to the instance itself
   */
  Steering96 *set_steering_angle(double angle);

  /**
   * @brief set steering angle speed
   * @return a this pointer to the instance itself
   */
  Steering96 *set_steering_angle_speed(double angle_speed);

 private:
 /** 
  * config detail: {'name': 'LWS', 'offset': 0.0, 'precision': 0.1,
  * 'len': 16, 'is_signed_var': True, 'physical_range': '[-40|40]',
  * 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit':
  * '\xb0'}
  */
  void set_steering_angle_p(uint8_t *data, double angle);

 /** 
  * config detail: {'name': 'LWS_Speed', 'offset': 0.0, 'precision': 4.0,
  * 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]',
  * 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit':
  * '\xb0/s'} 
  */

  void set_steering_angle_speed_p(uint8_t *data, double angle_speed);

  private:
  double steering_angle_ = 0.0;
  double steering_angle_speed_ = 0.0;
};

}  // namespace twizy 
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_TWIZY_PROTOCOL_STEERING_96_H_
