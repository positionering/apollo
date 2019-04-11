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

#ifndef MODULES_CANBUS_VEHICLE_TWIZY_PROTOCOL_STEERINGANGLE_0C0H_C0_H_
#define MODULES_CANBUS_VEHICLE_TWIZY_PROTOCOL_STEERINGANGLE_0C0H_C0_H_

#include "modules/drivers/canbus/can_comm/protocol_data.h"
#include "modules/canbus/proto/chassis_detail.pb.h"

namespace apollo {
namespace canbus {
namespace twizy {

class Steeringangle0c0hc0 : public ::apollo::drivers::canbus::ProtocolData<
							::apollo::canbus::ChassisDetail> {
 public:
  static const int32_t ID;
  Steeringangle0c0hc0();
  void Parse(const std::uint8_t* bytes, int32_t length,
                     ChassisDetail* chassis) const;

 private:

  // config detail: {'name': 'SAS_CHCKSUM', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 36, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int sas_chcksum(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'MSG_COUNT', 'offset': 0.0, 'precision': 1.0, 'len': 4, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 32, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int msg_count(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'LWS_STAT', 'offset': 0.0, 'precision': 1.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 24, 'type': 'int', 'order': 'intel', 'physical_unit': ''}
  int lws_stat(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'LWS_Speed', 'offset': 0.0, 'precision': 4.0, 'len': 8, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 16, 'type': 'double', 'order': 'intel', 'physical_unit': '\xb0/s'}
  double lws_speed(const std::uint8_t* bytes, const int32_t length) const;

  // config detail: {'name': 'LWS', 'offset': 0.0, 'precision': 0.1, 'len': 16, 'is_signed_var': True, 'physical_range': '[-40|40]', 'bit': 0, 'type': 'double', 'order': 'intel', 'physical_unit': '\xb0'}
  double lws(const std::uint8_t* bytes, const int32_t length) const;
};

}  // namespace twizy
}  // namespace canbus
}  // namespace apollo

#endif  // MODULES_CANBUS_VEHICL_TWIZY_PROTOCOL_STEERINGANGLE_0C0H_C0_H_
