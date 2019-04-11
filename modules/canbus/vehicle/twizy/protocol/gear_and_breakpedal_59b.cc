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

#include "modules/canbus/vehicle/twizy/protocol/gear_and_breakpedal_59b.h"


#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace twizy {

using ::apollo::drivers::canbus::Byte;

Gearandbreakpedal59b::Gearandbreakpedal59b() {}
const int32_t Gearandbreakpedal59b::ID = 0x59B;

void Gearandbreakpedal59b::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_twizy()->mutable_gear_and_pedal()->set_brake_pedalstatus(brake_pedalstatus(bytes, length));
  chassis->mutable_twizy()->mutable_gear_and_pedal()->set_gear_r(gear_r(bytes, length));
  chassis->mutable_twizy()->mutable_gear_and_pedal()->set_gear_n(gear_n(bytes, length));
  chassis->mutable_twizy()->mutable_gear_and_pedal()->set_gear_d(gear_d(bytes, length));
}

// config detail: {'name': 'brake_pedalstatus', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 8, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Gearandbreakpedal59b::brake_pedalstatus(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'gear_r', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 5, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Gearandbreakpedal59b::gear_r(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(5, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'gear_n', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 3, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Gearandbreakpedal59b::gear_n(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(3, 1);

  bool ret = x;
  return ret;
}

// config detail: {'name': 'gear_d', 'offset': 0.0, 'precision': 1.0, 'len': 1, 'is_signed_var': False, 'physical_range': '[0|0]', 'bit': 1, 'type': 'bool', 'order': 'intel', 'physical_unit': ''}
bool Gearandbreakpedal59b::gear_d(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(1, 1);

  bool ret = x;
  return ret;
}
}  // namespace twizy
}  // namespace canbus
}  // namespace apollo
