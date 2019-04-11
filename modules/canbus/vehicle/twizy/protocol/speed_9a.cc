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

#include "modules/canbus/vehicle/twizy/protocol/speed_9a.h"
#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace twizy {

using ::apollo::drivers::canbus::Byte;

const int32_t Speed9A::ID = 0x9A; //ID 154 på CAN i dec.

uint32_t Speed9A::GetPeriod() const {
  // receive rate??
  // receive timeout would trigger fault, letting en=0 and etc.
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Speed9A::UpdateData(uint8_t *data) {
  set_ref_speed_p(data, speed_);
}

void Speed9A::Reset() {
  speed_ = 0.0;
}

Speed9A *Speed9A::set_ref_speed(double speed) {
  speed_ = speed;
  return this;
}

// private

void Speed9A::set_ref_speed_p(uint8_t *data, double speed) {
  speed = ProtocolData::BoundedValue(0.0, 5.0, speed);

  char s = (char)((speed / 5.0) * 255);

  Byte frame_low(data + 0);
  frame_low.set_value(s, 0, 8);

  }


}  // namespace twizy
}  // namespace canbus
}  // namespace apollo
