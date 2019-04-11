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

#include "modules/canbus/vehicle/twizy/protocol/steering_96.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace twizy {

using ::apollo::drivers::canbus::Byte;

const int32_t Steering96::ID = 0x96; //ID 150 på CAN i dec.

uint32_t Steering96::GetPeriod() const {
  // receive rate??
  // receive timeout would trigger fault, letting en=0 and etc.
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Steering96::UpdateData(uint8_t *data) {
  set_steering_angle_p(data, steering_angle_);
  set_steering_angle_speed_p(data, steering_angle_speed_);
}

void Steering96::Reset() {
  steering_angle_ = 0.0;
  steering_angle_speed_ = 0.0;
}

Steering96 *Steering96::set_steering_angle(double angle) {
  steering_angle_ = angle;
  return this;
}

Steering96 *Steering96::set_steering_angle_speed(double angle_speed) {
  steering_angle_speed_ = angle_speed;
  return this;
}

// private

// positive for left, negative for right
void Steering96::set_steering_angle_p(uint8_t *data, double angle) {
  angle = ProtocolData::BoundedValue(-39.5, 39.5, angle);
  
  Byte frame_high(data + 1);
  //  Left turn is positive(+) and right turn is negative(-). write 255 for -, 0 for +.
  if (angle < 0) {
    frame_high.set_value(0xFF);
    angle = -angle;
  } else {
    frame_high.set_value(0x00);   
  }

  char a = (char)((angle / 39.5) * 255);
  
  Byte frame_low(data + 0);
  frame_low.set_value(a, 0, 8);
  //frame_low.set_value(a, 3, 8); //flyttade inte i vilken position som can medelande läggs ):
}

void Steering96::set_steering_angle_speed_p(uint8_t *data, double angle_speed) {
  angle_speed = ProtocolData::BoundedValue(0.0, 500.0, angle_speed);
  int8_t x = (int8_t) (angle_speed / 2);

  Byte frame(data + 2);
  frame.set_value(x, 0, 8);
  //frame.set_value(x, 3, 8);//flyttade inte i vilken position som can medelande läggs ):
}

}  // namespace twizy
}  // namespace canbus
}  // namespace apollo
