/* Copyright 2017 The Apollo Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

#include "modules/canbus/vehicle/twizy/twizy_message_manager.h"
#include "modules/canbus/vehicle/twizy/protocol/steering_96.h"
#include "modules/canbus/vehicle/twizy/protocol/gear_98.h"
#include "modules/canbus/vehicle/twizy/protocol/speed_9a.h"
#include "modules/canbus/vehicle/twizy/protocol/currentspeed_599.h"
#include "modules/canbus/vehicle/twizy/protocol/gear_and_breakpedal_59b.h"
#include "modules/canbus/vehicle/twizy/protocol/steeringangle_0c0h_c0.h"

namespace apollo {
namespace canbus {
namespace twizy {

TwizyMessageManager::TwizyMessageManager() {
  // Control Messages
  AddSendProtocolData<Steering96, true>();
  AddSendProtocolData<Speed9A, true>();
  
  // Report Messages
  AddRecvProtocolData<Steeringangle0c0hc0, true>();
  AddRecvProtocolData<Gearandbreakpedal59b, true>();
  AddRecvProtocolData<Currentspeed599, true>();
}

TwizyMessageManager::~TwizyMessageManager() {}

}  // namespace twizy
}  // namespace canbus
}  // namespace apollo
