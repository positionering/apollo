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

#include "modules/canbus/vehicle/twizy/protocol/currentspeed_599.h"


#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace twizy {

using ::apollo::drivers::canbus::Byte;

Currentspeed599::Currentspeed599() {}
const int32_t Currentspeed599::ID = 0x599;

void Currentspeed599::Parse(const std::uint8_t* bytes, int32_t length,
                         ChassisDetail* chassis) const {
  chassis->mutable_twizy()->mutable_curr_speed()->set_curr_speed(curr_speed(bytes, length));
}

// config detail: {'name': 'curr_speed', 'offset': 0.0, 'precision': 0.01, 'len': 16, 'is_signed_var': True, 'physical_range': '[0|0]', 'bit': 47, 'type': 'double', 'order': 'motorola', 'physical_unit': ''}
double Currentspeed599::curr_speed(const std::uint8_t* bytes, int32_t length) const {
    
    unsigned short i = 0;

    unsigned char* t = (unsigned char*) &i;

    *(t) = bytes[6];
    *(t+1) = bytes[5];

    unsigned long result = 0.0;

    unsigned long temp = 0 | i;
    const unsigned short signbit_mask = 0x8000;
    temp = temp & signbit_mask;
    result = (result | temp) << 48; //Shift up to signbit position

    const unsigned short exponent_mask = 0x7C00;
    temp = i & exponent_mask;
    //Shift down and cast to signed to be able to do the math required to fix the bias
    //and then shift up again to be bitwise ore:d back into the result
    //16-bit bias is 15, 64 bit is 1023, so add the difference (1008) to compensate
    const int bias_difference = 1023 - 15;
    //Uncomment the line below to test my wierd unscientific fuckery that makes the numbers line up
    //const int bias_difference = 1025 - 15;
    long exponent = ((temp >> 10) + bias_difference) << 52;
    //Add the corrected exponent to the result
    result = (result | exponent);

    //Mask out the mantissa
    const unsigned long mantissa_mask = 0x0FFFFFFFFFFFFF;
    temp = i & mantissa_mask;
    //Rightpad with zeroes
    temp = temp << 42;

    result = (result | temp);


    double speed = *((double*) &result);
    return speed;
}
}  // namespace twizy
}  // namespace canbus
}  // namespace apollo
