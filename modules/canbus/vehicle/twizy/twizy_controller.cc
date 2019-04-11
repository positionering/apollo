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

#include "modules/canbus/vehicle/twizy/twizy_controller.h"

//#include "modules/common/log.h"

#include "modules/drivers/canbus/can_comm/can_sender.h"
#include "modules/canbus/vehicle/twizy/twizy_message_manager.h"
#include "modules/common/time/time.h"

namespace apollo {
namespace canbus {
namespace twizy {

using ::apollo::common::ErrorCode;
using ::apollo::control::ControlCommand;

namespace {

const int32_t kMaxFailAttempt = 10;
const int32_t CHECK_RESPONSE_STEER_UNIT_FLAG = 1;
const int32_t CHECK_RESPONSE_SPEED_UNIT_FLAG = 2;
}

ErrorCode TwizyController::Init(const VehicleParameter& params,
                                  CanSender<::apollo::canbus::ChassisDetail>*
									  const can_sender,
                                  MessageManager<::apollo::canbus::ChassisDetail>*
									  const message_manager) {
  if (is_initialized_) {
    AINFO << "TwizyController has already been initiated.";
    return ErrorCode::CANBUS_ERROR;
  }

  params_.CopyFrom(params);
  if (!params_.has_driving_mode()) {
    AERROR << "Vehicle conf pb not set driving_mode.";
    return ErrorCode::CANBUS_ERROR;
  }

  if (can_sender == nullptr) {
    return ErrorCode::CANBUS_ERROR;
  }
  can_sender_ = can_sender;

  if (message_manager == nullptr) {
    AERROR << "protocol manager is null.";
    return ErrorCode::CANBUS_ERROR;
  }
  message_manager_ = message_manager;

  // sender part
  steering_96_ = dynamic_cast<Steering96 *>(
  message_manager_->GetMutableProtocolDataById(Steering96::ID)
  );
  if (steering_96_ == nullptr) {
    AERROR << "Steering96 does not exist in the TwizyMessageManager!";
	return ErrorCode::CANBUS_ERROR;
  }

  speed_9a_ = dynamic_cast<Speed9A *>(
  message_manager_->GetMutableProtocolDataById(Speed9A::ID)
  );
  if (speed_9a_ == nullptr) {
    AERROR << "Speed9A does not exist in the TwizyMessageManager!";
  return ErrorCode::CANBUS_ERROR;
  }

  can_sender_->AddMessage(Steering96::ID, steering_96_, false);
  can_sender_->AddMessage(Speed9A::ID, speed_9a_, false);

  // need sleep to ensure all messages received
  AINFO << "TwizyController is initialized.";

  is_initialized_ = true;
  return ErrorCode::OK;
}

TwizyController::~TwizyController() {}

bool TwizyController::Start() {
  if (!is_initialized_) {
    AERROR << "TwizyController has NOT been initiated.";
    return false;
  }
  const auto& update_func = [this] { SecurityDogThreadFunc(); };
  thread_.reset(new std::thread(update_func));

  return true;
}

void TwizyController::Stop() {
  if (!is_initialized_) {
    AERROR << "TwizyController stops or starts improperly!";
    return;
  }

  if (thread_ != nullptr && thread_->joinable()) {
    thread_->join();
    thread_.reset();
    AINFO << "TwizyController stopped.";
  }
}

Chassis TwizyController::chassis() {
  chassis_.Clear();

  ChassisDetail chassis_detail;

  message_manager_->GetSensorData(&chassis_detail);

  //std::string debugs = chassis_detail.DebugString();
  //ROS_WARN_STREAM(debugs);

  // 21, 22, previously 1, 2
  if (driving_mode() == Chassis::EMERGENCY_MODE) {
    set_chassis_error_code(Chassis::NO_ERROR);
  }

  chassis_.set_driving_mode(driving_mode());
  chassis_.set_error_code(chassis_error_code());

  // 3
  chassis_.set_engine_started(true);

  // ADD YOUR OWN CAR CHASSIS OPERATION
  // We assume that car isn't in motion when engine starts.

  chassis_.set_speed_mps(chassis_detail.twizy().curr_speed().curr_speed());
  chassis_.set_steering_percentage(-(chassis_detail.twizy().steering_angle().lws()+200.0)/5.5); //Changed 21/5
  chassis_.set_parking_brake(chassis_detail.twizy().gear_and_pedal().brake_pedalstatus());

  if (chassis_detail.twizy().gear_and_pedal().gear_d()) {
	  set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
	  chassis_.set_gear_location(apollo::canbus::Chassis::GEAR_DRIVE); //sets position to GEAR_DRIVE, see twizy.proto
  }

  return chassis_;
}

void TwizyController::Emergency() {
  set_driving_mode(Chassis::EMERGENCY_MODE);
  ResetProtocol();
}

ErrorCode TwizyController::EnableAutoMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    AINFO << "already in COMPLETE_AUTO_DRIVE mode";
    return ErrorCode::OK;
  }
  return ErrorCode::OK;
  // ADD YOUR OWN CAR CHASSIS OPERATION

  can_sender_->Update();
  const int32_t flag =
      CHECK_RESPONSE_STEER_UNIT_FLAG | CHECK_RESPONSE_SPEED_UNIT_FLAG;
  if (!CheckResponse(flag, true)) {
    AERROR << "Failed to switch to COMPLETE_AUTO_DRIVE mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);
    AINFO << "Switch to COMPLETE_AUTO_DRIVE mode ok.";
    return ErrorCode::OK;
  }
}

ErrorCode TwizyController::DisableAutoMode() {
  ResetProtocol();
  can_sender_->Update();
  set_driving_mode(Chassis::COMPLETE_MANUAL);
  set_chassis_error_code(Chassis::NO_ERROR);
  AINFO << "Switch to COMPLETE_MANUAL ok.";
  return ErrorCode::OK;
}

ErrorCode TwizyController::EnableSteeringOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_STEER_ONLY) {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Already in AUTO_STEER_ONLY mode";
    return ErrorCode::OK;
  }
  return ErrorCode::OK;
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  brake_60_->set_disable();
  throttle_62_->set_disable();
  steering_96_->set_enable();

  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::AUTO_STEER_ONLY);
    AINFO << "Switch to AUTO_STEER_ONLY mode ok.";
    return ErrorCode::OK;
  }
  */
}

ErrorCode TwizyController::EnableSpeedOnlyMode() {
  if (driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
      driving_mode() == Chassis::AUTO_SPEED_ONLY) {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Already in AUTO_SPEED_ONLY mode";
    return ErrorCode::OK;
  }
  return ErrorCode::OK;
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  brake_60_->set_enable();
  throttle_62_->set_enable();
  steering_96_->set_disable();

  can_sender_->Update();
  if (CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, true) == false) {
    AERROR << "Failed to switch to AUTO_STEER_ONLY mode.";
    Emergency();
    set_chassis_error_code(Chassis::CHASSIS_ERROR);
    return ErrorCode::CANBUS_ERROR;
  } else {
    set_driving_mode(Chassis::AUTO_SPEED_ONLY);
    AINFO << "Switch to AUTO_SPEED_ONLY mode ok.";
    return ErrorCode::OK;
  }
  */
}

// NEUTRAL, REVERSE, DRIVE
void TwizyController::Gear(Chassis::GearPosition gear_position) {
  if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "this drive mode no need to set gear.";
    return;
  }
  return;
  // ADD YOUR OWN CAR CHASSIS OPERATION
 /* switch (gear_position) {
    case Chassis::GEAR_NEUTRAL: {
      gear_98_->set_gear_neutral();
      break;
    }
    case Chassis::GEAR_REVERSE: {
      gear_98_->set_gear_reverse();
      break;
    }
    case Chassis::GEAR_DRIVE: {
      gear_98_->set_gear_drive();
      break;
    }
    case Chassis::GEAR_NONE: {
      gear_98_->set_gear_none();
      break;
    }
    case Chassis::GEAR_INVALID: {
      AERROR << "Gear command is invalid!";
      gear_98_->set_gear_none();
      break;
    }
    default: {
      gear_98_->set_gear_none();
      break;
    }
  } */

}

// brake with new acceleration
// acceleration:0.00~99.99, unit:
// acceleration:0.0 ~ 7.0, unit:m/s^2
// acceleration_spd:60 ~ 100, suggest: 90
// -> pedal
void TwizyController::Brake(double brake_percentage) {
  // double real_value = params_.max_acc() * acceleration / 100;
  // TODO Update brake value based on mode
  /*if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }*/
  // ADD YOUR OWN CAR CHASSIS OPERATION
  //gear_98_->set_brake_pedalstatus(true);
  speed_9a_->set_ref_speed(0.0);
}

// drive with old acceleration
// gas:0.00~99.99 unit:
void TwizyController::Throttle(double speed) {
  AERROR<<"SPEEED";
   AINFO<<"this is throttle func with speed in mp/s: " << speed
   << "\nand in km/h: " << speed*3.6;

  /*if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_SPEED_ONLY)) {
    AINFO << "The current drive mode does not need to set acceleration.";
    return;
  }*/
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  throttle_62_->set_pedal(pedal);
  */

  speed_9a_->set_ref_speed(2);
}

// twizy default, -40 ~ 40, left:+, right:-
// need to be compatible with control module, so reverse
// steering with old angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
void TwizyController::Steer(double angle) {
  /*if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  }*/
  // reverse sign
  // ADD YOUR OWN CAR CHASSIS OPERATION
  double angle2 = 80;
  angle2 = (angle2 / 100) * 39.5;
  steering_96_->set_steering_angle(angle2);
//AERROR<<"SKICKAR STEER";
}

// steering with new angle speed
// angle:-99.99~0.00~99.99, unit:, left:-, right:+
// angle_spd:0.00~99.99, unit:deg/s
void TwizyController::Steer(double angle, double angle_spd) {
  /* if (!(driving_mode() == Chassis::COMPLETE_AUTO_DRIVE ||
        driving_mode() == Chassis::AUTO_STEER_ONLY)) {
    AINFO << "The current driving mode does not need to set steer.";
    return;
  } */
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  const double real_angle = params_.max_steer_angle() * angle / 100.0;
  const double real_angle_spd = ProtocolData::BoundedValue(
      params_.min_steer_angle_spd(), params_.max_steer_angle_spd(),
      params_.max_steer_angle_spd() * angle_spd / 100.0);
  steering_96_->set_steering_angle(real_angle)
      ->set_steering_angle_speed(real_angle_spd);

  */
  double angle3 = 80;
  angle3 = (angle3 / 100) * 39.5;
  steering_96_->set_steering_angle(angle3);
//AERROR<<"SKICKAR STEER2";
}

void TwizyController::SetEpbBreak(const ControlCommand& command) {
  if (command.parking_brake()) {
    // None
  } else {
    // None
  }
}

void TwizyController::SetBeam(const ControlCommand& command) {
  if (command.signal().high_beam()) {
    // None
  } else if (command.signal().low_beam()) {
    // None
  } else {
    // None
  }
}

void TwizyController::SetHorn(const ControlCommand& command) {
  if (command.signal().horn()) {
    // None
  } else {
    // None
  }
}

void TwizyController::SetTurningSignal(const ControlCommand& command) {
  // Set Turn Signal
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  auto signal = command.signal().turn_signal();
  if (signal == Signal::TURN_LEFT) {
    turnsignal_68_->set_turn_left();
  } else if (signal == Signal::TURN_RIGHT) {
    turnsignal_68_->set_turn_right();
  } else {
    turnsignal_68_->set_turn_none();
  }
  */
}

void TwizyController::ResetProtocol() {
  message_manager_->ResetSendMessages();
}

bool TwizyController::CheckChassisError() {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  */
  return false;
}

void TwizyController::SecurityDogThreadFunc() {
  int32_t vertical_ctrl_fail = 0;
  int32_t horizontal_ctrl_fail = 0;

  if (can_sender_ == nullptr) {
    AERROR << "Fail to run SecurityDogThreadFunc() because can_sender_ is "
              "nullptr.";
    return;
  }
  while (!can_sender_->IsRunning()) {
    std::this_thread::yield();
  }

  std::chrono::duration<double, std::micro> default_period{50000};
  int64_t start = 0;
  int64_t end = 0;
  while (can_sender_->IsRunning()) {
    start = ::apollo::common::time::AsInt64<::apollo::common::time::micros>(
        ::apollo::common::time::Clock::Now());
    const Chassis::DrivingMode mode = driving_mode();
    bool emergency_mode = false;

    // 1. horizontal control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_STEER_ONLY) &&
        CheckResponse(CHECK_RESPONSE_STEER_UNIT_FLAG, false) == false) {
      ++horizontal_ctrl_fail;
      if (horizontal_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      horizontal_ctrl_fail = 0;
    }

    // 2. vertical control check
    if ((mode == Chassis::COMPLETE_AUTO_DRIVE ||
         mode == Chassis::AUTO_SPEED_ONLY) &&
        CheckResponse(CHECK_RESPONSE_SPEED_UNIT_FLAG, false) == false) {
      ++vertical_ctrl_fail;
      if (vertical_ctrl_fail >= kMaxFailAttempt) {
        emergency_mode = true;
        set_chassis_error_code(Chassis::MANUAL_INTERVENTION);
      }
    } else {
      vertical_ctrl_fail = 0;
    }
    if (CheckChassisError()) {
      set_chassis_error_code(Chassis::CHASSIS_ERROR);
      emergency_mode = true;
    }

    if (emergency_mode && mode != Chassis::EMERGENCY_MODE) {
      set_driving_mode(Chassis::EMERGENCY_MODE);
      message_manager_->ResetSendMessages();
    }
    end = ::apollo::common::time::AsInt64<::apollo::common::time::micros>(
        ::apollo::common::time::Clock::Now());
    std::chrono::duration<double, std::micro> elapsed{end - start};
    if (elapsed < default_period) {
      std::this_thread::sleep_for(default_period - elapsed);
    } else {
      AERROR
          << "Too much time consumption in TwizyController looping process:"
          << elapsed.count();
    }
  }
}

bool TwizyController::CheckResponse(const int32_t flags, bool need_wait) {
  /* ADD YOUR OWN CAR CHASSIS OPERATION
  */
  return true;
}

void TwizyController::set_chassis_error_mask(const int32_t mask) {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  chassis_error_mask_ = mask;
}

int32_t TwizyController::chassis_error_mask() {
  std::lock_guard<std::mutex> lock(chassis_mask_mutex_);
  return chassis_error_mask_;
}

Chassis::ErrorCode TwizyController::chassis_error_code() {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  return chassis_error_code_;
}

void TwizyController::set_chassis_error_code(
    const Chassis::ErrorCode& error_code) {
  std::lock_guard<std::mutex> lock(chassis_error_code_mutex_);
  chassis_error_code_ = error_code;
}

}  // namespace twizy
}  // namespace canbus
}  // namespace apollo