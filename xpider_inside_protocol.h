/*
 * Xpider mainboard software, running on Intel Curie
 * Copyright (C) 2015-2017  Roboeve, MakerCollider
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef XPIDER_INSIDE_PROTOCOL_H_
#define XPIDER_INSIDE_PROTOCOL_H_

#include "Arduino.h"

class XpiderInsideProtocol {
public:
  enum MessageType {
    kMove,
    kRotate,
    kSetEye,
    kSetFrontLeds,
    kHeartBeat,
    kUnknown,
    kMoveStep,
    kAutoMove,
    kGetRegister,
    kRegisterResponse
  };

  enum RegisterIndex {
    kControllerVersion
  };

  struct HeartBeatStruct {
    uint16_t step_counter;
    uint16_t obstacle_distance;
    float battery_voltage;
    float yaw_pitch_roll[3];
  };

  typedef void (* SendType) (const uint8_t *framebuffer, uint8_t frame_length);

  typedef void (* MoveCallbackType) (int8_t);
  typedef void (* StepCallbackType) (int8_t, uint8_t);
  typedef void (* AutoMoveCallbackType) (uint8_t, float, uint8_t, int8_t);
  typedef void (* RotateCallbackType) (int8_t);
  typedef void (* SetEyeCallbackType) (uint8_t);
  typedef void (* SetFrontLedsCallbackType) (const uint8_t[6]);
  typedef void (* HeartBeatCallbackType) (HeartBeatStruct);
  typedef void (* GetRegisterCallbackType) (RegisterIndex);
  typedef void (* RegisterResponseCallbackType) (RegisterIndex, const uint8_t*, uint8_t);

  struct CallbackListStruct {
    MoveCallbackType move;
    StepCallbackType step;
    AutoMoveCallbackType auto_move;
    RotateCallbackType rotate;
    SetEyeCallbackType set_eye;
    SetFrontLedsCallbackType set_front_leds;
    HeartBeatCallbackType heartbeat;
    GetRegisterCallbackType get_register;
    RegisterResponseCallbackType register_response;
  };

  XpiderInsideProtocol();
  ~XpiderInsideProtocol();

  void Initialize(SendType, CallbackListStruct);
  
  MessageType Decode(const uint8_t* buffer, uint16_t length);
  void SetStep(int8_t step_speed, uint8_t step_count);
  void SetEye(uint8_t angle);
  void SetMove(int8_t speed);
  void SetRotate(int8_t speed);
  void SetFrontLeds(const uint8_t[6]);
  void SetHeartBeat(HeartBeatStruct heatbeat);
  void SetAutoMove(uint8_t rotate_speed, float rotate_rad, uint8_t walk_speed, int8_t walk_step);
  void GetRegister(RegisterIndex register_index);
  void RegisterResponse(RegisterIndex register_index, const uint8_t *value, uint8_t length);
private:
  SendType Send;

private:
  CallbackListStruct callback_list_;
};

#endif // XPIDER_INSIDE_PROTOCOL_H_
