/**
 * Author: Ye Tian <flymaxty@foxmail.com>
 * Copyright (c) 2016 Maker Collider Corporation.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
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
