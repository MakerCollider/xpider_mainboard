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

#ifndef XPIDER_PROTOCOL_H_
#define XPIDER_PROTOCOL_H_

#include "xpider_info.h"

class XpiderProtocol {
public:
  #define MAX_MESSAGE_LENGTH    1024
  #define MAX_MESSAGE_NUM   64
  enum MessageType {
    kHeartBeat,
    kMove,
    kFrontLeds,
    kEye,
    kAutoPilot,
    kGroupInfo,
    kNNInfo,
    kRun,
    kEmergencyStop,
    kGetRegister,
    kUpdateRegister,
    kRegisterResponse,
    kUnknown,
    kStep,
    kAutoMove,
  };
  
  enum RegIndex {
    kRegUUID,
    kRegName,
    kRegHardwareVersion,
    kRegFirmwareVersion,
    kRegControllerVersion
  };

  XpiderProtocol();

  void Initialize(XpiderInfo* xpider_info);
  
  MessageType GetMessage(const uint8_t *buffer, const uint16_t &length);

  void GetBuffer(const MessageType &index, uint8_t **send_buffer, uint16_t *length);
  void GetRegister(const RegIndex &register_index, uint8_t **send_buffer, uint16_t *length);
  void UpdateRegister(const RegIndex &register_index, uint8_t **send_buffer, uint16_t *length);
  void RegisterResponse(const RegIndex &register_index, uint8_t **send_buffer, uint16_t *length);

  bool GetCRCResult(const uint8_t *buffer, const uint16_t &length);
  uint8_t SumCRC(const uint8_t *buffer, const uint16_t &length);

  void ParseGroupMsg(uint8_t *totalMsg, const uint16_t &length);
  void ParseNnMsg(uint8_t *totalMsg, const uint16_t &length);
  bool isWholeGroupInfoMsg() { return isWholeGroupInfo; }
  bool isWholeNnInfoMsg() { return isWholeNnInfo; }
  uint8_t * getWholeMsg() { return totalMsg; }
private:
  XpiderInfo *xpider_info_;
  uint8_t totalMsg[MAX_MESSAGE_LENGTH];
  uint8_t send_msg_buffer_[MAX_MESSAGE_NUM*20];
  uint8_t group_info_ver = 0;
  uint8_t nn_info_ver = 0;
  uint16_t group_info_unread_byte = 0;
  uint16_t nn_info_unread_byte = 0;
  /*every time recv a msg, it will record the total msg position in buffer*/
  uint16_t total_msg_position = 0;

  /*call this func multis times to combine a whole lerning/actionGroup msg*/
  void CombineMsg(const uint8_t *buffer, const uint8_t &one_time_buffer_length, uint8_t *totalMsg);
  bool isWholeGroupInfo = false;
  bool isWholeNnInfo = false;
};

#endif // XPIDER_PROTOCOL_H_
