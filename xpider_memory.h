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

#ifndef XPIDER_MEMORY_H_
#define XPIDER_MEMORY_H_

#include <SPI.h>
#include "SerialFlash.h"

#include "xpider_pin.h"
#include "xpider_info.h"

class XpiderMemory {
public:
  static const char* kCustomDataFileName;

  static const char* kGroupInfoFileName;
  static const char* kNnInfoFileName;

  static XpiderMemory* instance();

  bool Initialize();

  bool EraseFlash();
  bool OpenDataFile();
  bool OpenGroupInfoFile();
  bool OpenNnInfoFile();

  bool CheckCustomData();
  bool CheckGroupInfoData();
  bool CheckNnInfoData();

  bool InitializeCustomData();
  bool InitializeGroupInfoData();
  bool InitializeNnInfoData();

  bool ReadCustomData(XpiderInfo::CustomData *custom_data);
  bool ReadGroupInfo(uint8_t * totalMsg, uint16_t *length);
  bool ReadNnInfo(uint8_t * totalMsg, uint16_t *length);

  bool WriteCustomData(XpiderInfo::CustomData *custom_data);
  bool WriteGroupInfoData(uint8_t * totalMsg, uint16_t length);
  bool WriteNnInfoData(uint8_t * totalMsg, uint16_t length);

  void EraseAll();

private:
  static XpiderMemory* instance_;

  SerialFlashFile custom_data_file_;
  SerialFlashFile group_info_file_;
  SerialFlashFile nn_info_file_;

  XpiderMemory();
  ~XpiderMemory();
};

#endif // XPIDER_MEMORY_H_


