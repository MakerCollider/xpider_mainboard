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


