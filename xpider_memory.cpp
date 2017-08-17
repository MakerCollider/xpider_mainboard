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

#include "xpider_memory.h"
#include "arduino_log.h"

XpiderMemory* XpiderMemory::instance_ = NULL;
const char* XpiderMemory::kCustomDataFileName = "CustomData";

const char* XpiderMemory::kGroupInfoFileName = "GroupInfoData";
const char* XpiderMemory::kNnInfoFileName = "NnInfoData";

XpiderMemory::XpiderMemory() {

}

XpiderMemory::~XpiderMemory() {

}

XpiderMemory* XpiderMemory::instance() {
  if(instance_ == NULL) {
    instance_ = new XpiderMemory();
  }

  return instance_;
}

bool XpiderMemory::EraseFlash() {
  SerialFlash.eraseAll();
  while (!SerialFlash.ready()) {}

  return true;
}

bool XpiderMemory::Initialize() {
  SerialFlash.begin(FLASH_CHIP_SELECT);
  return true;
}

bool XpiderMemory::CheckCustomData() {
  return SerialFlash.exists(kCustomDataFileName) ? true : false;
}

bool XpiderMemory::CheckGroupInfoData() {
  return SerialFlash.exists(kGroupInfoFileName) ? true : false;
}
bool XpiderMemory::CheckNnInfoData() {
  return SerialFlash.exists(kNnInfoFileName) ? true : false;
}

bool XpiderMemory::OpenDataFile() {
  if(SerialFlash.exists(kCustomDataFileName)) {
    if(!custom_data_file_) {
      custom_data_file_ = SerialFlash.open(kCustomDataFileName);
    }
    return true;
  }

  return false;
}

bool XpiderMemory::OpenGroupInfoFile() {
  if(SerialFlash.exists(kGroupInfoFileName)) {
    if(!group_info_file_) {
      group_info_file_ = SerialFlash.open(kGroupInfoFileName);
    }
    return true;
  }
  return false;
}
bool XpiderMemory::OpenNnInfoFile() {
  if(SerialFlash.exists(kNnInfoFileName)) {
    if(!nn_info_file_) {
      nn_info_file_ = SerialFlash.open(kNnInfoFileName);
    }
    return true;
  }
  return false;
}

bool XpiderMemory::InitializeCustomData() {
  if(!CheckCustomData()) {
    // SerialFlash.create(kCustomDataFileName, sizeof(XpiderInfo::CustomData));
    SerialFlash.createErasable(kCustomDataFileName, sizeof(XpiderInfo::CustomData)); 
  }

  if (OpenDataFile()) {
    custom_data_file_.seek(0);
    XpiderInfo::CustomData* temp = const_cast<XpiderInfo::CustomData*>(&XpiderInfo::kInitializeCustomData);
    custom_data_file_.write(reinterpret_cast<void*>(temp), sizeof(XpiderInfo::CustomData)); 
    while (!SerialFlash.ready()) {}
  } else {
    return false;
  }
  
  return true;
}
bool XpiderMemory::InitializeGroupInfoData() {
  if(!CheckGroupInfoData()) {
    // SerialFlash.create(kCustomDataFileName, sizeof(XpiderInfo::CustomData));
    SerialFlash.createErasable(kGroupInfoFileName, 1024); 
  }

  if (OpenGroupInfoFile()) {
    group_info_file_.seek(0);
    XpiderInfo::DefGroupInfoData* temp = const_cast<XpiderInfo::DefGroupInfoData*>(&XpiderInfo::kInitializeGroupInfoData);
    group_info_file_.write(reinterpret_cast<void*>(temp), sizeof(XpiderInfo::DefGroupInfoData)); 
    while (!SerialFlash.ready()) {}
  } else {
    return false;
  }
  return true;
}

bool XpiderMemory::InitializeNnInfoData() {
  if(!CheckNnInfoData()) {
    // SerialFlash.create(kCustomDataFileName, sizeof(XpiderInfo::CustomData));
    SerialFlash.createErasable(kNnInfoFileName, 1024); 
  }

  if (OpenNnInfoFile()) {
    nn_info_file_.seek(0);
    XpiderInfo::DefNnInfoData* temp = const_cast<XpiderInfo::DefNnInfoData*>(&XpiderInfo::kInitializeNnInfoData);
    nn_info_file_.write(reinterpret_cast<void*>(temp), sizeof(XpiderInfo::DefNnInfoData)); 
    while (!SerialFlash.ready()) {}
  } else {
    return false;
  } 
  return true;
}

bool XpiderMemory::ReadCustomData(XpiderInfo::CustomData *custom_data) {
  if(custom_data_file_) {
    custom_data_file_.seek(0);
    custom_data_file_.read(reinterpret_cast<void*>(custom_data), sizeof(XpiderInfo::CustomData));
    return true;
  }

  return false;
}
bool XpiderMemory::ReadGroupInfo(uint8_t * totalMsg, uint16_t *length) {
  /*group info msg*/
  if(group_info_file_) {
    group_info_file_.seek(0);
    /*get length*/
    group_info_file_.read(reinterpret_cast<void*>(totalMsg), 4);
    memcpy(length, &totalMsg[2], 2);
    LOG_PRINT("GroupInfo in Flash length is ");
    LOG_PRINTLN(*length);
    /*Read whole data*/
    group_info_file_.seek(0);
    group_info_file_.read(reinterpret_cast<void*>(totalMsg), *length);
    for(int i = 0; i < *length; i++){
      LOG_PRINTF(totalMsg[i], HEX);
      LOG_PRINT(" ");
    }
    LOG_PRINTLN(" ");
    return true;
  }
  else {
    /*do nothing*/
    LOG_PRINTLN("No Preset GroupInfo");
    return false;
  }
}

bool XpiderMemory::ReadNnInfo(uint8_t * totalMsg, uint16_t *length) {
  /*nn info msg*/
  if(nn_info_file_) {
    nn_info_file_.seek(0);
    /*get length*/
    nn_info_file_.read(reinterpret_cast<void*>(totalMsg), 4);
    memcpy(length, &totalMsg[2], 2);
    LOG_PRINT("NNInfo in Flash length is ");
    LOG_PRINTLN(*length);
    /*Read whole data*/
    nn_info_file_.seek(0);
    nn_info_file_.read(reinterpret_cast<void*>(totalMsg), *length);
    for(int i = 0; i < *length; i++){
      LOG_PRINTF(totalMsg[i], HEX);
      LOG_PRINT(" ");
    }
    LOG_PRINTLN(" ");
    return true;
  }
  else {
    /*do nothing*/
    LOG_PRINTLN("No Preset NnInfo");
    return false;
  }
}

bool XpiderMemory::WriteCustomData(XpiderInfo::CustomData *custom_data) {
  if(custom_data_file_) {
    custom_data_file_.erase();
    custom_data_file_.seek(0);
    custom_data_file_.write(reinterpret_cast<void*>(custom_data), sizeof(XpiderInfo::CustomData));
    while (!SerialFlash.ready()) {}
    return true;
  }

  return false;
}

bool XpiderMemory::WriteGroupInfoData(uint8_t * totalMsg, uint16_t length) {
  if(group_info_file_) {
    group_info_file_.erase();
    group_info_file_.seek(0);
    group_info_file_.write(reinterpret_cast<void*>(totalMsg), length);
    while (!SerialFlash.ready()) {}
    return true;
  }

  return false;
}

bool XpiderMemory::WriteNnInfoData(uint8_t * totalMsg, uint16_t length) {
  if(nn_info_file_) {
    nn_info_file_.erase();
    nn_info_file_.seek(0);
    nn_info_file_.write(reinterpret_cast<void*>(totalMsg), length);
    while (!SerialFlash.ready()) {}
    return true;
  }

  return false;
}

void XpiderMemory::EraseAll(){
  if(group_info_file_) {
    LOG_PRINTLN("group file exist, remove it");
    group_info_file_.erase();
    group_info_file_.flush();
    group_info_file_.close();
  }
  if(nn_info_file_) {
    LOG_PRINTLN("nn file exist, remove it");
    nn_info_file_.erase();
    nn_info_file_.flush();
  }
}


