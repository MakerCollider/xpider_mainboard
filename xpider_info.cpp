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

#include "xpider_info.h"
#include "arduino_log.h"

const char* XpiderInfo::kCustomDataFileName = "CustomData";
const XpiderInfo::CustomData XpiderInfo::kInitializeCustomData = {
  "XP_UKN",
  "unknown",
  0x00000000
};
const XpiderInfo::DefGroupInfoData XpiderInfo::kInitializeGroupInfoData = {
  0x05, /*type*/
  0x00, /*ver*/
  0x07, /*length low*/
  0x00, /*length high*/
  0x00, /*pad group id*/
  0x00, /*pad group info*/
  0x0c  /*crc*/
};
const XpiderInfo::DefNnInfoData XpiderInfo::kInitializeNnInfoData = {
  0x06, /*type*/
  0x00, /*ver*/
  0x08, /*length low*/
  0x00, /*length high*/
  0x00, /*pad sensor num*/
  0x00, /*pad nn data length low*/
  0x00, /*pad nn data length high*/
  0x0d  /*crc*/
};

void XpiderInfo::ClearGroupList(int group_id) {
  int i; 
  for(i = 0; i < (action_group_list[group_id]).size(); i++)
  {
  	GroupElement *temp = action_group_list[group_id].get(i);
    if(temp != NULL) {
    	switch(temp->id)
    	{
        case idWalk: {
          WalkInfo * temp_data = (WalkInfo *)temp->data;
          delete temp_data;
          temp->data = NULL;
          break;
        }
        case idRotate: {
          RotateInfo * temp_data = (RotateInfo *)temp->data;
          delete temp_data;
          temp->data = NULL;
          break;
        }
        case idLed: {
          LedInfo * temp_data = (LedInfo *)temp->data;
          delete temp_data;
          temp->data = NULL;
          break;
        }
        case idEye: {
          EyeInfo * temp_data = (EyeInfo *)temp->data;
          delete temp_data;
          temp->data = NULL;
          break;
        }
        default: {
          LOG_PRINTLN("Error: Clear Action Group info");
          break;
        }
    	}
      delete temp;
      temp = NULL;
    }
  }
  action_group_list[group_id].clear();
}
void XpiderInfo::ClearAllGroupList() {
  for(int max = 0; max < MAX_GROUP_NUM ; max++) {
    XpiderInfo::ClearGroupList(max);
  }
}

void XpiderInfo::GroupListAdd(int group_id, XpiderInfo::ActionIndex id, uint8_t *buffer) {
  GroupElement *temp = new GroupElement();
  temp->id = id;
  switch(id)
  {
    case idWalk: {
      WalkInfo *walk_info = new WalkInfo();
      memcpy(walk_info, buffer, sizeof(WalkInfo));
      temp->data = (uint8_t *)walk_info;
      break;
    }
    case idRotate: {
      RotateInfo *rotate_info = new RotateInfo();
      memcpy(rotate_info, buffer, sizeof(RotateInfo));
      temp->data = (uint8_t *)rotate_info;
      break;
    }
    case idLed: {
      LedInfo *led_info = new LedInfo();
      memcpy(led_info, buffer, sizeof(LedInfo));
      temp->data = (uint8_t *)led_info;
      break;
    }
    case idEye: {
      EyeInfo *eye_info = new EyeInfo();
      memcpy(eye_info, buffer, sizeof(EyeInfo));
      temp->data = (uint8_t *)eye_info;
      break;
    }
    case idSpeaker: {
      SpeakerInfo *speaker_info = new SpeakerInfo();
      memcpy(speaker_info, buffer, sizeof(SpeakerInfo));
      temp->data = (uint8_t *)speaker_info;
      break;
    }
    default: {
      LOG_PRINTLN("Error: add unknow type element");
      break;
    }
  }
  action_group_list[group_id].add(temp);
}

bool XpiderInfo::ifGroupCanRun(uint8_t group_id) {
  return (is_free || (group_id!=running_group_id));
}

XpiderInfo::GroupElement * XpiderInfo::SetCurrentAction(uint8_t action_num, bool enable, uint32_t start_time)
{
  if(enable == true){
    running_action_id = action_num;
    LOG_PRINT("action_num = ");
    LOG_PRINTLN(action_num);
    LOG_PRINT("running_group_id = ");
    LOG_PRINTLN(running_group_id);
    LOG_PRINT("running_group_id.size = ");
    LOG_PRINTLN(action_group_list[running_group_id].size());

    if(action_group_list[running_group_id].size() > 0 &&
       running_action_id < action_group_list[running_group_id].size()){
      GroupElement * temp = action_group_list[running_group_id].get(action_num);
      action_status.id = temp->id;
      action_status.time_start = start_time;
      switch(temp->id){
        case idWalk:{
          action_status.time_counter = ((WalkInfo *)temp->data)->time;
          break;
        }
        case idRotate:{
          action_status.time_counter = ((RotateInfo *)temp->data)->time;
          break;
        }
        case idLed:{
          action_status.time_counter = ((LedInfo *)temp->data)->time;
          break;
        }
        case idEye:{
          action_status.time_counter = ((EyeInfo *)temp->data)->time;
          break;
        }
        case idSpeaker:{
          action_status.time_counter = 0;
          break;
        }
        default: {
          LOG_PRINTLN("ERROR: Unknow action when doing the action!");
        }
      }
      action_status.is_enable = true;
      return temp;
    } else {
      LOG_PRINTLN("Set Current Action, but action is empty, So no more actions in this group");
      action_status.is_enable = false;
      return NULL;
    }
  }else{
    LOG_PRINTLN("Set Current Action receive false");
    action_status.is_enable = false;
    return NULL;
  }
}
XpiderInfo::GroupElement * XpiderInfo::getCurrentAction(){
  GroupElement * temp = action_group_list[running_group_id].get(running_action_id);
  return temp;
}

uint8_t XpiderInfo::getCurrentActionId(){
  return running_action_id;
}

bool XpiderInfo::ifActionRunning(){
  return action_status.is_enable;
}

void XpiderInfo::Count(){
  if(action_status.time_counter < 10){
    action_status.time_counter = 0;
  }else {
    action_status.time_counter -= 10;
  }
  LOG_PRINT("timer = ");
  LOG_PRINTLN(action_status.time_counter);
}

bool XpiderInfo::ifActionShouldStop(uint32_t current_time){
  if((current_time - action_status.time_start) >= action_status.time_counter){
    return true;
  }else {
    return false;
  }
}



