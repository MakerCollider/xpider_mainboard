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

#ifndef XPIDER_INFO_H_
#define XPIDER_INFO_H_

#include <stdint.h>
#include <string.h>

#include "linked_list.h"

#define MAX_GROUP_NUM  64
#define MAX_ACTION_NUM_IN_GROUP  64
#define MAX_ACTION_INDEX_NUM  4
#define MAX_SENSOR_NUM 6
#define MAX_NN_NEURON_NUM  64
#define MAX_NN_NEURON_BYTES  64

class XpiderInfo {
public:
  
  struct CustomData {
    char name[20];
    char hardware_version[20];
    uint32_t uuid;
  };

  struct DefGroupInfoData {
    uint8_t type; /*0x05*/
    uint8_t ver;  /*default 0*/
    uint8_t length_low; /*default 6byte: 1 type + 1 ver + 2 length + 1byte 0 + 1byte CRC*/
    uint8_t length_high;
    uint8_t padGroupId;
    uint8_t padGroupInfo;  /* always 0*/
    uint8_t crc;
  };
  struct DefNnInfoData {
    uint8_t type; /*0x06*/
    uint8_t ver;  /*default 0*/
    uint8_t length_low; /*default 6byte: 1 type + 1 ver + 2 length + 1byte 0 + 1byte CRC*/
    uint8_t length_high;
    uint8_t pad1;  /* always 0, 0 byte sensor*/
    uint8_t nn_length_low;   /*0 byte nn data*/
    uint8_t nn_length_high;
    uint8_t crc;
  };

  static const char* kCustomDataFileName;
  static const CustomData kInitializeCustomData;
  static const DefGroupInfoData kInitializeGroupInfoData;
  static const DefNnInfoData kInitializeNnInfoData;

  int8_t move;
  int8_t rotate;
  int8_t count_speed;
  uint8_t count;

  uint8_t rotate_speed;
  float rotate_rad;
  uint8_t walk_speed;
  int8_t walk_step;

  uint16_t obstacle_distance;

  uint8_t left_led_rgb[3];
  uint8_t right_led_rgb[3];

  bool eye_power;
  int8_t eye_angle;

  bool autopilot_enable;
  float autopilot_heading;

  float voltage;
  uint16_t step_counter;

  float yaw_pitch_roll[3];

  char firmware_version_[20];

  CustomData custom_data_;

  uint8_t sound_level;

  enum ActionIndex {
    idWalk,
    idRotate,
    idLed,
    idEye,
    idSpeaker,
    idUnknow
  };
  
  enum SensorIndex {
    idDistance = 1,
    idMic,
    idGyro,
    idUnknowSensor
  };

  uint8_t nn_info_ver = 0xff;
  uint8_t store_sensor_num = 0;
  SensorIndex nn_sensor_list[MAX_SENSOR_NUM] = {idUnknowSensor};
  uint8_t nn_neuron_num = 0;

  typedef struct _NeuronInfo{
    uint8_t neuron_bytes[MAX_NN_NEURON_BYTES];
    uint8_t length;
    uint8_t cat;
  }NeuronInfo;

  LinkedList<NeuronInfo> nn_data_list = LinkedList<NeuronInfo>();

  typedef struct _WalkInfo{
    uint8_t speed;
    uint16_t time;
  } WalkInfo;
  typedef struct _RotateInfo{
    uint8_t speed;
    uint16_t time;
  } RotateInfo;
  typedef struct _LedInfo{
    uint8_t led[6];
    uint16_t time;
  } LedInfo;
  typedef struct _EyeInfo{
    uint8_t angle;
    uint16_t time;
  } EyeInfo;
  typedef struct _SpeakerInfo{
    uint8_t unknow;
    uint16_t time;
  } SpeakerInfo;

  typedef struct GroupElement_ {
    XpiderInfo::ActionIndex id;
    uint8_t * data;   /*data include the struct info*/
  }GroupElement;

  typedef struct ActionStatus_ {
    XpiderInfo::ActionIndex id;
    uint32_t time_counter;
    uint32_t time_start;    
    bool is_enable = false;
  }ActionStatus;

  uint8_t group_info_ver = 0xff;
  uint8_t store_group_num = 0;
  uint8_t store_group_id[MAX_GROUP_NUM];
  LinkedList<GroupElement*> *action_group_list = new LinkedList<GroupElement*>[MAX_GROUP_NUM]();

  uint8_t last_group_id = 0xff;
  uint8_t isActionFinish = 0; /*0 means finish, 1 means action is running*/

  ActionStatus action_status;

  void ClearGroupList(int group_id);
  void GroupListAdd(int group_id, XpiderInfo::ActionIndex id, uint8_t *buffer);
  void ClearAllGroupList() ;

  bool ifGroupCanRun(uint8_t group_id);
  bool ifActionRunning();
  void Count();
  bool ifActionShouldStop(uint32_t current_time);
  uint8_t running_group_id = 0xff;
  uint8_t running_action_id = 0xff;
  bool is_free = true;
  XpiderInfo::GroupElement *SetCurrentAction(uint8_t action_num, bool enable, uint32_t start_time); /*false: action not finish, or no more action to do*/
  XpiderInfo::GroupElement *getCurrentAction();
  uint8_t getCurrentActionId();

};
#endif // XPIDER_INFO_H_


