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
#include "xpider_protocol.h"

#ifdef ARDUINO
#include "arduino_log.h"
#ifdef TEST_MODE
#include <Arduino.h>
#endif
#endif

XpiderProtocol::XpiderProtocol() {
}

void XpiderProtocol::Initialize(XpiderInfo* xpider_info) {
  xpider_info_ = xpider_info;
  memset(totalMsg, 0, MAX_MESSAGE_LENGTH);
}

void XpiderProtocol::GetBuffer(const MessageType &index, uint8_t** send_buffer, uint16_t *length) {
  switch (index) {
    case kHeartBeat: {
      *length = 20;

      send_msg_buffer_[0] = kHeartBeat;
      memcpy(&send_msg_buffer_[1], &xpider_info_->step_counter, 2);
      memcpy(&send_msg_buffer_[3], &xpider_info_->obstacle_distance, 2);

      int16_t temp_voltage = static_cast<int16_t>(xpider_info_->voltage*100.0f);
      memcpy(&send_msg_buffer_[5], &temp_voltage, 2);

      memcpy(&send_msg_buffer_[7], xpider_info_->yaw_pitch_roll, sizeof(float)*3);
      send_msg_buffer_[19] = xpider_info_->sound_level;
      
      break;
    }
    case kEye: {
      *length = 3;
      
      send_msg_buffer_[0] = kEye;
      send_msg_buffer_[1] = xpider_info_->eye_power;
      send_msg_buffer_[2] = xpider_info_->eye_angle;
      
      break;
    }
    case kFrontLeds: {
      *length = 7;
      
      send_msg_buffer_[0] = kFrontLeds;
      memcpy(&send_msg_buffer_[1], &xpider_info_->left_led_rgb, 3);
      memcpy(&send_msg_buffer_[4], &xpider_info_->right_led_rgb, 3);
      
      break;
    }
    case kMove: {
      *length = 3;

      send_msg_buffer_[0] = kMove;
      send_msg_buffer_[1] = xpider_info_->move;
      send_msg_buffer_[2] = xpider_info_->rotate;

      break;
    }
    case kStep: {
      *length = 3;

      send_msg_buffer_[0] = kStep;
      send_msg_buffer_[1] = xpider_info_->count_speed;
      send_msg_buffer_[2] = xpider_info_->count;

      break;
    }
    case kAutoMove: {
      *length = 8;

      send_msg_buffer_[0] = kAutoMove;
      send_msg_buffer_[1] = xpider_info_->rotate_speed;
      memcpy(&send_msg_buffer_[2], &xpider_info_->rotate_rad, 4);
      send_msg_buffer_[6] = xpider_info_->walk_speed;
      send_msg_buffer_[7] = static_cast<uint8_t>(xpider_info_->walk_step);

      break;
    }
    case kAutoPilot: {
      if(xpider_info_->autopilot_enable) {
        *length = 4;

        int16_t temp_autopilot = static_cast<int16_t>(xpider_info_->autopilot_heading*100.0f);
        memcpy(&send_msg_buffer_[2], &temp_autopilot, 2);
      } else {
        *length = 2;
      }

      send_msg_buffer_[0] = kAutoPilot;
      send_msg_buffer_[1] = xpider_info_->autopilot_enable;

      break;
    }
    case kGroupInfo: {
      /*generate msg by link list*/
      *length = 0;
      send_msg_buffer_[0] = kGroupInfo;
      send_msg_buffer_[1] = xpider_info_->group_info_ver;
      *length +=4; /*add header length*/
      for (int i = 0; i < xpider_info_->store_group_num; i++){
        send_msg_buffer_[*length] = xpider_info_->store_group_id[i]; /*add cat or real group id here*/
        *length += 1;
        send_msg_buffer_[*length] = xpider_info_->action_group_list[i].size();
        *length += 1;
        for(int j = 0; j < xpider_info_->action_group_list[i].size(); j++){
          XpiderInfo::GroupElement *temp = xpider_info_->action_group_list[i].get(j);
          /*Serial.print("get group");
          Serial.print(i);
          Serial.print(", element ");
          Serial.print(j);
          Serial.print(", id is ");
          Serial.println(temp->id, HEX);*/
          switch(temp->id){
            case XpiderInfo::idWalk:{
              send_msg_buffer_[*length] = XpiderInfo::idWalk;
              *length += 1;
              memcpy(&send_msg_buffer_[*length], temp->data, sizeof(XpiderInfo::WalkInfo));
              *length += sizeof(XpiderInfo::WalkInfo);
              break;
            }
            case XpiderInfo::idRotate:{
              send_msg_buffer_[*length] = XpiderInfo::idRotate;
              *length += 1;
              memcpy(&send_msg_buffer_[*length], temp->data, sizeof(XpiderInfo::RotateInfo));
              *length += sizeof(XpiderInfo::RotateInfo);
              break;
            }
            case XpiderInfo::idLed:{
              send_msg_buffer_[*length] = XpiderInfo::idLed;
              *length += 1;
              memcpy(&send_msg_buffer_[*length], temp->data, sizeof(XpiderInfo::LedInfo));
              *length += sizeof(XpiderInfo::LedInfo);
              break;
            }
            case XpiderInfo::idEye:{
              send_msg_buffer_[*length] = XpiderInfo::idEye;
              *length += 1;
              memcpy(&send_msg_buffer_[*length], temp->data, sizeof(XpiderInfo::EyeInfo));
              *length += sizeof(XpiderInfo::EyeInfo);
              break;
            }
            case XpiderInfo::idSpeaker:{
              send_msg_buffer_[*length] = XpiderInfo::idSpeaker;
              *length += 1;
              memcpy(&send_msg_buffer_[*length], temp->data, sizeof(XpiderInfo::SpeakerInfo));
              *length += sizeof(XpiderInfo::SpeakerInfo);
              break;
            }
            default:{
              //Serial.println("ERROR: Unknow action id when package");
              break;
            }
          }
        }
      }
      uint16_t temp_length = 0;
      temp_length = *length + 1; /*since length is subtitle, real length is subtitle + 1*/ 
      memcpy(&send_msg_buffer_[2], &temp_length, 2); /*copy length to header*/
      send_msg_buffer_[*length] = SumCRC(send_msg_buffer_, *length); /*add CRC to tail*/

      /*Serial.print("CRC is ");
      Serial.println(send_msg_buffer_[*length], HEX);

      for(int m = 0; m < *length+1; m++){
        Serial.print(send_msg_buffer_[m], HEX);
        Serial.print(" ");
      }
      Serial.println(" ");*/
      // buffer = send_msg_buffer_;
      break;
    }
    case kNNInfo: {
      /*generate msg by link list*/
      *length = 0;
      send_msg_buffer_[0] = kNNInfo;
      send_msg_buffer_[1] = xpider_info_->nn_info_ver;
      *length +=4; /*add header length*/
      send_msg_buffer_[*length] = xpider_info_->store_sensor_num;
      *length += 1;
      for(int index = 0; index < xpider_info_->store_sensor_num; index++){
        send_msg_buffer_[*length] = xpider_info_->nn_sensor_list[index];
        *length += 1;
      }
      send_msg_buffer_[*length] = xpider_info_->nn_neuron_num;
      *length += 1;
      for(int i = 0; i < xpider_info_->nn_neuron_num; i++){
        send_msg_buffer_[*length] = (xpider_info_->nn_data_list.get(i)).length;
        *length += 1;
        memcpy(&send_msg_buffer_[*length], (xpider_info_->nn_data_list.get(i)).neuron_bytes, (xpider_info_->nn_data_list.get(i)).length);
        *length += (xpider_info_->nn_data_list.get(i)).length;
        send_msg_buffer_[*length] = (xpider_info_->nn_data_list.get(i)).cat;
        *length += 1;
      }

      uint16_t temp_length = 0;
      temp_length = *length + 1; /*since length is subtitle, real length is subtitle + 1*/ 
      memcpy(&send_msg_buffer_[2], &temp_length, 2); /*copy length to header*/
      send_msg_buffer_[*length] = SumCRC(send_msg_buffer_, *length); /*add CRC to tail*/

      /*for(int m = 0; m < *length+1; m++){
        Serial.print(send_msg_buffer_[m], HEX);
        Serial.print(" ");
      }
      Serial.println(" ");*/
      // buffer = send_msg_buffer_;
      break;
    }
    case kRun: {
      *length = 1;

      send_msg_buffer_[0] = kRun;
      break;
    }
    case kEmergencyStop: {
      *length = 1;

      send_msg_buffer_[0] = kEmergencyStop;
      break;
    }
    case kUnknown:
    default: {
    }
  }

  *send_buffer = send_msg_buffer_;
}

XpiderProtocol::MessageType XpiderProtocol::GetMessage(const uint8_t *buffer, const uint16_t &length) {
  MessageType index = static_cast<MessageType>(buffer[0]);
  switch (index) {
    case kHeartBeat: {
      memcpy(&xpider_info_->step_counter, &buffer[1], 2);
      memcpy(&xpider_info_->obstacle_distance, &buffer[3], 2);

      int16_t temp_voltage;
      memcpy(&temp_voltage, &buffer[5], 2);
      xpider_info_->voltage = static_cast<float>(temp_voltage/100.0f);

      memcpy(xpider_info_->yaw_pitch_roll, buffer+7, sizeof(float)*3);

      xpider_info_->sound_level = buffer[19];
      break;
    }
    case kEye: {
      xpider_info_->eye_power = buffer[1];
      xpider_info_->eye_angle = buffer[2];
      break;
    }
    case kFrontLeds: {
      memcpy(&xpider_info_->left_led_rgb, &buffer[1], 3);
      memcpy(&xpider_info_->right_led_rgb, &buffer[4], 3);
      break;
    }
    case kMove: {
      xpider_info_->move = buffer[1];
      xpider_info_->rotate = buffer[2];
      break;
    }
    case kStep: {
      xpider_info_->count_speed = buffer[1];
      xpider_info_->count = buffer[2];
      break;
    }
    case kAutoMove: {
      xpider_info_->rotate_speed = buffer[1];
      memcpy(&xpider_info_->rotate_rad, &buffer[2], 4);
      xpider_info_->walk_speed = buffer[6];
      xpider_info_->walk_step = static_cast<int8_t>(buffer[7]);
      break;
    }
    case kAutoPilot: {
      xpider_info_->autopilot_enable = buffer[1];
      if(xpider_info_->autopilot_enable) {
        int16_t temp_autopilot;
        memcpy(&temp_autopilot, &buffer[2], 2);
        xpider_info_->autopilot_heading = static_cast<float>(temp_autopilot/100.0f);
      }
      break;
    }
    case kGroupInfo: {
      /*for(int i = 0; i < length; i++){
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
      }
      Serial.println(" ");*/
      /*input message and combine msg*/
      CombineMsg(buffer, length, totalMsg);
     break;
    }
    case kNNInfo: {
      /*for(int i = 0; i < length; i++){
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
      }
      Serial.println(" ");*/
      /*input message and combine msg*/
      CombineMsg(buffer, length, totalMsg);
      break;
    }
    case kRun: {
      break;
    }
    case kEmergencyStop: {
      break;
    }
    case kGetRegister: {
      /* do nothing here */
      break;
    }
    case kUpdateRegister:
    case kRegisterResponse: {
      switch(buffer[1]) {
        case kRegName: {
          memcpy(&xpider_info_->custom_data_.name, &buffer[3], buffer[2]);
          break;
        }
        case kRegVersion: {
          memcpy(&xpider_info_->custom_data_.hardware_version, &buffer[3], buffer[2]);
          break;
        }
        case kRegFirmware: {
          memcpy(&xpider_info_->firmware_version_, &buffer[3], buffer[2]);
          break;
        }
        case kRegUUID: {
          memcpy(&xpider_info_->custom_data_.uuid, &buffer[3], buffer[2]);
          break;
        }
        default: {
        }
      }
      break;
    }
    default: {
      break;
    }
  }

  return index;
}

bool XpiderProtocol::GetCRCResult(const uint8_t *buffer, const uint16_t &length){
  uint8_t sum = 0;
  for (int i = 0; i < length-1; i++){
    sum += buffer[i];
  }
  /*Serial.print("Sum = ");
  Serial.print(sum, HEX);
  Serial.print(", recv crc is ");
  Serial.println(buffer[length - 1], HEX);*/
  if(sum == buffer[length - 1])
    return true;
  else
    return false;
}
uint8_t XpiderProtocol::SumCRC(const uint8_t *buffer, const uint16_t &length){
  uint8_t sum = 0;
  for (int i = 0; i < length; i++){
    sum += buffer[i];
  }
  return sum;
}

/*this func used to combine received msg to a whole msg*/
void XpiderProtocol::CombineMsg(const uint8_t *buffer, const uint8_t &one_time_buffer_length, uint8_t *totalMsg){
  if(totalMsg[0] == '\0') {
    /*receive new msg, this package should be the first package of whole msg*/
    if(buffer[0] == 0x05){
      isWholeGroupInfo = false;
      /*new Group Info msg*/
      total_msg_position = 0;
      group_info_ver = buffer[1];
      memcpy(&group_info_unread_byte, &buffer[2], 2);
      //Serial.print("Receive new Group Info msg length = ");
      //Serial.println(group_info_unread_byte);
      memcpy(totalMsg, buffer, one_time_buffer_length);
      group_info_unread_byte -= one_time_buffer_length;
      total_msg_position = one_time_buffer_length;
      //Serial.print("Receive Group Info msg, group_info_unread_byte = ");
      //Serial.println(group_info_unread_byte);
    }else if(buffer[0] == 0x06) {
      isWholeNnInfo = false;
      /*new NN Info msg*/
      total_msg_position = 0;
      nn_info_ver = buffer[1];
      memcpy(&nn_info_unread_byte, &buffer[2], 2);
      //Serial.print("Receive new NN Info msg length = ");
      //Serial.println(nn_info_unread_byte);
      memcpy(totalMsg, buffer, one_time_buffer_length);
      nn_info_unread_byte -= one_time_buffer_length;
      total_msg_position += one_time_buffer_length;
      //Serial.print("Receive NN Info msg, nn_info_unread_byte = ");
      //Serial.println(nn_info_unread_byte);
    }
  }else{
    if(buffer[0] == 0x05){
      isWholeGroupInfo = false;
      if(group_info_unread_byte != 0){
        /*copy pure data no 1 byte header*/
        memcpy(totalMsg+total_msg_position, buffer+1, one_time_buffer_length);
        group_info_unread_byte -= one_time_buffer_length-1;
        /*not record length of header*/
        total_msg_position += one_time_buffer_length-1;
        //Serial.print("Receive Group Info msg, group_info_unread_byte = ");
        //Serial.println(group_info_unread_byte);
      }else{
        //Serial.println("Error: That will never happen! Group Info");
      }
    }else if(buffer[0] == 0x06) {
      isWholeNnInfo = false;
      if(nn_info_unread_byte != 0){
        /*copy pure data no 1 byte header*/
        memcpy(totalMsg+total_msg_position, buffer+1, one_time_buffer_length);
        nn_info_unread_byte -= one_time_buffer_length-1;
        /*not record length of header*/
        total_msg_position += one_time_buffer_length-1;
        //Serial.print("Receive NN Info msg, nn_info_unread_byte = ");
        //Serial.println(nn_info_unread_byte);
      }else{
        //Serial.println("Error: That will never happen! NN Info");
      }
    }
  }
  /*
  for(int i = 0; i < total_msg_position; i++){
    Serial.print(totalMsg[i], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");*/

  if(group_info_unread_byte == 0 && totalMsg[0] == 0x05){
    if(GetCRCResult(totalMsg, total_msg_position)){
      /*Receive all msg data for Group Info, parse it, and save to linklist*/
      ParseGroupMsg(totalMsg, total_msg_position);
      isWholeGroupInfo = true;
    }else{
      memset(totalMsg, 0, MAX_MESSAGE_LENGTH);
    }
  }
  if(nn_info_unread_byte == 0 && totalMsg[0] == 0x06){
    if(GetCRCResult(totalMsg, total_msg_position)){
      /*Receive all msg data for Group Info, parse it, and save to linklist*/
      ParseNnMsg(totalMsg, total_msg_position);
      isWholeNnInfo = true;
    }else{
      memset(totalMsg, 0, MAX_MESSAGE_LENGTH);
    }
  }
}

void XpiderProtocol::ParseGroupMsg(uint8_t *totalMsg, const uint16_t &length){
  /*Serial.print("Receive Whole Group Info msg, length =");
  Serial.println(length);
  for(int i = 0; i < *(uint16_t *)(totalMsg+2); i++){
    Serial.print(totalMsg[i], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");*/
  /*compare ver*/
  //if(group_info_ver != xpider_info_->group_info_ver){
    xpider_info_->group_info_ver = group_info_ver;
    for(int i = 0; i < xpider_info_->store_group_num; i++){
      xpider_info_->ClearGroupList(i);
    }
    xpider_info_->store_group_num = 0;
    memset(xpider_info_->store_group_id, 0, MAX_GROUP_NUM);

    /**insert group info to xpider info list*/
    for(int i = 4; i < length-1; i++){   /*Start from group info, skip header and CRC*/
      //Serial.print("Fill group ");
      //Serial.println(xpider_info_->store_group_num); 
      xpider_info_->store_group_id[xpider_info_->store_group_num] = totalMsg[i];
      i++;
      uint8_t action_num_in_group = totalMsg[i];
      #ifdef TEST_MODE
      Serial.print("this group has ");
      Serial.print(action_num_in_group);
      Serial.println(" actions ");
      #endif
      i++; // jump to actions in group
      for(int j = 0; j < action_num_in_group; j++){
        XpiderInfo::ActionIndex index = static_cast<XpiderInfo::ActionIndex>(totalMsg[i]);
        i++;
        #ifdef TEST_MODE
        Serial.print("Action ");
        Serial.print(j);
        Serial.print(" is ");
        Serial.println(index);
        #endif
        switch (index) { 
          case XpiderInfo::idWalk: {
            XpiderInfo::WalkInfo w1;
            memcpy(&w1, &totalMsg[i], sizeof(XpiderInfo::WalkInfo));
            i += sizeof(XpiderInfo::WalkInfo);
            #ifdef TEST_MODE
            Serial.print("Walk speed is ");
            Serial.println(w1.speed, HEX);
            Serial.print("time is ");
            Serial.println(w1.time, HEX);
            #endif
            xpider_info_->GroupListAdd(xpider_info_->store_group_num, XpiderInfo::idWalk, (uint8_t*)&w1);
            break;
          }
          case XpiderInfo::idRotate: {
            XpiderInfo::RotateInfo r1;
            memcpy(&r1, &totalMsg[i], sizeof(XpiderInfo::RotateInfo));
            i += sizeof(XpiderInfo::RotateInfo);
            #ifdef TEST_MODE
            Serial.print("Rotate speed is ");
            Serial.println(r1.speed, HEX);
            Serial.print("time is ");
            Serial.println(r1.time, HEX);
            #endif
            xpider_info_->GroupListAdd(xpider_info_->store_group_num, XpiderInfo::idRotate, (uint8_t*)&r1);
            break;
          }
          case XpiderInfo::idLed: {
            XpiderInfo::LedInfo l1;
            memcpy(&l1, &totalMsg[i], sizeof(XpiderInfo::LedInfo));
            #ifdef TEST_MODE
            Serial.print("led is ");
            Serial.print(l1.led[0], HEX);
            Serial.print(l1.led[1], HEX);
            Serial.print(l1.led[2], HEX);
            Serial.print(l1.led[3], HEX);
            Serial.print(l1.led[4], HEX);
            Serial.println(l1.led[5], HEX);
            Serial.print("time is ");
            Serial.println(l1.time, HEX);
            #endif
            i += (sizeof(XpiderInfo::LedInfo)); //since when this group loop finish, i will add 1 automaticlly
            xpider_info_->GroupListAdd(xpider_info_->store_group_num, XpiderInfo::idLed, (uint8_t*)&l1);
            break;
          }
          case XpiderInfo::idEye: {
            XpiderInfo::EyeInfo e1;
            memcpy(&e1, &totalMsg[i], sizeof(XpiderInfo::EyeInfo));
            i += sizeof(XpiderInfo::EyeInfo);
            #ifdef TEST_MODE
            Serial.print("angle is ");
            Serial.println(e1.angle, HEX);
            Serial.print("time is ");
            Serial.println(e1.time, HEX);
            #endif
            xpider_info_->GroupListAdd(xpider_info_->store_group_num, XpiderInfo::idEye, (uint8_t*)&e1);
            break;
          }
          case XpiderInfo::idSpeaker: {
            XpiderInfo::SpeakerInfo s1;
            memcpy(&s1, &totalMsg[i], sizeof(XpiderInfo::SpeakerInfo));
            i += (sizeof(XpiderInfo::SpeakerInfo));
            xpider_info_->GroupListAdd(xpider_info_->store_group_num, XpiderInfo::idSpeaker, (uint8_t*)&s1);
            break;
          }
          default: {
            #ifdef TEST_MODE
            Serial.print("ERROR: Recv unknow action ID = ");
            Serial.println(index);
            #endif
            return;
          }
        }
      }
      i -= 1; //since when this group loop finish, i will add 1 automaticlly, So here 
      #ifdef TEST_MODE
      Serial.print("Parse Group ");
      Serial.print(xpider_info_->store_group_num);
      Serial.println(" Msg Finish, Go to next ");
      #endif
      xpider_info_->store_group_num++;
    }
    /*Save to EEPROM*/

//  } else {
//    /*same ver , no need to do anything*/
//    Serial.print("Same Group Info ver, Skip");
//  }
  return;
}

void XpiderProtocol::ParseNnMsg(uint8_t *totalMsg, const uint16_t &length){
  /*Serial.println("Receive Whole NN Info msg");
  for(int i = 0; i < length; i++){
    Serial.print(totalMsg[i], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");*/
  //if(nn_info_ver != xpider_info_->nn_info_ver) {
    xpider_info_->nn_info_ver = nn_info_ver;
    xpider_info_->store_sensor_num = 0;
    memset(xpider_info_->nn_sensor_list, XpiderInfo::idUnknowSensor, MAX_SENSOR_NUM);
    /*start to fill data*/
    uint8_t i = 0, j = 0;
    xpider_info_->store_sensor_num = totalMsg[4];
    i += 5;
    #ifdef TEST_MODE
    //Serial.println(xpider_info_->store_sensor_num);
    #endif
    for (j = 0; j < xpider_info_->store_sensor_num; j++){
      xpider_info_->nn_sensor_list[j] = static_cast<XpiderInfo::SensorIndex>(totalMsg[i]);
      #ifdef TEST_MODE
      //Serial.println(xpider_info_->nn_sensor_list[j]);
      #endif
      i++;
    }
    /*Clean all neuron info*/
    xpider_info_->nn_data_list.clear();
    /*get nn num*/
    xpider_info_->nn_neuron_num = totalMsg[i];
    i += 1;
    for( int n = 0; n < xpider_info_->nn_neuron_num; n++){
      XpiderInfo::NeuronInfo nnInfo;
      nnInfo.length = totalMsg[i];
      i += 1;
      memcpy(nnInfo.neuron_bytes, &totalMsg[i], nnInfo.length);
      i += nnInfo.length;
      nnInfo.cat = totalMsg[i];
      i += 1;
      xpider_info_->nn_data_list.add(nnInfo);
    }
    /*Save to EEPROM*/

//  } else {
//    /*same ver , no need to do anything*/
//    Serial.print("Same NN Info ver, Skip");
//  }
  /*To be add*/
  /*input the NN parameters to NN, here*/

  /********************************/
  return;
}

void XpiderProtocol::GetRegister(const RegIndex &register_index, uint8_t **send_buffer, uint16_t *length) {
  *length = 2;
  uint8_t *buffer = new uint8_t[2];

  buffer[0] = kGetRegister;
  buffer[1] = register_index;

  *send_buffer = buffer;
}

void XpiderProtocol::UpdateRegister(const RegIndex &register_index, uint8_t **send_buffer, uint16_t *length) {
  send_msg_buffer_[0] = kUpdateRegister;
  switch(register_index) {
    case kRegName: {
      send_msg_buffer_[1] = kRegName;
      send_msg_buffer_[2] = strlen(xpider_info_->custom_data_.name)+1;
      memcpy(&send_msg_buffer_[3], &xpider_info_->custom_data_.name, send_msg_buffer_[2]);
      *length = 3 + send_msg_buffer_[2];
      break;
    }
    case kRegVersion: {
      send_msg_buffer_[1] = kRegVersion;
      send_msg_buffer_[2] = strlen(xpider_info_->custom_data_.hardware_version)+1;
      memcpy(&send_msg_buffer_[3], &xpider_info_->custom_data_.hardware_version, send_msg_buffer_[2]);
      *length = 3 + send_msg_buffer_[2];
      break;
    }
    case kRegFirmware: {
      send_msg_buffer_[1] = kRegFirmware;
      send_msg_buffer_[2] = strlen(xpider_info_->firmware_version_)+1;
      memcpy(&send_msg_buffer_[3], &xpider_info_->firmware_version_, send_msg_buffer_[2]);
      *length = 3 + send_msg_buffer_[2];
      break;
    }
    case kRegUUID: {
      send_msg_buffer_[1] = kRegUUID;
      send_msg_buffer_[2] = 4;
      memcpy(&send_msg_buffer_[3], &xpider_info_->custom_data_.uuid, send_msg_buffer_[2]);
      *length = 3 + send_msg_buffer_[2];
    }
  }

  uint8_t* buffer = new uint8_t[*length];
  memcpy(buffer, send_msg_buffer_, *length);
  *send_buffer = buffer;
}

void XpiderProtocol::RegisterResponse(const RegIndex &register_index, uint8_t **send_buffer, uint16_t *length) {
  send_msg_buffer_[0] = kRegisterResponse;

  switch(register_index) {
    case kRegName: {
      send_msg_buffer_[1] = kRegName;
      send_msg_buffer_[2] = strlen(xpider_info_->custom_data_.name)+1;
      memcpy(&send_msg_buffer_[3], &xpider_info_->custom_data_.name, send_msg_buffer_[2]);
      *length = 3 + send_msg_buffer_[2];
      break;
    }
    case kRegVersion: {
      send_msg_buffer_[1] = kRegVersion;
      send_msg_buffer_[2] = strlen(xpider_info_->custom_data_.hardware_version)+1;
      memcpy(&send_msg_buffer_[3], &xpider_info_->custom_data_.hardware_version, send_msg_buffer_[2]);
      *length = 3 + send_msg_buffer_[2];
      break;
    }
    case kRegFirmware: {
      send_msg_buffer_[1] = kRegFirmware;
      send_msg_buffer_[2] = strlen(xpider_info_->firmware_version_)+1;
      memcpy(&send_msg_buffer_[3], &xpider_info_->firmware_version_, send_msg_buffer_[2]);
      *length = 3 + send_msg_buffer_[2];
      break;
    }
    case kRegUUID: {
      send_msg_buffer_[1] = kRegUUID;
      send_msg_buffer_[2] = 4;
      memcpy(&send_msg_buffer_[3], &xpider_info_->custom_data_.uuid, send_msg_buffer_[2]);
      *length = 3 + send_msg_buffer_[2];
    }
  }

  uint8_t *buffer = new uint8_t[*length];
  memcpy(buffer, send_msg_buffer_, *length);
  *send_buffer = buffer;
}
