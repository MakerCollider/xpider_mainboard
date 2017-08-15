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

#include <Arduino.h>
#include <SoftwareSerial.h>

#include "Arduhdlc.h"

#include "pid.h"
#include "task.h"
#include "linked_list.h"

#include "xpider_info.h"
#include "xpider_memory.h"
#include "xpider_neuron.h"
#include "xpider_protocol.h"
#include "xpider_inside_protocol.h"

#include "arduino_log.h"

#define FIRMWARE_VERSION "3.3.0"

#define HEART_BEAT_PERIOD 100000
#define COMM_CHECK_PERIOD 4000000
#define NN_INPUT_PERIOD   2000000
#define AUTO_PILOT_PERIOD 100000
#define NN_RUN_PERIOD     10000

#define MIN_VOLTAGE 3.4f

enum TASK_ID {
  TASK_LED = 0,
  TASK_COMMAND = 1,
  TASK_HEARTBEAT = 2,
  TASK_NN_INPUT = 3,
  TASK_NN_RUN = 4,
};

PID *g_pid;
Task g_task_heartbeat, g_task_nn_input, g_task_nn_run, g_task_auto_pilot;
Task g_task_check_comm;

bool g_controller_ack = false;
bool g_comm_watchdog = true;

XpiderInfo g_xpider_info;
XpiderNeuron g_xpider_neuron;
XpiderMemory *g_xpider_memory;
XpiderProtocol g_xpider_protocol;
XpiderInsideProtocol g_xpider_inside_protocol;

Arduhdlc *g_inside_hdlc, *g_network_hdlc;

SoftwareSerial g_controller_serial(3, 2);

void inside_sendframe(const uint8_t *databuffer, uint16_t bufferlength) {
  g_controller_serial.write(databuffer, bufferlength);
}

void network_sendframe(const uint8_t *databuffer, uint16_t bufferlength) {
  Serial1.write(databuffer, bufferlength);
}

void inside_frame_handler(const uint8_t *data, uint16_t length) {
  g_xpider_inside_protocol.Decode(data, length);
}

void network_frame_handler(const uint8_t *data, uint16_t length) {
  LOG_PRINT("Get Command: ");
  XpiderProtocol::MessageType message_type = g_xpider_protocol.GetMessage(data, length);

  switch (message_type) {
    case XpiderProtocol::kMove: {
      LOG_PRINT("Move: ");
      LOG_PRINT(g_xpider_info.move);
      LOG_PRINT(", Rotate: ");
      LOG_PRINTLN(g_xpider_info.rotate);
      g_xpider_inside_protocol.SetMove(g_xpider_info.move);
      g_xpider_inside_protocol.SetRotate(g_xpider_info.rotate);
      break;
    }
    case XpiderProtocol::kStep: {
      LOG_PRINT("Count Speed: ");
      LOG_PRINT(g_xpider_info.count_speed);
      LOG_PRINT(", Count: ");
      LOG_PRINT(g_xpider_info.count);
      g_xpider_inside_protocol.SetStep(g_xpider_info.count_speed, g_xpider_info.count);
      break;
    }
    case XpiderProtocol::kAutoMove: {
      LOG_PRINT("Auto Move: rotate_speed ");
      LOG_PRINT(g_xpider_info.rotate_speed);
      LOG_PRINT(", rotate_rad ");
      LOG_PRINT(g_xpider_info.rotate_rad);
      LOG_PRINT(", walk_speed ");
      LOG_PRINT(g_xpider_info.walk_speed);
      LOG_PRINT(", walk_step ");
      LOG_PRINTLN(g_xpider_info.walk_step);
      g_xpider_inside_protocol.SetAutoMove(g_xpider_info.rotate_speed, g_xpider_info.rotate_rad,
                                           g_xpider_info.walk_speed, g_xpider_info.walk_step);
      break;
    }
    case XpiderProtocol::kFrontLeds: {
      uint8_t leds[6];
      memcpy(leds, &g_xpider_info.left_led_rgb, 3);
      memcpy(leds+3, &g_xpider_info.right_led_rgb, 3);
      g_xpider_inside_protocol.SetFrontLeds(leds);

      LOG_PRINT("LED ");
      for(uint8_t i=0; i<6; i++) {
        LOG_PRINTF(leds[i], HEX);
        LOG_PRINT(" ");
      }
      LOG_PRINTLN("");
      break;
    }
    case XpiderProtocol::kEye: {
      LOG_PRINT("Eye: ");
      LOG_PRINTLN(g_xpider_info.eye_angle);
      g_xpider_inside_protocol.SetEye(g_xpider_info.eye_angle);
      break;
    }
    case XpiderProtocol::kAutoPilot: {
      g_task_auto_pilot.setEnabled(true);
      break;
    }
    case XpiderProtocol::kGroupInfo: {
      LOG_PRINTLN("GroupInfo");
      if(g_xpider_protocol.isWholeGroupInfoMsg()){
        /*recv whole Group Info msg, need to write to flash*/
        uint8_t *tempbuffer = NULL;
        tempbuffer = g_xpider_protocol.getWholeMsg();
        LOG_PRINT("get Group info msg from protocol, length is ");
        LOG_PRINTLN(*(uint16_t*)&tempbuffer[2]);
        g_xpider_memory->WriteGroupInfoData(tempbuffer, *(uint16_t*)&tempbuffer[2]);
        memset(tempbuffer, 0, MAX_MESSAGE_LENGTH);
      }
      break;
    }
    case XpiderProtocol::kNNInfo: {
      LOG_PRINTLN("NNInfo");
      if(g_xpider_protocol.isWholeNnInfoMsg()){
        /*recv whole Nn Info msg, need to write to flash*/
        uint8_t *tempbuffer = NULL;
        tempbuffer = g_xpider_protocol.getWholeMsg();
        LOG_PRINT("get Nn Info msg from protocol, length is ");
        LOG_PRINTLN(*(uint16_t*)&tempbuffer[2]);
        g_xpider_memory->WriteNnInfoData(tempbuffer, *(uint16_t*)&tempbuffer[2]);
        memset(tempbuffer, 0, MAX_MESSAGE_LENGTH);
      }
      break;
    } 
    case XpiderProtocol::kRun: {
      /*send nn data to neural network, and run sensor collection task.
      In this task, it will classify the input and give a output,
      then do every action in group info*/
      /*clean all the neuron info before learn*/
      LOG_PRINTLN("nn run");
      g_xpider_neuron.Forget();
      for(int i=0; i<g_xpider_info.nn_neuron_num; i++){
        XpiderInfo::NeuronInfo nnInfo = g_xpider_info.nn_data_list.get(i);
        g_xpider_neuron.Learn(nnInfo.neuron_bytes, 
                              nnInfo.length,
                              nnInfo.cat);
      }
      g_task_nn_input.setEnabled(true);
      g_task_nn_run.setEnabled(true);
      break;
    }
    case XpiderProtocol::kEmergencyStop: {
      g_task_nn_input.setEnabled(false);
      g_task_nn_run.setEnabled(false);
      g_task_auto_pilot.setEnabled(false);

      g_xpider_info.SetCurrentAction(0, false, 0);
      g_xpider_info.is_free = true;
      g_xpider_info.running_group_id = 0xff;
      g_xpider_info.running_action_id = 0xff;

      g_xpider_inside_protocol.SetMove(0);
      g_xpider_inside_protocol.SetRotate(0);
      uint8_t leds[6] = {0};
      g_xpider_inside_protocol.SetFrontLeds(leds);
      break;
    }
    case XpiderProtocol::kGetRegister: {
      LOG_PRINT("Get Register: ");
      LOG_PRINTLN(static_cast<int>(data[1]));
      UploadRegister(static_cast<XpiderProtocol::RegIndex>(data[1]));
      break;
    }
    case XpiderProtocol::kUpdateRegister: {
      /*
       * GetMessage function has already update info on g_xpider_info,
       * so here we just update spi flash
       */
      g_xpider_memory->WriteCustomData(&g_xpider_info.custom_data_);

      /* we also need upload new register value to master */
      UploadRegister(static_cast<XpiderProtocol::RegIndex>(data[1]));
      break;
    }
    default: {
      LOG_PRINTLN("Unknown");
    }
  }

  // LOG_PRINTLN("");
}

void InsideSendData(const uint8_t* buffer, uint8_t buffer_length) {
  g_inside_hdlc->frameDecode(buffer, buffer_length);
}

void AutoPilot() {
  float delta_rad, speed_value;

  if(g_xpider_info.autopilot_enable) {
    delta_rad = g_xpider_info.autopilot_heading - g_xpider_info.yaw_pitch_roll[0];
    int direction = delta_rad>0 ? -1 : 1;
    delta_rad = abs(delta_rad)>PI ? (2.0*PI-abs(delta_rad))*direction : delta_rad;

    if(abs(delta_rad) > 0.27) {
      g_xpider_inside_protocol.SetMove(0);
      speed_value = g_pid->update(0, delta_rad);
      speed_value = constrain(speed_value, -100, 100);
      g_xpider_inside_protocol.SetRotate(static_cast<int8_t>((-1)*speed_value));
    } else {
      g_xpider_inside_protocol.SetRotate(0);
      g_xpider_inside_protocol.SetMove(80);
    }

     String serial_string = "AutoPilotDegree: " + String(g_xpider_info.autopilot_heading);
     serial_string += ", Heading: " + String(g_xpider_info.yaw_pitch_roll[0]) + ", Delta: " + delta_rad;
     LOG_PRINTLN(serial_string);
  } else {
    g_xpider_inside_protocol.SetRotate(0);
    g_xpider_inside_protocol.SetMove(0);
    g_pid->clear();
    g_task_auto_pilot.setEnabled(false);
  }
}

void UploadRegister(XpiderProtocol::RegIndex register_index) {
  uint8_t *buffer;
  uint16_t buffer_length;

  g_xpider_protocol.RegisterResponse(register_index, &buffer, &buffer_length);
  g_network_hdlc->frameDecode(buffer, buffer_length);
}

void GetHeartBeat(XpiderInsideProtocol::HeartBeatStruct heartbeat) {
  g_xpider_info.step_counter = heartbeat.step_counter;
  g_xpider_info.obstacle_distance = heartbeat.obstacle_distance;
  g_xpider_info.voltage = heartbeat.battery_voltage;
  memcpy(g_xpider_info.yaw_pitch_roll, heartbeat.yaw_pitch_roll, sizeof(float)*3);

  /* get extra sensor value when receive data from controller */
  int sound_level = analogRead(SOUND_SENSOR);
  g_xpider_info.sound_level = map(sound_level, 0, 1023, 0, 255);

  // LOG_PRINT("Get heartbeat, ");
  // LOG_PRINT("Step: ");
  // LOG_PRINT(g_xpider_info.step_counter);
  // LOG_PRINT(", Obstacle: ");
  // LOG_PRINT(g_xpider_info.obstacle_distance);
  // LOG_PRINT(", Voltage: ");
  // LOG_PRINT(g_xpider_info.voltage);
  // LOG_PRINT(", Ypr: ");
  // LOG_PRINT(g_xpider_info.yaw_pitch_roll[0]);
  // LOG_PRINT(" ");
  // LOG_PRINT(g_xpider_info.yaw_pitch_roll[1]);
  // LOG_PRINT(" ");
  // LOG_PRINT(g_xpider_info.yaw_pitch_roll[2]);
  // LOG_PRINT(", Sound: ");
  // LOG_PRINT(g_xpider_info.sound_level);
  // LOG_PRINTLN("");
}

void UploadHeartBeat() {
  uint8_t *buffer;
  uint16_t buffer_length;

  /* Get heartbeat buffer */
  g_xpider_protocol.GetBuffer(g_xpider_protocol.kHeartBeat, &buffer, &buffer_length);
  
  /* Encode with hdlc and send */
  g_network_hdlc->frameDecode(buffer, buffer_length);
}

void RegisterResponse(XpiderInsideProtocol::RegisterIndex register_index, const uint8_t *value, uint8_t length) {
  switch(register_index) {
    case XpiderInsideProtocol::kControllerVersion: {
      g_controller_ack = true;
      memcpy(g_xpider_info.controller_version, value, length);
      LOG_PRINT("Controller Version: ");
      LOG_PRINT(g_xpider_info.controller_version);
      LOG_PRINT(", length: ");
      LOG_PRINT(strlen(g_xpider_info.controller_version));
      LOG_PRINT(", input length: ");
      LOG_PRINTLN(static_cast<int>(length));
    }
  }
}

void CommCheck() {
  if(g_comm_watchdog == true) {
    g_comm_watchdog = false;
  } else {
    g_task_nn_input.setEnabled(false);
    g_task_nn_run.setEnabled(false);
    g_task_auto_pilot.setEnabled(false);

    g_xpider_inside_protocol.SetMove(0);
    g_xpider_inside_protocol.SetRotate(0);
    uint8_t leds[6] = {0};
    g_xpider_inside_protocol.SetFrontLeds(leds);
  }
}

void ReceiveData() {
  uint16_t bytes_available = 0;

  /* Receive data from wifi module */
  bytes_available = Serial1.available();
  while(bytes_available > 0) {
    /* put this flag into frame handler */
    g_comm_watchdog = true;
    g_network_hdlc->charReceiver(Serial1.read());
    bytes_available --;
  }

  /* Receive data from controller */
  bytes_available = g_controller_serial.available();
  while(bytes_available > 0) {
    g_inside_hdlc->charReceiver(g_controller_serial.read());
    bytes_available --;
  }
}

void DriveXpiderByNN(uint8_t action_id){
  LOG_PRINTLN("DriveXpiderByNN");
  XpiderInfo::GroupElement * temp = g_xpider_info.SetCurrentAction(action_id, true, millis());
  if(temp != NULL)  {
    LOG_PRINT("NN action ");
    LOG_PRINTF(temp->id, HEX);
    LOG_PRINTLN(" start ");
    switch(temp->id){
      case XpiderInfo::idWalk:{
        g_xpider_inside_protocol.SetMove(((XpiderInfo::WalkInfo *)temp->data)->speed);
        break;
      }
      case XpiderInfo::idRotate:{
        g_xpider_inside_protocol.SetRotate(((XpiderInfo::RotateInfo *)temp->data)->speed);
        break;
      }
      case XpiderInfo::idLed:{

        g_xpider_inside_protocol.SetFrontLeds(((XpiderInfo::LedInfo *)temp->data)->led);
        break;
      }
      case XpiderInfo::idEye:{
        g_xpider_inside_protocol.SetEye(((XpiderInfo::EyeInfo *)temp->data)->angle);
        break;
      }
      case XpiderInfo::idSpeaker:{
        //g_xpider_control->MoveEye(((XpiderInfo::EyeInfo *)temp->data)->angle);
        break;
      }
      default: {
        LOG_PRINTLN("ERROR: Unknow action when doing the action!");
      }
    }
  }else{
    /*no action to do*/
    LOG_PRINT("No Action to do, Group finished ");
    LOG_PRINTLN(g_xpider_info.running_group_id);
    g_xpider_info.SetCurrentAction(action_id, false, 0);
    g_xpider_info.is_free = true;
    g_xpider_info.running_group_id = 0xff;
    g_xpider_info.running_action_id = 0xff;
  }
}

void NN_Input() {
  LOG_PRINTLN("Read Sensor Input and execute action");
  /*get sensor value, and combine nn input*/
  uint8_t temp[128] = {0};
  int length = 0;
  for (int i = 0; i < g_xpider_info.store_sensor_num; i++){
    LOG_PRINT("Sensor ");
    LOG_PRINT(i);
    LOG_PRINT(", value: ");
    switch(g_xpider_info.nn_sensor_list[i]){
      case XpiderInfo::idDistance: {
        temp[length] = map(g_xpider_info.obstacle_distance, 40, 500, 0, 255);
        LOG_PRINTF(g_xpider_info.obstacle_distance, HEX);
        LOG_PRINT(", remap: ");
        LOG_PRINTLN(temp[length]);
        length += 1;
        break;
      }
      case XpiderInfo::idMic: {
        temp[length] = g_xpider_info.sound_level;
        LOG_PRINTF(g_xpider_info.sound_level, HEX);
        LOG_PRINT(", remap: ");
        LOG_PRINTLN(temp[length]);        
        length += 1;
        break;
      }
      case XpiderInfo::idGyro: {
        for(uint8_t i=0; i<3; i++) {
          temp[length] = (g_xpider_info.yaw_pitch_roll[i]-(PI*2)) * (255-0) / (PI*2 - 0) + 0;
          LOG_PRINTF(g_xpider_info.yaw_pitch_roll[i], HEX);
          LOG_PRINT(" ");
          length += 1;
        }
        LOG_PRINT(", remap");
        LOG_PRINT(temp[length-3]);
        LOG_PRINT(", ");
        LOG_PRINT(temp[length-2]);
        LOG_PRINT(", ");
        LOG_PRINTLN(temp[length-1]);
        break;
      }
      case XpiderInfo::idUnknowSensor: {
        break;
      }
      default:{
        break;
      }
    }
  }
  LOG_PRINT("Sensor data buffer is");
  for(int i = 0; i < length; i++){
    LOG_PRINT(" ");
    LOG_PRINTF(temp[i], HEX);
  }
  LOG_PRINTLN("");

  int cat = g_xpider_neuron.Classify(&temp[0], length);
  if(cat != -1) {
    int group_id = -1;
    /*
     * there has mapping relation between cat and group ID
     * cat maybe 1,3, but in list cat1 and cat3 stored in postion 1 and 2
     */
    for(int i = 0; i < g_xpider_info.store_group_num; i++) {
      if(g_xpider_info.store_group_id[i] == cat) {
        group_id = i;
      }
    }

    /*
     * different action group can be run or
     * the previous action finish
     */
    if(g_xpider_info.ifGroupCanRun(group_id)) {
      LOG_PRINT("Change running group to ");
      LOG_PRINTLN(group_id);
      g_xpider_inside_protocol.SetMove(0);
      g_xpider_inside_protocol.SetRotate(0);
      uint8_t leds[6] = {0};
      g_xpider_inside_protocol.SetFrontLeds(leds);

      g_xpider_info.is_free = false;
      g_xpider_info.running_group_id = group_id;
      g_xpider_info.running_action_id = 0;
      DriveXpiderByNN(g_xpider_info.running_action_id);
    } else {
      LOG_PRINT("Cat ");
      LOG_PRINT(cat);
      LOG_PRINTLN(" can not be run!");
    }
  } else {
    LOG_PRINTLN("Classify unknow");
  }
}

void NN_Run() {
  if(g_xpider_info.ifActionRunning()) {
    /*action is running*/
    // g_xpider_info.Count();
    if(g_xpider_info.ifActionShouldStop(millis())){
      XpiderInfo::GroupElement * temp = g_xpider_info.getCurrentAction();
      if(temp != NULL) {
        switch(temp->id){
          case XpiderInfo::idWalk:
          case XpiderInfo::idRotate:
          case XpiderInfo::idLed:
          case XpiderInfo::idEye:
          case XpiderInfo::idSpeaker: {
            g_xpider_inside_protocol.SetMove(0);
            g_xpider_inside_protocol.SetRotate(0);
            uint8_t leds[6] = {0};
            g_xpider_inside_protocol.SetFrontLeds(leds);
            break;
          }
          default: {
            LOG_PRINTLN("ERROR: Unknow action when doing the action!");
          }
        }
        LOG_PRINT("Action ");
        LOG_PRINT(temp->id);
        LOG_PRINTLN(" finished !");
      }
      /*run next action*/
      DriveXpiderByNN(g_xpider_info.running_action_id+1);
    }else{
      //LOG_PRINTLN("Action running, waiting");
    }
  }else{
    //LOG_PRINTLN("No action running");
  }
}

void LowVoltageAlert() {
  uint8_t led_on[6] = {50, 0, 0, 0, 50, 0};
  uint8_t led_off[6] = {0, 0, 0, 0, 0, 0};

  LOG_PRINT("Low Voltage Alert....");
  g_xpider_inside_protocol.SetFrontLeds(led_on);
  delay(500);
  g_xpider_inside_protocol.SetFrontLeds(led_off);
  delay(500);
  tone(5, 1200, 200);
  delay(500);
  tone(5, 1200, 200);
  LOG_PRINTLN("Done!");
}

void setup() {
  #ifdef LOG
    Serial.begin(115200);
    while(!Serial);
  #endif // LOG

  LOG_PRINTLN("Xpider Initialize...");

  tone(SPEAKER, 1000, 200);
  delay(500);
  tone(SPEAKER, 1000, 200);

  pinMode(SOUND_SENSOR, INPUT);

  LOG_PRINT("Initialize hdlc...");
  g_inside_hdlc = new Arduhdlc(&inside_sendframe, &inside_frame_handler, 256);
  g_network_hdlc = new  Arduhdlc(&network_sendframe, &network_frame_handler, 256);
  LOG_PRINTLN("OK!");

  LOG_PRINT("Check Custom data...");
  g_xpider_memory = XpiderMemory::instance();
  g_xpider_memory->Initialize();
  if(!g_xpider_memory->CheckCustomData()) {
    LOG_PRINTLN("No custom data on SPI flash");

    /* Format spi flash and initialize custom data file */
    g_xpider_memory->EraseFlash();
    g_xpider_memory->InitializeCustomData();
  }
  g_xpider_memory->OpenDataFile();
  g_xpider_memory->ReadCustomData(&g_xpider_info.custom_data_);
  String temp = FIRMWARE_VERSION;
  memcpy(g_xpider_info.firmware_version, temp.c_str(), temp.length());
  LOG_PRINTLN("OK!");
  LOG_PRINT("Name: ");
  LOG_PRINT(g_xpider_info.custom_data_.name);
  LOG_PRINT(", hardware_version: ");
  LOG_PRINT(g_xpider_info.custom_data_.hardware_version);
  LOG_PRINT(", UUID: ");
  LOG_PRINT(g_xpider_info.custom_data_.uuid);
  LOG_PRINT(", firmware_version: ");
  LOG_PRINTLN(g_xpider_info.firmware_version);

  LOG_PRINT("Initialize protocol...");
  g_xpider_protocol.Initialize(&g_xpider_info);

  XpiderInsideProtocol::CallbackListStruct callback_list;
  callback_list.heartbeat = &GetHeartBeat;
  callback_list.register_response = &RegisterResponse;
  g_xpider_inside_protocol.Initialize(&InsideSendData, callback_list);
  LOG_PRINTLN("OK!");

  LOG_PRINT("NN initialize...");
  g_xpider_neuron.Initialize();
  g_task_nn_input.init(NN_INPUT_PERIOD, NN_Input);
  g_task_nn_input.setEnabled(false);
  g_task_nn_run.init(NN_RUN_PERIOD, NN_Run);
  g_task_nn_run.setEnabled(false);
  LOG_PRINTLN("OK!");

  LOG_PRINT("Autopilot task initialize...");
  g_pid = new PID(55, 16, 0, -100, 100);
  g_task_auto_pilot.init(AUTO_PILOT_PERIOD, AutoPilot);
  g_task_auto_pilot.setEnabled(false);
  LOG_PRINTLN("OK!");

  LOG_PRINT("Heartbeat task initialize...");
  g_task_heartbeat.init(HEART_BEAT_PERIOD, UploadHeartBeat);
  g_task_heartbeat.setEnabled(true);
  LOG_PRINTLN("OK!");

  LOG_PRINT("CommCheck task initialize...");
  g_task_check_comm.init(COMM_CHECK_PERIOD, CommCheck);
  g_task_check_comm.setEnabled(true);
  LOG_PRINTLN("OK!");

  LOG_PRINT("Start Serial...");
  Serial1.begin(115200);
  g_controller_serial.begin(9600);
  LOG_PRINTLN("OK!");

  while(g_controller_ack == false) {
    g_xpider_inside_protocol.GetRegister(XpiderInsideProtocol::kControllerVersion);
    delay(500);
    ReceiveData();
    LOG_PRINTLN("Get controller version...");
  }

  LOG_PRINTLN("All Done!");
}

void loop() {
  /* Receive data from wifi and controller*/
  ReceiveData();

  /* Check comm status */
  g_task_check_comm.trigger(micros());

  /* Upload xpider info through wifi */
  // UploadHeartBeat();
  g_task_heartbeat.trigger(micros());

  if(g_xpider_info.voltage >= MIN_VOLTAGE) {
    g_task_auto_pilot.trigger(micros());
    g_task_nn_input.trigger(micros());
    g_task_nn_run.trigger(micros());
  } else if(g_xpider_info.voltage != 0.0f) {
    g_task_nn_input.setEnabled(false);
    g_task_nn_run.setEnabled(false);
    g_task_auto_pilot.setEnabled(false);

    g_xpider_inside_protocol.SetMove(0);
    g_xpider_inside_protocol.SetRotate(0);  
    
    LowVoltageAlert();
  }
}
