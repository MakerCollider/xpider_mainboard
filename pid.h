/**
 * Author: Kang Li <likangqd@163.com>
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

#ifndef PID_H_
#define PID_H_

#include <Arduino.h>

class PID {
public:
  PID(float kp, float ki, float kd, float out_max, float out_min);
  ~PID();

  float k_p() { return k_p_; }
  float k_i() { return k_i_; }
  float k_d() { return k_d_; }

  void set_k_p(float value) { k_p_ = value; }
  void set_k_i(float value) { k_i_ = value; }
  void set_k_d(float value) { k_d_ = value; }

  bool clear();
  float update(float target, float actual);

private:
  float k_p_;                // 比例系数
  float k_i_;                // 积分系数
  float k_d_;                // 微分系数

  float integral_;           // 积分值
  int8_t i_enable_;          // 积分项使能

  float delta_value_;        // 偏差值
  float last_delta_value_;   // 上一个偏差值

  float output_;             // 输出值
  float last_output_;        // 上一次输出值
  float output_max_;         // 最大输出值
  float output_min_;         // 最小输出值
};

#endif // PID_H_


