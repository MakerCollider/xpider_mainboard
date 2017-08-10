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

#include "PID.h"

PID::PID(float kp, float ki, float kd, float out_max, float out_min) {
  k_p_ = kp;
  k_i_ = ki;
  k_d_ = kd;

  output_max_ = out_max;
  output_min_ = out_min;
}

PID::~PID() {

}

float PID::update(float target_value, float actual_value) {
  delta_value_ = target_value - actual_value;

  if(abs(delta_value_) < 0.8) {
    i_enable_ = 1;
    if((last_output_ > output_max_ && delta_value_ < 0) ||
       (last_output_ < output_min_ && delta_value_ > 0) ||
       (last_output_ < output_max_ && actual_value > output_min_)) {
        // TODO: add description
      integral_ += delta_value_;
    }
  } else {
    i_enable_ = 0;
  }

  output_ = k_p_ * delta_value_;
  output_ += i_enable_ * k_i_ * integral_;
  output_ += k_d_ * (delta_value_ - last_delta_value_);

  last_output_ = output_;
  last_delta_value_ = delta_value_;

  return output_;
}

bool PID::clear() {
  last_output_ = 0;
  integral_ = 0;
  return true;
}


