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

#ifndef ARDUINO_LOG_H_
#define ARDUINO_LOG_H_

#ifdef ARDUINO
  #include <Arduino.h>

  // #define DEBUG
  #ifdef DEBUG
    #define DEBUG_PRINT(x) Serial.print(x)
    #define DEBUG_PRINTF(x, y) Serial.print(x, y)
    #define DEBUG_PRINTLN(x) Serial.println(x)
    #define DEBUG_PRINTLNF(x, y) Serial.println(x, y)
  #else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(x, y)
    #define DEBUG_PRINTLN(x)
    #define DEBUG_PRINTLNF(x, y)
  #endif // DEBUG

  // #define LOG
  #ifdef LOG
    #define LOG_PRINT(x) Serial.print(x)
    #define LOG_PRINTF(x, y) Serial.print(x, y)
    #define LOG_PRINTLN(x) Serial.println(x)
    #define LOG_PRINTLNF(x, y) Serial.println(x, y)
  #else
    #define LOG_PRINT(x)
    #define LOG_PRINTF(x, y)
    #define LOG_PRINTLN(x)
    #define LOG_PRINTLNF(x, y)
  #endif // LOG
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTF(x, y)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTLNF(x, y)
  #define LOG_PRINT(x)
  #define LOG_PRINTF(x, y)
  #define LOG_PRINTLN(x)
  #define LOG_PRINTLNF(x, y)
#endif // ARDUINO

#endif // ARDUINO_LOG_H_
