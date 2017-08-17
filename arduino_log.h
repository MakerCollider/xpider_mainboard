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
