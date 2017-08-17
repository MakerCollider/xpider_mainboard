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

#include "xpider_neuron.h"
#include "arduino_log.h"
#include <Arduino.h>

XpiderNeuron::XpiderNeuron() {

}

XpiderNeuron::~XpiderNeuron() {

}

bool XpiderNeuron::Initialize(){
  hNN.begin();
  return true;
}
void XpiderNeuron::Learn(uint8_t *buffer, uint8_t length, uint8_t cat){
  LOG_PRINTLN("set NN parameters to NN!");
  for(int i = 0; i < length; i++){
    LOG_PRINT(" ");
    LOG_PRINTF(buffer[i], HEX);
  }
  LOG_PRINTLN(" ");
  hNN.learn(buffer, length, cat);
}
int XpiderNeuron::Classify(uint8_t *buffer, int length){
  int dist, cat, nid;
  hNN.setKNN();
  uint8_t result = hNN.classify(buffer, length, &dist, &cat, &nid);
  return cat;	
}
void XpiderNeuron::Forget()
{
  hNN.forget();
}

