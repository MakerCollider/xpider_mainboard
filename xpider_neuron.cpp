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

