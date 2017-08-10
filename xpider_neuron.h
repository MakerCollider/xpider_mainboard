#ifndef XPIDER_NEURON_H_
#define XPIDER_NEURON_H_

#include <Arduino.h>
#include "xpider_info.h"
#include <CurieNeuronsPro.h>

class XpiderNeuron {
public:
  XpiderNeuron();
  ~XpiderNeuron();

  bool Initialize();
  void Learn(uint8_t *buffer, uint8_t length, uint8_t cat);
  int Classify(uint8_t *buffer, int length);
  void Forget();
private:
  CurieNeurons hNN;
};
#endif

