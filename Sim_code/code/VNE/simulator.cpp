/*
 * simulator.cpp
 *
 *  Created on: 14-Jul-2008
 *      Author: nmmkchow
 */

#include "simulator.h"

Simulator::Simulator() {

}

Simulator::~Simulator() {

}

const Event &Simulator::top() {
  return PQ.top();
}

void Simulator::pop() {
  PQ.pop();
}

bool Simulator::empty() {
  return PQ.empty();
}

Event::Event(int _type, int _time, int _index)
  : type(_type), time(_time), index(_index) {

}

Event::~Event() {

}
