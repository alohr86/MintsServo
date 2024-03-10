#include <stdint.h>
#pragma once

#include "Arduino.h"

class Motor {
  public:
    Motor(uint8_t pwm) : pwm(pwm) {}

    virtual void set(int speed) = 0;
  protected:
    uint8_t pwm;
};