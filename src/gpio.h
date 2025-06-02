#ifndef MAX31855_GPIO_H
#define MAX31855_GPIO_H
#include <cstddef>
#include <stdint-gcc.h>

class gpio_arduino{
public:
  gpio_arduino(){}
  virtual ~gpio_arduino(){}
  virtual void pinMode(uint8_t p, uint8_t d) = 0;
  virtual void digitalWrite(uint8_t p, uint8_t d) = 0;
};


#endif //MAX31855_GPIO_H
