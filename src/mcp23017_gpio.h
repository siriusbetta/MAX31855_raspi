#ifndef MAX31855_MCP23017_GPIO_H
#define MAX31855_MCP23017_GPIO_H

#include "gpio.h"
#include "MCP23017.h"
#include "memory"

class mcp23017_gpio : public gpio_arduino {
public:
  mcp23017_gpio(std::unique_ptr<mcp32017::MCP23017> pmcp23017);
  ~mcp23017_gpio();
  
  void pinMode(uint8_t p, uint8_t d);
  void digitalWrite(uint8_t p, uint8_t d);

private:
  std::unique_ptr<mcp32017::MCP23017> mcp23017;
};


#endif //MAX31855_MCP23017_GPIO_H
