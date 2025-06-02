#include "mcp23017_gpio.h"

mcp23017_gpio::mcp23017_gpio(std::unique_ptr<mcp32017::MCP23017> pmcp23017) : mcp23017(std::move(pmcp23017)){

}

mcp23017_gpio::~mcp23017_gpio(){

}

void mcp23017_gpio::pinMode(uint8_t p, uint8_t d){
  mcp23017->pinMode(p, d);
}

void mcp23017_gpio::digitalWrite(uint8_t p, uint8_t d){
  mcp23017->digitalWrite(p, d);
}

