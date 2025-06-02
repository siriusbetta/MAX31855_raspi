#include "MCP23017.h"
#include "MAX31855.h"
#include "mcp23017_gpio.h"
#include "memory"

int main(int argc, char *argv[]){
  std::unique_ptr<mcp32017::MCP23017> mcp23017;
  mcp23017->addCS(1);
  std::unique_ptr<mcp23017_gpio> mcp23017Gpio =
    std::make_unique<mcp23017_gpio>(std::move(mcp23017));
  
  std::unique_ptr<max31855::MAX31855> max318551 =
    std::make_unique<max31855::MAX31855>(std::move(mcp23017Gpio));
  max318551->setCS(1);
  
  return 0;
}