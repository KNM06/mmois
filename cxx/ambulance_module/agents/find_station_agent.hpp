#pragma once

#include <sc-memory/sc_agent.hpp>

#include "keynodes/ambulance_keynodes.hpp"

namespace ambulance_module
{

class FindStationAgent : public ScActionInitiatedAgent
{
public:
  ScAddr GetActionClass() const override;

  ScResult DoProgram(ScAction & action) override;
};

} // namespace ambulance_module
