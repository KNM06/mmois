#include "ambulance_module.hpp"
#include "agents/find_station_agent.hpp"
#include "keynodes/ambulance_keynodes.hpp"

using namespace ambulance_module;

SC_MODULE_REGISTER(AmbulanceModule)
    ->Agent<FindStationAgent>();
