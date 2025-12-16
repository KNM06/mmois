#include "ambulance_module.hpp"
#include "agents/calculate_distances_agent.hpp"
#include "agents/find_center_agent.hpp"
#include "agents/find_optimal_agent.hpp" 
#include "agents/find_problem_zones_agent.hpp"
#include "keynodes/ambulance_keynodes.hpp"

using namespace ambulance_module;

SC_MODULE_REGISTER(AmbulanceModule)
    ->Agent<CalculateDistancesAgent>()
    ->Agent<FindCenterAgent>()
    ->Agent<FindOptimalAgent>()
    ->Agent<FindProblemZonesAgent>();
