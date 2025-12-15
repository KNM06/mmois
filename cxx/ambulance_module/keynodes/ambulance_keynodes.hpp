#pragma once

#include <sc-memory/sc_keynodes.hpp>

namespace ambulance_module
{

class AmbulanceKeynodes : public ScKeynodes
{
public:
  static inline ScKeynode const action_find_optimal_station {
      "action_find_optimal_station", ScType::ConstNodeClass};

  static inline ScKeynode const concept_village {
      "concept_village", ScType::ConstNodeClass};

  static inline ScKeynode const nrel_population {
      "nrel_population", ScType::ConstNodeNonRole};

  static inline ScKeynode const nrel_coordinate_x {
      "nrel_coordinate_x", ScType::ConstNodeNonRole};

  static inline ScKeynode const nrel_coordinate_y {
      "nrel_coordinate_y", ScType::ConstNodeNonRole};

  static inline ScKeynode const nrel_optimal_location {
      "nrel_optimal_location", ScType::ConstNodeNonRole};
};

} // namespace ambulance_module
