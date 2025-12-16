#pragma once
#include <sc-memory/sc_keynodes.hpp>

namespace ambulance_module
{
class AmbulanceKeynodes : public ScKeynodes
{
public:
  static inline ScKeynode const action_calculate_distances {
      "action_calculate_distances", ScType::ConstNodeClass};
  static inline ScKeynode const action_find_graph_center {
      "action_find_graph_center", ScType::ConstNodeClass};
  static inline ScKeynode const action_find_optimal_station {
      "action_find_optimal_station", ScType::ConstNodeClass};
  

  static inline ScKeynode const action_find_problem_zones {
      "action_find_problem_zones", ScType::ConstNodeClass};


  static inline ScKeynode const concept_village {
      "concept_village", ScType::ConstNodeClass};

  static inline ScKeynode const nrel_population {
      "nrel_population", ScType::ConstNodeNonRole};
  static inline ScKeynode const nrel_coordinate_x {
      "nrel_coordinate_x", ScType::ConstNodeNonRole};
  static inline ScKeynode const nrel_coordinate_y {
      "nrel_coordinate_y", ScType::ConstNodeNonRole};


  static inline ScKeynode const nrel_distance {
      "nrel_distance", ScType::ConstNodeNonRole};
  static inline ScKeynode const nrel_eccentricity {
      "nrel_eccentricity", ScType::ConstNodeNonRole};
  static inline ScKeynode const nrel_graph_center {
      "nrel_graph_center", ScType::ConstNodeNonRole};
  static inline ScKeynode const nrel_optimal_location {
      "nrel_optimal_location", ScType::ConstNodeNonRole};
      

  static inline ScKeynode const nrel_problem_zone {
      "nrel_problem_zone", ScType::ConstNodeNonRole};
};
}
