#include "calculate_distances_agent.hpp"
#include <cmath>
#include <string>
#include <vector>

using namespace ambulance_module;

ScAddr CalculateDistancesAgent::GetActionClass() const
{
  return AmbulanceKeynodes::action_calculate_distances;
}

ScResult CalculateDistancesAgent::DoProgram(ScAction & action)
{
  struct Node { ScAddr addr; double x; double y; };
  std::vector<Node> nodes;
  
  ScIterator3Ptr it3 = m_context.CreateIterator3(
      AmbulanceKeynodes::concept_village, ScType::ConstPermPosArc, ScType::Unknown);
  
  while(it3->Next()) {
      ScAddr village = it3->Get(2);
      
      auto GetValue = [&](ScAddr const & rel) -> double {
        ScIterator5Ptr const it5 = m_context.CreateIterator5(
            village,
            ScType::ConstCommonArc,
            ScType::NodeLink,
            ScType::ConstPermPosArc,
            rel);

        if (it5->Next()) {
            ScAddr const linkAddr = it5->Get(2);
            std::string content_str;
            m_context.GetLinkContent(linkAddr, content_str);
            try { return std::stod(content_str); } catch (...) { return 0.0; }
        }
        return -1.0;
      };

      double valX = GetValue(AmbulanceKeynodes::nrel_coordinate_x);
      double valY = GetValue(AmbulanceKeynodes::nrel_coordinate_y);
      
      if (valX != -1.0 && valY != -1.0)
          nodes.push_back({village, valX, valY});
  }

  if (nodes.empty()) {
      m_logger.Error("No villages found.");
      return action.FinishWithError();
  }

  for (size_t i = 0; i < nodes.size(); ++i) {
      for (size_t j = i + 1; j < nodes.size(); ++j) {
          double dist = std::hypot(nodes[i].x - nodes[j].x, nodes[i].y - nodes[j].y);
          
          ScAddr link = m_context.GenerateLink(ScType::NodeLink);
          m_context.SetLinkContent(link, std::to_string(dist));
          
          ScAddr commonArc = m_context.GenerateConnector(
              ScType::ConstCommonArc, nodes[i].addr, nodes[j].addr);
          
          m_context.GenerateConnector(
              ScType::ConstPermPosArc, AmbulanceKeynodes::nrel_distance, commonArc);
           
          m_context.GenerateConnector(
               ScType::ConstPermPosArc, link, commonArc); 
      }
  }

  m_logger.Info("Distances calculated.");
  return action.FinishSuccessfully();
}
