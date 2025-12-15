#include "find_center_agent.hpp"
#include <cmath>
#include <limits>
#include <vector>
#include <string>
#include <algorithm>

using namespace ambulance_module;

ScAddr FindCenterAgent::GetActionClass() const
{
  return AmbulanceKeynodes::action_find_graph_center;
}

ScResult FindCenterAgent::DoProgram(ScAction & action)
{
  ScAddr const actionNode = action;
  
  struct VillageData { ScAddr addr; double x; double y; };
  std::vector<VillageData> villages;
  
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
          villages.push_back({village, valX, valY});
  }
  
  if (villages.empty()) {
      m_logger.Error("No villages found.");
      return action.FinishWithError();
  }

  double minMaxDist = std::numeric_limits<double>::max(); 
  ScAddr centerNode;
  bool centerFound = false;

  ScStructure resultStruct = m_context.GenerateStructure();

  for (const auto& v1 : villages) {
      double maxDistForV1 = 0.0;
      
      for (const auto& v2 : villages) {
          if (v1.addr == v2.addr) continue;
          double d = std::hypot(v1.x - v2.x, v1.y - v2.y);
          if (d > maxDistForV1) maxDistForV1 = d;
      }

      ScAddr link = m_context.GenerateLink(ScType::NodeLink);
      m_context.SetLinkContent(link, std::to_string(maxDistForV1));
      
      ScAddr arc = m_context.GenerateConnector(ScType::ConstCommonArc, v1.addr, link);
      m_context.GenerateConnector(ScType::ConstPermPosArc, AmbulanceKeynodes::nrel_eccentricity, arc);
      resultStruct << link << arc;

      if (maxDistForV1 < minMaxDist) {
          minMaxDist = maxDistForV1;
          centerNode = v1.addr;
          centerFound = true;
      }
  }

  if (!centerFound) return action.FinishWithError();

  ScAddr resArc = m_context.GenerateConnector(ScType::ConstCommonArc, actionNode, centerNode);
  m_context.GenerateConnector(ScType::ConstPermPosArc, AmbulanceKeynodes::nrel_graph_center, resArc);
  
  resultStruct << centerNode << resArc << AmbulanceKeynodes::nrel_graph_center;
  action.SetResult(resultStruct);

  m_logger.Info("Graph Center found. Radius: " + std::to_string(minMaxDist));
  return action.FinishSuccessfully();
}
