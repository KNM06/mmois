#include "find_optimal_agent.hpp" // Исправлен хедер

#include <cmath>
#include <limits>
#include <vector>
#include <string>

using namespace ambulance_module;

ScAddr FindOptimalAgent::GetActionClass() const
{
  return AmbulanceKeynodes::action_find_optimal_station;
}

ScResult FindOptimalAgent::DoProgram(ScAction & action)
{
  ScAddr const actionNode = action;
  
  struct VillageData
  {
    ScAddr addr;
    double x;
    double y;
    int population;
  };

  std::vector<VillageData> villages;
  ScAddrVector allVillageNodes;

  ScIterator3Ptr const it3 = m_context.CreateIterator3(
      AmbulanceKeynodes::concept_village,
      ScType::ConstPermPosArc,
      ScType::Unknown);

  while (it3->Next())
  {
    ScAddr const villageAddr = it3->Get(2);
    allVillageNodes.push_back(villageAddr);
  }

  if (allVillageNodes.empty())
  {
    m_logger.Error("No villages found in the knowledge base.");
    return action.FinishWithError();
  }

  for (ScAddr const & villageAddr : allVillageNodes)
  {
    double x = 0.0;
    double y = 0.0;
    int pop = 0;
    bool dataFound = true;

    auto GetValue = [&](ScAddr const & rel) -> double {
      ScIterator5Ptr const it5 = m_context.CreateIterator5(
          villageAddr,
          ScType::ConstCommonArc,
          ScType::NodeLink,
          ScType::ConstPermPosArc,
          rel);

      if (it5->Next())
      {
        ScAddr const linkAddr = it5->Get(2);
        std::string content_str;
        m_context.GetLinkContent(linkAddr, content_str);
        try
        {
          return std::stod(content_str);
        }
        catch (...)
        {
          return 0.0;
        }
      }
      return -1.0;
    };

    double valX = GetValue(AmbulanceKeynodes::nrel_coordinate_x);
    double valY = GetValue(AmbulanceKeynodes::nrel_coordinate_y);
    double valPop = GetValue(AmbulanceKeynodes::nrel_population);

    if (valX == -1.0 || valY == -1.0 || valPop == -1.0)
    {
      m_logger.Warning("Incomplete data for village with addr: " + std::to_string(villageAddr.Hash()));
      continue; 
    }

    villages.push_back({villageAddr, valX, valY, (int)valPop});
  }

  if (villages.empty())
  {
    m_logger.Error("No valid village data found.");
    return action.FinishWithError();
  }

  double minScore = std::numeric_limits<double>::max();
  ScAddr bestVillageAddr;
  bool found = false;

  for (const auto & candidate : villages)
  {
    double currentScore = 0.0;

    for (const auto & target : villages)
    {
      double dist = std::hypot(candidate.x - target.x, candidate.y - target.y);
      currentScore += dist * target.population;
    }

    if (currentScore < minScore)
    {
      minScore = currentScore;
      bestVillageAddr = candidate.addr;
      found = true;
    }
  }

  if (!found)
  {
    return action.FinishWithError();
  }

  ScStructure resultStructure = m_context.GenerateStructure();

  ScAddr const resultArc = m_context.GenerateConnector(
      ScType::ConstCommonArc,
      actionNode,
      bestVillageAddr);

  ScAddr const relArc = m_context.GenerateConnector(
      ScType::ConstPermPosArc,
      AmbulanceKeynodes::nrel_optimal_location,
      resultArc);

  resultStructure << bestVillageAddr << resultArc << relArc << AmbulanceKeynodes::nrel_optimal_location;

  action.SetResult(resultStructure);
  
  m_logger.Info("Optimal station found. Min weighted score: " + std::to_string(minScore));

  return action.FinishSuccessfully();
}
