#include "find_problem_zones_agent.hpp"
#include <cmath>
#include <vector>
#include <string>
#include <numeric>
#include <tuple> 

using namespace ambulance_module;

ScAddr FindProblemZonesAgent::GetActionClass() const
{
  return AmbulanceKeynodes::action_find_problem_zones;
}

ScResult FindProblemZonesAgent::DoProgram(ScAction & action)
{
  ScAddr const actionNode = action;

  ScAddr optimalStation;
  bool stationFound = false;

  // Ищем любую дугу помеченную как nrel_optimal_location
  ScIterator3Ptr itOpt = m_context.CreateIterator3(
      AmbulanceKeynodes::nrel_optimal_location,//отношение "Оптимальное расположение"
      ScType::ConstPermPosArc,//принадлежности
      ScType::Unknown //неизвестная дуга между action и village, котрая помечена отношением
  );

  while (itOpt->Next())
  {
      ScAddr resultArc = itOpt->Get(2);//адрес дуги
      
      //кортеж 
      auto [source, target] = m_context.GetConnectorIncidentElements(resultArc);
      // source - действие, target - найденная оптимальная деревня
      
      //проверяем что target это деревня 
      if (m_context.CheckConnector(
          AmbulanceKeynodes::concept_village, target, ScType::ConstPermPosArc)) 
      {
          optimalStation = target;
          stationFound = true;
          break;
      }
  }

  if (!stationFound)
  {
      m_logger.Error("Optimal station not found. Run FindOptimalAgent first.");
      return action.FinishWithError();
  }

  //считываем координаты оптимальной станции
  auto GetCoord = [&](ScAddr const & node, ScAddr const & rel) -> double {
      ScIterator5Ptr it = m_context.CreateIterator5(
          node, ScType::ConstCommonArc, ScType::NodeLink, ScType::ConstPermPosArc, rel);
      if (it->Next()) {
          std::string str;
          m_context.GetLinkContent(it->Get(2), str);
          try { return std::stod(str); } catch(...) { return 0.0; }
      }
      return 0.0;
  };

  double optX = GetCoord(optimalStation, AmbulanceKeynodes::nrel_coordinate_x);
  double optY = GetCoord(optimalStation, AmbulanceKeynodes::nrel_coordinate_y);

  //считываем остальные деревни
  struct VillageDist { ScAddr addr; double dist; };
  std::vector<VillageDist> list;
  double totalDist = 0;

  ScIterator3Ptr itV = m_context.CreateIterator3(
      AmbulanceKeynodes::concept_village, ScType::ConstPermPosArc, ScType::Unknown);
  
  while(itV->Next())
  {
      ScAddr v = itV->Get(2);//адрес деревни
      if (v == optimalStation) continue;
//получаем координаты
      double x = GetCoord(v, AmbulanceKeynodes::nrel_coordinate_x);
      double y = GetCoord(v, AmbulanceKeynodes::nrel_coordinate_y);
      //считаем дистанцию между оптимальной и текущей
      double d = std::hypot(x - optX, y - optY);
      list.push_back({v, d});//добавляем в вектор хранения адресов и дистанций
      totalDist += d;//общая сумма расстояний
  }

  if (list.empty()) return action.FinishSuccessfully();

  double avgDist = totalDist / list.size();//среднее расстояние для деревень
  double threshold = avgDist * 1.5; //критерий плохой зоны

  m_logger.Info("Average distance: " + std::to_string(avgDist) + 
                ", Threshold: " + std::to_string(threshold));

  ScStructure resultStruct = m_context.GenerateStructure();

  //помечаем проблемные зоны
  for (const auto& item : list)//проходимся по вектору с деревнями и дистанциями 
  {
      if (item.dist > threshold)
      {
          ScAddr resArc = m_context.GenerateConnector(ScType::ConstCommonArc, actionNode, item.addr);//создаем дугу между действием и далекой деревней
          m_context.GenerateConnector(ScType::ConstPermPosArc, AmbulanceKeynodes::nrel_problem_zone, resArc);//помечаем дугу как проблемная зона
          resultStruct << item.addr << resArc << AmbulanceKeynodes::nrel_problem_zone;
          
          m_logger.Info("Problem zone found! Distance: " + std::to_string(item.dist));
      }
  }

  action.SetResult(resultStruct);
  return action.FinishSuccessfully();
}
