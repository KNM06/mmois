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
  
    //ищем деревни
  ScIterator3Ptr it3 = m_context.CreateIterator3(
      AmbulanceKeynodes::concept_village, ScType::ConstPermPosArc, ScType::Unknown);//класс деревень, дуга принадлежности, неизвестная деревня
  
  while(it3->Next()) {
      ScAddr village = it3->Get(2);//получаем адрес деревни
      
      auto GetValue = [&](ScAddr const & rel) -> double {
        ScIterator5Ptr const it5 = m_context.CreateIterator5(
            village,//деревня
            ScType::ConstCommonArc,//дуга общего вида
            ScType::NodeLink,//ссылка где чило
            ScType::ConstPermPosArc,//принадлежности
            rel);//отношение(nrel_coordinate_x или nrel_coordinate_y)

        if (it5->Next()) {
            ScAddr const linkAddr = it5->Get(2);//получаем адрес где число
            std::string content_str;
            m_context.GetLinkContent(linkAddr, content_str);//копируем значение в строку
            try { return std::stod(content_str); } catch (...) { return 0.0; }
        } //stod это текст в число
        return -1.0;
      };

      double valX = GetValue(AmbulanceKeynodes::nrel_coordinate_x);
      double valY = GetValue(AmbulanceKeynodes::nrel_coordinate_y);
            //если найдены, то добавляем в вектор
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

  for (const auto& v1 : villages) {//перебирем все деревни
      double maxDistForV1 = 0.0;
      
      for (const auto& v2 : villages) {//измеряем расстояние от v1 до всех v2
          if (v1.addr == v2.addr) continue;
          double d = std::hypot(v1.x - v2.x, v1.y - v2.y);//считаем расстояние(гипотенуза)
          if (d > maxDistForV1) maxDistForV1 = d;//сравниваем с максимальным
      }

//записываем макисмальное расстояние для v1
      ScAddr link = m_context.GenerateLink(ScType::NodeLink);
      m_context.SetLinkContent(link, std::to_string(maxDistForV1));
      
      //создаем связь между эксцентриситетом и максимального расстояния
      ScAddr arc = m_context.GenerateConnector(ScType::ConstCommonArc, v1.addr, link);
      m_context.GenerateConnector(ScType::ConstPermPosArc, AmbulanceKeynodes::nrel_eccentricity, arc);
      resultStruct << link << arc;
//ищем минимум среди максимумов
      if (maxDistForV1 < minMaxDist) {
          minMaxDist = maxDistForV1;
          centerNode = v1.addr;//записываем вершину как центр графа
          centerFound = true;
      }
  }

  if (!centerFound) return action.FinishWithError();
  
  //связываем действие наъхождения центра с центром
  ScAddr resArc = m_context.GenerateConnector(ScType::ConstCommonArc, actionNode, centerNode);//общая дуга, действие, центр
  m_context.GenerateConnector(ScType::ConstPermPosArc, AmbulanceKeynodes::nrel_graph_center, resArc);//принадлежности, отношение центра, результат
  
  resultStruct << centerNode << resArc << AmbulanceKeynodes::nrel_graph_center;
  action.SetResult(resultStruct);

  m_logger.Info("Graph Center found. Radius: " + std::to_string(minMaxDist));
  return action.FinishSuccessfully();
}
