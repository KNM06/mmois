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
          nodes.push_back({village, valX, valY});
  }

  if (nodes.empty()) {
      m_logger.Error("No villages found.");
      return action.FinishWithError();
  }

  for (size_t i = 0; i < nodes.size(); ++i) {//перебираем все деревни
      for (size_t j = i + 1; j < nodes.size(); ++j) {//перебираем все деревни, кроме тех что уже прошли
          double dist = std::hypot(nodes[i].x - nodes[j].x, nodes[i].y - nodes[j].y); //считаем дистанцию
          
          ScAddr link = m_context.GenerateLink(ScType::NodeLink);//создаем место хранения дистанции
          m_context.SetLinkContent(link, std::to_string(dist));//записываем дистанцию
          
          //создаем ребро графа
          ScAddr commonArc = m_context.GenerateConnector(
              ScType::ConstCommonArc, nodes[i].addr, nodes[j].addr);
              //дорога,деревня,деревня
          
          m_context.GenerateConnector(
              ScType::ConstPermPosArc, AmbulanceKeynodes::nrel_distance, commonArc);//помечаем дорогу как дистанция
           
           //привязываем число к ребру дистанции
          m_context.GenerateConnector(
               ScType::ConstPermPosArc, link, commonArc); 
      }
  }

  m_logger.Info("Distances calculated.");
  return action.FinishSuccessfully();
}
