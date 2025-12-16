#include <gtest/gtest.h>

#include <sc-memory/sc_agent.hpp>
#include <sc-memory/sc_memory.hpp>
#include <sc-memory/test/sc_test.hpp>

// Подключаем заголовочные файлы всех агентов
#include "agents/find_optimal_agent.hpp"
#include "agents/calculate_distances_agent.hpp"
#include "agents/find_center_agent.hpp"
#include "agents/find_problem_zones_agent.hpp"

#include "keynodes/ambulance_keynodes.hpp"

using namespace ambulance_module;

// Тестовый класс
class AmbulanceAgentTest : public ScMemoryTest
{
protected:
  void SetUp() override
  {
      ScMemoryTest::SetUp();

      // Подписываем всех агентов на события
      m_ctx->SubscribeAgent<FindOptimalAgent>();
      m_ctx->SubscribeAgent<CalculateDistancesAgent>();
      m_ctx->SubscribeAgent<FindCenterAgent>();
      m_ctx->SubscribeAgent<FindProblemZonesAgent>();
  }

  void TearDown() override
  {
      // Отписываем агентов (обратный порядок хорошая практика, но не обязательно)
      m_ctx->UnsubscribeAgent<FindProblemZonesAgent>();
      m_ctx->UnsubscribeAgent<FindCenterAgent>();
      m_ctx->UnsubscribeAgent<CalculateDistancesAgent>();
      m_ctx->UnsubscribeAgent<FindOptimalAgent>();
      
      ScMemoryTest::TearDown();
  }

  // Вспомогательная функция: Создать деревню с параметрами
  ScAddr CreateVillage(std::string const & sysIdtf, double x, double y, int population)
  {
      ScAddr village = m_ctx->GenerateNode(ScType::ConstNode);
      
      // В API 0.10.0 сначала строка, потом адрес
      m_ctx->SetElementSystemIdentifier(sysIdtf, village);
      
      // Добавляем в множество concept_village
      m_ctx->GenerateConnector(
          ScType::ConstPermPosArc, 
          AmbulanceKeynodes::concept_village, 
          village);

      // Лямбда для создания свойств (координат, населения)
      auto AddProperty = [&](ScAddr const & rel, std::string const & val) {
          ScAddr link = m_ctx->GenerateLink(ScType::NodeLink);
          m_ctx->SetLinkContent(link, val);
          
          // Деревня -> (дуга) -> Ссылка
          ScAddr arc = m_ctx->GenerateConnector(ScType::ConstCommonArc, village, link);
          // Отношение -> (дуга) -> (дуга выше)
          m_ctx->GenerateConnector(ScType::ConstPermPosArc, rel, arc);
      };

      AddProperty(AmbulanceKeynodes::nrel_coordinate_x, std::to_string(x));
      AddProperty(AmbulanceKeynodes::nrel_coordinate_y, std::to_string(y));
      AddProperty(AmbulanceKeynodes::nrel_population, std::to_string(population));

      return village;
  }

  // Вспомогательная функция: Прочитать число из ссылки
  double GetLinkValue(ScAddr const & linkAddr)
  {
      std::string content;
      m_ctx->GetLinkContent(linkAddr, content);
      try {
          return std::stod(content);
      } catch(...) {
          return -1.0;
      }
  }
};

// ==========================================
// ТЕСТ 1: Поиск оптимальной станции (Медиана)
// ==========================================
TEST_F(AmbulanceAgentTest, FindOptimalStationSuccess)
{
    // Центральная деревня (население 2500) должна победить
    ScAddr vCentral = CreateVillage("Village_Central", 7.5, 6.0, 2500);
    CreateVillage("Village_Northern", 3.0, 10.0, 1200);
    CreateVillage("Village_Southern", 11.0, 2.0, 1800);

    ScAction action = m_ctx->GenerateAction(AmbulanceKeynodes::action_find_optimal_station);

    EXPECT_TRUE(action.InitiateAndWait(2000));
    EXPECT_TRUE(action.IsFinishedSuccessfully());

    // Проверяем результат: Action -> (nrel_optimal_location) -> Village
    ScIterator5Ptr it5 = m_ctx->CreateIterator5(
        action, 
        ScType::ConstCommonArc,
        ScType::ConstNode,
        ScType::ConstPermPosArc,
        AmbulanceKeynodes::nrel_optimal_location
    );

    EXPECT_TRUE(it5->Next());
    EXPECT_EQ(it5->Get(2), vCentral);
}

// ==========================================
// ТЕСТ 2: Расчет расстояний (Граф)
// ==========================================
TEST_F(AmbulanceAgentTest, CalculateDistancesSuccess)
{
    // Треугольник 3-4-5. Точка (0,0) и (3,4). Расстояние = 5.
    ScAddr v1 = CreateVillage("V1", 0.0, 0.0, 100);
    ScAddr v2 = CreateVillage("V2", 3.0, 4.0, 100);

    ScAction action = m_ctx->GenerateAction(AmbulanceKeynodes::action_calculate_distances);

    EXPECT_TRUE(action.InitiateAndWait(2000));
    EXPECT_TRUE(action.IsFinishedSuccessfully());

    // Ищем дугу между V1 и V2 (порядок не важен, проверяем оба)
    ScAddr commonArc;
    bool arcFound = false;

    ScIterator5Ptr it5 = m_ctx->CreateIterator5(
        v1, ScType::ConstCommonArc, v2, ScType::ConstPermPosArc, AmbulanceKeynodes::nrel_distance
    );
    if (it5->Next()) {
        commonArc = it5->Get(1);
        arcFound = true;
    } else {
        it5 = m_ctx->CreateIterator5(
            v2, ScType::ConstCommonArc, v1, ScType::ConstPermPosArc, AmbulanceKeynodes::nrel_distance
        );
        if (it5->Next()) {
            commonArc = it5->Get(1);
            arcFound = true;
        }
    }

    EXPECT_TRUE(arcFound) << "Не найдена дуга расстояния между V1 и V2";
    
    // Проверяем значение на дуге. Структура: Link -> (arc) -> CommonArc
    ScIterator3Ptr it3 = m_ctx->CreateIterator3(
        ScType::NodeLink,
        ScType::ConstPermPosArc,
        commonArc
    );

    EXPECT_TRUE(it3->Next()) << "Не найдено числовое значение (link) для дуги";
    double dist = GetLinkValue(it3->Get(0));
    
    EXPECT_DOUBLE_EQ(dist, 5.0);
}

// ==========================================
// ТЕСТ 3: Поиск центра графа и эксцентриситета
// ==========================================
TEST_F(AmbulanceAgentTest, FindCenterSuccess)
{
    // Точки на линии: A(0)--2--B(2)--8--C(10)
    // Эксцентриситеты: A=10, B=8, C=10.
    // Центр: B (min = 8).
    
    ScAddr vA = CreateVillage("A", 0.0, 0.0, 10);
    ScAddr vB = CreateVillage("B", 2.0, 0.0, 10);
    ScAddr vC = CreateVillage("C", 10.0, 0.0, 10);

    ScAction action = m_ctx->GenerateAction(AmbulanceKeynodes::action_find_graph_center);

    EXPECT_TRUE(action.InitiateAndWait(2000));
    EXPECT_TRUE(action.IsFinishedSuccessfully());

    // 1. Проверяем, что B - центр
    ScIterator5Ptr itCenter = m_ctx->CreateIterator5(
        action,
        ScType::ConstCommonArc,
        ScType::ConstNode,
        ScType::ConstPermPosArc,
        AmbulanceKeynodes::nrel_graph_center
    );

    EXPECT_TRUE(itCenter->Next());
    EXPECT_EQ(itCenter->Get(2), vB);

    // 2. Проверяем эксцентриситет для B (должен быть 8.0)
    ScIterator5Ptr itEcc = m_ctx->CreateIterator5(
        vB,
        ScType::ConstCommonArc,
        ScType::NodeLink,
        ScType::ConstPermPosArc,
        AmbulanceKeynodes::nrel_eccentricity
    );
    
    EXPECT_TRUE(itEcc->Next());
    double eccVal = GetLinkValue(itEcc->Get(2));
    EXPECT_DOUBLE_EQ(eccVal, 8.0);
}

// ==========================================
// ТЕСТ 4: Поиск проблемных зон (доступность)
// ==========================================
TEST_F(AmbulanceAgentTest, FindProblemZonesSuccess)
{
    // Оптимальная: (0,0).
    // Близкая: (2,0) -> dist=2.
    // Далекая: (10,0) -> dist=10.
    // Среднее: (2+10)/2 = 6. Порог: 6 * 1.5 = 9.
    // Далекая (10 > 9) -> Проблемная зона.
    
    ScAddr vOpt = CreateVillage("Opt", 0.0, 0.0, 1000);
    ScAddr vNear = CreateVillage("Near", 2.0, 0.0, 100);
    ScAddr vFar = CreateVillage("Far", 10.0, 0.0, 100);
    
    // Эмуляция работы FindOptimalAgent: назначаем Opt оптимальной
    ScAddr actionOld = m_ctx->GenerateAction(AmbulanceKeynodes::action_find_optimal_station);
    ScAddr resArc = m_ctx->GenerateConnector(ScType::ConstCommonArc, actionOld, vOpt);
    m_ctx->GenerateConnector(ScType::ConstPermPosArc, AmbulanceKeynodes::nrel_optimal_location, resArc);

    // Запуск агента проблемных зон
    ScAction action = m_ctx->GenerateAction(AmbulanceKeynodes::action_find_problem_zones);
    EXPECT_TRUE(action.InitiateAndWait(2000));
    EXPECT_TRUE(action.IsFinishedSuccessfully());

    // Проверяем Far
    ScIterator5Ptr itFar = m_ctx->CreateIterator5(
        action,
        ScType::ConstCommonArc,
        vFar,
        ScType::ConstPermPosArc,
        AmbulanceKeynodes::nrel_problem_zone
    );
    EXPECT_TRUE(itFar->Next()) << "Far village должна быть проблемной зоной";

    // Проверяем Near (не должна быть проблемной)
    ScIterator5Ptr itNear = m_ctx->CreateIterator5(
        action,
        ScType::ConstCommonArc,
        vNear,
        ScType::ConstPermPosArc,
        AmbulanceKeynodes::nrel_problem_zone
    );
    EXPECT_FALSE(itNear->Next()) << "Near village НЕ должна быть проблемной зоной";
}
