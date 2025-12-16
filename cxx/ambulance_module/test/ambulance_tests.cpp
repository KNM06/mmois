#include <gtest/gtest.h>

#include <sc-memory/sc_agent.hpp>
#include <sc-memory/sc_memory.hpp>
#include <sc-memory/test/sc_test.hpp>

#include "agents/find_optimal_agent.hpp"
#include "agents/calculate_distances_agent.hpp"
#include "agents/find_center_agent.hpp"
#include "agents/find_problem_zones_agent.hpp"

#include "keynodes/ambulance_keynodes.hpp"

using namespace ambulance_module;

class AmbulanceAgentTest : public ScMemoryTest
{
protected:
  void SetUp() override
  {
      ScMemoryTest::SetUp();
  //подписываем агентов
      m_ctx->SubscribeAgent<FindOptimalAgent>();
      m_ctx->SubscribeAgent<CalculateDistancesAgent>();
      m_ctx->SubscribeAgent<FindCenterAgent>();
      m_ctx->SubscribeAgent<FindProblemZonesAgent>();
  }

  void TearDown() override
  {
  //отписываем агентов
      m_ctx->UnsubscribeAgent<FindProblemZonesAgent>();
      m_ctx->UnsubscribeAgent<FindCenterAgent>();
      m_ctx->UnsubscribeAgent<CalculateDistancesAgent>();
      m_ctx->UnsubscribeAgent<FindOptimalAgent>();
      
      ScMemoryTest::TearDown();
  }

  // создаем деревню с параметрами
  ScAddr CreateVillage(std::string const & sysIdtf, double x, double y, int population)
  {
      ScAddr village = m_ctx->GenerateNode(ScType::ConstNode);//создаем узел деревни
      
      m_ctx->SetElementSystemIdentifier(sysIdtf, village);//устанавливаем идентификатор
      
      //привязываем к классу деревень
      m_ctx->GenerateConnector(
          ScType::ConstPermPosArc, 
          AmbulanceKeynodes::concept_village, 
          village);


      auto AddProperty = [&](ScAddr const & rel, std::string const & val) {
          ScAddr link = m_ctx->GenerateLink(ScType::NodeLink);//создаем ссылку
          m_ctx->SetLinkContent(link, val);//записываем значение
          
          ScAddr arc = m_ctx->GenerateConnector(ScType::ConstCommonArc, village, link);//создаем дугу между деревней и значением

          m_ctx->GenerateConnector(ScType::ConstPermPosArc, rel, arc);//связываем отношение и прошлую дугу
      };
//записываем значения
      AddProperty(AmbulanceKeynodes::nrel_coordinate_x, std::to_string(x));
      AddProperty(AmbulanceKeynodes::nrel_coordinate_y, std::to_string(y));
      AddProperty(AmbulanceKeynodes::nrel_population, std::to_string(population));

      return village;
  }

  //читаем число из ссылки
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

//поиск оптимальной станции 
TEST_F(AmbulanceAgentTest, FindOptimalStationSuccess)
{
    //центральная деревня должна победить
    ScAddr vCentral = CreateVillage("Village_Central", 7.5, 6.0, 2500);
    CreateVillage("Village_Northern", 3.0, 10.0, 1200);
    CreateVillage("Village_Southern", 11.0, 2.0, 1800);

    ScAction action = m_ctx->GenerateAction(AmbulanceKeynodes::action_find_optimal_station);

    EXPECT_TRUE(action.InitiateAndWait(2000));
    EXPECT_TRUE(action.IsFinishedSuccessfully());

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


//расчет расстояний 
TEST_F(AmbulanceAgentTest, CalculateDistancesSuccess)
{
    ScAddr v1 = CreateVillage("V1", 0.0, 0.0, 100);
    ScAddr v2 = CreateVillage("V2", 3.0, 4.0, 100);

    ScAction action = m_ctx->GenerateAction(AmbulanceKeynodes::action_calculate_distances);

    EXPECT_TRUE(action.InitiateAndWait(2000));
    EXPECT_TRUE(action.IsFinishedSuccessfully());


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
    
    //проверяем значение на дуге
    ScIterator3Ptr it3 = m_ctx->CreateIterator3(
        ScType::NodeLink,
        ScType::ConstPermPosArc,
        commonArc
    );

    EXPECT_TRUE(it3->Next()) << "Не найдено числовое значение (link) для дуги";
    double dist = GetLinkValue(it3->Get(0));
    
    EXPECT_DOUBLE_EQ(dist, 5.0);
}


//поиск центра графа и эксцентриситета
TEST_F(AmbulanceAgentTest, FindCenterSuccess)
{
    // A(0)--2--B(2)--8--C(10)
    //эксцентриситеты: A=10, B=8, C=10
    //нентр: B min = 8
    
    ScAddr vA = CreateVillage("A", 0.0, 0.0, 10);
    ScAddr vB = CreateVillage("B", 2.0, 0.0, 10);
    ScAddr vC = CreateVillage("C", 10.0, 0.0, 10);

    ScAction action = m_ctx->GenerateAction(AmbulanceKeynodes::action_find_graph_center);

    EXPECT_TRUE(action.InitiateAndWait(2000));
    EXPECT_TRUE(action.IsFinishedSuccessfully());

    //проверяем что B центр
    ScIterator5Ptr itCenter = m_ctx->CreateIterator5(
        action,
        ScType::ConstCommonArc,
        ScType::ConstNode,
        ScType::ConstPermPosArc,
        AmbulanceKeynodes::nrel_graph_center
    );

    EXPECT_TRUE(itCenter->Next());
    EXPECT_EQ(itCenter->Get(2), vB);

    //проверяем эксцентриситет для B должен быть 8
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

//поиск проблемных зон 
TEST_F(AmbulanceAgentTest, FindProblemZonesSuccess)
{
    //оптимальная (0,0)
    //близкая: (2,0) -> dist=2
    //далекая: (10,0) -> dist=10
    //среднее: (2+10)/2 = 6 порог: 6 * 1.5 = 9
    //далекая (10 > 9) -> проблемная зона
    
    ScAddr vOpt = CreateVillage("Opt", 0.0, 0.0, 1000);
    ScAddr vNear = CreateVillage("Near", 2.0, 0.0, 100);
    ScAddr vFar = CreateVillage("Far", 10.0, 0.0, 100);
    
    //назначаем Opt оптимальной
    ScAddr actionOld = m_ctx->GenerateAction(AmbulanceKeynodes::action_find_optimal_station);
    ScAddr resArc = m_ctx->GenerateConnector(ScType::ConstCommonArc, actionOld, vOpt);
    m_ctx->GenerateConnector(ScType::ConstPermPosArc, AmbulanceKeynodes::nrel_optimal_location, resArc);

    //запуск агента проблемных зон
    ScAction action = m_ctx->GenerateAction(AmbulanceKeynodes::action_find_problem_zones);
    EXPECT_TRUE(action.InitiateAndWait(2000));
    EXPECT_TRUE(action.IsFinishedSuccessfully());

    //проверяем Far
    ScIterator5Ptr itFar = m_ctx->CreateIterator5(
        action,
        ScType::ConstCommonArc,
        vFar,
        ScType::ConstPermPosArc,
        AmbulanceKeynodes::nrel_problem_zone
    );
    EXPECT_TRUE(itFar->Next()) << "Far village должна быть проблемной зоной";

    //проверяем Near (не должна быть проблемной)
    ScIterator5Ptr itNear = m_ctx->CreateIterator5(
        action,
        ScType::ConstCommonArc,
        vNear,
        ScType::ConstPermPosArc,
        AmbulanceKeynodes::nrel_problem_zone
    );
    EXPECT_FALSE(itNear->Next()) << "Near village НЕ должна быть проблемной зоной";
}
