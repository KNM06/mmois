#include <gtest/gtest.h>

#include <sc-memory/sc_agent.hpp>
#include <sc-memory/sc_memory.hpp>
#include <sc-memory/test/sc_test.hpp>

#include "agents/find_station_agent.hpp"
#include "keynodes/ambulance_keynodes.hpp"

using namespace ambulance_module;

class AmbulanceAgentTest : public ScMemoryTest
{
protected:
  void SetUp() override
  {
      ScMemoryTest::SetUp();
      m_ctx->SubscribeAgent<FindStationAgent>();
  }

  void TearDown() override
  {
      m_ctx->UnsubscribeAgent<FindStationAgent>();
      ScMemoryTest::TearDown();
  }

  ScAddr CreateVillage(std::string const & sysIdtf, double x, double y, int population)
  {
      ScAddr village = m_ctx->GenerateNode(ScType::ConstNode);
      
      // ИСПРАВЛЕНИЕ: Сначала идентификатор (строка), потом адрес (ScAddr)
      m_ctx->SetElementSystemIdentifier(sysIdtf, village);
      
      m_ctx->GenerateConnector(
          ScType::ConstPermPosArc, 
          AmbulanceKeynodes::concept_village, 
          village);

      auto AddProperty = [&](ScAddr const & rel, std::string const & val) {
          ScAddr link = m_ctx->GenerateLink(ScType::NodeLink);
          m_ctx->SetLinkContent(link, val);
          
          ScAddr arc = m_ctx->GenerateConnector(ScType::ConstCommonArc, village, link);
          m_ctx->GenerateConnector(ScType::ConstPermPosArc, rel, arc);
      };

      AddProperty(AmbulanceKeynodes::nrel_coordinate_x, std::to_string(x));
      AddProperty(AmbulanceKeynodes::nrel_coordinate_y, std::to_string(y));
      AddProperty(AmbulanceKeynodes::nrel_population, std::to_string(population));

      return village;
  }
};

TEST_F(AmbulanceAgentTest, FindOptimalStationSuccess)
{
    // Подготовка тестовых данных
    ScAddr vCentral = CreateVillage("Village_Central", 7.5, 6.0, 2500);
    CreateVillage("Village_Northern", 3.0, 10.0, 1200);
    CreateVillage("Village_Southern", 11.0, 2.0, 1800);

    // Запуск действия
    ScAction action = m_ctx->GenerateAction(AmbulanceKeynodes::action_find_optimal_station);

    EXPECT_TRUE(action.InitiateAndWait(2000));
    EXPECT_TRUE(action.IsFinishedSuccessfully());

    // Проверка результата
    // Передаем action напрямую (неявное приведение к ScAddr)
    ScIterator5Ptr it5 = m_ctx->CreateIterator5(
        action, 
        ScType::ConstCommonArc,
        ScType::ConstNode,
        ScType::ConstPermPosArc,
        AmbulanceKeynodes::nrel_optimal_location
    );

    EXPECT_TRUE(it5->Next());
    ScAddr resultVillage = it5->Get(2);

    EXPECT_EQ(resultVillage, vCentral);
}

TEST_F(AmbulanceAgentTest, NoVillagesError)
{
    ScAction action = m_ctx->GenerateAction(AmbulanceKeynodes::action_find_optimal_station);
    
    EXPECT_TRUE(action.InitiateAndWait(1000));
    
    EXPECT_TRUE(action.IsFinishedWithError());
}
