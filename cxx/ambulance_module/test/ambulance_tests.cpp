#include <gtest/gtest.h>

#include <sc-memory/sc_agent.hpp>
#include <sc-memory/sc_memory.hpp>
#include <sc-memory/test/sc_test.hpp>

#include "agents/find_optimal_agent.hpp"
#include "agents/calculate_distances_agent.hpp"
#include "agents/find_center_agent.hpp"

#include "keynodes/ambulance_keynodes.hpp"

using namespace ambulance_module;

class AmbulanceAgentTest : public ScMemoryTest
{
protected:
  void SetUp() override
  {
      ScMemoryTest::SetUp();

      m_ctx->SubscribeAgent<FindOptimalAgent>();
      m_ctx->SubscribeAgent<CalculateDistancesAgent>();
      m_ctx->SubscribeAgent<FindCenterAgent>();
  }

  void TearDown() override
  {
      m_ctx->UnsubscribeAgent<FindOptimalAgent>();
      m_ctx->UnsubscribeAgent<CalculateDistancesAgent>();
      m_ctx->UnsubscribeAgent<FindCenterAgent>();
      
      ScMemoryTest::TearDown();
  }

  ScAddr CreateVillage(std::string const & sysIdtf, double x, double y, int population)
  {
      ScAddr village = m_ctx->GenerateNode(ScType::ConstNode);
      
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


TEST_F(AmbulanceAgentTest, FindOptimalStationSuccess)
{
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

    EXPECT_TRUE(arcFound) << "Distance relation between V1 and V2 not found";
    
    ScIterator3Ptr it3 = m_ctx->CreateIterator3(
        ScType::NodeLink,
        ScType::ConstPermPosArc,
        commonArc
    );

    EXPECT_TRUE(it3->Next()) << "Link with distance value not found";
    double dist = GetLinkValue(it3->Get(0));
    
    EXPECT_DOUBLE_EQ(dist, 5.0);
}
TEST_F(AmbulanceAgentTest, FindCenterSuccess)
{
    
    ScAddr vA = CreateVillage("A", 0.0, 0.0, 10);
    ScAddr vB = CreateVillage("B", 2.0, 0.0, 10);
    ScAddr vC = CreateVillage("C", 10.0, 0.0, 10);

    ScAction action = m_ctx->GenerateAction(AmbulanceKeynodes::action_find_graph_center);

    EXPECT_TRUE(action.InitiateAndWait(2000));
    EXPECT_TRUE(action.IsFinishedSuccessfully());

    ScIterator5Ptr itCenter = m_ctx->CreateIterator5(
        action,
        ScType::ConstCommonArc,
        ScType::ConstNode,
        ScType::ConstPermPosArc,
        AmbulanceKeynodes::nrel_graph_center
    );

    EXPECT_TRUE(itCenter->Next());
    EXPECT_EQ(itCenter->Get(2), vB);

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
