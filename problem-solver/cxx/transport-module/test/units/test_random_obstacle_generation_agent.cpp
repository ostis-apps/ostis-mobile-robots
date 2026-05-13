#include "test_utils.hpp"

#include <atomic>
#include <random>
#include <thread>
#include <unordered_map>

#define private public
#include <agents/random_obstacle_generation_agent.hpp>
#undef private

namespace
{

struct ObstacleGeneratorTestData
{
  ScAddr robotAddr;
  ScAddr firstPointAddr;
  ScAddr secondPointAddr;
  ScAddr thirdPointAddr;
};

ObstacleGeneratorTestData LoadObstacleGeneratorTestData(ScMemoryContext & context)
{
  ScsLoader loader;
  loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "obstacle_generation_agent_test.scs");

  ObstacleGeneratorTestData data;
  data.robotAddr = context.SearchElementBySystemIdentifier("obstacle_generator_test_robot");
  data.firstPointAddr = context.SearchElementBySystemIdentifier("obstacle_generator_test_point_1");
  data.secondPointAddr = context.SearchElementBySystemIdentifier("obstacle_generator_test_point_2");
  data.thirdPointAddr = context.SearchElementBySystemIdentifier("obstacle_generator_test_point_3");

  EXPECT_TRUE(data.robotAddr.IsValid());
  EXPECT_TRUE(data.firstPointAddr.IsValid());
  EXPECT_TRUE(data.secondPointAddr.IsValid());
  EXPECT_TRUE(data.thirdPointAddr.IsValid());

  return data;
}

}  // namespace

TEST_F(TransportModuleTest, RandomObstacleGenerationAgentCreatesObstacle)
{
  ScAgentContext context;
  LoadObstacleGeneratorTestData(context);

  RandomObstacleGenerationAgent agent;
  ScAddr const obstacleAddr = agent.GenerateObstacle(context);

  ASSERT_TRUE(obstacleAddr.IsValid());
  EXPECT_TRUE(context.CheckConnector(
      MobileRobotsKeynodes::concept_obstacle,
      obstacleAddr,
      ScType::ConstPermPosArc));

  ScIterator5Ptr const obstaclePositionIt5 = context.CreateIterator5(
      obstacleAddr,
      ScType::ConstCommonArc,
      ScType::ConstNode,
      ScType::ConstActualTempPosArc,
      MobileRobotsKeynodes::nrel_obstacle_position);

  EXPECT_TRUE(obstaclePositionIt5->Next());
}

TEST_F(TransportModuleTest, RandomObstacleGenerationAgentSelectsPositionFromActiveRobotRoute)
{
  ScAgentContext context;
  ObstacleGeneratorTestData const data = LoadObstacleGeneratorTestData(context);

  RandomObstacleGenerationAgent agent;
  EXPECT_EQ(agent.GetRobotCurrentPosition(context, data.robotAddr), data.firstPointAddr);
  EXPECT_TRUE(agent.FindRouteByPoint(context, data.firstPointAddr).IsValid());
  EXPECT_FALSE(agent.CollectActiveRoutePoints(context).empty());

  ScAddr const obstaclePositionAddr = agent.SelectObstaclePosition(context);

  ASSERT_TRUE(obstaclePositionAddr.IsValid());
  EXPECT_TRUE(obstaclePositionAddr == data.secondPointAddr || obstaclePositionAddr == data.thirdPointAddr);
}

TEST_F(TransportModuleTest, RandomObstacleGenerationAgentRemovesExpiredObstacle)
{
  ScAgentContext context;
  LoadObstacleGeneratorTestData(context);

  RandomObstacleGenerationAgent agent;
  ScAddr const obstacleAddr = agent.GenerateObstacle(context);
  ASSERT_TRUE(obstacleAddr.IsValid());

  agent.m_obstacleExpirationTicks[obstacleAddr] = agent.m_currentTick;
  agent.RemoveExpiredObstacles(context);

  EXPECT_FALSE(context.IsElement(obstacleAddr));
}
