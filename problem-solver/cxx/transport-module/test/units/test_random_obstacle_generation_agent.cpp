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
  ScAddr routeAddr;
  ScAddr firstPointAddr;
  ScAddr secondPointAddr;
  ScAddr thirdPointAddr;
};

ScAddr AddNextPointRelation(ScMemoryContext & context, ScAddr const & routeAddr, ScAddr const & sourceAddr, ScAddr const & targetAddr)
{
  ScAddr const nextPointArcAddr = context.GenerateConnector(ScType::ConstCommonArc, sourceAddr, targetAddr);
  context.GenerateConnector(ScType::ConstPermPosArc, MobileRobotsKeynodes::nrel_next_point, nextPointArcAddr);
  context.GenerateConnector(ScType::ConstPermPosArc, routeAddr, nextPointArcAddr);

  return nextPointArcAddr;
}

ObstacleGeneratorTestData GenerateActiveRobotRoute(ScMemoryContext & context)
{
  ObstacleGeneratorTestData data;

  data.robotAddr = context.GenerateNode(ScType::ConstNode);
  data.routeAddr = context.GenerateNode(ScType::ConstNodeStructure);
  data.firstPointAddr = context.GenerateNode(ScType::ConstNode);
  data.secondPointAddr = context.GenerateNode(ScType::ConstNode);
  data.thirdPointAddr = context.GenerateNode(ScType::ConstNode);

  ScAddr const robotClassArcAddr =
      context.GenerateConnector(ScType::ConstPermPosArc, MobileRobotsKeynodes::concept_mobile_robot, data.robotAddr);
  EXPECT_TRUE(robotClassArcAddr.IsValid());

  ScAddr const launchedArcAddr =
      context.GenerateConnector(ScType::ConstActualTempPosArc, MobileRobotsKeynodes::concept_launched, data.robotAddr);
  EXPECT_TRUE(launchedArcAddr.IsValid());

  ScAddr const locationArcAddr = context.GenerateConnector(ScType::ConstCommonArc, data.robotAddr, data.firstPointAddr);
  EXPECT_TRUE(locationArcAddr.IsValid());

  ScAddr const locationRelationArcAddr =
      context.GenerateConnector(ScType::ConstActualTempPosArc, MobileRobotsKeynodes::nrel_location, locationArcAddr);
  EXPECT_TRUE(locationRelationArcAddr.IsValid());

  EXPECT_TRUE(context.CreateIterator3(
      MobileRobotsKeynodes::concept_launched,
      ScType::ConstActualTempPosArc,
      ScType::Node)->Next());
  EXPECT_TRUE(context.CreateIterator5(
      data.robotAddr,
      ScType::ConstCommonArc,
      ScType::Node,
      ScType::ConstActualTempPosArc,
      MobileRobotsKeynodes::nrel_location)->Next());

  AddNextPointRelation(context, data.routeAddr, data.firstPointAddr, data.secondPointAddr);
  AddNextPointRelation(context, data.routeAddr, data.secondPointAddr, data.thirdPointAddr);

  EXPECT_TRUE(context.CreateIterator5(
      ScType::Node,
      ScType::ConstCommonArc,
      ScType::Node,
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::nrel_next_point)->Next());

  return data;
}

}  // namespace

TEST_F(TransportModuleTest, RandomObstacleGenerationAgentCreatesObstacle)
{
  ScAgentContext context;
  GenerateActiveRobotRoute(context);

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
  ObstacleGeneratorTestData const data = GenerateActiveRobotRoute(context);

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
  GenerateActiveRobotRoute(context);

  RandomObstacleGenerationAgent agent;
  ScAddr const obstacleAddr = agent.GenerateObstacle(context);
  ASSERT_TRUE(obstacleAddr.IsValid());

  agent.m_obstacleExpirationTicks[obstacleAddr] = agent.m_currentTick;
  agent.RemoveExpiredObstacles(context);

  EXPECT_FALSE(context.IsElement(obstacleAddr));
}
