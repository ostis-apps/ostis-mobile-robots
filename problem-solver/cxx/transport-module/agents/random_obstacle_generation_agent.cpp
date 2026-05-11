#include "random_obstacle_generation_agent.hpp"

#include <chrono>

RandomObstacleGenerationAgent::~RandomObstacleGenerationAgent()
{
  Stop();
}

void RandomObstacleGenerationAgent::Start()
{
  if (m_isRunning.exchange(true))
    return;

  m_worker = std::thread(&RandomObstacleGenerationAgent::WorkerLoop, this);
}

void RandomObstacleGenerationAgent::Stop()
{
  if (!m_isRunning.exchange(false))
    return;

  if (m_worker.joinable())
    m_worker.join();
}

void RandomObstacleGenerationAgent::WorkerLoop()
{
  ScMemoryContext context;

  while (m_isRunning)
  {
    GenerateStep(context);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void RandomObstacleGenerationAgent::GenerateStep(ScMemoryContext & context)
{
  ++m_currentTick;
  RemoveExpiredObstacles(context);

  if (ShouldGenerateObstacle())
    GenerateObstacle(context);
}

bool RandomObstacleGenerationAgent::ShouldGenerateObstacle()
{
  ++m_ticksSinceLastObstacle;

  if (m_ticksSinceLastObstacle < m_nextObstacleInterval)
    return false;

  m_ticksSinceLastObstacle = 0;
  m_nextObstacleInterval = GenerateNextObstacleInterval();

  std::uniform_real_distribution<double> probabilityDistribution(0.0, 1.0);
  return probabilityDistribution(m_randomGenerator) < ObstacleProbability;
}

void RandomObstacleGenerationAgent::RemoveExpiredObstacles(ScMemoryContext & context)
{
  for (auto it = m_obstacleExpirationTicks.begin(); it != m_obstacleExpirationTicks.end();)
  {
    if (it->second > m_currentTick)
    {
      ++it;
      continue;
    }

    ScAddr const obstacleAddr = it->first;
    it = m_obstacleExpirationTicks.erase(it);
    RemoveObstacle(context, obstacleAddr);
  }
}

void RandomObstacleGenerationAgent::RemoveObstacle(ScMemoryContext & context, ScAddr const & obstacleAddr)
{
  if (obstacleAddr.IsValid() && context.IsElement(obstacleAddr))
    context.EraseElement(obstacleAddr);
}

ScAddr RandomObstacleGenerationAgent::GenerateObstacle(ScMemoryContext & context)
{
  ScAddr const obstaclePositionAddr = SelectObstaclePosition(context);
  if (!obstaclePositionAddr.IsValid())
    return ScAddr::Empty;

  ScAddr const obstacleAddr = context.GenerateNode(ScType::ConstNode);

  context.GenerateConnector(
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::concept_obstacle,
      obstacleAddr);

  ScAddr const obstaclePositionArc = context.GenerateConnector(
      ScType::ConstCommonArc,
      obstacleAddr,
      obstaclePositionAddr);
  context.GenerateConnector(
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::nrel_obstacle_position,
      obstaclePositionArc);

  int const obstacleLifetime = GenerateObstacleLifetime();
  m_obstacleExpirationTicks.emplace(obstacleAddr, m_currentTick + obstacleLifetime);

  return obstacleAddr;
}

ScAddr RandomObstacleGenerationAgent::SelectObstaclePosition(ScMemoryContext & context)
{
  ScAddrVector routePoints = CollectActiveRoutePoints(context);

  if (routePoints.empty())
    return ScAddr::Empty;

  std::uniform_int_distribution<size_t> positionDistribution(0, routePoints.size() - 1);
  return routePoints[positionDistribution(m_randomGenerator)];
}

ScAddrVector RandomObstacleGenerationAgent::CollectActiveRoutePoints(ScMemoryContext & context)
{
  ScAddrVector routePoints;
  ScIterator3Ptr const activeRobotIt3 = context.CreateIterator3(
      MobileRobotsKeynodes::concept_launched,
      ScType::ConstActualTempPosArc,
      ScType::ConstNode);

  while (activeRobotIt3->Next())
  {
    ScAddr const robotAddr = activeRobotIt3->Get(2);
    if (!context.CheckConnector(MobileRobotsKeynodes::concept_mobile_robot, robotAddr, ScType::ConstPermPosArc))
      continue;

    ScAddr const currentPositionAddr = GetRobotCurrentPosition(context, robotAddr);
    if (!currentPositionAddr.IsValid())
      continue;

    ScAddr const activeRouteAddr = FindRouteByPoint(context, currentPositionAddr);
    if (!activeRouteAddr.IsValid())
      continue;

    ScIterator5Ptr const routePointIt5 = context.CreateIterator5(
        activeRouteAddr,
        ScType::ConstPermPosArc,
        ScType::ConstCommonArc,
        ScType::ConstPermPosArc,
        MobileRobotsKeynodes::nrel_next_point);

    while (routePointIt5->Next())
    {
      auto const [_, targetPointAddr] = context.GetConnectorIncidentElements(routePointIt5->Get(2));
      routePoints.push_back(targetPointAddr);
    }
  }

  return routePoints;
}

ScAddr RandomObstacleGenerationAgent::FindRouteByPoint(ScMemoryContext & context, ScAddr const & pointAddr)
{
  ScIterator5Ptr const routeArcIt5 = context.CreateIterator5(
      ScType::ConstNode,
      ScType::ConstCommonArc,
      ScType::ConstNode,
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::nrel_next_point);

  while (routeArcIt5->Next())
  {
    ScAddr const routeArcAddr = routeArcIt5->Get(1);
    auto const [sourcePointAddr, targetPointAddr] = context.GetConnectorIncidentElements(routeArcAddr);

    if (sourcePointAddr != pointAddr && targetPointAddr != pointAddr)
      continue;

    ScIterator3Ptr const routeIt3 = context.CreateIterator3(
        ScType::ConstNodeStructure,
        ScType::ConstPermPosArc,
        routeArcAddr);

    if (routeIt3->Next())
      return routeIt3->Get(0);
  }

  return ScAddr::Empty;
}

ScAddr RandomObstacleGenerationAgent::GetRobotCurrentPosition(ScMemoryContext & context, ScAddr const & robotAddr)
{
  ScIterator5Ptr const locationIt5 = context.CreateIterator5(
      robotAddr,
      ScType::ConstCommonArc,
      ScType::ConstNode,
      ScType::ConstActualTempPosArc,
      MobileRobotsKeynodes::nrel_location);

  if (locationIt5->Next())
    return locationIt5->Get(2);

  return ScAddr::Empty;
}

int RandomObstacleGenerationAgent::GenerateNextObstacleInterval()
{
  std::uniform_int_distribution<int> intervalDistribution(MinObstacleInterval, MaxObstacleInterval);
  return intervalDistribution(m_randomGenerator);
}

int RandomObstacleGenerationAgent::GenerateObstacleLifetime()
{
  std::uniform_int_distribution<int> lifetimeDistribution(MinObstacleLifetime, MaxObstacleLifetime);
  return lifetimeDistribution(m_randomGenerator);
}
