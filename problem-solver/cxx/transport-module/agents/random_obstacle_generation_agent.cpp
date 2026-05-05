#include "random_obstacle_generation_agent.hpp"

ScAddr RandomObstacleGenerationAgent::GetActionClass() const
{
  return MobileRobotsKeynodes::action_generate_random_obstacle;
}

bool RandomObstacleGenerationAgent::CheckInitiationCondition(ScEventGenerateSimulationTimeTick const & event)
{
  return event.GetArcSourceElement() == MobileRobotsKeynodes::concept_simulation_time_tick;
}

ScResult RandomObstacleGenerationAgent::DoProgram(
    ScEventGenerateSimulationTimeTick const & event,
    ScAction & action)
{
  ++m_currentTick;
  RemoveExpiredObstacles();

  if (!ShouldGenerateObstacle())
    return action.FinishSuccessfully();

  ScAddr const obstacleAddr = GenerateObstacle();
  if (obstacleAddr.IsValid())
    action.FormResult(obstacleAddr);

  return action.FinishSuccessfully();
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

void RandomObstacleGenerationAgent::RemoveExpiredObstacles()
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
    RemoveObstacle(obstacleAddr);
  }
}

void RandomObstacleGenerationAgent::RemoveObstacle(ScAddr const & obstacleAddr)
{
  if (obstacleAddr.IsValid() && m_context.IsElement(obstacleAddr))
    m_context.EraseElement(obstacleAddr);
}

ScAddr RandomObstacleGenerationAgent::GenerateObstacle()
{
  ScAddr const obstaclePositionAddr = SelectObstaclePosition();
  if (!obstaclePositionAddr.IsValid())
    return ScAddr::Empty;

  ScAddr const obstacleAddr = m_context.GenerateNode(ScType::ConstNode);

  m_context.GenerateConnector(
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::concept_obstacle,
      obstacleAddr);

  ScAddr const obstaclePositionArc = m_context.GenerateConnector(
      ScType::ConstCommonArc,
      obstacleAddr,
      obstaclePositionAddr);
  m_context.GenerateConnector(
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::nrel_obstacle_position,
      obstaclePositionArc);

  int const obstacleLifetime = GenerateObstacleLifetime();
  m_obstacleExpirationTicks.emplace(obstacleAddr, m_currentTick + obstacleLifetime);

  return obstacleAddr;
}

ScAddr RandomObstacleGenerationAgent::SelectObstaclePosition()
{
  ScAddrVector routeArcs;
  ScIterator5Ptr const it5 = m_context.CreateIterator5(
      ScType::ConstNode,
      ScType::ConstCommonArc,
      ScType::ConstNode,
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::nrel_next_point);

  while (it5->Next())
    routeArcs.push_back(it5->Get(1));

  if (routeArcs.empty())
    return ScAddr::Empty;

  std::uniform_int_distribution<size_t> positionDistribution(0, routeArcs.size() - 1);
  return routeArcs[positionDistribution(m_randomGenerator)];
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
