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
  ScAddr const & tickAddr = event.GetArcTargetElement();

  if (!ShouldGenerateObstacle())
    return action.FinishSuccessfully();

  ScAddr const & eventAddr = GenerateObstacleAppearanceEvent(tickAddr);
  action.FormResult(eventAddr);

  return action.FinishSuccessfully();
}

bool RandomObstacleGenerationAgent::ShouldGenerateObstacle()
{
  ++m_ticksSinceLastObstacle;

  if (m_ticksSinceLastObstacle < m_nextObstacleInterval)
    return false;

  std::uniform_real_distribution<double> probabilityDistribution(0.0, 1.0);
  if (probabilityDistribution(m_randomGenerator) >= ObstacleProbability)
    return false;

  m_ticksSinceLastObstacle = 0;
  m_nextObstacleInterval = GenerateNextObstacleInterval();

  return true;
}

ScAddr RandomObstacleGenerationAgent::GenerateObstacleAppearanceEvent(ScAddr const & tickAddr)
{
  ScAddr const & obstacleAddr = GenerateObstacle(tickAddr);
  ScAddr const & eventAddr = m_context.GenerateNode(ScType::ConstNode);

  m_context.GenerateConnector(
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::concept_obstacle_appearance_event,
      eventAddr);

  ScAddr const & generatedObstacleArc = m_context.GenerateConnector(
      ScType::ConstCommonArc,
      eventAddr,
      obstacleAddr);
  m_context.GenerateConnector(
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::nrel_generated_obstacle,
      generatedObstacleArc);

  ScAddr const & eventTimeArc = m_context.GenerateConnector(
      ScType::ConstCommonArc,
      eventAddr,
      tickAddr);
  m_context.GenerateConnector(
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::nrel_event_time,
      eventTimeArc);

  return eventAddr;
}

ScAddr RandomObstacleGenerationAgent::GenerateObstacle(ScAddr const & tickAddr)
{
  ScAddr const & obstacleAddr = m_context.GenerateNode(ScType::ConstNode);

  m_context.GenerateConnector(
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::concept_obstacle,
      obstacleAddr);

  ScAddr const & obstaclePositionAddr = SelectObstaclePosition();
  if (obstaclePositionAddr.IsValid())
  {
    ScAddr const & obstaclePositionArc = m_context.GenerateConnector(
        ScType::ConstCommonArc,
        obstacleAddr,
        obstaclePositionAddr);
    m_context.GenerateConnector(
        ScType::ConstPermPosArc,
        MobileRobotsKeynodes::nrel_obstacle_position,
        obstaclePositionArc);
  }

  return obstacleAddr;
}

ScAddr RandomObstacleGenerationAgent::SelectObstaclePosition()
{
  ScIterator5Ptr const it5 = m_context.CreateIterator5(
      ScType::ConstNode,
      ScType::ConstCommonArc,
      ScType::ConstNode,
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::nrel_next_point);

  if (it5->Next())
    return it5->Get(0);

  return ScAddr::Empty;
}

int RandomObstacleGenerationAgent::GenerateNextObstacleInterval()
{
  std::uniform_int_distribution<int> intervalDistribution(MinObstacleInterval, MaxObstacleInterval);
  return intervalDistribution(m_randomGenerator);
}
