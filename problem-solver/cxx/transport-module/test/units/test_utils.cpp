#include "test_utils.hpp"

void SubscribeAgents(ScAgentContext & context)
{
  ScIterator3Ptr const it3 =
      context.CreateIterator3(MobileRobotsKeynodes::concept_mobile_robot, ScType::ConstPermPosArc, ScType::ConstNode);
  while (it3->Next())
  {
    ScAddr const & robotAddr = it3->Get(2);
    context.SubscribeAgent<MobileRobotCoordinationAgent>(robotAddr);
    context.SubscribeAgent<MobileRobotInterpretationAgent>(robotAddr);
    context.SubscribeAgent<MobileRobotAnalyzerAgent>(robotAddr);
  }
}

void UnsubscribeAgents(ScAgentContext & context)
{
  ScIterator3Ptr const it3 =
      context.CreateIterator3(MobileRobotsKeynodes::concept_mobile_robot, ScType::ConstPermPosArc, ScType::ConstNode);
  while (it3->Next())
  {
    ScAddr const & robotAddr = it3->Get(2);
    context.UnsubscribeAgent<MobileRobotCoordinationAgent>(robotAddr);
    context.UnsubscribeAgent<MobileRobotInterpretationAgent>(robotAddr);
    context.UnsubscribeAgent<MobileRobotAnalyzerAgent>(robotAddr);
  }
}

void SubscribeInterCoordAgents(ScAgentContext & context)
{
  ScIterator3Ptr const it3 =
      context.CreateIterator3(MobileRobotsKeynodes::concept_mobile_robot, ScType::ConstPermPosArc, ScType::ConstNode);
  while (it3->Next())
  {
    ScAddr const & robotAddr = it3->Get(2);
    context.SubscribeAgent<MobileRobotCoordinationAgent>(robotAddr);
    context.SubscribeAgent<MobileRobotInterpretationAgent>(robotAddr);
  }
}

void UnsubscribeInterCoordAgents(ScAgentContext & context)
{
  ScIterator3Ptr const it3 =
      context.CreateIterator3(MobileRobotsKeynodes::concept_mobile_robot, ScType::ConstPermPosArc, ScType::ConstNode);
  while (it3->Next())
  {
    ScAddr const & robotAddr = it3->Get(2);
    context.UnsubscribeAgent<MobileRobotCoordinationAgent>(robotAddr);
    context.UnsubscribeAgent<MobileRobotInterpretationAgent>(robotAddr);
  }
}

void WaitAgents(ScAgentContext & context)
{
  while (true)
  {
    bool still_working = false;
    ScIterator3Ptr it3 = context.CreateIterator3(
        MobileRobotsKeynodes::concept_launched, ScType::ConstActualTempPosArc, ScType::ConstNode);
    while (it3->Next())
    {
      ScAddr robotAddr = it3->Get(2);
      if (context.CheckConnector(MobileRobotsKeynodes::concept_mobile_robot, robotAddr, ScType::ConstPermPosArc))
      {
        still_working = true;
        break;
      }
    }
    if (still_working)
      sleep(5);
    else
      break;
  }
}

void DeleteObstacle(ScAgentContext & context){
  ScIterator3Ptr const it3 =
      context.CreateIterator3(MobileRobotsKeynodes::concept_obstacle, ScType::ConstPermPosArc, ScType::ConstNode);
  if (it3->Next())
  {
    ScAddr obstacle = it3->Get(2);
    context.EraseElement(obstacle);
    SC_LOG_INFO("Obstacle is deleted");
  }
}