#include "test_utils.hpp"

void SubscribeAgents(ScAgentContext & context)
{
  ScIterator3Ptr const it3 =
      context.CreateIterator3(MobileRobotsKeynodes::concept_mobile_robot, ScType::ConstPermPosArc, ScType::ConstNode);
  while (it3->Next())
  {
    ScAddr const & robotAddr = it3->Get(2);
    context.SubscribeAgent<MobileRobotCoordinationAgent>(robotAddr);
    // context.SubscribeAgent<MobileRobotInterpretationAgent>(robotAddr);
    // context.SubscribeAgent<MobileRobotAnalyzerAgent>(robotAddr);
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
    // context.UnsubscribeAgent<MobileRobotInterpretationAgent>(robotAddr);
    // context.UnsubscribeAgent<MobileRobotAnalyzerAgent>(robotAddr);
  }
}