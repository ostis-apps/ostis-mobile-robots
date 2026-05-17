#include "test_utils.hpp"

void SubscribeAgents(ScAgentContext & context)
{
  CountBoxes(context);

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
  CountBoxes(context);

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
      usleep(200);
    else
      break;
  }
  sleep(1);
}

void DeleteObstacle(ScAgentContext & context){
  ScIterator3Ptr const it3 =
      context.CreateIterator3(MobileRobotsKeynodes::concept_obstacle, ScType::ConstPermPosArc, ScType::ConstNode);
  while (it3->Next())
  {
    ScAddr obstacle = it3->Get(2);
    context.EraseElement(obstacle);
    SC_LOG_INFO("Obstacle is deleted");
  }
}

void CountBoxes(ScAgentContext & context){
  ScIterator3Ptr const it3 =
      context.CreateIterator3(
        MobileRobotsKeynodes::concept_route, 
        ScType::ConstPermPosArc, 
        ScType::ConstNodeStructure);
  while (it3->Next())
  {
    ScAddr routeAddr = it3->Get(2);
    int box_count = 0;
    ScIterator5Ptr const it5 = context.CreateIterator5(
        routeAddr, 
        ScType::ConstPermPosArc, 
        ScType::ConstNode,
        ScType::ConstPermPosArc,
        MobileRobotsKeynodes::rrel_start_point);
    while(it5->Next()){
      ScAddr const startPointAddr = it5->Get(2);
      ScIterator5Ptr const it5_1 =
      context.CreateIterator5(ScType::ConstNode, ScType::ConstCommonArc, startPointAddr, ScType::ConstActualTempPosArc, MobileRobotsKeynodes::nrel_location);
      while(it5_1->Next()){
        if(context.CheckConnector(MobileRobotsKeynodes::concept_box, it5_1->Get(0), ScType::ConstPermPosArc))
          box_count++;
      }
      ScIterator5Ptr const it5_2 = context.CreateIterator5(
        startPointAddr, 
        ScType::ConstCommonArc, 
        ScType::ConstNodeLink,
        ScType::ConstActualTempPosArc,
        MobileRobotsKeynodes::nrel_box_count);
      if(it5_2->Next())
        context.SetLinkContent(it5_2->Get(2), std::to_string(box_count));
      else{
        ScAddr countAddr = context.GenerateLink(ScType::ConstNodeLink);
        context.SetLinkContent(countAddr, std::to_string(box_count));
        ScAddr arcAddr = context.GenerateConnector(ScType::ConstCommonArc, startPointAddr, countAddr);
        context.GenerateConnector(ScType::ConstActualTempPosArc, MobileRobotsKeynodes::nrel_box_count, arcAddr);
      }
    }
  }
}
