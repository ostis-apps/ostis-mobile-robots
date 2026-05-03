#include "mobile_robot_coordination_agent.hpp"

ScAddr MobileRobotCoordinationAgent::GetActionClass() const
{
  return MobileRobotsKeynodes::action_coordinate_mobile_robot;
}

bool MobileRobotCoordinationAgent::CheckInitiationCondition(ScEventChangeMobileRobotState const & event)
{
  ScAddr const & stateAddr = event.GetArcSourceElement();
  ScAddrToValueUnorderedMap<InterpreterCallback> states = {
      {MobileRobotsKeynodes::concept_ready_being_launched,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateReadyBeingLoaded(action, robotAddr);
       }},
      {MobileRobotsKeynodes::concept_ready_being_unlaunched,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateReadyBeingUnloaded(action, robotAddr);
       }},
  };
  auto const & it = states.find(stateAddr);
  if (it == states.cend())
    return false;

  m_interpreterCallback = it->second;
  return true;
}

ScResult MobileRobotCoordinationAgent::InterpreterStateReadyBeingLoaded(ScAction & action, ScAddr const & robotAddr)
{
  // Установить для робота состояние "загружается"
  // Найти местоположение робота и из этого местоположения взять свободную коробку и установить через какой-то
  // промежуток времени её в качестве груза робота
  // После этого убрать состояние "загружается" и установить состояние "загружен"
  return action.FinishSuccessfully();
}

ScResult MobileRobotCoordinationAgent::InterpreterStateReadyBeingUnloaded(ScAction & action, ScAddr const & robotAddr)
{
  // Установить для робота состояние "разгружается"
  // Найти местоположение робота и из этого местоположения взять свободную коробку и установить через какой-то
  // промежуток времени её в качестве груза робота
  // После этого убрать состояние "разгружается" и установить состояние "разгружен"
  return action.FinishSuccessfully();
}

ScResult MobileRobotCoordinationAgent::DoProgram(ScEventChangeMobileRobotState const & event, ScAction & action)
{
  ScAddr const & robotAddr = event.GetArcTargetElement();
  return m_interpreterCallback(action, robotAddr);
}
