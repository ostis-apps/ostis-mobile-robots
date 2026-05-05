#include "mobile_robot_interpretation_agent.hpp"

ScAddr MobileRobotInterpretationAgent::GetActionClass() const
{
  return MobileRobotsKeynodes::action_interpreter_mobile_robot;
}

bool MobileRobotInterpretationAgent::CheckInitiationCondition(ScEventChangeMobileRobotState const & event)
{
  ScAddr const & stateAddr = event.GetArcSourceElement();
  ScAddrToValueUnorderedMap<InterpreterCallback> states = {
    {MobileRobotsKeynodes::concept_launched, [this](ScAction & action, ScAddr const & robotAddr) -> ScResult { return InterpreterStateLaunched(action, robotAddr); }},
    {MobileRobotsKeynodes::concept_box_loaded, [this](ScAction & action, ScAddr const & robotAddr) -> ScResult { return InterpreterStateBoxLoaded(action, robotAddr); }},
    {MobileRobotsKeynodes::concept_box_unloaded, [this](ScAction & action, ScAddr const & robotAddr) -> ScResult { return InterpreterStateBoxUnloaded(action, robotAddr); }},
    {MobileRobotsKeynodes::concept_stopped, [this](ScAction & action, ScAddr const & robotAddr) -> ScResult { return InterpreterStateStopped(action, robotAddr); }},
  };
  auto const & it = states.find(stateAddr);
  if (it == states.cend())
    return false;

  m_interpreterCallback = it->second;
  return true;
}

ScResult MobileRobotInterpretationAgent::InterpreterStateLaunched(ScAction & action, ScAddr const & robotAddr)
{
  //добавление состояния "перемещается" 
  //добавление скорости
  //изменение местоположения на точку, принадлежащую пути
  //перемещение в точку загрузки
  //удаление состояния "перемещается"
  //удаление скорости
  //добавление состояния "готов к загрузке" (StateReadyBeingLoaded)
  return action.FinishSuccessfully();
}

ScResult MobileRobotInterpretationAgent::InterpreterStateBoxLoaded(ScAction & action, ScAddr const & robotAddr)
{
  //добавление состояние "перемещается"
  //добавление скорости
  //перемещение в следующую точку маршрута
  //
  //если припятствие:
  //  удаление состояния "перемещается"
  //  удаление скорости
  //  добавление состояния "остановлен" ??? (не уверен что нужно полностью отключать робота во время ожидания, скорее добавить состояние ожидания)
  //  пока припятствие:
  //    проверка на припятствие
  //  удаление состояния "остановлен" ??? (ожидание)
  //  добавление состояния "перемещается"
  //  добавление скорости
  //  
  //если точка разгрузки:
  //  удаление состояние "перемещается"
  //  удаления скорости
  //  добавление состояния "готов к разгрузке"(StateReadyBeingUnloaded)
  return action.FinishSuccessfully();
}

ScResult MobileRobotInterpretationAgent::InterpreterStateBoxUnloaded(ScAction & action, ScAddr const & robotAddr)
{
  //аналогично предыдущему методу, движение в точку погрузки
  return action.FinishSuccessfully();
}

ScResult MobileRobotInterpretationAgent::InterpreterStateStopped(ScAction & action, ScAddr const & robotAddr)
{
  //??? возможно перемещение в пункт хранения роботов, если не использоуется во время ожидания удаления припятствия
  m_logger.Info("stopped");
  return action.FinishSuccessfully();
}

ScResult MobileRobotInterpretationAgent::DoProgram(ScEventChangeMobileRobotState const & event, ScAction & action)
{
  ScAddr const & robotAddr = event.GetArcTargetElement();
  m_logger.Info("Test call");
  return m_interpreterCallback(action, robotAddr);
  //прорбный комик
}
