#include "mobile_robot_coordination_agent.hpp"

#include "keynodes/keynodes.hpp"

ScAddr MobileRobotCoordinationAgent::GetActionClass() const
{
  return Keynodes::action_coordinate_mobile_robot;
}

bool MobileRobotCoordinationAgent::CheckInitiationCondition(ScEventChangeMobileRobotState const & event)
{
  // Тут надо задать шаблоны и проверять по ним наличие в базе знаний ситуаций, соответствующих командам 
  // агента интерпретации состояний мобильного робота, для дальнейшего вызова DoProgram

  // Подробнее тут https://ostis-ai.github.io/sc-machine/sc-memory/api/cpp/extended/agents/agents/#checkinitiationcondition

  return true;
}

ScResult MobileRobotCoordinationAgent::DoProgram(
    ScEventChangeMobileRobotState const & event,
    ScAction & action)
{
  ScAddr const & robotAddr = event.GetArcTargetElement();

  // Тут должна быть логика обработки команд от агента интерпретации состояния мобильного робота

  // Для создания и ожидания ситуации в базе знаний необходимо использовать это API 
  // https://ostis-ai.github.io/sc-machine/sc-memory/api/cpp/extended/agents/waiters/#examples-of-using-waiters

  // Для вызова агента дампа статистики необходимо использовать это API 
  // https://ostis-ai.github.io/sc-machine/sc-memory/api/cpp/extended/agents/actions/

  return action.FinishSuccessfully();
}
