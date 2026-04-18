#include "mobile_robot_interpretation_agent.hpp"

#include "keynodes/keynodes.hpp"

ScAddr MobileRobotInterpretationAgent::GetActionClass() const
{
  return Keynodes::action_interpreter_mobile_robot;
}

bool MobileRobotInterpretationAgent::CheckInitiationCondition(ScEventChangeMobileRobotState const & event)
{
  // Тут надо задать шаблоны и проверять по ним наличие в базе знаний ситуаций, соответствующих командам 
  // агента координации мобильного робота, для дальнейшего вызова DoProgram

  // Подробнее тут https://ostis-ai.github.io/sc-machine/sc-memory/api/cpp/extended/agents/agents/#checkinitiationcondition

  return true;
}

ScResult MobileRobotInterpretationAgent::DoProgram(
    ScEventChangeMobileRobotState const & event,
    ScAction & action)
{
  ScAddr const & robotAddr = event.GetArcTargetElement();

  // Тут должна быть логика обработки команд от агента координатора

  // Для создания и ожидания ситуации в базе знаний необходимо использовать это API 
  // https://ostis-ai.github.io/sc-machine/sc-memory/api/cpp/extended/agents/waiters/#examples-of-using-waiters

  return action.FinishSuccessfully();
}
