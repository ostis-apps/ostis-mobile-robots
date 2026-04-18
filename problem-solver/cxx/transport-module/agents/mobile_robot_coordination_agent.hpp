#pragma once

#include <sc-memory/sc_agent.hpp>

using ScEventChangeMobileRobotState = ScEventAfterGenerateIncomingArc<ScType::ConstMembershipArc>;
// Реагируем на появление новой дуги принадлежности в узел мобильного робота
// Подробнее тут https://ostis-ai.github.io/sc-machine/sc-memory/api/cpp/extended/agents/events

class MobileRobotCoordinationAgent : public ScAgent<ScEventChangeMobileRobotState>
{
public:
  ScAddr GetActionClass() const override;

  bool CheckInitiationCondition(ScEventChangeMobileRobotState const & event) override;

  ScResult DoProgram(
      ScEventChangeMobileRobotState const & event,
      ScAction & action) override;
};
