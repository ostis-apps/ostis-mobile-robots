#pragma once

#include <sc-memory/sc_agent.hpp>
#include "keynodes/keynodes.hpp"

using ScEventChangeMobileRobotState = ScEventAfterGenerateIncomingArc<ScType::ConstMembershipArc>;

class MobileRobotAnalyzerAgent : public ScAgent<ScEventChangeMobileRobotState>
{
public:
  ScAddr GetActionClass() const override;
  bool CheckInitiationCondition(ScEventChangeMobileRobotState const & event) override;
  ScResult DoProgram(ScEventChangeMobileRobotState const & event, ScAction & action) override;
};
