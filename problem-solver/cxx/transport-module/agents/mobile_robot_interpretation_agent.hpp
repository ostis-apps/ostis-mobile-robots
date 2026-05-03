#pragma once

#include <sc-memory/sc_agent.hpp>

#include "keynodes/keynodes.hpp"

using ScEventChangeMobileRobotState = ScEventAfterGenerateIncomingArc<ScType::ConstPosArc>;

class MobileRobotInterpretationAgent : public ScAgent<ScEventChangeMobileRobotState>
{
public:
  ScAddr GetActionClass() const override;

  bool CheckInitiationCondition(ScEventChangeMobileRobotState const & event) override;

  ScResult InterpreterStateLaunched(ScAction & action, ScAddr const & robotAddr);

  ScResult InterpreterStateBoxLoaded(ScAction & action, ScAddr const & robotAddr);

  ScResult InterpreterStateBoxUnloaded(ScAction & action, ScAddr const & robotAddr);

  ScResult InterpreterStateStopped(ScAction & action, ScAddr const & robotAddr);

  ScResult DoProgram(
      ScEventChangeMobileRobotState const & event,
      ScAction & action) override;
  
private:
  InterpreterCallback m_interpreterCallback;
};
