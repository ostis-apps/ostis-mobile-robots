#pragma once

#include <sc-memory/sc_agent.hpp>
#include "keynodes/keynodes.hpp"

using ScEventChangeMobileRobotState = ScEventAfterGenerateIncomingArc<ScType::ConstActualTempPosArc>;

class MobileRobotAnalyzerAgent : public ScAgent<ScEventChangeMobileRobotState>
{
public:
  ScAddr GetActionClass() const override;
  bool CheckInitiationCondition(ScEventChangeMobileRobotState const & event);
  ScResult InterpreterStateLaunched(ScAction & action, ScAddr const & robotAddr);
  ScResult InterpreterStateStopped(ScAction & action, ScAddr const & robotAddr);
  ScResult InterpreterStateIsMoving(ScAction & action, ScAddr const & robotAddr);
  ScResult InterpreterStateIsNotMoving(ScAction & action, ScAddr const & robotAddr);
  ScResult InterpreterStateIsWaiting(ScAction & action, ScAddr const & robotAddr);
  ScResult InterpreterStateIsNotWaiting(ScAction & action, ScAddr const & robotAddr);
  ScResult InterpreterStateIsLoading(ScAction & action, ScAddr const & robotAddr);
  ScResult InterpreterStateIsNotLoading(ScAction & action, ScAddr const & robotAddr);
  ScResult InterpreterStateIsUnloading(ScAction & action, ScAddr const & robotAddr);
  ScResult InterpreterStateIsNotUnloading(ScAction & action, ScAddr const & robotAddr);
  void LogTotalStats();
  ScResult DoProgram(ScEventChangeMobileRobotState const & event, ScAction & action) override;

private:
  InterpreterCallback m_interpreterCallback;
  double CalculateDiffInSeconds(size_t const & robotHash, size_t const & classHash);
  void LogTotalStats();
};
