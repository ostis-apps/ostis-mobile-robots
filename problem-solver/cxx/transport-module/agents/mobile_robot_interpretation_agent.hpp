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

  void MoveToNextPoint(ScAddr const & robotAddr, ScAddr const & next_point);

  void StartMooving(ScAddr const & robotAddr);

  void StopMooving(ScAddr const & robotAddr);

  bool ObstacleCheck(ScAddr const & next_point);

  ScAddr GetNextPoint(ScAddr const & robotAddr);

  bool UnloadingPointCheck(ScAddr const & next_point);

  bool UploadingPointCheck(ScAddr const & next_point);

  bool SetWaitingState(ScAddr const & robotAddr, bool state);
  
  void SetSpeed(ScAddr const & robotAddr, int speed);

  ScResult InterpreterStateStopped(ScAction & action, ScAddr const & robotAddr);

  ScResult DoProgram(
      ScEventChangeMobileRobotState const & event,
      ScAction & action) override;
  
private:
  InterpreterCallback m_interpreterCallback;
};
