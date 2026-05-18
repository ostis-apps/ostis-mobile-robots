#pragma once

#include <sc-memory/sc_agent.hpp>

#include "keynodes/keynodes.hpp"

using ScEventChangeMobileRobotState = ScEventAfterGenerateIncomingArc<ScType::ConstActualTempPosArc>;

class MobileRobotInterpretationAgent : public ScAgent<ScEventChangeMobileRobotState>
{
public:
  ScAddr GetActionClass() const override;

  bool CheckInitiationCondition(ScEventChangeMobileRobotState const & event) override;

  ScResult InterpreterStateLaunched(ScAction & action, ScAddr const & robotAddr);

  ScResult InterpreterStateBoxLoaded(ScAction & action, ScAddr const & robotAddr);

  ScResult InterpreterStateBoxUnloaded(ScAction & action, ScAddr const & robotAddr);

  void MoveToNextPoint(ScAddr const & robotAddr, ScAddr const & next_point);

  void StartMoving(ScAddr const & robotAddr);

  void StopMoving(ScAddr const & robotAddr);

  bool ObstacleCheck(ScAddr const & routePoint, ScAddr const & robotAddr);

  ScAddr GetNextPoint(ScAddr const & robotAddr);

  bool UnloadingPointCheck(ScAddr const & routePoint, ScAddr const & robotAddr);

  bool UploadingPointCheck(ScAddr const & routePoint, ScAddr const & robotAddr);

  void SetWaitingState(ScAddr const & robotAddr, bool state);

  void SetCurrentSpeed(ScAddr const & robotAddr, const double &speed);

  ScResult InterpreterStateStopped(ScAction & action, ScAddr const & robotAddr);

  ScResult DoProgram(ScEventChangeMobileRobotState const & event, ScAction & action) override;

private:
  InterpreterCallback m_interpreterCallback;

  void ChangeActualTempArcToPos(ScAddr const & addr1, ScAddr const & addr2);
  void ChangeActualTempArcToNeg(ScAddr const & addr1, ScAddr const & addr2);
  bool IsStopped(ScAddr const & robotAddr);
  double GetCurrentSpeed(ScAddr const & robotAddr);
  double GetMaxSpeed(ScAddr const & robotAddr);
  double GetDistanceToNextPoint(ScAddr const & routePointAddr);
};
