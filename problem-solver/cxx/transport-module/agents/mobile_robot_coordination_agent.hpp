#pragma once

#include <random>
#include <sc-memory/sc_agent.hpp>
#include "keynodes/keynodes.hpp"

using ScEventChangeMobileRobotState = ScEventAfterGenerateIncomingArc<ScType::ConstMembershipArc>;

class MobileRobotCoordinationAgent : public ScAgent<ScEventChangeMobileRobotState>
{
public:
  ScAddr GetActionClass() const override;

  bool CheckInitiationCondition(ScEventChangeMobileRobotState const & event) override;

  ScResult InterpreterStateReadyBeingLoaded(ScAction & action, ScAddr const & robotAddr);

  ScResult InterpreterStateReadyBeingUnloaded(ScAction & action, ScAddr const & robotAddr);

  ScResult DoProgram(
      ScEventChangeMobileRobotState const & event,
      ScAction & action) override;
  
private:
  InterpreterCallback m_interpreterCallback;

  void ChangeActualTempArcToNeg(const ScAddr &addr1, const ScAddr &addr2);
  void ChangeActualTempArcToPos(const ScAddr &addr1, const ScAddr &addr2);
  double MobileRobotCoordinationAgent::GetLoadTime(ScAddr const &routeAddr);
  double MobileRobotCoordinationAgent::GetUnloadTime(ScAddr const &routeAddr);
  double MobileRobotCoordinationAgent::GenerateTime(double const &min, double const &max);
  std::mt19937 m_randomGenerator{std::random_device{}()};
};
