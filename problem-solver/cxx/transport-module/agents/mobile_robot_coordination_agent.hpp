#pragma once

#include <random>
#include <sc-memory/sc_agent.hpp>
#include "keynodes/keynodes.hpp"

using ScEventChangeMobileRobotState = ScEventAfterGenerateIncomingArc<ScType::ConstActualTempPosArc>;

class MobileRobotCoordinationAgent : public ScAgent<ScEventChangeMobileRobotState>
{
public:
  ScAddr GetActionClass() const override;

  bool CheckInitiationCondition(ScEventChangeMobileRobotState const & event) override;

  ScResult InterpreterStateReadyBeingLoaded(ScAction & action, ScAddr const & robotAddr);

  ScResult InterpreterStateReadyBeingUnloaded(ScAction & action, ScAddr const & robotAddr);

  ScResult DoProgram(ScEventChangeMobileRobotState const & event, ScAction & action) override;

private:
  InterpreterCallback m_interpreterCallback;

  static bool box_counted;
  static int box_count;

  void ChangeActualTempArcToNeg(ScAddr const & addr1, ScAddr const & addr2);
  void ChangeActualTempArcToPos(ScAddr const & addr1, ScAddr const & addr2);
  double GetLoadTime(ScAddr const & routeAddr);
  double GetUnloadTime(ScAddr const & routeAddr);
  double GenerateTime(int const & min, int const & max);
  bool AreThereFreeBoxes(ScAddr const &robotAddr);
  ScAddr const &GetRouteEndPoint(ScAddr const &robotAddr);
  void ReduceBoxCount(ScAddr const &routeStartPoint);


  std::mt19937 m_randomGenerator{std::random_device{}()};
};
