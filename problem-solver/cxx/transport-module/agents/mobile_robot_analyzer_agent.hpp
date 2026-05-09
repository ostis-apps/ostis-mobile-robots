// mobile_robot_analyzer_agent.hpp

#pragma once

#include <sc-memory/sc_agent.hpp>
#include <sc-memory/sc_event.hpp>
#include <sc-memory/sc_action.hpp>
#include "keynodes.hpp"

#include <chrono>
#include <unordered_map>
#include <functional>

using namespace std::chrono;


using ScEventChangeMobileRobotState = ScEventAfterGenerateIncomingArc<ScType::ConstMembershipArc>;

class MobileRobotAnalyzerAgent : public ScAgent<ScEventChangeMobileRobotState>
{
  SC_CLASS(
    Agent,
    Event(
      MobileRobotsKeynodes::concept_mobile_robot_state,
      ScEvent::Type::AddOutputEdge))

  SC_GENERATED_BODY()

public:
  ScAddr GetActionClass() const override;

  bool CheckInitiationCondition(
      ScEventChangeMobileRobotState const & event) override;

  ScResult DoProgram(
      ScEventChangeMobileRobotState const & event,
      ScAction & action) override;

private:
  using AnalyzerCallback = std::function<ScResult(ScAction &, ScAddr const &)>;
  AnalyzerCallback m_analyzerCallback;

  struct RobotStatistics
  {
    double waitingTime = 0;
    double loadingTime = 0;
    double unloadingTime = 0;

    steady_clock::time_point waitStart;
    steady_clock::time_point loadStart;
    steady_clock::time_point unloadStart;
  };

  std::unordered_map<ScAddr, RobotStatistics> m_statistics;
  steady_clock::time_point m_experimentStart;

  double m_totalWaitingTime = 0;
  double m_totalLoadingTime = 0;
  double m_totalUnloadingTime = 0;

private:
  ScResult ProcessStart(ScAction & action, ScAddr const & robotAddr);
  ScResult ProcessLoadingStart(ScAction & action, ScAddr const & robotAddr);
  ScResult ProcessLoadingEnd(ScAction & action, ScAddr const & robotAddr);
  ScResult ProcessUnloadingStart(ScAction & action, ScAddr const & robotAddr);
  ScResult ProcessUnloadingEnd(ScAction & action, ScAddr const & robotAddr);
  ScResult ProcessWaitingStart(ScAction & action, ScAddr const & robotAddr);
  ScResult ProcessWaitingEnd(ScAction & action, ScAddr const & robotAddr);
  ScResult ProcessStop(ScAction & action, ScAddr const & robotAddr);

  void GenerateReport();
};
