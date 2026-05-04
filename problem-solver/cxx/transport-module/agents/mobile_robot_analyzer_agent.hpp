#pragma once

#include <sc-memory/sc_agent.hpp>
#include <chrono>
#include <map>

#include "keynodes/keynodes.hpp"

using ScEventChangeMobileRobotState = ScEventAfterGenerateIncomingArc<ScType::ConstPosArc>;

class MobileRobotAnalyzerAgent : public ScAgent<ScEventChangeMobileRobotState>
{
public:
  ScAddr GetActionClass() const override;

  bool CheckInitiationCondition(ScEventChangeMobileRobotState const & event) override;

  ScResult DoProgram(
      ScEventChangeMobileRobotState const & event, 
      ScAction & action) override;

private:
  void UpdateStatistic(ScAddr const & robotAddr, ScAddr const & statRelation, double deltaTime);
  
  double ReadLinkValue(ScAddr const & linkAddr);
  void WriteLinkValue(ScAddr const & linkAddr, double value);

  std::map<size_t, std::chrono::steady_clock::time_point> m_startTimes;
};
