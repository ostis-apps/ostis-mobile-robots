#pragma once

#include <sc-memory/sc_agent.hpp>
#include "keynodes/keynodes.hpp"

using ScEventChangeMobileRobotStateForAnalyzer = ScEventAfterGenerateIncomingArc<ScType::ConstActualTempPosArc>;

class MobileRobotAnalyzerAgent : public ScAgent<ScEventChangeMobileRobotStateForAnalyzer>
{
public:
  ScAddr GetActionClass() const override;
  bool CheckInitiationCondition(ScEventChangeMobileRobotStateForAnalyzer const & event) override;
  ScResult DoProgram(ScEventChangeMobileRobotStateForAnalyzer const & event, ScAction & action) override;
};
