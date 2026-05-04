#include "mobile_robot_analyzer_agent.hpp"
#include <sc-memory/sc_link.hpp>

ScAddr MobileRobotAnalyzerAgent::GetActionClass() const
{
  return MobileRobotsKeynodes::action_analyze_mobile_robot;
}

bool MobileRobotAnalyzerAgent::CheckInitiationCondition(ScEventChangeMobileRobotState const & event)
{
  ScAddr const & stateAddr = event.GetArcSourceElement();
  
  return (stateAddr == MobileRobotsKeynodes::concept_loading_process ||
          stateAddr == MobileRobotsKeynodes::concept_unloading_process ||
          stateAddr == MobileRobotsKeynodes::concept_waiting_obstacle ||
          stateAddr == MobileRobotsKeynodes::concept_stopped);
}

ScResult MobileRobotAnalyzerAgent::DoProgram(ScEventChangeMobileRobotState const & event, ScAction & action)
{
  ScAddr const & robotAddr = event.GetArcTargetElement(); 
  ScAddr const & newStateAddr = event.GetArcSourceElement(); 
  size_t robotHash = robotAddr.GetHash();

  auto now = std::chrono::steady_clock::now();

  if (m_startTimes.count(robotHash))
  {
    auto startTime = m_startTimes[robotHash];
    double duration = std::chrono::duration<double>(now - startTime).count();

    UpdateStatistic(robotAddr, MobileRobotsKeynodes::nrel_total_simulation_time, duration);
  }

  if (newStateAddr == MobileRobotsKeynodes::concept_stopped)
  {
    m_startTimes.erase(robotHash);
    m_logger.Info("Анализатор: Моделирование для робота " + std::to_string(robotHash) + " завершено.");
  }
  else
  {
    m_startTimes[robotHash] = now;
    
    if (newStateAddr == MobileRobotsKeynodes::concept_loading_process || 
        newStateAddr == MobileRobotsKeynodes::concept_unloading_process)
    {
       m_logger.Info("Анализатор: Замер времени погрузки/разгрузки...");
    }
  }

  return action.FinishSuccessfully();
}

void MobileRobotAnalyzerAgent::UpdateStatistic(ScAddr const & robotAddr, ScAddr const & statRelation, double deltaTime)
{
  ScIterator5Ptr it = m_memoryCtx.Iterator5(
      robotAddr,
      ScType::EdgeDCommonConst,
      ScType::NodeVar,
      ScType::EdgeAccessConstPos,
      statRelation
  );

  if (it->Next())
  {
    ScAddr linkAddr = it->Get(2);
    double currentValue = ReadLinkValue(linkAddr);
    WriteLinkValue(linkAddr, currentValue + deltaTime);
  }
  else
  {
    m_logger.Error("Анализатор: Не найдена ссылка для записи статистики!");
  }
}

double MobileRobotAnalyzerAgent::ReadLinkValue(ScAddr const & linkAddr)
{
  ScLink link(m_memoryCtx, linkAddr);
  try {
    return link.Get<double>();
  } catch (...) {
    return 0.0; 
  }
}

void MobileRobotAnalyzerAgent::WriteLinkValue(ScAddr const & linkAddr, double value)
{
  ScLink link(m_memoryCtx, linkAddr);
  link.Set(value);
}
