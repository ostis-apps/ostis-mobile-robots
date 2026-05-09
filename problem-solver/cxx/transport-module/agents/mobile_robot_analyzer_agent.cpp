// mobile_robot_analyzer_agent.cpp

#include "mobile_robot_analyzer_agent.hpp"

ScAddr MobileRobotAnalyzerAgent::GetActionClass() const
{
  return MobileRobotsKeynodes::action_analyze_mobile_robot;
}

bool MobileRobotAnalyzerAgent::CheckInitiationCondition(
    ScEventChangeMobileRobotState const & event)
{
  ScAddr const & stateAddr =
      event.GetArcSourceElement();

  ScAddrToValueUnorderedMap<AnalyzerCallback> states =
  {
    {
      MobileRobotsKeynodes::concept_launched,
      [this](ScAction & action,
             ScAddr const & robotAddr)
      {
        return ProcessStart(action, robotAddr);
      }
    },

    {
      MobileRobotsKeynodes::concept_robot_is_loading,
      [this](ScAction & action,
             ScAddr const & robotAddr)
      {
        return ProcessLoadingStart(action, robotAddr);
      }
    },

    {
      MobileRobotsKeynodes::concept_box_loaded,
      [this](ScAction & action,
             ScAddr const & robotAddr)
      {
        return ProcessLoadingEnd(action, robotAddr);
      }
    },

    {
      MobileRobotsKeynodes::concept_robot_is_unloading,
      [this](ScAction & action,
             ScAddr const & robotAddr)
      {
        return ProcessUnloadingStart(action, robotAddr);
      }
    },

    {
      MobileRobotsKeynodes::concept_box_unloaded,
      [this](ScAction & action,
             ScAddr const & robotAddr)
      {
        return ProcessUnloadingEnd(action, robotAddr);
      }
    },

    {
      MobileRobotsKeynodes::concept_robot_waiting,
      [this](ScAction & action,
             ScAddr const & robotAddr)
      {
        return ProcessWaitingStart(action, robotAddr);
      }
    },

    {
      MobileRobotsKeynodes::concept_robot_not_waiting,
      [this](ScAction & action,
             ScAddr const & robotAddr)
      {
        return ProcessWaitingEnd(action, robotAddr);
      }
    },

    {
      MobileRobotsKeynodes::concept_stopped,
      [this](ScAction & action,
             ScAddr const & robotAddr)
      {
        return ProcessStop(action, robotAddr);
      }
    }
  };

  auto const & it = states.find(stateAddr);

  if (it == states.cend())
    return false;

  m_analyzerCallback = it->second;

  return true;
}

ScResult MobileRobotAnalyzerAgent::ProcessStart(
    ScAction & action,
    ScAddr const & robotAddr)
{
  m_experimentStart = steady_clock::now();

  m_totalWaitingTime = 0;
  m_totalLoadingTime = 0;
  m_totalUnloadingTime = 0;

  m_statistics.clear();

  m_logger.Info("Experiment started");

  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::ProcessLoadingStart(
    ScAction & action,
    ScAddr const & robotAddr)
{
  m_statistics[robotAddr].loadStart =
      steady_clock::now();

  m_logger.Info("Loading started");

  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::ProcessLoadingEnd(
    ScAction & action,
    ScAddr const & robotAddr)
{
  auto end = steady_clock::now();

  double loadTime =
      duration_cast<seconds>(
          end -
          m_statistics[robotAddr].loadStart)
      .count();

  m_statistics[robotAddr].loadingTime += loadTime;

  m_totalLoadingTime += loadTime;

  m_logger.Info(
      "Loading finished: "
      + std::to_string(loadTime));

  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::ProcessUnloadingStart(
    ScAction & action,
    ScAddr const & robotAddr)
{
  m_statistics[robotAddr].unloadStart =
      steady_clock::now();

  m_logger.Info("Unloading started");

  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::ProcessUnloadingEnd(
    ScAction & action,
    ScAddr const & robotAddr)
{
  auto end = steady_clock::now();

  double unloadTime =
      duration_cast<seconds>(
          end -
          m_statistics[robotAddr].unloadStart)
      .count();

  m_statistics[robotAddr].unloadingTime += unloadTime;

  m_totalUnloadingTime += unloadTime;

  m_logger.Info(
      "Unloading finished: "
      + std::to_string(unloadTime));

  return action.FinishSuccessfully();
}


ScResult MobileRobotAnalyzerAgent::ProcessWaitingStart(
    ScAction & action,
    ScAddr const & robotAddr)
{
  m_statistics[robotAddr].waitStart =
      steady_clock::now();

  m_logger.Info("Waiting started");

  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::ProcessWaitingEnd(
    ScAction & action,
    ScAddr const & robotAddr)
{
  auto end = steady_clock::now();

  double waitTime =
      duration_cast<seconds>(
          end -
          m_statistics[robotAddr].waitStart)
      .count();

  m_statistics[robotAddr].waitingTime += waitTime;

  m_totalWaitingTime += waitTime;

  m_logger.Info(
      "Waiting finished: "
      + std::to_string(waitTime));

  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::ProcessStop(
    ScAction & action,
    ScAddr const & robotAddr)
{
  GenerateReport();

  m_logger.Info("Experiment finished");

  return action.FinishSuccessfully();
}

void MobileRobotAnalyzerAgent::GenerateReport()
{
  auto end = steady_clock::now();

  double totalTime =
      duration_cast<seconds>(
          end -
          m_experimentStart)
      .count();

  double movementTime =
      totalTime
      - m_totalLoadingTime
      - m_totalUnloadingTime
      - m_totalWaitingTime;

  m_logger.Info("========== REPORT ==========");

  m_logger.Info(
      "Total time: "
      + std::to_string(totalTime));

  m_logger.Info(
      "Movement time: "
      + std::to_string(movementTime));

  m_logger.Info(
      "Waiting time: "
      + std::to_string(m_totalWaitingTime));

  m_logger.Info(
      "Loading/unloading time: "
      + std::to_string(
          m_totalLoadingTime +
          m_totalUnloadingTime));

  for (auto const & robot : m_statistics)
  {
    m_logger.Info("----- ROBOT -----");

    m_logger.Info(
        "Waiting: "
        + std::to_string(
            robot.second.waitingTime));

    m_logger.Info(
        "Loading: "
        + std::to_string(
            robot.second.loadingTime));

    m_logger.Info(
        "Unloading: "
        + std::to_string(
            robot.second.unloadingTime));
  }

  m_logger.Info("============================");
}

ScResult MobileRobotAnalyzerAgent::DoProgram(
    ScEventChangeMobileRobotState const & event,
    ScAction & action)
{
  ScAddr const & robotAddr =
      event.GetArcTargetElement();

  return m_analyzerCallback(
      action,
      robotAddr);
}
