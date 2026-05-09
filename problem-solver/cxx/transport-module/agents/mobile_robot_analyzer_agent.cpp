#include "mobile_robot_analyzer_agent.hpp"
#include <sc-memory/sc_link.hpp>
#include <chrono>
#include <map>

using namespace std::chrono;

static std::map<size_t, steady_clock::time_point> startTimes;
static std::map<size_t, ScAddr> lastStates;

ScAddr MobileRobotAnalyzerAgent::GetActionClass() const
{
  return MobileRobotsKeynodes::action_analyze_mobile_robot;
}

bool MobileRobotAnalyzerAgent::CheckInitiationCondition(ScEventChangeMobileRobotState const & event)
{
  ScAddr const & stateAddr = event.GetArcSourceElement();
  
  return (stateAddr == MobileRobotsKeynodes::concept_robot_is_loading ||
          stateAddr == MobileRobotsKeynodes::concept_robot_is_unloading ||
          stateAddr == MobileRobotsKeynodes::concept_launched ||
          stateAddr == MobileRobotsKeynodes::concept_stopped);
}

// Отслеживаемые состояния:
//   - concept_robot_is_loading   (робот загружается)
//   - concept_robot_is_unloading (робот разгружается)
//   - concept_launched           (робот движется)
//   - concept_stopped            (робот остановлен)
//
// При каждом изменении состояния запоминается текущее время
// При следующем изменении вычисляется разница между временами
// Разница выводится в лог
// При состоянии stopped данные о роботе удаляются

ScResult MobileRobotAnalyzerAgent::DoProgram(ScEventChangeMobileRobotState const & event, ScAction & action)
{
  ScAddr const & robotAddr = event.GetArcTargetElement();
  ScAddr const & newStateAddr = event.GetArcSourceElement();
  size_t robotHash = robotAddr.Hash();
  auto now = steady_clock::now();

  // Если у робота уже было состояние - считаем длительность
  if (startTimes.count(robotHash))
  {
    auto startTime = startTimes[robotHash];
    double seconds = duration<double>(now - startTime).count();
    
    // Логируем
    m_logger.Info("Analyzer: Robot state duration: " + std::to_string(seconds) + "s");
  }

  // Если новое состояние - stopped, удаляем, иначе обновляем
  if (newStateAddr == MobileRobotsKeynodes::concept_stopped)
  {
    startTimes.erase(robotHash);
    lastStates.erase(robotHash);
    m_logger.Info("Analyzer: Robot stopped");
  }
  else
  {
    startTimes[robotHash] = now;
    lastStates[robotHash] = newStateAddr;
    m_logger.Info("Analyzer: Robot state changed");
  }

  return action.FinishSuccessfully();
}
