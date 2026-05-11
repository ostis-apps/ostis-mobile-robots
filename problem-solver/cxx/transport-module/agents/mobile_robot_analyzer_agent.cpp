#include "mobile_robot_analyzer_agent.hpp"
#include <sc-memory/sc_link.hpp>
#include <chrono>
#include <map>

using namespace std::chrono;

// Структура для хранения статистики по роботу
struct RobotStats
{
  double waitingTime = 0;    // время ожидания препятствий
  double loadingTime = 0;    // время загрузки
  double unloadingTime = 0;  // время разгрузки

  steady_clock::time_point waitStart;
  steady_clock::time_point loadStart;
  steady_clock::time_point unloadStart;
};

static std::map<size_t, RobotStats> robotStats;
static std::map<size_t, steady_clock::time_point> stateStartTimes;
static std::map<size_t, ScAddr> lastStates;

// Глобальные счётчики для суммарной статистики
static double totalWaitingTime = 0;
static double totalLoadUnloadTime = 0;
static steady_clock::time_point experimentStartTime;
static bool experimentRunning = false;

ScAddr MobileRobotAnalyzerAgent::GetActionClass() const
{
  return MobileRobotsKeynodes::action_analyze_mobile_robot;
}

bool MobileRobotAnalyzerAgent::CheckInitiationCondition(ScEventChangeMobileRobotStateForAnalyzer const & event)
{
  ScAddr const & stateAddr = event.GetArcSourceElement();

  // Добавляем отслеживание ожидания препятствия
  return (
      stateAddr == MobileRobotsKeynodes::concept_robot_is_loading
      || stateAddr == MobileRobotsKeynodes::concept_robot_is_unloading
      || stateAddr == MobileRobotsKeynodes::concept_launched || stateAddr == MobileRobotsKeynodes::concept_robot_waiting
      || stateAddr == MobileRobotsKeynodes::concept_robot_not_waiting
      || stateAddr == MobileRobotsKeynodes::concept_stopped);
}

ScResult MobileRobotAnalyzerAgent::DoProgram(ScEventChangeMobileRobotStateForAnalyzer const & event, ScAction & action)
{
  ScAddr const & robotAddr = event.GetArcTargetElement();
  ScAddr const & newStateAddr = event.GetArcSourceElement();
  size_t robotHash = robotAddr.Hash();
  auto now = steady_clock::now();

  if (newStateAddr == MobileRobotsKeynodes::concept_launched && !experimentRunning)
  {
    experimentRunning = true;
    experimentStartTime = now;
    totalWaitingTime = 0;
    totalLoadUnloadTime = 0;
    robotStats.clear();
    m_logger.Info("=== АНАЛИЗАТОР: Эксперимент начат ===");
  }

  if (stateStartTimes.count(robotHash))
  {
    auto startTime = stateStartTimes[robotHash];
    double seconds = duration<double>(now - startTime).count();
    ScAddr lastState = lastStates[robotHash];

    if (lastState == MobileRobotsKeynodes::concept_robot_waiting)
    {
      robotStats[robotHash].waitingTime += seconds;
      totalWaitingTime += seconds;
      m_logger.Info("Анализатор: Робот ожидал " + std::to_string(seconds) + "с");
    }
    else if (lastState == MobileRobotsKeynodes::concept_robot_is_loading)
    {
      robotStats[robotHash].loadingTime += seconds;
      totalLoadUnloadTime += seconds;
      m_logger.Info("Анализатор: Загрузка длилась " + std::to_string(seconds) + "с");
    }
    else if (lastState == MobileRobotsKeynodes::concept_robot_is_unloading)
    {
      robotStats[robotHash].unloadingTime += seconds;
      totalLoadUnloadTime += seconds;
      m_logger.Info("Анализатор: Разгрузка длилась " + std::to_string(seconds) + "с");
    }
  }

  if (newStateAddr == MobileRobotsKeynodes::concept_stopped)
  {
    double totalTime = duration<double>(now - experimentStartTime).count();
    double movementTime = totalTime - totalWaitingTime - totalLoadUnloadTime;

    m_logger.Info("========== ИТОГОВЫЙ ОТЧЁТ ==========");
    m_logger.Info("Общее время эксперимента: " + std::to_string(totalTime) + "с");
    m_logger.Info("Время движения: " + std::to_string(movementTime) + "с");
    m_logger.Info("Время ожидания препятствий: " + std::to_string(totalWaitingTime) + "с");
    m_logger.Info("Время погрузки/разгрузки: " + std::to_string(totalLoadUnloadTime) + "с");

    for (auto const & [hash, stats] : robotStats)
    {
      m_logger.Info("--- Робот ---");
      m_logger.Info("  Ожидание: " + std::to_string(stats.waitingTime) + "с");
      m_logger.Info("  Загрузка: " + std::to_string(stats.loadingTime) + "с");
      m_logger.Info("  Разгрузка: " + std::to_string(stats.unloadingTime) + "с");
    }
    m_logger.Info("====================================");

    stateStartTimes.erase(robotHash);
    lastStates.erase(robotHash);
    robotStats.erase(robotHash);
    experimentRunning = false;
  }
  else
  {
    stateStartTimes[robotHash] = now;
    lastStates[robotHash] = newStateAddr;
  }

  return action.FinishSuccessfully();
}
