#include "mobile_robot_analyzer_agent.hpp"
#include <sc-memory/sc_link.hpp>
#include <chrono>
#include <thread>
#include <map>

// Структура для хранения статистики по роботу
struct RobotStats
{
  double waitingTime = 0;    // время ожидания препятствий
  double loadingTime = 0;    // время загрузки
  double unloadingTime = 0;  // время разгрузки
  double movingTime = 0;
};

static std::map<size_t, RobotStats> robotStats;
static std::map<size_t, std::map<size_t, std::chrono::steady_clock::time_point>> stateStartTimes;

// Глобальные счётчики для суммарной статистики
static double totalWaitingTime = 0;
static double totalLoadUnloadTime = 0;
static std::chrono::steady_clock::time_point experimentStartTime;
static bool experimentRunning = false;

ScAddr MobileRobotAnalyzerAgent::GetActionClass() const
{
  return MobileRobotsKeynodes::action_analyze_mobile_robot;
}

bool MobileRobotAnalyzerAgent::CheckInitiationCondition(ScEventChangeMobileRobotState const & event)
{
  ScAddr const & stateAddr = event.GetArcSourceElement();
  ScAddrToValueUnorderedMap<InterpreterCallback> states = {
      {MobileRobotsKeynodes::concept_launched,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateLaunched(action, robotAddr);
       }},
      {MobileRobotsKeynodes::concept_stopped,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateStopped(action, robotAddr);
       }},
      {MobileRobotsKeynodes::concept_is_moving,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateIsMoving(action, robotAddr);
       }},
      {MobileRobotsKeynodes::concept_is_not_moving,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateIsNotMoving(action, robotAddr);
       }},
      {MobileRobotsKeynodes::concept_robot_is_waiting,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateIsWaiting(action, robotAddr);
       }},
      {MobileRobotsKeynodes::concept_robot_is_not_waiting,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateIsNotWaiting(action, robotAddr);
       }},
      {MobileRobotsKeynodes::concept_robot_is_loading,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateIsLoading(action, robotAddr);
       }},
      {MobileRobotsKeynodes::concept_robot_is_not_loading,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateIsNotLoading(action, robotAddr);
       }},
      {MobileRobotsKeynodes::concept_robot_is_unloading,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateIsUnloading(action, robotAddr);
       }},
      {MobileRobotsKeynodes::concept_robot_is_not_unloading,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateIsNotUnloading(action, robotAddr);
       }},

  };
  auto const & it = states.find(stateAddr);
  if (it == states.cend())
    return false;

  m_interpreterCallback = it->second;
  return true;
}

ScResult MobileRobotAnalyzerAgent::DoProgram(ScEventChangeMobileRobotState const & event, ScAction & action)
{
  ScAddr const & robotAddr = event.GetArcTargetElement();
  return m_interpreterCallback(action, robotAddr);
}

ScResult MobileRobotAnalyzerAgent::InterpreterStateLaunched(ScAction & action, ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  auto now = std::chrono::steady_clock::now();
  experimentRunning = true;
  experimentStartTime = now;
  totalWaitingTime = 0;
  totalLoadUnloadTime = 0;
  robotStats.clear();
  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::InterpreterStateStopped(ScAction & action, ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  experimentRunning = false;
  std::this_thread::sleep_for(std::chrono::seconds(1));
  RobotStats stats = robotStats[robotHash];
  m_logger.Info("====================================");
  m_logger.Info("--- Робот " + m_context.GetElementSystemIdentifier(robotAddr) + " ---");
  m_logger.Info("  Ожидание: " + std::to_string(stats.waitingTime) + "с");
  m_logger.Info("  Движение: " + std::to_string(stats.movingTime) + "с");
  m_logger.Info("  Загрузка: " + std::to_string(stats.loadingTime) + "с");
  m_logger.Info("  Разгрузка: " + std::to_string(stats.unloadingTime) + "с");
  m_logger.Info("====================================");
  stateStartTimes.erase(robotHash);
  robotStats.erase(robotHash);
  experimentRunning = false;
  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::InterpreterStateIsMoving(ScAction & action, ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  size_t classHash = MobileRobotsKeynodes::concept_is_moving.Hash();
  auto now = std::chrono::steady_clock::now();
  stateStartTimes[robotHash][classHash] = now;
  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::InterpreterStateIsNotMoving(ScAction & action, ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  size_t classHash = MobileRobotsKeynodes::concept_is_moving.Hash();
  double seconds = CalculateDiffInSeconds(robotHash, classHash);
  robotStats[robotHash].movingTime += seconds;
  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::InterpreterStateIsWaiting(ScAction & action, ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  size_t classHash = MobileRobotsKeynodes::concept_robot_is_waiting.Hash();
  auto now = std::chrono::steady_clock::now();
  stateStartTimes[robotHash][classHash] = now;
  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::InterpreterStateIsNotWaiting(ScAction & action, ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  size_t classHash = MobileRobotsKeynodes::concept_robot_is_waiting.Hash();
  double seconds = CalculateDiffInSeconds(robotHash, classHash);
  robotStats[robotHash].waitingTime += seconds;
  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::InterpreterStateIsLoading(ScAction & action, ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  size_t classHash = MobileRobotsKeynodes::concept_robot_is_loading.Hash();
  auto now = std::chrono::steady_clock::now();
  stateStartTimes[robotHash][classHash] = now;
  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::InterpreterStateIsNotLoading(ScAction & action, ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  size_t classHash = MobileRobotsKeynodes::concept_robot_is_loading.Hash();
  double seconds = CalculateDiffInSeconds(robotHash, classHash);
  robotStats[robotHash].loadingTime += seconds;
  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::InterpreterStateIsUnloading(ScAction & action, ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  size_t classHash = MobileRobotsKeynodes::concept_robot_is_unloading.Hash();
  auto now = std::chrono::steady_clock::now();
  stateStartTimes[robotHash][classHash] = now;
  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::InterpreterStateIsNotUnloading(ScAction & action, ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  size_t classHash = MobileRobotsKeynodes::concept_robot_is_unloading.Hash();
  double seconds = CalculateDiffInSeconds(robotHash, classHash);
  robotStats[robotHash].unloadingTime += seconds;
  return action.FinishSuccessfully();
}

double MobileRobotAnalyzerAgent::CalculateDiffInSeconds(size_t const & robotHash, size_t const & classHash)
{
  auto now = std::chrono::steady_clock::now();
  auto startTime = stateStartTimes[robotHash][classHash];
  double seconds = std::chrono::duration<double>(now - startTime).count();
  return seconds;
}


    // ScResult MobileRobotAnalyzerAgent::DoProgram(ScEventChangeMobileRobotState const & event, ScAction & action)
    // {
    //   ScAddr const & robotAddr = event.GetArcTargetElement();
    //   ScAddr const & newStateAddr = event.GetArcSourceElement();
    //   size_t robotHash = robotAddr.Hash();
    //   auto now = steady_clock::now();

    //   if (newStateAddr == MobileRobotsKeynodes::concept_launched && !experimentRunning)
    //   {
    //     experimentRunning = true;
    //     experimentStartTime = now;
    //     totalWaitingTime = 0;
    //     totalLoadUnloadTime = 0;
    //     robotStats.clear();
    //     m_logger.Info("=== АНАЛИЗАТОР: Эксперимент начат ===");
    //   }

    //   if (stateStartTimes.count(robotHash))
    //   {
    //     auto startTime = stateStartTimes[robotHash];
    //     double seconds = duration<double>(now - startTime).count();
    //     ScAddr lastState = lastStates[robotHash];

    //     if (lastState == MobileRobotsKeynodes::concept_robot_is_waiting)
    //     {
    //       robotStats[robotHash].waitingTime += seconds;
    //       totalWaitingTime += seconds;
    //       m_logger.Info("Анализатор: Робот ожидал " + std::to_string(seconds) + "с");
    //     }
    //     else if (lastState == MobileRobotsKeynodes::concept_robot_is_loading)
    //     {
    //       robotStats[robotHash].loadingTime += seconds;
    //       totalLoadUnloadTime += seconds;
    //       m_logger.Info("Анализатор: Загрузка длилась " + std::to_string(seconds) + "с");
    //     }
    //     else if (lastState == MobileRobotsKeynodes::concept_robot_is_unloading)
    //     {
    //       robotStats[robotHash].unloadingTime += seconds;
    //       totalLoadUnloadTime += seconds;
    //       m_logger.Info("Анализатор: Разгрузка длилась " + std::to_string(seconds) + "с");
    //     }
    //   }

    //   if (newStateAddr == MobileRobotsKeynodes::concept_stopped)
    //   {
    //     double totalTime = duration<double>(now - experimentStartTime).count();
    //     double movementTime = totalTime - totalWaitingTime - totalLoadUnloadTime;

    //     m_logger.Info("========== ИТОГОВЫЙ ОТЧЁТ ==========");
    //     m_logger.Info("Общее время эксперимента: " + std::to_string(totalTime) + "с");
    //     m_logger.Info("Время движения: " + std::to_string(movementTime) + "с");
    //     m_logger.Info("Время ожидания препятствий: " + std::to_string(totalWaitingTime) + "с");
    //     m_logger.Info("Время погрузки/разгрузки: " + std::to_string(totalLoadUnloadTime) + "с");

    //     for (auto const & [hash, stats] : robotStats)
    //     {
    //       m_logger.Info("--- Робот ---");
    //       m_logger.Info("  Ожидание: " + std::to_string(stats.waitingTime) + "с");
    //       m_logger.Info("  Загрузка: " + std::to_string(stats.loadingTime) + "с");
    //       m_logger.Info("  Разгрузка: " + std::to_string(stats.unloadingTime) + "с");
    //     }
    //     m_logger.Info("====================================");

    //     stateStartTimes.erase(robotHash);
    //     lastStates.erase(robotHash);
    //     robotStats.erase(robotHash);
    //     experimentRunning = false;
    //   }
    //   else
    //   {
    //     stateStartTimes[robotHash] = now;
    //     lastStates[robotHash] = newStateAddr;
    //   }

    //   return action.FinishSuccessfully();
    // }
