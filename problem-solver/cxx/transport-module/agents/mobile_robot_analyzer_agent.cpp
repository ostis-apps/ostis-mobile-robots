#include "mobile_robot_analyzer_agent.hpp"
#include <sc-memory/sc_link.hpp>
#include <chrono>
#include <thread>
#include <map>

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
static double totalMovingTime = 0;
static std::chrono::steady_clock::time_point experimentStartTime;

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
  experimentStartTime = now;
  totalWaitingTime = 0;
  totalLoadUnloadTime = 0;
  robotStats.clear();
  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::InterpreterStateStopped(ScAction & action, ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();

  std::this_thread::sleep_for(std::chrono::seconds(1));
  RobotStats stats = robotStats[robotHash];
  SC_LOG_INFO("====================================");
  SC_LOG_INFO("--- " + m_context.GetElementSystemIdentifier(robotAddr) + " ---");
  SC_LOG_INFO("Ожидание: " + std::to_string(stats.waitingTime) + "с");
  SC_LOG_INFO("Движение: " + std::to_string(stats.movingTime) + "с");
  SC_LOG_INFO("Загрузка: " + std::to_string(stats.loadingTime) + "с");
  SC_LOG_INFO("Разгрузка: " + std::to_string(stats.unloadingTime) + "с");
  SC_LOG_INFO("====================================");
  stateStartTimes.erase(robotHash);
  robotStats.erase(robotHash);

  bool other_is_launched = false;
  ScIterator3Ptr const it3 =
      m_context.CreateIterator3(robotAddr, ScType::ConstPermPosArc, ScType::ConstNodeTuple);
  while (it3->Next()){
    ScAddr robotGroupAddr = it3->Get(2);
    ScIterator3Ptr const it3_1 = m_context.CreateIterator3(robotGroupAddr, ScType::ConstPermPosArc, ScType::ConstNode);
    while(it3_1->Next())
    {
      if (m_context.CheckConnector(MobileRobotsKeynodes::concept_launched, it3_1->Get(2), ScType::ConstActualTempPosArc)){
        other_is_launched = true;
        break;
      }
    }
    if (other_is_launched)
      break;
  }
  if(!other_is_launched)
    LogTotalStats();
  
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
  totalMovingTime += seconds;
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
  totalWaitingTime += seconds;
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
  totalLoadUnloadTime += seconds;
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
  totalLoadUnloadTime += seconds;
  return action.FinishSuccessfully();
}

double MobileRobotAnalyzerAgent::CalculateDiffInSeconds(size_t const & robotHash, size_t const & classHash)
{
  auto now = std::chrono::steady_clock::now();
  auto startTime = stateStartTimes[robotHash][classHash];
  double seconds = std::chrono::duration<double>(now - startTime).count();
  return seconds;
}

void MobileRobotAnalyzerAgent::LogTotalStats()
{
  auto now = std::chrono::steady_clock::now();
  double seconds = std::chrono::duration<double>(now - experimentStartTime).count();
  SC_LOG_INFO("====================================");
  SC_LOG_INFO("--- Total Stats ---");
  SC_LOG_INFO("Эксперимент: " + std::to_string(seconds) + "с");
  SC_LOG_INFO("Движение: " + std::to_string(totalMovingTime) + "с");
  SC_LOG_INFO("Загрузка/Разгрузка: " + std::to_string(totalLoadUnloadTime) + "с");
  SC_LOG_INFO("Ожидание: " + std::to_string(totalWaitingTime) + "с");
  SC_LOG_INFO("====================================");
}
