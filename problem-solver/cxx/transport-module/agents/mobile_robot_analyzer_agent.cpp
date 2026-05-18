#include "mobile_robot_analyzer_agent.hpp"
#include <sc-memory/sc_link.hpp>
#include <chrono>
#include <thread>
#include <map>

// ------------------------------------------------------------
// Структура накопления статистики по одному роботу
// Хранит суммарное время пребывания в ключевых состояниях
// ------------------------------------------------------------
struct RobotStats
{
  double waitingTime = 0;    // время ожидания препятствий
  double loadingTime = 0;    // время загрузки коробки
  double unloadingTime = 0;  // время разгрузки коробки
  double movingTime = 0;     // общее время движения
};

// ------------------------------------------------------------
// Глобальные хранилища состояния эксперимента
// ------------------------------------------------------------

// Статистика по каждому роботу (ключ — hash SC-адреса робота)
static std::map<size_t, RobotStats> robotStats;

// Временные метки входа в состояния:
// robotHash -> (stateHash -> time_point)
static std::map<size_t, std::map<size_t, std::chrono::steady_clock::time_point>> stateStartTimes;

// ------------------------------------------------------------
// Глобальные агрегированные метрики эксперимента
// Используются для финального отчёта
// ------------------------------------------------------------
static double totalWaitingTime = 0;
static double totalLoadUnloadTime = 0;
static double totalMovingTime = 0;

// Время старта всего эксперимента
static std::chrono::steady_clock::time_point experimentStartTime;

// ------------------------------------------------------------
// Возвращает SC-адрес класса действия, на который реагирует агент
// ------------------------------------------------------------
ScAddr MobileRobotAnalyzerAgent::GetActionClass() const
{
  return MobileRobotsKeynodes::action_analyze_mobile_robot;
}

// ------------------------------------------------------------
// Проверка условия активации агента
// Агент вызывается при изменении состояния робота (SC-event)
// ------------------------------------------------------------
bool MobileRobotAnalyzerAgent::CheckInitiationCondition(ScEventChangeMobileRobotState const & event)
{
  ScAddr const & stateAddr = event.GetArcSourceElement();

  // Соответствие SC-состояний и обработчиков агента
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

  // Если состояние не поддерживается — агент не активируется
  auto const & it = states.find(stateAddr);
  if (it == states.cend())
    return false;

  m_interpreterCallback = it->second;
  return true;
}

// ------------------------------------------------------------
// Основной вход агента: выполнение выбранного обработчика
// ------------------------------------------------------------
ScResult MobileRobotAnalyzerAgent::DoProgram(
    ScEventChangeMobileRobotState const & event,
    ScAction & action)
{
  ScAddr const & robotAddr = event.GetArcTargetElement();
  return m_interpreterCallback(action, robotAddr);
}

// ------------------------------------------------------------
// Инициализация эксперимента
// Сбрасывает статистику и фиксирует стартовое время
// ------------------------------------------------------------
ScResult MobileRobotAnalyzerAgent::InterpreterStateLaunched(
    ScAction & action,
    ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  auto now = std::chrono::steady_clock::now();

  experimentStartTime = now;

  // очистка всей предыдущей статистики
  totalWaitingTime = 0;
  totalLoadUnloadTime = 0;
  robotStats.clear();

  return action.FinishSuccessfully();
}

// ------------------------------------------------------------
// Финализация эксперимента для конкретного робота
// Вывод индивидуальной статистики
// ------------------------------------------------------------
ScResult MobileRobotAnalyzerAgent::InterpreterStateStopped(
    ScAction & action,
    ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();

  // небольшая задержка для стабилизации событий SC-памяти
  std::this_thread::sleep_for(std::chrono::seconds(1));

  RobotStats stats = robotStats[robotHash];

  // логирование итогов по конкретному роботу
  SC_LOG_INFO("====================================");
  SC_LOG_INFO("--- " + m_context.GetElementSystemIdentifier(robotAddr) + " ---");
  SC_LOG_INFO("Waiting: " + std::to_string(stats.waitingTime) + "с");
  SC_LOG_INFO("Moving: " + std::to_string(stats.movingTime) + "с");
  SC_LOG_INFO("Loading: " + std::to_string(stats.loadingTime) + "с");
  SC_LOG_INFO("Unloading: " + std::to_string(stats.unloadingTime) + "с");
  SC_LOG_INFO("====================================");

  // очистка локальных данных робота
  stateStartTimes.erase(robotHash);
  robotStats.erase(robotHash);

  // проверка: завершены ли все роботы в эксперименте
  bool other_is_launched = false;

  ScIterator3Ptr const it3 =
      m_context.CreateIterator3(robotAddr, ScType::ConstPermPosArc, ScType::ConstNodeTuple);

  while (it3->Next())
  {
    ScAddr robotGroupAddr = it3->Get(2);

    ScIterator3Ptr const it3_1 =
        m_context.CreateIterator3(robotGroupAddr, ScType::ConstPermPosArc, ScType::ConstNode);

    while (it3_1->Next())
    {
      if (m_context.CheckConnector(
              MobileRobotsKeynodes::concept_launched,
              it3_1->Get(2),
              ScType::ConstActualTempPosArc))
      {
        other_is_launched = true;
        break;
      }
    }

    if (other_is_launched)
      break;
  }

  // если больше нет активных роботов — печать общей статистики
  if (!other_is_launched)
    LogTotalStats();

  return action.FinishSuccessfully();
}

// ------------------------------------------------------------
// Обработка входа в состояние "движение"
// ------------------------------------------------------------
ScResult MobileRobotAnalyzerAgent::InterpreterStateIsMoving(
    ScAction & action,
    ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  size_t classHash = MobileRobotsKeynodes::concept_is_moving.Hash();

  stateStartTimes[robotHash][classHash] =
      std::chrono::steady_clock::now();

  return action.FinishSuccessfully();
}

// ------------------------------------------------------------
// Выход из состояния "движение"
// вычисление длительности и накопление статистики
// ------------------------------------------------------------
ScResult MobileRobotAnalyzerAgent::InterpreterStateIsNotMoving(
    ScAction & action,
    ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  size_t classHash = MobileRobotsKeynodes::concept_is_moving.Hash();

  double seconds = CalculateDiffInSeconds(robotHash, classHash);

  robotStats[robotHash].movingTime += seconds;
  totalMovingTime += seconds;

  return action.FinishSuccessfully();
}

// ------------------------------------------------------------
// Аналогичные обработчики для WAITING
// ------------------------------------------------------------
ScResult MobileRobotAnalyzerAgent::InterpreterStateIsWaiting(
    ScAction & action,
    ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  size_t classHash = MobileRobotsKeynodes::concept_robot_is_waiting.Hash();

  stateStartTimes[robotHash][classHash] =
      std::chrono::steady_clock::now();

  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::InterpreterStateIsNotWaiting(
    ScAction & action,
    ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  size_t classHash = MobileRobotsKeynodes::concept_robot_is_waiting.Hash();

  double seconds = CalculateDiffInSeconds(robotHash, classHash);

  robotStats[robotHash].waitingTime += seconds;
  totalWaitingTime += seconds;

  return action.FinishSuccessfully();
}

// ------------------------------------------------------------
// Аналогичные обработчики для LOADING
// ------------------------------------------------------------
ScResult MobileRobotAnalyzerAgent::InterpreterStateIsLoading(
    ScAction & action,
    ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  size_t classHash = MobileRobotsKeynodes::concept_robot_is_loading.Hash();

  stateStartTimes[robotHash][classHash] =
      std::chrono::steady_clock::now();

  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::InterpreterStateIsNotLoading(
    ScAction & action,
    ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  size_t classHash = MobileRobotsKeynodes::concept_robot_is_loading.Hash();

  double seconds = CalculateDiffInSeconds(robotHash, classHash);

  robotStats[robotHash].loadingTime += seconds;
  totalLoadUnloadTime += seconds;

  return action.FinishSuccessfully();
}

// ------------------------------------------------------------
// Аналогичные обработчики для UNLOADING
// ------------------------------------------------------------
ScResult MobileRobotAnalyzerAgent::InterpreterStateIsUnloading(
    ScAction & action,
    ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  size_t classHash = MobileRobotsKeynodes::concept_robot_is_unloading.Hash();

  stateStartTimes[robotHash][classHash] =
      std::chrono::steady_clock::now();

  return action.FinishSuccessfully();
}

ScResult MobileRobotAnalyzerAgent::InterpreterStateIsNotUnloading(
    ScAction & action,
    ScAddr const & robotAddr)
{
  size_t robotHash = robotAddr.Hash();
  size_t classHash = MobileRobotsKeynodes::concept_robot_is_unloading.Hash();

  double seconds = CalculateDiffInSeconds(robotHash, classHash);

  robotStats[robotHash].unloadingTime += seconds;
  totalLoadUnloadTime += seconds;

  return action.FinishSuccessfully();
}

// ------------------------------------------------------------
// Вычисление разницы времени между входом и выходом состояния
// ------------------------------------------------------------
double MobileRobotAnalyzerAgent::CalculateDiffInSeconds(
    size_t const & robotHash,
    size_t const & classHash)
{
  auto now = std::chrono::steady_clock::now();
  auto startTime = stateStartTimes[robotHash][classHash];

  return std::chrono::duration<double>(now - startTime).count();
}

// ------------------------------------------------------------
// Печать глобальной статистики эксперимента
// ------------------------------------------------------------
void MobileRobotAnalyzerAgent::LogTotalStats()
{
  auto now = std::chrono::steady_clock::now();
  double seconds = std::chrono::duration<double>(now - experimentStartTime).count();

  SC_LOG_INFO("====================================");
  SC_LOG_INFO("--- Total Stats ---");
  SC_LOG_INFO("Experiment: " + std::to_string(seconds) + "с");
  SC_LOG_INFO("Moving: " + std::to_string(totalMovingTime) + "с");
  SC_LOG_INFO("Loading/Unloading: " + std::to_string(totalLoadUnloadTime) + "с");
  SC_LOG_INFO("Waiting: " + std::to_string(totalWaitingTime) + "с");
  SC_LOG_INFO("====================================");
}
