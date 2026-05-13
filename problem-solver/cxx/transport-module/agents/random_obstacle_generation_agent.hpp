#pragma once

#include <atomic>
#include <random>
#include <thread>
#include <unordered_map>

#include <sc-memory/sc_memory.hpp>

#include "keynodes/keynodes.hpp"

class RandomObstacleGenerationAgent
{
public:
  RandomObstacleGenerationAgent() = default;

  ~RandomObstacleGenerationAgent();

  void Start();

  void Stop();

private:
  void WorkerLoop();

  void GenerateStep(ScMemoryContext & context);

  bool ShouldGenerateObstacle();

  void RemoveExpiredObstacles(ScMemoryContext & context);

  void RemoveObstacle(ScMemoryContext & context, ScAddr const & obstacleAddr);

  ScAddr GenerateObstacle(ScMemoryContext & context);

  ScAddr SelectObstaclePosition(ScMemoryContext & context);

  ScAddrVector CollectActiveRoutePoints(ScMemoryContext & context);

  ScAddr FindRouteByPoint(ScMemoryContext & context, ScAddr const & pointAddr);

  ScAddr GetRobotCurrentPosition(ScMemoryContext & context, ScAddr const & robotAddr);

  int GenerateNextObstacleInterval();

  int GenerateObstacleLifetime();

private:
  static constexpr double ObstacleProbability = 1.0;
  static constexpr int MinObstacleInterval = 1;
  static constexpr int MaxObstacleInterval = 1;
  static constexpr int MinObstacleLifetime = 8;
  static constexpr int MaxObstacleLifetime = 12;

  std::atomic_bool m_isRunning = false;
  std::thread m_worker;

  int m_currentTick = 0;
  int m_ticksSinceLastObstacle = 0;
  int m_nextObstacleInterval = MinObstacleInterval;
  std::unordered_map<ScAddr, int, ScAddrHashFunc> m_obstacleExpirationTicks;

  std::mt19937 m_randomGenerator{std::random_device{}()};
};
