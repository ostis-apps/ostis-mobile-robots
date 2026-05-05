#pragma once

#include <random>
#include <unordered_map>

#include <sc-memory/sc_agent.hpp>

#include "keynodes/keynodes.hpp"

using ScEventGenerateSimulationTimeTick = ScEventAfterGenerateOutgoingArc<ScType::ConstPosArc>;

class RandomObstacleGenerationAgent : public ScAgent<ScEventGenerateSimulationTimeTick>
{
public:
  ScAddr GetActionClass() const override;

  bool CheckInitiationCondition(ScEventGenerateSimulationTimeTick const & event) override;

  ScResult DoProgram(
      ScEventGenerateSimulationTimeTick const & event,
      ScAction & action) override;

private:
  bool ShouldGenerateObstacle();

  void RemoveExpiredObstacles();

  void RemoveObstacle(ScAddr const & obstacleAddr);

  ScAddr GenerateObstacle();

  ScAddr SelectObstaclePosition();

  int GenerateNextObstacleInterval();

  int GenerateObstacleLifetime();

private:
  static constexpr double ObstacleProbability = 0.2;
  static constexpr int MinObstacleInterval = 3;
  static constexpr int MaxObstacleInterval = 8;
  static constexpr int MinObstacleLifetime = 2;
  static constexpr int MaxObstacleLifetime = 5;

  int m_currentTick = 0;
  int m_ticksSinceLastObstacle = 0;
  int m_nextObstacleInterval = MinObstacleInterval;
  std::unordered_map<ScAddr, int, ScAddrHashFunc> m_obstacleExpirationTicks;

  std::mt19937 m_randomGenerator{std::random_device{}()};
};
