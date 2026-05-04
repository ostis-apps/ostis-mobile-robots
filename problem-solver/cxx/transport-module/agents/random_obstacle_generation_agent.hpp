#pragma once

#include <random>

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

  ScAddr GenerateObstacleAppearanceEvent(ScAddr const & tickAddr);

  ScAddr GenerateObstacle(ScAddr const & tickAddr);

  ScAddr SelectObstaclePosition();

  int GenerateNextObstacleInterval();

private:
  static constexpr double ObstacleProbability = 0.2;
  static constexpr int MinObstacleInterval = 3;
  static constexpr int MaxObstacleInterval = 8;

  int m_ticksSinceLastObstacle = 0;
  int m_nextObstacleInterval = MinObstacleInterval;

  std::mt19937 m_randomGenerator{std::random_device{}()};
};
