#pragma once

#include <sc-memory/sc_module.hpp>

#include "agents/random_obstacle_generation_agent.hpp"

class TransportModule : public ScModule
{
public:
  void Initialize(ScMemoryContext * context);

  void Shutdown(ScMemoryContext * context);

private:
  RandomObstacleGenerationAgent m_randomObstacleGenerationAgent;
};
