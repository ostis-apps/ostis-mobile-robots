#pragma once

#include <sc-memory/sc_keynodes.hpp>

class Keynodes : public ScKeynodes
{
public:
  static inline ScKeynode const action_coordinate_mobile_robot{"action_coordinate_mobile_robot", ScType::ConstNodeClass};
  static inline ScKeynode const action_interpreter_mobile_robot{"action_interpreter_mobile_robot", ScType::ConstNodeClass};
};

// Подробнее о ключевых элементах тут: https://ostis-ai.github.io/sc-machine/sc-memory/api/cpp/extended/agents/keynodes/
