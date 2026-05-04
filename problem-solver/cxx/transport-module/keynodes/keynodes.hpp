#pragma once

#include <sc-memory/sc_keynodes.hpp>

using InterpreterCallback = std::function<ScResult(ScAction & action, ScAddr const &)>;

class MobileRobotsKeynodes : public ScKeynodes
{
public:
  static inline ScKeynode const action_coordinate_mobile_robot{"action_coordinate_mobile_robot", ScType::ConstNodeClass};
  static inline ScKeynode const action_interpreter_mobile_robot{"action_interpreter_mobile_robot", ScType::ConstNodeClass};

  static inline ScKeynode const concept_launched{"concept_launched", ScType::ConstNodeClass};
  static inline ScKeynode const concept_stopped{"concept_stopped", ScType::ConstNodeClass};
  static inline ScKeynode const concept_box_loaded{"concept_box_loaded", ScType::ConstNodeClass};
  static inline ScKeynode const concept_box_unloaded{"concept_box_unloaded", ScType::ConstNodeClass};
  static inline ScKeynode const concept_ready_being_launched{"concept_ready_being_launched", ScType::ConstNodeClass};
  static inline ScKeynode const concept_ready_being_unlaunched{"concept_ready_being_unlaunched", ScType::ConstNodeClass};
  
  static inline ScKeynode const concept_mobile_robot{"concept_mobile_robot", ScType::ConstNodeClass};
  static inline ScKeynode const concept_box{"concept_box", ScType::ConstNodeClass};
  
  static inline ScKeynode const nrel_location{"nrel_location", ScType::ConstNodeNonRole};

  static inline ScKeynode const concept_simulation_time_tick{"concept_simulation_time_tick", ScType::ConstNodeClass};
  static inline ScKeynode const concept_obstacle{"concept_obstacle", ScType::ConstNodeClass};
  static inline ScKeynode const concept_obstacle_appearance_event{"concept_obstacle_appearance_event",ScType::ConstNodeClass};

  static inline ScKeynode const nrel_generated_obstacle{"nrel_generated_obstacle",ScType::ConstNodeNonRole};
  static inline ScKeynode const nrel_event_time{ "nrel_event_time", ScType::ConstNodeNonRole };

};

// Подробнее о ключевых элементах тут: https://ostis-ai.github.io/sc-machine/sc-memory/api/cpp/extended/agents/keynodes/
