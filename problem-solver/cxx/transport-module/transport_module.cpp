#include "transport_module.hpp"

#include "agents/mobile_robot_coordination_agent.hpp"
#include "agents/mobile_robot_interpretation_agent.hpp"
#include "agents/mobile_robot_analyzer_agent.hpp"
#include "agents/random_obstacle_generation_agent.hpp"


#include "keynodes/keynodes.hpp"

// Тут надо будет добавить заголовочные файлы для агента генерации случайных препятствий и агента дампа статистики

SC_MODULE_REGISTER(TransportModule)
  ->Agent<MobileRobotAnalyzerAgent>();
// Тут надо зарегистрировать агент генерации случайных препятствий и агент дампа статистики
// ->Agent<MyAgent>();
// Подробнее тут https://ostis-ai.github.io/sc-machine/sc-memory/api/cpp/extended/agents/modules/

void TransportModule::Initialize(ScMemoryContext * _)
{
  ScAgentContext context;
  ScIterator3Ptr const it3 = context.CreateIterator3(
    MobileRobotsKeynodes::concept_mobile_robot,
    ScType::ConstPermPosArc,
    ScType::ConstNode
  );
  while (it3->Next())
  {
    ScAddr const & robotAddr = it3->Get(2);
    context.SubscribeAgent<MobileRobotCoordinationAgent>(robotAddr);
    context.SubscribeAgent<MobileRobotInterpretationAgent>(robotAddr);
    context.SubscribeAgent<MobileRobotAnalyzerAgent>(robotAddr);
  }
  context.SubscribeAgent<RandomObstacleGenerationAgent>(MobileRobotsKeynodes::concept_simulation_time_tick);
}

void TransportModule::Shutdown(ScMemoryContext * _)
{  
  ScAgentContext context;
  ScIterator3Ptr const it3 = context.CreateIterator3(
    MobileRobotsKeynodes::concept_mobile_robot,
    ScType::ConstPermPosArc,
    ScType::ConstNode
  );
  while (it3->Next())
  {
    ScAddr const & robotAddr = it3->Get(2);
    context.UnsubscribeAgent<MobileRobotCoordinationAgent>(robotAddr);
    context.UnsubscribeAgent<MobileRobotInterpretationAgent>(robotAddr);
    context.UnsubscribeAgent<MobileRobotAnalyzerAgent>(robotAddr);
  }
  context.SubscribeAgent<RandomObstacleGenerationAgent>(MobileRobotsKeynodes::concept_simulation_time_tick);
}
