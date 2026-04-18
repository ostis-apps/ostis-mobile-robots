#include "transport_module.hpp"

#include "agents/mobile_robot_coordination_agent.hpp"
#include "agents/mobile_robot_interpretation_agent.hpp"
// Тут надо будет добавить заголовочные файлы для агента генерации случайных препятствий и агента дампа статистики

SC_MODULE_REGISTER(TransportModule);
// Тут надо зарегистрировать агент генерации случайных препятствий и агент дампа статистики
// ->Agent<MyAgent>();
// Подробнее тут https://ostis-ai.github.io/sc-machine/sc-memory/api/cpp/extended/agents/modules/

void TransportModule::Initialize(ScMemoryContext * _)
{
  ScAgentContext context;
  // Тут надо пройтись по всем мобильным роботам в базе знаний и для них зарегистрировать агент координации и агент интерпретации

  // Подробнее тут https://ostis-ai.github.io/sc-machine/sc-memory/api/cpp/extended/agents/modules/
}

void TransportModule::Shutdown(ScMemoryContext * _)
{
  ScAgentContext context;
  // Тут надо пройтись по всем мобильным роботам в базе знаний и для них дерегистрировать агент координации и агент интерпретации

  // Подробнее тут https://ostis-ai.github.io/sc-machine/sc-memory/api/cpp/extended/agents/modules/
}
