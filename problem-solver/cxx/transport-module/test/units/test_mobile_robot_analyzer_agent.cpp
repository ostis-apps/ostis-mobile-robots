#include "test_utils.hpp"
#include <chrono>
#include <thread>

TEST_F(TransportModuleTest, CallMobileRobotAnalyzerAgent)
{
  try
  {
    ScAgentContext context;
    ScsLoader loader;
    
    // Загружаем тестовую структуру
    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "example_analyzer.scs");
    
    // Подписываем агентов 
    SubscribeAgents(context);
    
    // Загружаем начальные состояния робота
    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "example_analyzer_initial_states.scs");
    
    // Ждём завершения 
    WaitAgents(context);
    
    // Отписываем агентов
    UnsubscribeAgents(context);
  }
  catch (utils::ScException & e)
  {
    std::cout << e.Message() << std::endl;
  }
}
