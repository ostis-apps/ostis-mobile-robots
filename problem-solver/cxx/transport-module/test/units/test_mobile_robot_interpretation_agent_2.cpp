#include "test_utils.hpp"

TEST_F(TransportModuleTest, CallMobileRobotInterpretationAgent2)
{
  try
  {
    ScAgentContext context;
    ScsLoader loader;
    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "example_4.scs");

    SubscribeAgents(context);

    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "example_4_robots_initial_states.scs");

    sleep(15);
    // переписать на ожидание событий остановки всех роботов

    UnsubscribeAgents(context);
  }
  catch (utils::ScException & e)
  {
    std::cout << e.Message() << std::endl;
  }
}
