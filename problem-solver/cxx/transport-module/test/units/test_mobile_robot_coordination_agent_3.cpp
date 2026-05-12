#include "test_utils.hpp"

TEST_F(TransportModuleTest, CallMobileRobotCoordinationAgent3)
{
  try
  {
    ScAgentContext context;
    ScsLoader loader;
    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "example_5.scs");

    SubscribeAgents(context);

    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "example_5_robots_initial_states.scs");

    sleep(30);
    // переписать на ожидание событий остановки всех роботов

    UnsubscribeAgents(context);
  }
  catch (utils::ScException & e)
  {
    std::cout << e.Message() << std::endl;
  }
}
