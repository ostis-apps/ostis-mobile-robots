#include "test_utils.hpp"

TEST_F(TransportModuleTest, CallMobileRobotInterpritationAgent1)
{
  try
  {
    ScAgentContext context;
    ScsLoader loader;
    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "Test_robot_1_route.scs");

    SubscribeAgents(context);

    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "Test_robot_1_state.scs");

    sleep(5);
    // переписать на ожидание событий остановки всех роботов

    UnsubscribeAgents(context);
  }
  catch (utils::ScException & e)
  {
    std::cout << e.Message() << std::endl;
  }
}
