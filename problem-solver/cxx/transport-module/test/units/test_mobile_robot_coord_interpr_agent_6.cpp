#include "test_utils.hpp"

TEST_F(TransportModuleTest, CallMobileRobotCoordInterprAgent6)
{
  try
  {
    ScAgentContext context;
    ScsLoader loader;
    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "coord_interpr_agent_test_6.scs");

    SubscribeInterCoordAgents(context);

    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "coord_interpr_agent_test_6_initial_states.scs");

    sleep(4);

    DeleteObstacle(context);

    WaitAgents(context);

    UnsubscribeInterCoordAgents(context);
  }
  catch (utils::ScException & e)
  {
    std::cout << e.Message() << std::endl;
  }
}
