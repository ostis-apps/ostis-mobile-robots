#include "test_utils.hpp"

TEST_F(TransportModuleTest, CallMobileRobotCoordInterprAgent2)
{
  try
  {
    ScAgentContext context;
    ScsLoader loader;
    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "coord_interpr_agent_test_2.scs");

    SubscribeInterCoordAgents(context);

    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "coord_interpr_agent_test_2_initial_states.scs");

    WaitAgents(context);

    UnsubscribeAgents(context);
  }
  catch (utils::ScException & e)
  {
    std::cout << e.Message() << std::endl;
  }
}
