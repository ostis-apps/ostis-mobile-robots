#include "test_utils.hpp"

TEST_F(TransportModuleTest, CallMobileRobotCoordinationAgent1)
{
  try
  {
    ScAgentContext context;
    ScsLoader loader;
    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "coordination_agent_test_1.scs");

    SubscribeInterCoordAgents(context);

    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "coordination_agent_test_1_initial_states.scs");

    WaitAgents(context);

    UnsubscribeInterCoordAgents(context);
  }
  catch (utils::ScException & e)
  {
    std::cout << e.Message() << std::endl;
  }
}
