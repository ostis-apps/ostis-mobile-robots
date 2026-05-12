#include "test_utils.hpp"

TEST_F(TransportModuleTest, CallMobileRobotInterpretationAgent1)
{
  try
  {
    ScAgentContext context;
    ScsLoader loader;
    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "interpretation_agent_test_1.scs");

    SubscribeAgents(context);

    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "interpretation_agent_test_1_initial_states.scs");

    sleep(10);

    DeleteObstacle(context);

    WaitAgents(context);

    UnsubscribeAgents(context);
  }
  catch (utils::ScException & e)
  {
    std::cout << e.Message() << std::endl;
  }
}