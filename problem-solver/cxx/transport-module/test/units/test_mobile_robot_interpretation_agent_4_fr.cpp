#include "test_utils.hpp"
#include "mobile"

TEST_F(TransportModuleTest, CallMobileRobotInterpretationAgent3fr)
{
  try
  {
    ScAgentContext context;
    ScsLoader loader;
    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "example_3_fr.scs");

    SubscribeAgents(context);

    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "example_3_robots_initial_states_fr.scs");

    sleep(15);
    UnsubscribeAgents(context);
  }
  catch (utils::ScException & e)
  {
    std::cout << e.Message() << std::endl;
  }
}
