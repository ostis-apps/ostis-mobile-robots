#include "test_utils.hpp"

#define private public
#include <agents/random_obstacle_generation_agent.hpp>
#undef private

TEST_F(TransportModuleTest, CallMobileRobotCoordInterprAgent5)
{
  try
  {
    ScAgentContext context;
    ScsLoader loader;
    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "coord_interpr_agent_test_5.scs");

    SubscribeInterCoordAgents(context);

    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "coord_interpr_agent_test_5_initial_states.scs");

    RandomObstacleGenerationAgent obstacleGenerator;
    obstacleGenerator.GenerateObstacle(context);

    sleep(6);

    DeleteObstacle(context);

    WaitAgents(context);

    UnsubscribeInterCoordAgents(context);
  }
  catch (utils::ScException & e)
  {
    std::cout << e.Message() << std::endl;
  }
}
