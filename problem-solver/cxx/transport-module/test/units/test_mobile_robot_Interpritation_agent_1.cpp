#include "test_utils.hpp"

#define private public
#include <agents/random_obstacle_generation_agent.hpp>
#undef private

TEST_F(TransportModuleTest, CallMobileRobotInterpritationAgent1)
{
  try
  {
    ScAgentContext context;
    ScsLoader loader;
    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "Test_robot_1_route.scs");

    SubscribeAgents(context);

    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "Test_robot_1_state.scs");

    RandomObstacleGenerationAgent obstacleGenerator;
    obstacleGenerator.GenerateObstacle(context);

    sleep(6);

    DeleteObstacle(context);

    WaitAgents(context);

    UnsubscribeAgents(context);
  }
  catch (utils::ScException & e)
  {
    std::cout << e.Message() << std::endl;
  }
}
