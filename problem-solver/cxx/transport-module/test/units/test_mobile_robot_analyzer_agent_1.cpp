#include "test_utils.hpp"
#include <chrono>
#include <thread>

TEST_F(TransportModuleTest, SingleRobotComplexPathTest)
{
  try
  {
    ScAgentContext context;
    ScsLoader loader;

    loader.loadScsFile(context, "test-structures/analyzer_agent_test_1.scs");

    SubscribeAgents(context);

    // Достаем адреса элементов для имитации действий
    ScAddr robotAddr = context.SearchElementBySystemIdentifier("robot_alpha");
    ScAddr launchedAddr = context.SearchElementBySystemIdentifier("concept_launched");
    ScAddr stoppedAddr = context.SearchElementBySystemIdentifier("concept_stopped");


    context.GenerateConnector(ScType::ConstActualTempPosArc, launchedAddr, robotAddr);
      
    std::this_thread::sleep_for(std::chrono::seconds(4));

    context.GenerateConnector(ScType::ConstActualTempPosArc, stoppedAddr, robotAddr);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    WaitAgents(context);
    UnsubscribeAgents(context);
  }
  catch (utils::ScException & e)
  {
    std::cout << "SC-Error: " << e.Message() << std::endl;
  }
}
