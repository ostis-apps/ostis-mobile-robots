#include "test_utils.hpp"
#include <thread>
#include <gtest/gtest.h>

TEST_F(TransportModuleTest, AnalyzerPerformanceTest)
{
    try
    {
        ScAgentContext context;
        ScsLoader loader;
        loader.loadScsFile(context, std::string(EXAMPLE_MODULE_TEST_FILES_DIR_PATH) + "analyzer_agent_test.scs");

        SubscribeAgents(context); 
      
        loader.loadScsFile(context, std::string(EXAMPLE_MODULE_TEST_FILES_DIR_PATH) + "analyzer_agent_test_init.scs");

        std::this_thread::sleep_for(std::chrono::seconds(2));

        DeleteObstacle(context);

        std::this_thread::sleep_for(std::chrono::seconds(3));

        ScAddr robotAddr = context.SearchElementBySystemIdentifier("robot_test_analyzer");
        ScAddr stoppedAddr = context.SearchElementBySystemIdentifier("concept_stopped");
        
        context.GenerateConnector(ScType::ConstActualTempPosArc, stoppedAddr, robotAddr);

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        UnsubscribeAgents(context);
    }
    catch (utils::ScException & e)
    {
        FAIL() << "ScException: " << e.Message();
    }
}
