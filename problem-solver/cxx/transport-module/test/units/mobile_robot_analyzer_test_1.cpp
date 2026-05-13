#include "test_utils.hpp"
#include <thread>
#include <gtest/gtest.h>
#include <sc-memory/sc_link.hpp>

TEST_F(TransportModuleTest, ComplexRouteCycleTest)
{
    try
    {
        ScAgentContext context;
        ScsLoader loader;

        loader.loadScsFile(context, std::string(EXAMPLE_MODULE_TEST_FILES_DIR_PATH) + "analyzer_agent_test_1.scs");
        SubscribeAgents(context);

        // Ищем все необходимые узлы 
        ScAddr robotAddr = context.SearchElementBySystemIdentifier("robot_complex");
        ScAddr loadingAddr = context.SearchElementBySystemIdentifier("concept_robot_is_loading");
        ScAddr notLoadingAddr = context.SearchElementBySystemIdentifier("concept_robot_is_not_loading");
        ScAddr waitingAddr = context.SearchElementBySystemIdentifier("concept_robot_is_waiting");
        ScAddr notWaitingAddr = context.SearchElementBySystemIdentifier("concept_robot_is_not_waiting");
        ScAddr unloadingAddr = context.SearchElementBySystemIdentifier("concept_robot_is_unloading");
        ScAddr notUnloadingAddr = context.SearchElementBySystemIdentifier("concept_robot_is_not_unloading");
        ScAddr stoppedAddr = context.SearchElementBySystemIdentifier("concept_stopped");

        ASSERT_TRUE(robotAddr.IsValid() && loadingAddr.IsValid() && notLoadingAddr.IsValid());
        ASSERT_TRUE(waitingAddr.IsValid() && notWaitingAddr.IsValid());

        SC_LOG_INFO("--- [ТЕСТ] Шаг 1: LAUNCHED ---");
        loader.loadScsFile(context, std::string(EXAMPLE_MODULE_TEST_FILES_DIR_PATH) + "analyzer_agent_test_1_init.scs");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        SC_LOG_INFO("--- [ТЕСТ] Шаг 2: Погрузка ---");
        context.GenerateConnector(ScType::ConstActualTempPosArc, loadingAddr, robotAddr);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        context.GenerateConnector(ScType::ConstActualTempPosArc, notLoadingAddr, robotAddr); 
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        SC_LOG_INFO("--- [ТЕСТ] Шаг 3: Ожидание ---");
        context.GenerateConnector(ScType::ConstActualTempPosArc, waitingAddr, robotAddr);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        context.GenerateConnector(ScType::ConstActualTempPosArc, notWaitingAddr, robotAddr); 
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        SC_LOG_INFO("--- [ТЕСТ] Шаг 4: Разгрузка ---");
        context.GenerateConnector(ScType::ConstActualTempPosArc, unloadingAddr, robotAddr);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        context.GenerateConnector(ScType::ConstActualTempPosArc, notUnloadingAddr, robotAddr); 
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        SC_LOG_INFO("--- [ТЕСТ] Шаг 5: STOPPED ---");
        context.GenerateConnector(ScType::ConstActualTempPosArc, stoppedAddr, robotAddr);

        std::this_thread::sleep_for(std::chrono::seconds(2));
        UnsubscribeAgents(context);
    }
    catch (utils::ScException & e)
    {
        FAIL() << "ScException: " << e.Message();
    }
}
