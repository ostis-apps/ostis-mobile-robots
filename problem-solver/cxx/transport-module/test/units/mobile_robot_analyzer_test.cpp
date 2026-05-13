#include "test_utils.hpp"
#include <thread>
#include <gtest/gtest.h>

TEST_F(TransportModuleTest, AnalyzerPerformanceTest)
{
    try
    {
        ScAgentContext context;
        ScsLoader loader;

        // 1. Загружаем статику (база знаний: робот, маршрут, препятствия)
        loader.loadScsFile(context, std::string(EXAMPLE_MODULE_TEST_FILES_DIR_PATH) + "analyzer_agent_test.scs");

        // 2. Подписываем всех агентов (Интерпретатор, Координатор и Анализатор)
        // Используем общую функцию SubscribeAgents из test_utils.cpp
        SubscribeAgents(context); 

        // 3. Активируем робота (состояние launched)
        // В этот момент интерпретатор увидит робота и начнет имитацию движения
        loader.loadScsFile(context, std::string(EXAMPLE_MODULE_TEST_FILES_DIR_PATH) + "analyzer_agent_test_init.scs");

        // 4. Имитация реальной работы:
        // Даем роботу постоять перед препятствием (Анализатор должен начать считать время ожидания)
        std::this_thread::sleep_for(std::chrono::seconds(2));

        // Удаляем препятствие, чтобы робот "поехал" дальше
        DeleteObstacle(context);

        // Даем роботу "проехать" еще немного (Анализатор считает время движения)
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // 5. Завершение эксперимента
        // Ищем робота и ключевой узел остановки
        ScAddr robotAddr = context.SearchElementBySystemIdentifier("robot_test_analyzer");
        ScAddr stoppedAddr = context.SearchElementBySystemIdentifier("concept_stopped");
        
        // Генерируем статус "stopped". Это триггер для твоего агента вывести финальный отчет.
        context.GenerateConnector(ScType::ConstActualTempPosArc, stoppedAddr, robotAddr);

        // Ждем завершения работы агента (вывода таблицы в консоль)
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        // 6. Чистим подписки
        UnsubscribeAgents(context);
    }
    catch (utils::ScException & e)
    {
        FAIL() << "ScException: " << e.Message();
    }
}
