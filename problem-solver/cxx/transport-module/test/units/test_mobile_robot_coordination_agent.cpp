#include <sc-memory/test/sc_test.hpp>
#include <sc-builder/scs_loader.hpp>

#include <agents/mobile_robot_coordination_agent.hpp>
#include <agents/mobile_robot_interpretation_agent.hpp>
#include <keynodes/keynodes.hpp>

using TransportModuleTest = ScMemoryTest;
std::string const EXAMPLE_MODULE_TEST_FILES_DIR_PATH = "../test-structures/";

void SubscribeAgents(ScAgentContext & context)
{
  ScIterator3Ptr const it3 = context.CreateIterator3(
    MobileRobotsKeynodes::concept_mobile_robot,
    ScType::ConstPermPosArc,
    ScType::ConstNode
  );
  while (it3->Next())
  {
    ScAddr const & robotAddr = it3->Get(2);
    context.SubscribeAgent<MobileRobotCoordinationAgent>(robotAddr);
    context.SubscribeAgent<MobileRobotInterpretationAgent>(robotAddr);
  }
}

void UnsubscribeAgents(ScAgentContext & context)
{
  ScIterator3Ptr const it3 = context.CreateIterator3(
    MobileRobotsKeynodes::concept_mobile_robot,
    ScType::ConstPermPosArc,
    ScType::ConstNode
  );
  while (it3->Next())
  {
    ScAddr const & robotAddr = it3->Get(2);
    context.UnsubscribeAgent<MobileRobotCoordinationAgent>(robotAddr);
    context.UnsubscribeAgent<MobileRobotInterpretationAgent>(robotAddr);
  }
}

TEST_F(TransportModuleTest, CallMobileRobotCoordinationAgent)
{
  try
  {
    ScAgentContext context;
    ScsLoader loader;
    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "example_1.scs");

    SubscribeAgents(context);

    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "example_1_robots_initial_states.scs");

    UnsubscribeAgents(context);
  }
  catch (utils::ScException & e) {
    std::cout << e.Message() << std::endl;
  }
}
/*#include <sc-memory/test/sc_test.hpp>
#include <sc-builder/scs_loader.hpp>

#include <agents/mobile_robot_coordination_agent.hpp>
#include <agents/mobile_robot_interpretation_agent.hpp>
#include <keynodes/keynodes.hpp>

using TransportModuleTest = ScMemoryTest;
std::string const EXAMPLE_MODULE_TEST_FILES_DIR_PATH = "../test-structures/";

void SubscribeAgents(ScAgentContext & context)
{
  ScIterator3Ptr const it3 = context.CreateIterator3(
    MobileRobotsKeynodes::concept_mobile_robot,
    ScType::ConstPermPosArc,
    ScType::ConstNode
  );
  while (it3->Next())
  {
    ScAddr const & robotAddr = it3->Get(2);
    context.SubscribeAgent<MobileRobotCoordinationAgent>(robotAddr);
    context.SubscribeAgent<MobileRobotInterpretationAgent>(robotAddr);
  }
}

void UnsubscribeAgents(ScAgentContext & context)
{
  ScIterator3Ptr const it3 = context.CreateIterator3(
    MobileRobotsKeynodes::concept_mobile_robot,
    ScType::ConstPermPosArc,
    ScType::ConstNode
  );
  while (it3->Next())
  {
    ScAddr const & robotAddr = it3->Get(2);
    context.UnsubscribeAgent<MobileRobotCoordinationAgent>(robotAddr);
    context.UnsubscribeAgent<MobileRobotInterpretationAgent>(robotAddr);
  }
}

// ============================================================================
// ТЕСТ 1: Базовый вызов агента координации (существующий)
// ============================================================================
TEST_F(TransportModuleTest, CallMobileRobotCoordinationAgent)
{
  try
  {
    ScAgentContext context;
    ScsLoader loader;
    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "example_1.scs");

    SubscribeAgents(context);

    loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "example_1_robots_initial_states.scs");

    UnsubscribeAgents(context);
  }
  catch (utils::ScException & e) {
    std::cout << e.Message() << std::endl;
  }
}

// ============================================================================
// ТЕСТ 2: Переход в состояние "запущен" (InterpreterStateLaunched)
// ============================================================================
TEST_F(TransportModuleTest, InterpreterStateLaunched_AddsMovingState)
{
  ScAgentContext context;
  ScsLoader loader;
  
  // Загружаем базовую онтологию
  loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "ontology.scs");
  
  // Создаём тестового робота
  ScAddr const robotAddr = context.HelperResolveSystemIdtf("test_robot_1", ScType::NodeConst);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm, 
                     MobileRobotsKeynodes::concept_mobile_robot, 
                     robotAddr);
  
  // Устанавливаем начальное местоположение (пункт А)
  ScAddr const pointA = context.HelperResolveSystemIdtf("point_A", ScType::NodeConst);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::rrel_start_point,
                     pointA);
  
  ScAddr const locationArc = context.CreateEdge(ScType::EdgeDCommonConst, robotAddr, pointA);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::nrel_location,
                     locationArc);
  
  SubscribeAgents(context);
  
  // Создаём событие изменения состояния на "запущен"
  ScAddr const actionAddr = context.CreateNode(ScType::NodeConstClass);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::action_interpreter_mobile_robot,
                     actionAddr);
  
  // Аргумент 1: робот
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     actionAddr,
                     robotAddr);
  
  // Аргумент 2: новое состояние "запущен"
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     actionAddr,
                     MobileRobotsKeynodes::concept_launched);
  
  // Создаём событие для запуска агента
  ScAddr const eventAddr = context.CreateEdge(ScType::EdgeDCommonConst,
                                               MobileRobotsKeynodes::concept_launched,
                                               robotAddr);
  
  // Создаем экземпляр агента и вызываем метод напрямую для тестирования
  MobileRobotInterpretationAgent agent;
  agent.Init(&context);
  
  // Проверяем, что условие инициации выполняется
  ScEventChangeMobileRobotState testEvent(eventAddr);
  EXPECT_TRUE(agent.CheckInitiationCondition(testEvent));
  
  // Выполняем интерпретацию состояния
  ScAction action(actionAddr);
  ScResult result = agent.InterpreterStateLaunched(action, robotAddr);
  
  // Проверяем результат
  EXPECT_EQ(result, ScResult::SC_RESULT_OK);
  
  // Проверяем, что добавлено состояние "перемещается"
  ScIterator3Ptr movingIter = context.CreateIterator3(
    MobileRobotsKeynodes::concept_is_mooving,
    ScType::EdgeAccessConstPosPerm,
    robotAddr);
  EXPECT_TRUE(movingIter->Next());
  
  UnsubscribeAgents(context);
}

// ============================================================================
// ТЕСТ 3: Проверка обнаружения препятствия (ObstacleCheck)
// ============================================================================
TEST_F(TransportModuleTest, ObstacleCheck_DetectsObstacle)
{
  ScAgentContext context;
  ScsLoader loader;
  loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "ontology.scs");
  
  // Создаём точку маршрута
  ScAddr const pointAddr = context.HelperResolveSystemIdtf("test_point_1", ScType::NodeConst);
  
  // Создаём препятствие
  ScAddr const obstacleAddr = context.HelperResolveSystemIdtf("obstacle_1", ScType::NodeConst);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::concept_obstacle,
                     obstacleAddr);
  
  // Связываем препятствие с точкой
  ScAddr const positionArc = context.CreateEdge(ScType::EdgeDCommonConst, obstacleAddr, pointAddr);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::nrel_obstacle_position,
                     positionArc);
  
  MobileRobotInterpretationAgent agent;
  agent.Init(&context);
  
  // Проверяем обнаружение препятствия
  EXPECT_TRUE(agent.ObstacleCheck(pointAddr));
}

TEST_F(TransportModuleTest, ObstacleCheck_NoObstacle)
{
  ScAgentContext context;
  ScsLoader loader;
  loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "ontology.scs");
  
  // Создаём точку без препятствий
  ScAddr const pointAddr = context.HelperResolveSystemIdtf("test_point_2", ScType::NodeConst);
  
  MobileRobotInterpretationAgent agent;
  agent.Init(&context);
  
  // Проверяем отсутствие препятствия
  EXPECT_FALSE(agent.ObstacleCheck(pointAddr));
}

// ============================================================================
// ТЕСТ 4: Проверка точки разгрузки (UnloadingPointCheck)
// ============================================================================
TEST_F(TransportModuleTest, UnloadingPointCheck_IsUnloadingPoint)
{
  ScAgentContext context;
  ScsLoader loader;
  loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "ontology.scs");
  
  // Создаём структуру "пункт Б" (точка разгрузки)
  ScAddr const structureAddr = context.CreateNode(ScType::NodeConstClass);
  ScAddr const pointB = context.HelperResolveSystemIdtf("point_B", ScType::NodeConst);
  
  // Связываем точку со структурой через отношение rrel_end_point
  ScAddr const relArc = context.CreateEdge(ScType::EdgeDCommonConst, structureAddr, pointB);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::rrel_end_point,
                     relArc);
  
  MobileRobotInterpretationAgent agent;
  agent.Init(&context);
  
  EXPECT_TRUE(agent.UnloadingPointCheck(pointB));
}

TEST_F(TransportModuleTest, UnloadingPointCheck_IsNotUnloadingPoint)
{
  ScAgentContext context;
  ScsLoader loader;
  loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "ontology.scs");
  
  // Создаём обычную точку (не пункт Б)
  ScAddr const pointAddr = context.HelperResolveSystemIdtf("intermediate_point", ScType::NodeConst);
  
  MobileRobotInterpretationAgent agent;
  agent.Init(&context);
  
  EXPECT_FALSE(agent.UnloadingPointCheck(pointAddr));
}

// ============================================================================
// ТЕСТ 5: Проверка точки погрузки (LoadingPointCheck)
// ============================================================================
TEST_F(TransportModuleTest, LoadingPointCheck_IsLoadingPoint)
{
  ScAgentContext context;
  ScsLoader loader;
  loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "ontology.scs");
  
  // Создаём структуру "пункт А" (точка погрузки)
  ScAddr const structureAddr = context.CreateNode(ScType::NodeConstClass);
  ScAddr const pointA = context.HelperResolveSystemIdtf("point_A", ScType::NodeConst);
  
  // Связываем точку со структурой через отношение rrel_start_point
  ScAddr const relArc = context.CreateEdge(ScType::EdgeDCommonConst, structureAddr, pointA);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::rrel_start_point,
                     relArc);
  
  MobileRobotInterpretationAgent agent;
  agent.Init(&context);
  
  EXPECT_TRUE(agent.LoadingPointCheck(pointA));
}

// ============================================================================
// ТЕСТ 6: Управление состоянием ожидания (SetWaitingState)
// ============================================================================
TEST_F(TransportModuleTest, SetWaitingState_AddsWaitingState)
{
  ScAgentContext context;
  ScsLoader loader;
  loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "ontology.scs");
  
  ScAddr const robotAddr = context.HelperResolveSystemIdtf("test_robot_2", ScType::NodeConst);
  
  MobileRobotInterpretationAgent agent;
  agent.Init(&context);
  
  // Устанавливаем состояние ожидания
  agent.SetWaitingState(robotAddr, true);
  
  // Проверяем, что состояние добавлено
  ScIterator3Ptr waitingIter = context.CreateIterator3(
    MobileRobotsKeynodes::concept_waiting_obstacle,
    ScType::EdgeAccessConstPosPerm,
    robotAddr);
  EXPECT_TRUE(waitingIter->Next());
  
  // Убираем состояние ожидания
  agent.SetWaitingState(robotAddr, false);
  
  // Проверяем, что состояние удалено
  ScIterator3Ptr waitingIter2 = context.CreateIterator3(
    MobileRobotsKeynodes::concept_waiting_obstacle,
    ScType::EdgeAccessConstNegPerm,
    robotAddr);
  EXPECT_TRUE(waitingIter2->Next());
}

// ============================================================================
// ТЕСТ 7: Полная цепочка: запуск → движение → препятствие → ожидание → продолжение
// ============================================================================
TEST_F(TransportModuleTest, FullScenario_WithObstacle)
{
  ScAgentContext context;
  ScsLoader loader;
  loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "ontology.scs");
  
  // Создаём робота и точки маршрута
  ScAddr const robotAddr = context.HelperResolveSystemIdtf("robot_scenario", ScType::NodeConst);
  ScAddr const pointA = context.HelperResolveSystemIdtf("point_A", ScType::NodeConst);
  ScAddr const pointB = context.HelperResolveSystemIdtf("point_B", ScType::NodeConst);
  ScAddr const intermediate = context.HelperResolveSystemIdtf("point_mid", ScType::NodeConst);
  
  // Настраиваем маршрут: A -> mid -> B
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::rrel_start_point, pointA);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::rrel_end_point, pointB);
  
  ScAddr const nextArc1 = context.CreateEdge(ScType::EdgeDCommonConst, pointA, intermediate);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::nrel_next_point, nextArc1);
  
  ScAddr const nextArc2 = context.CreateEdge(ScType::EdgeDCommonConst, intermediate, pointB);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::nrel_next_point, nextArc2);
  
  // Устанавливаем начальное положение робота в точке A
  ScAddr const locArc = context.CreateEdge(ScType::EdgeDCommonConst, robotAddr, pointA);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::nrel_location, locArc);
  
  MobileRobotInterpretationAgent agent;
  agent.Init(&context);
  
  // Шаг 1: Запуск робота
  ScAction action1(context.CreateNode(ScType::NodeConstClass));
  ScResult res1 = agent.InterpreterStateLaunched(action1, robotAddr);
  EXPECT_EQ(res1, ScResult::SC_RESULT_OK);
  
  // Проверяем состояние "движение"
  ScIterator3Ptr movingIter = context.CreateIterator3(
    MobileRobotsKeynodes::concept_is_mooving,
    ScType::EdgeAccessConstPosPerm,
    robotAddr);
  EXPECT_TRUE(movingIter->Next());
  
  // Шаг 2: Добавляем препятствие на промежуточную точку
  ScAddr const obstacle = context.HelperResolveSystemIdtf("obstacle_test", ScType::NodeConst);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::concept_obstacle, obstacle);
  ScAddr const obsPosArc = context.CreateEdge(ScType::EdgeDCommonConst, obstacle, intermediate);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::nrel_obstacle_position, obsPosArc);
  
  // Проверяем обнаружение препятствия
  EXPECT_TRUE(agent.ObstacleCheck(intermediate));
  
  // Шаг 3: Переход в состояние ожидания
  agent.SetWaitingState(robotAddr, true);
  ScIterator3Ptr waitingIter = context.CreateIterator3(
    MobileRobotsKeynodes::concept_waiting_obstacle,
    ScType::EdgeAccessConstPosPerm,
    robotAddr);
  EXPECT_TRUE(waitingIter->Next());
  
  // Шаг 4: Убираем препятствие (симуляция)
  context.EraseElement(obsPosArc);
  context.EraseElement(obstacle);
  
  // Проверяем, что препятствие исчезло
  EXPECT_FALSE(agent.ObstacleCheck(intermediate));
  
  // Шаг 5: Возврат в состояние движения
  agent.SetWaitingState(robotAddr, false);
  
  UnsubscribeAgents(context);
}

// ============================================================================
// ТЕСТ 8: Переход в состояние "готов к разгрузке" (InterpreterStateBoxLoaded)
// ============================================================================
TEST_F(TransportModuleTest, InterpreterStateBoxLoaded_ReachesUnloadingPoint)
{
  ScAgentContext context;
  ScsLoader loader;
  loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "ontology.scs");
  
  // Создаём робота и точку разгрузки
  ScAddr const robotAddr = context.HelperResolveSystemIdtf("robot_unload", ScType::NodeConst);
  ScAddr const pointB = context.HelperResolveSystemIdtf("point_B", ScType::NodeConst);
  
  // Настраиваем точку Б как конечную
  ScAddr const structure = context.CreateNode(ScType::NodeConstClass);
  ScAddr const relArc = context.CreateEdge(ScType::EdgeDCommonConst, structure, pointB);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::rrel_end_point, relArc);
  
  // Устанавливаем положение робота
  ScAddr const locArc = context.CreateEdge(ScType::EdgeDCommonConst, robotAddr, pointB);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::nrel_location, locArc);
  
  MobileRobotInterpretationAgent agent;
  agent.Init(&context);
  
  // Вызываем интерпретацию состояния "коробка загружена"
  ScAction action(context.CreateNode(ScType::NodeConstClass));
  ScResult result = agent.InterpreterStateBoxLoaded(action, robotAddr);
  
  EXPECT_EQ(result, ScResult::SC_RESULT_OK);
  
  // Проверяем, что добавлено состояние "готов к разгрузке"
  ScIterator3Ptr readyIter = context.CreateIterator3(
    MobileRobotsKeynodes::concept_ready_being_unloaded,
    ScType::EdgeAccessConstPosPerm,
    robotAddr);
  EXPECT_TRUE(readyIter->Next());
}

// ============================================================================
// ТЕСТ 9: Переход в состояние "готов к загрузке" (InterpreterStateBoxUnLoaded)
// ============================================================================
TEST_F(TransportModuleTest, InterpreterStateBoxUnLoaded_ReachesLoadingPoint)
{
  ScAgentContext context;
  ScsLoader loader;
  loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "ontology.scs");
  
  // Создаём робота и точку погрузки
  ScAddr const robotAddr = context.HelperResolveSystemIdtf("robot_load", ScType::NodeConst);
  ScAddr const pointA = context.HelperResolveSystemIdtf("point_A", ScType::NodeConst);
  
  // Настраиваем точку А как начальную
  ScAddr const structure = context.CreateNode(ScType::NodeConstClass);
  ScAddr const relArc = context.CreateEdge(ScType::EdgeDCommonConst, structure, pointA);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::rrel_start_point, relArc);
  
  // Устанавливаем положение робота
  ScAddr const locArc = context.CreateEdge(ScType::EdgeDCommonConst, robotAddr, pointA);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::nrel_location, locArc);
  
  MobileRobotInterpretationAgent agent;
  agent.Init(&context);
  
  // Вызываем интерпретацию состояния "коробка разгружена"
  ScAction action(context.CreateNode(ScType::NodeConstClass));
  ScResult result = agent.InterpreterStateBoxUnLoaded(action, robotAddr);
  
  EXPECT_EQ(result, ScResult::SC_RESULT_OK);
  
  // Проверяем, что добавлено состояние "готов к загрузке"
  // (используем правильный keynode из вашего кода)
  ScIterator3Ptr readyIter = context.CreateIterator3(
    MobileRobotsKeynodes::concept_ready_being_loaded,
    ScType::EdgeAccessConstPosPerm,
    robotAddr);
  EXPECT_TRUE(readyIter->Next());
}

// ============================================================================
// ТЕСТ 10: Получение следующей точки маршрута (GetNextPoint)
// ============================================================================
TEST_F(TransportModuleTest, GetNextPoint_ReturnsCorrectPoint)
{
  ScAgentContext context;
  ScsLoader loader;
  loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "ontology.scs");
  
  // Создаём точки маршрута
  ScAddr const point1 = context.HelperResolveSystemIdtf("route_point_1", ScType::NodeConst);
  ScAddr const point2 = context.HelperResolveSystemIdtf("route_point_2", ScType::NodeConst);
  
  // Создаём связь "следующая точка"
  ScAddr const nextArc = context.CreateEdge(ScType::EdgeDCommonConst, point1, point2);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::nrel_next_point, nextArc);
  
  // Создаём робота в точке 1
  ScAddr const robotAddr = context.HelperResolveSystemIdtf("robot_route", ScType::NodeConst);
  ScAddr const locArc = context.CreateEdge(ScType::EdgeDCommonConst, robotAddr, point1);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::nrel_location, locArc);
  
  MobileRobotInterpretationAgent agent;
  agent.Init(&context);
  
  // Получаем следующую точку
  ScAddr nextPoint = agent.GetNextPoint(robotAddr);
  
  EXPECT_EQ(nextPoint, point2);
}

// ============================================================================
// ТЕСТ 11: Обновление местоположения робота (MoveToNextPoint)
// ============================================================================
TEST_F(TransportModuleTest, MoveToNextPoint_UpdatesLocation)
{
  ScAgentContext context;
  ScsLoader loader;
  loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "ontology.scs");
  
  // Создаём робота и две точки
  ScAddr const robotAddr = context.HelperResolveSystemIdtf("robot_move", ScType::NodeConst);
  ScAddr const fromPoint = context.HelperResolveSystemIdtf("from_point", ScType::NodeConst);
  ScAddr const toPoint = context.HelperResolveSystemIdtf("to_point", ScType::NodeConst);
  
  // Устанавливаем начальное положение
  ScAddr const oldLocArc = context.CreateEdge(ScType::EdgeDCommonConst, robotAddr, fromPoint);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     MobileRobotsKeynodes::nrel_location, oldLocArc);
  
  MobileRobotInterpretationAgent agent;
  agent.Init(&context);
  
  // Перемещаем робота
  agent.MoveToNextPoint(robotAddr, toPoint);
  
  // Проверяем, что старая связь удалена
  ScIterator5Ptr oldIter = context.CreateIterator5(
    robotAddr,
    ScType::EdgeDCommonConst,
    fromPoint,
    ScType::EdgeAccessConstPosPerm,
    MobileRobotsKeynodes::nrel_location);
  EXPECT_FALSE(oldIter->Next());
  
  // Проверяем, что новая связь создана
  ScIterator5Ptr newIter = context.CreateIterator5(
    robotAddr,
    ScType::EdgeDCommonConst,
    toPoint,
    ScType::EdgeAccessConstPosPerm,
    MobileRobotsKeynodes::nrel_location);
  EXPECT_TRUE(newIter->Next());
}

// ============================================================================
// ТЕСТ 12: Остановка робота (StopMooving)
// ============================================================================
TEST_F(TransportModuleTest, StopMooving_RemovesMovingState)
{
  ScAgentContext context;
  ScsLoader loader;
  loader.loadScsFile(context, EXAMPLE_MODULE_TEST_FILES_DIR_PATH + "ontology.scs");
  
  ScAddr const robotAddr = context.HelperResolveSystemIdtf("robot_stop", ScType::NodeConst);
  
  // Сначала добавляем состояние "движение"
  ScAddr const movingArc = context.CreateEdge(ScType::EdgeDCommonConst, 
                                               MobileRobotsKeynodes::concept_is_mooving,
                                               robotAddr);
  context.CreateEdge(ScType::EdgeAccessConstPosPerm,
                     ScType::EdgeAccessConstPosPerm,
                     movingArc);
  
  MobileRobotInterpretationAgent agent;
  agent.Init(&context);
  
  // Останавливаем робота
  agent.StopMooving(robotAddr);
  
  // Проверяем, что состояние "движение" удалено (или помечено как негативное)
  ScIterator3Ptr movingIter = context.CreateIterator3(
    MobileRobotsKeynodes::concept_is_mooving,
    ScType::EdgeAccessConstNegPerm,
    robotAddr);
  EXPECT_TRUE(movingIter->Next());
}*/
