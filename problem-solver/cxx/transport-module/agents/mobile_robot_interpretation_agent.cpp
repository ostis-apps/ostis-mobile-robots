#include "mobile_robot_interpretation_agent.hpp"

ScAddr MobileRobotInterpretationAgent::GetActionClass() const
{
  return MobileRobotsKeynodes::action_interpreter_mobile_robot;
}

bool MobileRobotInterpretationAgent::CheckInitiationCondition(ScEventChangeMobileRobotState const & event)
{
  ScAddr const & stateAddr = event.GetArcSourceElement();
  ScAddrToValueUnorderedMap<InterpreterCallback> states = {
    {MobileRobotsKeynodes::concept_launched, [this](ScAction & action, ScAddr const & robotAddr) -> ScResult { return InterpreterStateLaunched(action, robotAddr); }},
    {MobileRobotsKeynodes::concept_box_loaded, [this](ScAction & action, ScAddr const & robotAddr) -> ScResult { return InterpreterStateBoxLoaded(action, robotAddr); }},
    {MobileRobotsKeynodes::concept_box_unloaded, [this](ScAction & action, ScAddr const & robotAddr) -> ScResult { return InterpreterStateBoxUnloaded(action, robotAddr); }},
    {MobileRobotsKeynodes::concept_stopped, [this](ScAction & action, ScAddr const & robotAddr) -> ScResult { return InterpreterStateStopped(action, robotAddr); }},
  };
  auto const & it = states.find(stateAddr);
  if (it == states.cend())
    return false;

  m_interpreterCallback = it->second;
  return true;
}

ScResult MobileRobotInterpretationAgent::InterpreterStateLaunched(ScAction & action, ScAddr const & robotAddr)
{
  // добавление состояния "готов к загрузке" (StateReadyBeingLoaded)
  // поиск структуры состояние -дуга-> робот
  ScIterator3Ptr it3 = m_context.CreateIterator3(
    MobileRobotsKeynodes::concept_ready_being_loaded,
    ScType::ActualTempNegArc,
    robotAddr);
  if (it3->Next())
    // удаление дуги
    m_context.EraseElement(it3->Get(1));

  // создание новой дуги
  m_context.GenerateConnector(
    ScType::ActualTempPosArc,
    MobileRobotsKeynodes::concept_ready_being_loaded,
    robotAddr);
  
  return action.FinishSuccessfully();
}

ScResult MobileRobotInterpretationAgent::InterpreterStateBoxLoaded(ScAction & action, ScAddr const & robotAddr)
{
  StartMooving(robotAddr);

  //перемещение в следующую точку маршрута
  ScAddr next_point = GetNextPoint(robotAddr);
  while(!UnloadingPointCheck(next_point)){
    if(ObstacleCheck(next_point)){
    StopMooving(robotAddr);

    //  добавление состояния "ожидание"

    while(ObstacleCheck(next_point)){
      continue;
    }

    //  удаление состояния "ожидание"

    StartMooving(robotAddr);
    }
    MoveToNextPoint(robotAddr, next_point);
    next_point = GetNextPoint(robotAddr);
  }
  // следующая точка - точка разгрузки
  MoveToNextPoint(robotAddr, next_point);
  StopMooving(robotAddr)
 
  //  добавление состояния "готов к разгрузке"
  ScIterator3Ptr it3 = m_context.CreateIterator3(
    MobileRobotsKeynodes::concept_ready_being_unloaded,
    ScType::ActualTempNegArc,
    robotAddr);
  if (it3->Next()){
    // удаление дуги
    m_context.EraseElement(it3->Get(1));
  }
  // создание новой дуги
  m_context.GenerateConnector(
    ScType::ActualTempPosArc,
    MobileRobotsKeynodes::concept_ready_being_unloaded,
    robotAddr);

  return action.FinishSuccessfully();
}

bool MobileRobotInterpretationAgent::UnloadingPointCheck(ScAddr const & next_point){
  ScIterator5Ptr it5 = m_context.CreateIterator5(
    ScType::Node,//?? здесь должен быть путь, @route_example_1
    ScType::ConstCommonArc,
    ScType::next_point,
    MobileRobotsKeynodes::rrel_end_point,
    ScType::ConstCommonArc);
  if (it5->Next()){
    return true
  }
  else{return false}
}

void MobileRobotInterpretationAgent::MoveToNextPoint(ScAddr const & robotAddr, ScAddr const & next_point){
  ScAddr current_point;
  ScIterator5Ptr it5 = m_context.CreateIterator5(
    robotAddr,
    ScType::ConstCommonArc,
    ScType::Node,
    MobileRobotsKeynodes::nrel_location,
    ScType::ActualTempPosArc);
  if (it5->Next()){
    m_context.EraseElement(it5->Get(1))
    m_context.EraseElement(it5->Get(4))
  }
  ScAddr arc = m_context.GenerateConnector(
    ScType::ConstCommonArc,
    robotAddr,
    next_point);
  m_context.GenerateConnector(
    ScType::ActualTempPosArc,
    MobileRobotsKeynodes::nrel_location,
    arc);
}

void MobileRobotInterpretationAgent::StartMooving(ScAddr const & robotAddr){
  //добавление состояние "перемещается"
  ScIterator3Ptr it3 = m_context.CreateIterator3(
    MobileRobotsKeynodes::concept_is_mooving,
    ScType::ActualTempNegArc,
    robotAddr);
  if (it3->Next()){
    // удаление дуги
    m_context.EraseElement(it3->Get(1));
  }
  // создание новой дуги
  m_context.GenerateConnector(
    ScType::ActualTempPosArc,
    MobileRobotsKeynodes::concept_is_mooving,
    robotAddr);

  //добавление скорости
}

void MobileRobotInterpretationAgent::StopMooving(ScAddr const & robotAddr){
  // удаление состояние "перемещается"
  ScIterator3Ptr it3 = m_context.CreateIterator3(
    MobileRobotsKeynodes::concept_is_mooving,
    ScType::ActualTempPosArc,
    robotAddr);
  if (it3->Next()){
    // удаление дуги
    m_context.EraseElement(it3->Get(1));
  }
  // создание новой дуги
  m_context.GenerateConnector(
    ScType::ActualTempNegArc,
    MobileRobotsKeynodes::concept_is_mooving,
    robotAddr);

  //  удаление скорости
}

bool MobileRobotInterpretationAgent::ObstacleCheck(ScAddr const & next_point){
  ScIterator3Ptr it3 = m_context.CreateIterator3(
    ScType::Node,
    ScType::ConstCommonArc,
    next_point);
  if (it3->Next()){
    obstacle = it->Get(0)
    ScIterator3Ptr it3_1 = m_context.CreateIterator3(
    MobileRobotsKeynodes::concept_obstacle,
    ScType::ConstCommonArc,
    obstacle);
    if (it3_1->Next()){
      return true
    }
  }else{
    return false
  }
}

ScAddr MobileRobotInterpretationAgent::GetNextPoint(ScAddr const & robotAddr){
  //нахождение текущего положения
  ScAddr current_point;
  ScIterator5Ptr it5 = m_context.CreateIterator5(
    robotAddr,
    ScType::ConstCommonArc,
    ScType::Node,
    MobileRobotsKeynodes::nrel_location,
    ScType::ActualTempPosArc);
  if (it5->Next()){
    current_point = it5->Get(2)
  }
  //нахождения следующего положения
  ScAddr next_point;
  it5 = m_context.CreateIterator5(
    current_point,
    ScType::ConstCommonArc,
    ScType::Node,
    MobileRobotsKeynodes::nrel_next_point,
    ScType::ConstCommonArc);//дуга принадлежности?
  if (it5->Next()){
    next_point = it5->Get(2)
  }
  return next_point;
}

ScResult MobileRobotInterpretationAgent::InterpreterStateBoxUnloaded(ScAction & action, ScAddr const & robotAddr)
{
  //аналогично предыдущему методу, движение в точку погрузки
  return action.FinishSuccessfully();
}

ScResult MobileRobotInterpretationAgent::InterpreterStateStopped(ScAction & action, ScAddr const & robotAddr)
{
  //??? возможно перемещение в пункт хранения роботов, если не использоуется во время ожидания удаления припятствия
  m_logger.Info("stopped");
  return action.FinishSuccessfully();
}

ScResult MobileRobotInterpretationAgent::DoProgram(ScEventChangeMobileRobotState const & event, ScAction & action)
{
  // нужно ли чтото дополнять в этом методе? 
  ScAddr const & robotAddr = event.GetArcTargetElement();
  m_logger.Info("Test call");
  return m_interpreterCallback(action, robotAddr);
  //прорбный комик
}
// влияет ли на чтото что я создатель ветки? могут ли быть изза этого у сани какието проблемы?