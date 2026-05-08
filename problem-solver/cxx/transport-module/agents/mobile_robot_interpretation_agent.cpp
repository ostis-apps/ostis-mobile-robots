#include "mobile_robot_interpretation_agent.hpp"
#include <time.h>
#include <thread>   // для std::this_thread::sleep_for
#include <chrono>   // для std::chrono::milliseconds

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
  // логика движения со склада

  // добавление состояния "готов к загрузке" (StateReadyBeingLoaded)
  // поиск структуры состояние -дуга-> робот
  ScIterator3Ptr it3 = m_context.CreateIterator3(
    MobileRobotsKeynodes::concept_ready_being_loaded,// добавить
    ScType::ConstActualTempNegArc,
    robotAddr);
  if (it3->Next())
    // удаление дуги
    m_context.EraseElement(it3->Get(1));

  // создание новой дуги
  m_context.GenerateConnector(
    ScType::ConstActualTempPosArc,
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
    SetWaitingState(robotAddr, true);

    while(ObstacleCheck(next_point)){
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    //  удаление состояния "ожидание"
    SetWaitingState(robotAddr, false);

    StartMooving(robotAddr);
    }
    MoveToNextPoint(robotAddr, next_point);
    next_point = GetNextPoint(robotAddr);
  }
  // следующая точка - точка разгрузки
  MoveToNextPoint(robotAddr, next_point);
  StopMooving(robotAddr);
 
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

ScResult MobileRobotInterpretationAgent::InterpreterStateBoxUnloaded(ScAction & action, ScAddr const & robotAddr)
{
  StartMooving(robotAddr);

  //перемещение в следующую точку маршрута
  ScAddr next_point = GetNextPoint(robotAddr);
  while(!UploadingPointCheck(next_point)){
    if(ObstacleCheck(next_point)){
    StopMooving(robotAddr);

    //  добавление состояния "ожидание"
    SetWaitingState(robotAddr, true);

    while(ObstacleCheck(next_point)){
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }

    //  удаление состояния "ожидание"
    SetWaitingState(robotAddr, false);

    StartMooving(robotAddr);
    }
    MoveToNextPoint(robotAddr, next_point);
    next_point = GetNextPoint(robotAddr);
  }
  // следующая точка - точка разгрузки
  MoveToNextPoint(robotAddr, next_point);
  StopMooving(robotAddr);
 
  //  добавление состояния "готов к разгрузке"
  ScIterator3Ptr it3 = m_context.CreateIterator3(
    MobileRobotsKeynodes::concept_ready_being_loaded,
    ScType::ActualTempNegArc,
    robotAddr);
  if (it3->Next()){
    // удаление дуги
    m_context.EraseElement(it3->Get(1));
  }
  // создание новой дуги
  m_context.GenerateConnector(
    ScType::ActualTempPosArc,
    MobileRobotsKeynodes::concept_ready_being_loaded,
    robotAddr);

  return action.FinishSuccessfully();
}

bool MobileRobotInterpretationAgent::SetWaitingState(ScAddr const & robotAddr, bool state)
{
  if (state){
    ScIterator3Ptr it3 = m_context.CreateIterator3(
    MobileRobotsKeynodes::concept_waiting_obstacle,
    ScType::ActualTempNegArc,
    robotAddr);
  if (it3->Next()){
    // удаление дуги
    m_context.EraseElement(it3->Get(1));
  }
  // создание новой дуги
  m_context.GenerateConnector(
    ScType::ActualTempPosArc,
    MobileRobotsKeynodes::concept_waiting_obstacle,
    robotAddr);
  }else{
    ScIterator3Ptr it3 = m_context.CreateIterator3(
    MobileRobotsKeynodes::concept_waiting_obstacle,
    ScType::ActualTempPosArc,
    robotAddr);
  if (it3->Next()){
    // удаление дуги
    m_context.EraseElement(it3->Get(1));
  }
  // создание новой дуги
  m_context.GenerateConnector(
    ScType::ActualTempNegArc,
    MobileRobotsKeynodes::concept_waiting_obstacle,
    robotAddr);
  }
}

bool MobileRobotInterpretationAgent::UnloadingPointCheck(ScAddr const & next_point)
{
  ScIterator5Ptr it5 = m_context.CreateIterator5(
    ScType::ConstNodeStructure,
    ScType::ConstCommonArc,
    next_point,
    ScType::ConstPermPosArc,
    MobileRobotsKeynodes::rrel_end_point);
  if (it5->Next()){
    return true;
  }
  else{return false;}
}

bool MobileRobotInterpretationAgent::UploadingPointCheck(ScAddr const & next_point)
{
  ScIterator5Ptr it5 = m_context.CreateIterator5(
    ScType::ConstNodeStructure,
    ScType::ConstCommonArc,
    next_point,
    ScType::ConstPermPosArc,
    MobileRobotsKeynodes::rrel_start_point);
  if (it5->Next()){
    return true;
  }
  else{return false;}
}

void MobileRobotInterpretationAgent::MoveToNextPoint(ScAddr const & robotAddr, ScAddr const & next_point)
{
  ScAddr current_point;
  ScIterator5Ptr it5 = m_context.CreateIterator5(
    robotAddr,
    ScType::ConstCommonArc,
    ScType::Node,
    ScType::ActualTempPosArc,
    MobileRobotsKeynodes::nrel_location);
  if (it5->Next()){
    m_context.EraseElement(it5->Get(1));
    m_context.EraseElement(it5->Get(4));
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

void MobileRobotInterpretationAgent::StartMooving(ScAddr const & robotAddr)
{
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
  SetSpeed(robotAddr, 0.5);
}

void MobileRobotInterpretationAgent::StopMooving(ScAddr const & robotAddr)
{
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
  SetSpeed(robotAddr, 0);
}

bool MobileRobotInterpretationAgent::ObstacleCheck(ScAddr const & next_point)
{
  ScIterator5Ptr it5 = m_context.CreateIterator5(
    ScType::Node,
    ScType::ConstCommonArc,
    next_point,
    ScType::ConstPermPosArc,
    MobileRobotsKeynodes::nrel_obstacle_position);
  if (it5->Next()){
    ScAddr obstacleAddr = it5->Get(0);
    ScIterator3Ptr it3 = m_context.CreateIterator3(
    MobileRobotsKeynodes::concept_obstacle,
    ScType::ConstPermPosArc,
    obstacleAddr);
    if (it3->Next()){
      return true;
    }
  }
  return false;
}

ScAddr MobileRobotInterpretationAgent::GetNextPoint(ScAddr const & robotAddr)
{
  //нахождение текущего положения
  ScAddr current_point;
  ScIterator5Ptr it5 = m_context.CreateIterator5(
    robotAddr,
    ScType::ConstCommonArc,
    ScType::Node,
    ScType::ConstActualTempPosArc,
    MobileRobotsKeynodes::nrel_location);
  if (it5->Next()){
    current_point = it5->Get(2);
  }
  //нахождения следующего положения
  ScAddr next_point;
  it5 = m_context.CreateIterator5(
    current_point,
    ScType::ConstCommonArc,
    ScType::Node,
    ScType::ConstPermPosArc,
    MobileRobotsKeynodes::nrel_next_point);
  if (it5->Next()){
    next_point = it5->Get(2);
  }
  return next_point;
}

void MobileRobotInterpretationAgent::SetSpeed(ScAddr const & robotAddr, int speedValue)
{
  ScIterator3Ptr it3 = m_context.CreateIterator3(
    ScType::VarNodeLink,
    ScType::ActualTempPosArc,
    robotAddr);
    if (it3->Next()){
      m_context.EraseElement(it3->Get(1));
      m_context.EraseElement(it3->Get(0));// можно ли просто удалить один узел, удалится ли связь автоматически?
    }

  ScAddr speed = m_context.GenerateLink(ScType::VarNodeLink);
  m_context.SetLinkContent(speed, speedValue);
  m_context.GenerateConnector(
    ScType::ActualTempPosArc,
    speed,
    robotAddr);
}


ScResult MobileRobotInterpretationAgent::InterpreterStateStopped(ScAction & action, ScAddr const & robotAddr)
{
  // логика движения на склад
  m_logger.Info("stopped");
  return action.FinishSuccessfully();
}

ScResult MobileRobotInterpretationAgent::DoProgram(ScEventChangeMobileRobotState const & event, ScAction & action)
{
  ScAddr const & robotAddr = event.GetArcTargetElement();
  m_logger.Info("Test call");
  return m_interpreterCallback(action, robotAddr);
}
