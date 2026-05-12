#include "mobile_robot_interpretation_agent.hpp"
#include <time.h>
#include <thread>  // для std::this_thread::sleep_for
#include <chrono>  // для std::chrono::milliseconds

ScAddr MobileRobotInterpretationAgent::GetActionClass() const
{
  return MobileRobotsKeynodes::action_interpreter_mobile_robot;
}

bool MobileRobotInterpretationAgent::CheckInitiationCondition(ScEventChangeMobileRobotState const & event)
{
  ScAddr const & stateAddr = event.GetArcSourceElement();
  ScAddrToValueUnorderedMap<InterpreterCallback> states = {
      {MobileRobotsKeynodes::concept_launched,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateLaunched(action, robotAddr);
       }},
      {MobileRobotsKeynodes::concept_box_loaded,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateBoxLoaded(action, robotAddr);
       }},
      {MobileRobotsKeynodes::concept_box_unloaded,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateBoxUnloaded(action, robotAddr);
       }},
      {MobileRobotsKeynodes::concept_stopped,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateStopped(action, robotAddr);
       }},
  };
  auto const & it = states.find(stateAddr);
  if (it == states.cend())
    return false;

  m_interpreterCallback = it->second;
  return true;
}

ScResult MobileRobotInterpretationAgent::InterpreterStateLaunched(ScAction & action, ScAddr const & robotAddr)
{
  if (IsStopped(robotAddr))
    return action.FinishSuccessfully();
  SC_LOG_INFO("InterpreterStateLaunched");
  ScIterator5Ptr it5 = m_context.CreateIterator5(
      ScType::ConstNode,
      ScType::ConstCommonArc,
      robotAddr,
      ScType::ConstActualTempPosArc,
      MobileRobotsKeynodes::nrel_location);
  while (it5->Next())
  {
    ScAddr const & boxAddr = it5->Get(0);
    ScIterator3Ptr it3 = m_context.CreateIterator3(MobileRobotsKeynodes::concept_box, ScType::ConstPermPosArc, boxAddr);
    if (it3->Next())
    {
      SC_LOG_INFO("INITIALLY THERE IS ONE BOX ON ROBOT");
      ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_box_loaded, robotAddr);
      return action.FinishSuccessfully();
    }
  }
  SC_LOG_INFO("INITIALLY THERE IS NO BOX ON ROBOT");
  ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_box_unloaded, robotAddr);
  return action.FinishSuccessfully();
}

ScResult MobileRobotInterpretationAgent::InterpreterStateBoxLoaded(ScAction & action, ScAddr const & robotAddr)
{
  if (IsStopped(robotAddr))
    return action.FinishSuccessfully();

  SC_LOG_INFO("InterpreterStateBoxLoaded");

  StartMoving(robotAddr);

  // перемещение в следующую точку маршрута
  ScAddr next_point = GetNextPoint(robotAddr);
  while (!UnloadingPointCheck(next_point))
  {
    if (ObstacleCheck(next_point))
    {
      StopMoving(robotAddr);

      //  добавление состояния "ожидание"
      SetWaitingState(robotAddr, true);

      while (ObstacleCheck(next_point))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      //  удаление состояния "ожидание"
      SetWaitingState(robotAddr, false);

      StartMoving(robotAddr);
    }
    MoveToNextPoint(robotAddr, next_point);
    next_point = GetNextPoint(robotAddr);
  }
  // следующая точка - точка разгрузки
  MoveToNextPoint(robotAddr, next_point);
  StopMoving(robotAddr);

  //  добавление состояния "готов к разгрузке"
  ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_ready_being_unloaded, robotAddr);

  return action.FinishSuccessfully();
}

ScResult MobileRobotInterpretationAgent::InterpreterStateBoxUnloaded(ScAction & action, ScAddr const & robotAddr)
{
  if (IsStopped(robotAddr))
    return action.FinishSuccessfully();

  SC_LOG_INFO("InterpreterStateBoxUnloaded");

  StartMoving(robotAddr);

  // перемещение в следующую точку маршрута
  ScAddr next_point = GetNextPoint(robotAddr);
  while (!UploadingPointCheck(next_point))
  {
    if (ObstacleCheck(next_point))
    {
      StopMoving(robotAddr);

      //  добавление состояния "ожидание"
      SetWaitingState(robotAddr, true);

      while (ObstacleCheck(next_point))
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      //  удаление состояния "ожидание"
      SetWaitingState(robotAddr, false);

      StartMoving(robotAddr);
    }
    MoveToNextPoint(robotAddr, next_point);
    next_point = GetNextPoint(robotAddr);
  }
  // следующая точка - точка разгрузки
  MoveToNextPoint(robotAddr, next_point);
  StopMoving(robotAddr);

  //  добавление состояния "готов к разгрузке"
  ScIterator3Ptr it3 = m_context.CreateIterator3(
      MobileRobotsKeynodes::concept_ready_being_loaded, ScType::ConstActualTempNegArc, robotAddr);
  if (it3->Next())
  {
    // удаление дуги
    m_context.EraseElement(it3->Get(1));
  }
  // создание новой дуги
  m_context.GenerateConnector(
      ScType::ConstActualTempPosArc, MobileRobotsKeynodes::concept_ready_being_loaded, robotAddr);

  return action.FinishSuccessfully();
}

void MobileRobotInterpretationAgent::SetWaitingState(ScAddr const & robotAddr, bool state)
{
  if (state)
  {
    ScIterator3Ptr it3 =
        m_context.CreateIterator3(MobileRobotsKeynodes::concept_waiting_obstacle, ScType::ActualTempNegArc, robotAddr);
    if (it3->Next())
    {
      // удаление дуги
      m_context.EraseElement(it3->Get(1));
    }
    // создание новой дуги
    m_context.GenerateConnector(ScType::ActualTempPosArc, MobileRobotsKeynodes::concept_waiting_obstacle, robotAddr);
  }
  else
  {
    ScIterator3Ptr it3 =
        m_context.CreateIterator3(MobileRobotsKeynodes::concept_waiting_obstacle, ScType::ActualTempPosArc, robotAddr);
    if (it3->Next())
    {
      // удаление дуги
      m_context.EraseElement(it3->Get(1));
    }
    // создание новой дуги
    m_context.GenerateConnector(ScType::ActualTempNegArc, MobileRobotsKeynodes::concept_waiting_obstacle, robotAddr);
  }
}

bool MobileRobotInterpretationAgent::UnloadingPointCheck(ScAddr const & routePoint)
{
  ScIterator5Ptr it5 = m_context.CreateIterator5(
      ScType::ConstNodeStructure,
      ScType::ConstPermPosArc,
      routePoint,
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::rrel_end_point);
  if (it5->Next())
  {
    SC_LOG_INFO("EndPoint");
    return true;
  }
  else
  {
    SC_LOG_INFO("not EndPoint");
    return false;
  }
}

bool MobileRobotInterpretationAgent::UploadingPointCheck(ScAddr const & routePoint)
{
  ScIterator5Ptr it5 = m_context.CreateIterator5(
      ScType::ConstNodeStructure,
      ScType::ConstPermPosArc,
      routePoint,
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::rrel_start_point);
  if (it5->Next())
  {
    SC_LOG_INFO("StartPoint");
    return true;
  }
  else
  {
    SC_LOG_INFO("not StartPoint");
    return false;
  }
}

void MobileRobotInterpretationAgent::MoveToNextPoint(ScAddr const & robotAddr, ScAddr const & next_point)
{
  SC_LOG_INFO("Move to next point " + m_context.GetElementSystemIdentifier(robotAddr));
  ScAddr current_point;
  ScIterator5Ptr it5 = m_context.CreateIterator5(
      robotAddr,
      ScType::ConstCommonArc,
      ScType::ConstNode,
      ScType::ConstActualTempPosArc,
      MobileRobotsKeynodes::nrel_location);
  if (it5->Next())
  {
    m_context.EraseElement(it5->Get(1));
  }
  ScAddr const & arc = m_context.GenerateConnector(ScType::ConstCommonArc, robotAddr, next_point);
  m_context.GenerateConnector(ScType::ConstActualTempPosArc, MobileRobotsKeynodes::nrel_location, arc);

  // задержка для иммитации скорости
  // ScIterator3Ptr it3 = m_context.CreateIterator3(
  //   ScType::ConstNodeLink,
  //   ScType::ConstActualTempPosArc,
  //   robotAddr);
  // if (it3->Next()){
  //   ScAddr speedNode = it3->Get(0);
  //   double speed = speedNode.GetLinkContent();
  // }

  double speed = 20;
  double distance = 20;
  double time = distance / speed;
  std::this_thread::sleep_for(std::chrono::duration<double>(time));
}

void MobileRobotInterpretationAgent::StartMoving(ScAddr const & robotAddr)
{
  ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_is_moving, robotAddr);
  
  // добавление скорости
  SetSpeed(robotAddr, 1);
}

void MobileRobotInterpretationAgent::StopMoving(ScAddr const & robotAddr)
{
  ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_is_moving, robotAddr);

  //  удаление скорости
  // SetSpeed(robotAddr, 0);
}

bool MobileRobotInterpretationAgent::ObstacleCheck(ScAddr const & routePoint)
{
  ScIterator5Ptr it5 = m_context.CreateIterator5(
      ScType::ConstNode,
      ScType::ConstCommonArc,
      routePoint,
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::nrel_obstacle_position);
  if (it5->Next())
  {
    ScAddr obstacleAddr = it5->Get(0);
    ScIterator3Ptr it3 =
        m_context.CreateIterator3(MobileRobotsKeynodes::concept_obstacle, ScType::ConstPermPosArc, obstacleAddr);
    if (it3->Next())
    {
      return true;
    }
  }
  return false;
}

ScAddr MobileRobotInterpretationAgent::GetNextPoint(ScAddr const & robotAddr)
{
  // нахождение текущего положения
  ScAddr current_point;
  ScIterator5Ptr it5 = m_context.CreateIterator5(
      robotAddr,
      ScType::ConstCommonArc,
      ScType::ConstNode,
      ScType::ConstActualTempPosArc,
      MobileRobotsKeynodes::nrel_location);
  if (it5->Next())
  {
    current_point = it5->Get(2);
  }
  // нахождения следующего положения
  ScAddr next_point;
  it5 = m_context.CreateIterator5(
      current_point,
      ScType::ConstCommonArc,
      ScType::ConstNode,
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::nrel_next_point);
  if (it5->Next())
  {
    next_point = it5->Get(2);
  }
  return next_point;
}

void MobileRobotInterpretationAgent::SetSpeed(ScAddr const & robotAddr, double speedValue)
{
  ScIterator3Ptr it3 = m_context.CreateIterator3(ScType::ConstNodeLink, ScType::ConstActualTempPosArc, robotAddr);
  if (it3->Next())
  {
    m_context.EraseElement(it3->Get(0));
  }

  ScAddr speed = m_context.GenerateLink(ScType::ConstNodeLink);
  m_context.SetLinkContent(speed, speedValue);
  m_context.GenerateConnector(ScType::ConstActualTempPosArc, speed, robotAddr);
}

ScResult MobileRobotInterpretationAgent::InterpreterStateStopped(ScAction & action, ScAddr const & robotAddr)
{
  SC_LOG_INFO("InterpreterStateStopped " + m_context.GetElementSystemIdentifier(robotAddr));
  return action.FinishSuccessfully();
}

void MobileRobotInterpretationAgent::ChangeActualTempArcToPos(ScAddr const & addr1, ScAddr const & addr2)
{
  ScIterator3Ptr it3 = m_context.CreateIterator3(addr1, ScType::ConstActualTempNegArc, addr2);
  if (it3->Next())
  {
    m_context.EraseElement(it3->Get(1));
  }
  m_context.GenerateConnector(ScType::ConstActualTempPosArc, addr1, addr2);
}

void MobileRobotInterpretationAgent::ChangeActualTempArcToNeg(ScAddr const & addr1, ScAddr const & addr2)
{
  ScIterator3Ptr it3 = m_context.CreateIterator3(addr1, ScType::ConstActualTempPosArc, addr2);
  if (it3->Next())
  {
    m_context.EraseElement(it3->Get(1));
  }
  m_context.GenerateConnector(ScType::ConstActualTempNegArc, addr1, addr2);
}

ScResult MobileRobotInterpretationAgent::DoProgram(ScEventChangeMobileRobotState const & event, ScAction & action)
{
  ScAddr const & robotAddr = event.GetArcTargetElement();
  return m_interpreterCallback(action, robotAddr);
}

bool MobileRobotInterpretationAgent::IsStopped(ScAddr const & robotAddr)
{
  return m_context.CheckConnector(MobileRobotsKeynodes::concept_stopped, robotAddr, ScType::ConstActualTempPosArc);
}