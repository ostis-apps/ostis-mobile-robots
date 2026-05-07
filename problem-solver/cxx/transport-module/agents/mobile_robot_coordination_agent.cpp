#include "mobile_robot_coordination_agent.hpp"
#include <sc-memory/sc_link.hpp>
#include <chrono>
#include <thread>

ScAddr MobileRobotCoordinationAgent::GetActionClass() const
{
  return MobileRobotsKeynodes::action_coordinate_mobile_robot;
}

bool MobileRobotCoordinationAgent::CheckInitiationCondition(ScEventChangeMobileRobotState const & event)
{
  ScAddr const & stateAddr = event.GetArcSourceElement();
  ScAddrToValueUnorderedMap<InterpreterCallback> states = {
      {MobileRobotsKeynodes::concept_ready_being_loaded,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateReadyBeingLoaded(action, robotAddr);
       }},
      {MobileRobotsKeynodes::concept_ready_being_unloaded,
       [this](ScAction & action, ScAddr const & robotAddr) -> ScResult
       {
         return InterpreterStateReadyBeingUnloaded(action, robotAddr);
       }},
  };
  auto const & it = states.find(stateAddr);
  if (it == states.cend())
    return false;

  m_interpreterCallback = it->second;
  return true;
}

// Установить для робота состояние "загружается"
// Найти местоположение робота и из этого местоположения взять свободную коробку и установить через какой-то
// промежуток времени её в качестве груза робота
// После этого убрать состояние "загружается" и установить состояние "загружен"
// убираем состояние "Готов к загрузке"
// Ставим состояние "Загружается"
// Добавить временной промежуток на загрузку
// Нахождение коробки, перемещение ее на агента робота и добавление состояния "Загружен"
// При отсутствии коробки робот завершает свою работу
ScResult MobileRobotCoordinationAgent::InterpreterStateReadyBeingLoaded(ScAction & action, ScAddr const & robotAddr)
{
  ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_ready_being_loaded, robotAddr);
  ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_robot_is_loading, robotAddr);

  ScIterator5Ptr it5 = m_context.CreateIterator5(robotAddr, ScType::ConstCommonArc, ScType::ConstNode, 
    MobileRobotsKeynodes::nrel_location, ScType::ActualTempPosArc);
  if (it5->Next()){
    ScAddr const &routeStartPointAddr = it5->Get(2);
    ScIterator5Ptr it5_1 = m_context.CreateIterator5(ScType::ConstNode, ScType::ConstCommonArc, routeStartPointAddr,
     MobileRobotsKeynodes::nrel_location, ScType::ActualTempPosArc);
    while (it5_1->Next()){
      ScAddr const &boxAddr = it5_1->Get(0);
      ScIterator3Ptr it3 = m_context.CreateIterator3(MobileRobotsKeynodes::concept_box, ScType::ConstPermPosArc, boxAddr);
      if(it3->Next()){
        m_context.EraseElement(it5_1->Get(1));
        m_context.EraseElement(it5_1->Get(4));

        ScIterator5Ptr it5_2 = m_context.CreateIterator5(ScType::ConstNodeStructure, ScType::ConstPermPosArc, routeStartPointAddr,
        MobileRobotsKeynodes::rrel_start_point, ScType::ConstPermPosArc);
        if(it5_2->Next()){
          ScAddr const &routeAddr = it5_2->Get(0);

          int load_time = GetLoadTime(routeAddr);

          std::this_thread::sleep_for(std::chrono::seconds(5));
          ScAddr const &arcAddr = m_context.GenerateConnector(ScType::ConstCommonArc, boxAddr, robotAddr);
          m_context.GenerateConnector(ScType::ActualTempPosArc, MobileRobotsKeynodes::nrel_location, arcAddr);

          ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_box_unloaded, robotAddr);
          ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_box_loaded, robotAddr);
          ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_robot_is_loading, robotAddr);

          break;
        }
      }
      else{
        ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_launched, robotAddr);
      }
    }
  }
  return action.FinishSuccessfully();
}

// Установить для робота состояние "разгружается"
// Найти местоположение робота и из этого местоположения взять свободную коробку и установить через какой-то
// промежуток времени её в качестве груза робота
// После этого убрать состояние "разгружается" и установить состояние "разгружен"

// Убираем состояние "Готов к разгрузке"
// Ставим состояние "Разгружается"
// Добавить временной промежуток на разгрузку

// Перемещение коробки с агента-робота в пункт разгрузки (процесс разгрузка)
// Добавление состояния "Разгружен"
// Если в пункте загрузки нет коробок - прекращение работы
ScResult MobileRobotCoordinationAgent::InterpreterStateReadyBeingUnloaded(ScAction & action, ScAddr const & robotAddr)
{
  ScAddr const &routeEndPointAddr = NULL;
  ScAddr const &routeAddr = NULL;

  ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_ready_being_unloaded, robotAddr);
  ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_robot_is_unloading, robotAddr);

  ScIterator5Ptr it5 = m_context.CreateIterator5(ScType::ConstNode, ScType::ConstCommonArc, robotAddr, 
    MobileRobotsKeynodes::nrel_location, ScType::ActualTempPosArc);
  while (it5->Next()){
    ScAddr const &boxAddr = it5->Get(0);
    ScIterator3Ptr it3 = m_context.CreateIterator3(MobileRobotsKeynodes::concept_box, ScType::ConstPermPosArc, boxAddr);
    if(it3->Next()){
      ScIterator5Ptr it5_1 = m_context.CreateIterator5(robotAddr, ScType::ConstCommonArc, ScType::ConstNode,
      MobileRobotsKeynodes::nrel_location, ScType::ActualTempPosArc);
      if(it5_1->Next()){
        ScAddr const &routeEndPointAddr = it5_1->Get(2);
        m_context.EraseElement(it5->Get(1));
        m_context.EraseElement(it5->Get(4));        

        ScIterator5Ptr it5_2 = m_context.CreateIterator5(ScType::ConstNodeStructure, ScType::ConstPermPosArc, routeEndPointAddr,
        MobileRobotsKeynodes::rrel_end_point, ScType::ConstPermPosArc);
        if(it5_2->Next()){
          ScAddr const &routeAddr = it5_2->Get(0);

          int unload_time = GetUnloadTime(routeAddr);

          std::this_thread::sleep_for(std::chrono::seconds(unload_time));
          ScAddr const &arcAddr = m_context.GenerateConnector(ScType::ConstCommonArc, boxAddr, routeEndPointAddr);
          m_context.GenerateConnector(ScType::ActualTempPosArc, MobileRobotsKeynodes::nrel_location, arcAddr);

          ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_box_loaded, robotAddr);
          ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_box_unloaded, robotAddr);
          ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_robot_is_unloading, robotAddr);

          break;
        }
      }
    }
  }

  it5 = m_context.CreateIterator5(ScType::ConstNodeStructure, ScType::ConstPermPosArc, routeEndPointAddr,
     MobileRobotsKeynodes::rrel_end_point, ScType::ConstPermPosArc);
  if(it5->Next()){
    ScAddr const &routeAddr = it5->Get(0);
    ScIterator5Ptr it5_1 = m_context.CreateIterator5(routeAddr, ScType::ConstPermPosArc, ScType::ConstNode,
      MobileRobotsKeynodes::rrel_start_point, ScType::ConstPermPosArc);
    if(it5_1->Next()){
      ScAddr const &routeStartPointAddr = it5_1->Get(2);
      ScIterator5Ptr it5_2 = m_context.CreateIterator5(ScType::ConstNode, ScType::ConstCommonArc, routeStartPointAddr,
      MobileRobotsKeynodes::nrel_location, ScType::ActualTempPosArc);
      if(it5_2->Next()){
        ScAddr const &boxAddr = it5_2->Get(0);
        if(m_context.CheckConnector(MobileRobotsKeynodes::concept_box, boxAddr, ScType::ConstPermPosArc))
          return action.FinishSuccessfully();
      }
      ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_launched, robotAddr);
      return action.FinishSuccessfully();
    }
  }
}

ScResult MobileRobotCoordinationAgent::DoProgram(ScEventChangeMobileRobotState const & event, ScAction & action)
{
  ScAddr const & robotAddr = event.GetArcTargetElement();
  return m_interpreterCallback(action, robotAddr);
}

void MobileRobotCoordinationAgent::ChangeActualTempArcToPos(const ScAddr &addr1, const ScAddr &addr2){
  ScIterator3Ptr it3 = m_context.CreateIterator3(addr1, ScType::ActualTempNegArc, addr2);
  if (it3->Next()){
    m_context.EraseElement(it3->Get(1));
  }
  m_context.GenerateConnector(ScType::ActualTempPosArc, addr1, addr2);
}

void MobileRobotCoordinationAgent::ChangeActualTempArcToNeg(const ScAddr &addr1, const ScAddr &addr2){
  ScIterator3Ptr it3 = m_context.CreateIterator3(addr1, ScType::ActualTempPosArc, addr2);
  if (it3->Next()){
    m_context.EraseElement(it3->Get(1));
  }
  m_context.GenerateConnector(ScType::ActualTempNegArc, addr1, addr2);
}

double MobileRobotCoordinationAgent::GetLoadTime(ScAddr const &routeAddr){
  double minLoadTime;
  ScIterator5Ptr it5 = m_context.CreateIterator5(routeAddr, ScType::ConstPermPosArc, ScType::LinkConst, 
    MobileRobotsKeynodes::rrel_min_load_time, ScType::ConstPermPosArc);
  if(it5->Next()){
    ScAddr const &minLoadTimeAddr = it5->Get(2);
    m_context.GetLinkContent(minLoadTimeAddr, minLoadTime);
  }
  double maxLoadTime;
  ScIterator5Ptr it5 = m_context.CreateIterator5(routeAddr, ScType::ConstPermPosArc, ScType::LinkConst, 
    MobileRobotsKeynodes::rrel_max_load_time, ScType::ConstPermPosArc);
  if(it5->Next()){
    ScAddr const &maxLoadTimeAddr = it5->Get(2);
    m_context.GetLinkContent(maxLoadTimeAddr, maxLoadTime);
  }
  return GenerateTime(minLoadTime, maxLoadTime);
}

double MobileRobotCoordinationAgent::GetUnloadTime(ScAddr const &routeAddr){
  double minUnloadTime;
  ScIterator5Ptr it5 = m_context.CreateIterator5(routeAddr, ScType::ConstPermPosArc, ScType::LinkConst, 
    MobileRobotsKeynodes::rrel_min_load_time, ScType::ConstPermPosArc);
  if(it5->Next()){
    ScAddr const &minUnloadTimeAddr = it5->Get(2);
    m_context.GetLinkContent(minUnloadTimeAddr, minUnloadTime);
  }
  double maxUnloadTime;
  ScIterator5Ptr it5 = m_context.CreateIterator5(routeAddr, ScType::ConstPermPosArc, ScType::LinkConst, 
    MobileRobotsKeynodes::rrel_max_load_time, ScType::ConstPermPosArc);
  if(it5->Next()){
    ScAddr const &maxUnloadTimeAddr = it5->Get(2);
    m_context.GetLinkContent(maxUnloadTimeAddr, maxUnloadTime);
  }
  return GenerateTime(minUnloadTime, maxUnloadTime);
}

double MobileRobotCoordinationAgent::GenerateTime(double const &min, double const &max)
{
  std::uniform_int_distribution<double> timeDistribution(min, max);
  return timeDistribution(m_randomGenerator);
}