#include "mobile_robot_coordination_agent.hpp"
#include <sc-memory/sc_link.hpp>
#include <chrono>
#include <thread>
#include <mutex>

std::mutex unloading_mutex;
std::mutex loading_mutex;

bool MobileRobotCoordinationAgent::box_counted = false;

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

ScResult MobileRobotCoordinationAgent::InterpreterStateReadyBeingLoaded(ScAction & action, ScAddr const & robotAddr)
{
  SC_LOG_INFO("Start InterpreterStateReadyBeingLoaded (" + m_context.GetElementSystemIdentifier(robotAddr) + ")");
  ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_ready_being_loaded, robotAddr);
  ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_robot_is_loading, robotAddr);
  ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_robot_is_not_loading, robotAddr);

  ScIterator5Ptr it5 = m_context.CreateIterator5(
      robotAddr,
      ScType::ConstCommonArc,
      ScType::ConstNode,
      ScType::ConstActualTempPosArc,
      MobileRobotsKeynodes::nrel_location);
  if (it5->Next())
  {
    std::unique_lock<std::mutex> lock(loading_mutex);

    ScAddr const & routeStartPointAddr = it5->Get(2);
    ScIterator5Ptr it5_1 = m_context.CreateIterator5(
        ScType::ConstNode,
        ScType::ConstCommonArc,
        routeStartPointAddr,
        ScType::ConstActualTempPosArc,
        MobileRobotsKeynodes::nrel_location);
    bool box_is_founded = false;
    while (it5_1->Next())
    {
      ScAddr const & boxAddr = it5_1->Get(0);
      ScIterator3Ptr it3 =
          m_context.CreateIterator3(MobileRobotsKeynodes::concept_box, ScType::ConstPermPosArc, boxAddr);
      if (it3->Next())
      {
        SC_LOG_INFO("A box is found at StartPoint (" + m_context.GetElementSystemIdentifier(robotAddr) + ")");
        m_context.EraseElement(it5_1->Get(1));

        ReduceBoxCount(routeStartPointAddr);

        lock.unlock();
        
        ScIterator5Ptr it5_2 = m_context.CreateIterator5(
            ScType::ConstNodeStructure,
            ScType::ConstPermPosArc,
            routeStartPointAddr,
            ScType::ConstPermPosArc,
            MobileRobotsKeynodes::rrel_start_point);
        if (it5_2->Next())
        {
          ScAddr const & routeAddr = it5_2->Get(0);

          int load_time = GetLoadTime(routeAddr);

          SC_LOG_INFO("Loading will take " + std::to_string(load_time) + " seconds (" + m_context.GetElementSystemIdentifier(robotAddr) + ")");


          std::this_thread::sleep_for(std::chrono::seconds(load_time));
          ScAddr const & arcAddr = m_context.GenerateConnector(ScType::ConstCommonArc, boxAddr, robotAddr);
          m_context.GenerateConnector(ScType::ConstActualTempPosArc, MobileRobotsKeynodes::nrel_location, arcAddr);

          ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_box_unloaded, robotAddr);
          ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_box_loaded, robotAddr);
          ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_robot_is_loading, robotAddr);
          ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_robot_is_not_loading, robotAddr);

          box_is_founded = true;
          SC_LOG_INFO("Finish InterpreterStateReadyBeingLoaded (" + m_context.GetElementSystemIdentifier(robotAddr) + ")");
          return action.FinishSuccessfully();
        }
      }
    }
    if (!box_is_founded)
    {
      SC_LOG_INFO("No boxes at StartPoint (" + m_context.GetElementSystemIdentifier(robotAddr) + ")");
      ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_launched, robotAddr);
      ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_stopped, robotAddr);
      SC_LOG_INFO("Finish InterpreterStateReadyBeingLoaded (" + m_context.GetElementSystemIdentifier(robotAddr) + ")");
      return action.FinishSuccessfully();
    }
  }
  SC_LOG_INFO("Finish unsuccessfully InterpreterStateReadyBeingLoaded (" + m_context.GetElementSystemIdentifier(robotAddr) + ")");
  return action.FinishUnsuccessfully();
}

void MobileRobotCoordinationAgent::ReduceBoxCount(ScAddr const &routeStartPoint){
  ScAddr boxCountAddr;
  int box_count;
  ScIterator5Ptr it5 = m_context.CreateIterator5(
    routeStartPoint,
    ScType::ConstCommonArc,
    ScType::ConstNodeLink,
    ScType::ConstActualTempPosArc,
    MobileRobotsKeynodes::nrel_box_count);
  if(it5->Next()){
    boxCountAddr = it5->Get(2);
    m_context.GetLinkContent(boxCountAddr, box_count);
    box_count--;
    m_context.SetLinkContent(boxCountAddr, std::to_string(box_count));
  }     
}

ScResult MobileRobotCoordinationAgent::InterpreterStateReadyBeingUnloaded(ScAction & action, ScAddr const & robotAddr)
{
  SC_LOG_INFO("Start InterpreterStateReadyBeingUnloaded (" + m_context.GetElementSystemIdentifier(robotAddr) + ")");
  ScAddr routeEndPointAddr = GetRouteEndPoint(robotAddr);

  std::unique_lock<std::mutex> lock(unloading_mutex);

  bool keep_working = AreThereFreeBoxes(robotAddr);

  lock.unlock();

  ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_ready_being_unloaded, robotAddr);
  ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_robot_is_unloading, robotAddr);
  ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_robot_is_not_unloading, robotAddr);

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
      SC_LOG_INFO("A box is found on " + m_context.GetElementSystemIdentifier(robotAddr));

      ScIterator5Ptr it5_2 = m_context.CreateIterator5(
          ScType::ConstNodeStructure,
          ScType::ConstPermPosArc,
          routeEndPointAddr,
          ScType::ConstPermPosArc,
          MobileRobotsKeynodes::rrel_end_point);
      if (it5_2->Next())
      {
        ScAddr const & routeAddr = it5_2->Get(0);

        int unload_time = GetUnloadTime(routeAddr);

        SC_LOG_INFO("Unloading will take " + std::to_string(unload_time) + " seconds (" + m_context.GetElementSystemIdentifier(robotAddr) + ")");

        std::this_thread::sleep_for(std::chrono::seconds(unload_time));
        ScAddr const & arcAddr = m_context.GenerateConnector(ScType::ConstCommonArc, boxAddr, routeEndPointAddr);
        m_context.GenerateConnector(ScType::ConstActualTempPosArc, MobileRobotsKeynodes::nrel_location, arcAddr);

        ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_robot_is_unloading, robotAddr);
        ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_robot_is_not_unloading, robotAddr);

        ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_box_loaded, robotAddr);

        if (!keep_working)
        {
          SC_LOG_INFO("There are no boxes at StartPoint, " + m_context.GetElementSystemIdentifier(robotAddr) + " is shutting down");
          ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_launched, robotAddr);
          ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_stopped, robotAddr);
          SC_LOG_INFO("Finish InterpreterStateReadyBeingUnloaded (" + m_context.GetElementSystemIdentifier(robotAddr) + ")");
          return action.FinishSuccessfully();
        }
        
        SC_LOG_INFO("There is at least one free box at StartPoint, " + m_context.GetElementSystemIdentifier(robotAddr) + " keeps working");
        ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_box_unloaded, robotAddr);

        SC_LOG_INFO("Finish InterpreterStateReadyBeingUnloaded (" + m_context.GetElementSystemIdentifier(robotAddr) + ")");
        return action.FinishSuccessfully();
      }
    }
  }
  SC_LOG_INFO("Finish unsuccessfully InterpreterStateReadyBeingUnloaded (" + m_context.GetElementSystemIdentifier(robotAddr) + ")");
  return action.FinishUnsuccessfully();
}

ScResult MobileRobotCoordinationAgent::DoProgram(ScEventChangeMobileRobotState const & event, ScAction & action)
{
  ScAddr const & robotAddr = event.GetArcTargetElement();

  return m_interpreterCallback(action, robotAddr);
}

void MobileRobotCoordinationAgent::ChangeActualTempArcToPos(ScAddr const & addr1, ScAddr const & addr2)
{
  ScIterator3Ptr it3 = m_context.CreateIterator3(addr1, ScType::ConstActualTempNegArc, addr2);
  if (it3->Next())
  {
    m_context.EraseElement(it3->Get(1));
  }
  m_context.GenerateConnector(ScType::ConstActualTempPosArc, addr1, addr2);
}

void MobileRobotCoordinationAgent::ChangeActualTempArcToNeg(ScAddr const & addr1, ScAddr const & addr2)
{
  ScIterator3Ptr it3 = m_context.CreateIterator3(addr1, ScType::ConstActualTempPosArc, addr2);
  if (it3->Next())
  {
    m_context.EraseElement(it3->Get(1));
  }
  m_context.GenerateConnector(ScType::ConstActualTempNegArc, addr1, addr2);
}

bool MobileRobotCoordinationAgent::AreThereFreeBoxes(ScAddr const &robotAddr){
  ScAddr routeEndPointAddr = GetRouteEndPoint(robotAddr);
  ScIterator5Ptr it5 = m_context.CreateIterator5(
      ScType::ConstNodeStructure,
      ScType::ConstPermPosArc,
      routeEndPointAddr,
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::rrel_end_point);
  if (it5->Next())
  {
    ScAddr const & routeAddr = it5->Get(0);

    ScIterator5Ptr it5 = m_context.CreateIterator5(
        routeAddr,
        ScType::ConstPermPosArc,
        ScType::ConstNode,
        ScType::ConstPermPosArc,
        MobileRobotsKeynodes::rrel_start_point);
    if(it5->Next()){
      ScAddr routeStartPoint = it5->Get(2);
      ScAddr freeBoxCountAddr;
      int free_box_count;
      ScIterator5Ptr it5 = m_context.CreateIterator5(
        routeStartPoint,
        ScType::ConstCommonArc,
        ScType::ConstNodeLink,
        ScType::ConstActualTempPosArc,
        MobileRobotsKeynodes::nrel_free_box_count);
      if(it5->Next())
      {
        freeBoxCountAddr = it5->Get(2);
        m_context.GetLinkContent(freeBoxCountAddr, free_box_count);
      }
      else
      {
        ScIterator5Ptr it5_2 = m_context.CreateIterator5(
            routeStartPoint,
            ScType::ConstCommonArc,
            ScType::ConstNodeLink,
            ScType::ConstActualTempPosArc,
            MobileRobotsKeynodes::nrel_box_count);
        if(it5_2->Next()){
          int box_count;
          m_context.GetLinkContent(it5_2->Get(2), box_count);
          free_box_count = box_count;
          freeBoxCountAddr = m_context.GenerateLink(ScType::ConstNodeLink);
          m_context.SetLinkContent(freeBoxCountAddr, free_box_count);
          ScAddr arcAddr = m_context.GenerateConnector(ScType::ConstCommonArc, routeStartPoint, freeBoxCountAddr);
          m_context.GenerateConnector(ScType::ConstActualTempPosArc, MobileRobotsKeynodes::nrel_free_box_count, arcAddr);
        }
      }
      if (free_box_count){
        free_box_count--;
        m_context.SetLinkContent(freeBoxCountAddr, std::to_string(free_box_count));
        return true;
      }     
    }
  }
  return false;
}

ScAddr const &MobileRobotCoordinationAgent::GetRouteEndPoint(ScAddr const &robotAddr){
  ScIterator5Ptr it5 = m_context.CreateIterator5(
      robotAddr,
      ScType::ConstCommonArc,
      ScType::ConstNode,
      ScType::ConstActualTempPosArc,
      MobileRobotsKeynodes::nrel_location);
  if (it5->Next())
  {
    ScAddr const &routeEndPointAddr = it5->Get(2);
    return routeEndPointAddr;
  }
  throw std::exception();
}

double MobileRobotCoordinationAgent::GetLoadTime(ScAddr const & routeAddr)
{
  double minLoadTime;
  ScIterator5Ptr it5 = m_context.CreateIterator5(
      routeAddr,
      ScType::ConstCommonArc,
      ScType::ConstNodeLink,
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::nrel_min_load_time);
  if (it5->Next())
  {
    ScAddr const & minLoadTimeAddr = it5->Get(2);
    m_context.GetLinkContent(minLoadTimeAddr, minLoadTime);
  }
  double maxLoadTime;
  it5 = m_context.CreateIterator5(
      routeAddr,
      ScType::ConstCommonArc,
      ScType::ConstNodeLink,
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::nrel_max_load_time);
  if (it5->Next())
  {
    ScAddr const & maxLoadTimeAddr = it5->Get(2);
    m_context.GetLinkContent(maxLoadTimeAddr, maxLoadTime);
  }
  return GenerateTime(minLoadTime, maxLoadTime);
}

double MobileRobotCoordinationAgent::GetUnloadTime(ScAddr const & routeAddr)
{
  double minUnloadTime;
  ScIterator5Ptr it5 = m_context.CreateIterator5(
      routeAddr,
      ScType::ConstCommonArc,
      ScType::ConstNodeLink,
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::nrel_min_unload_time);
  if (it5->Next())
  {
    ScAddr const & minUnloadTimeAddr = it5->Get(2);
    m_context.GetLinkContent(minUnloadTimeAddr, minUnloadTime);
  }
  double maxUnloadTime;
  it5 = m_context.CreateIterator5(
      routeAddr,
      ScType::ConstCommonArc,
      ScType::ConstNodeLink,
      ScType::ConstPermPosArc,
      MobileRobotsKeynodes::nrel_max_unload_time);
  if (it5->Next())
  {
    ScAddr const & maxUnloadTimeAddr = it5->Get(2);
    m_context.GetLinkContent(maxUnloadTimeAddr, maxUnloadTime);
  }
  return GenerateTime(minUnloadTime, maxUnloadTime);
}

double MobileRobotCoordinationAgent::GenerateTime(int const & min, int const & max)
{
  std::uniform_int_distribution<int> timeDistribution(min, max);
  return timeDistribution(m_randomGenerator);
}