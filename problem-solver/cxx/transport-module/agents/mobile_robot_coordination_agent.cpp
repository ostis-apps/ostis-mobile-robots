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

ScResult MobileRobotCoordinationAgent::InterpreterStateReadyBeingLoaded(ScAction & action, ScAddr const & robotAddr)
{
  SC_LOG_INFO("Start InterpreterStateReadyBeingLoaded");
  ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_ready_being_loaded, robotAddr);
  ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_robot_is_loading, robotAddr);

  ScIterator5Ptr it5 = m_context.CreateIterator5(
      robotAddr,
      ScType::ConstCommonArc,
      ScType::ConstNode,
      ScType::ConstActualTempPosArc,
      MobileRobotsKeynodes::nrel_location);
  if (it5->Next())
  {
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
        SC_LOG_INFO("Box is found at StartPoint");
        m_context.EraseElement(it5_1->Get(1));

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

          SC_LOG_INFO(std::to_string(load_time));

          std::this_thread::sleep_for(std::chrono::seconds(load_time));
          ScAddr const & arcAddr = m_context.GenerateConnector(ScType::ConstCommonArc, boxAddr, robotAddr);
          m_context.GenerateConnector(ScType::ConstActualTempPosArc, MobileRobotsKeynodes::nrel_location, arcAddr);

          ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_box_unloaded, robotAddr);
          ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_box_loaded, robotAddr);
          ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_robot_is_loading, robotAddr);

          box_is_founded = true;
          SC_LOG_INFO("Finish InterpreterStateReadyBeingLoaded");
          return action.FinishSuccessfully();
        }
      }
    }
    if (!box_is_founded)
    {
      SC_LOG_INFO("No boxes at StartPoint");
      ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_launched, robotAddr);
      ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_stopped, robotAddr);
      SC_LOG_INFO("Finish InterpreterStateReadyBeingLoaded");
      return action.FinishSuccessfully();
    }
  }
  SC_LOG_INFO("Finish unsuccessfully InterpreterStateReadyBeingLoaded");
  return action.FinishUnsuccessfully();
}

ScResult MobileRobotCoordinationAgent::InterpreterStateReadyBeingUnloaded(ScAction & action, ScAddr const & robotAddr)
{
  SC_LOG_INFO("Start InterpreterStateReadyBeingUnloaded");
  ScAddr routeEndPointAddr;
  int box_count = 0;
  bool keep_working = false;

  ScIterator5Ptr it5 = m_context.CreateIterator5(
      robotAddr,
      ScType::ConstCommonArc,
      ScType::ConstNode,
      ScType::ConstActualTempPosArc,
      MobileRobotsKeynodes::nrel_location);
  if (it5->Next())
  {
    routeEndPointAddr = it5->Get(2);
    ScIterator5Ptr it5 = m_context.CreateIterator5(
        ScType::ConstNodeStructure,
        ScType::ConstPermPosArc,
        routeEndPointAddr,
        ScType::ConstPermPosArc,
        MobileRobotsKeynodes::rrel_end_point);
    if (it5->Next())
    {
      SC_LOG_INFO("Route is found");
      ScAddr const & routeAddr = it5->Get(0);

      ScIterator5Ptr it5_1 = m_context.CreateIterator5(
          routeAddr,
          ScType::ConstPermPosArc,
          ScType::ConstNode,
          ScType::ConstPermPosArc,
          MobileRobotsKeynodes::rrel_start_point);
      if (it5_1->Next())
      {
        SC_LOG_INFO("StartPoint is found");
        ScAddr const & routeStartPointAddr = it5_1->Get(2);
        ScIterator5Ptr it5_2 = m_context.CreateIterator5(
            ScType::ConstNode,
            ScType::ConstCommonArc,
            routeStartPointAddr,
            ScType::ConstActualTempPosArc,
            MobileRobotsKeynodes::nrel_location);
        while (it5_2->Next())
        {
          SC_LOG_INFO("Box found");
          ScAddr const & boxAddr = it5_2->Get(0);
          ScIterator3Ptr it3 =
              m_context.CreateIterator3(MobileRobotsKeynodes::concept_box, ScType::ConstPermPosArc, boxAddr);
          if (it3->Next())
            box_count++;
          if (box_count > 0)
            break;
        }
        keep_working = box_count > 0;
        SC_LOG_INFO("boxes at " + std::to_string(box_count));
      }
    }
  }

  ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_ready_being_unloaded, robotAddr);
  ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_robot_is_unloading, robotAddr);

  it5 = m_context.CreateIterator5(
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
      SC_LOG_INFO("Box is found on robot");

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

        SC_LOG_INFO(std::to_string(unload_time));

        std::this_thread::sleep_for(std::chrono::seconds(unload_time));
        ScAddr const & arcAddr = m_context.GenerateConnector(ScType::ConstCommonArc, boxAddr, routeEndPointAddr);
        m_context.GenerateConnector(ScType::ConstActualTempPosArc, MobileRobotsKeynodes::nrel_location, arcAddr);

        ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_robot_is_unloading, robotAddr);
        ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_box_loaded, robotAddr);

        if (!keep_working)
        {
          ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_stopped, robotAddr);
          ChangeActualTempArcToNeg(MobileRobotsKeynodes::concept_launched, robotAddr);
        }

        ChangeActualTempArcToPos(MobileRobotsKeynodes::concept_box_unloaded, robotAddr);

        SC_LOG_INFO("Finish InterpreterStateReadyBeingUnloaded");
        return action.FinishSuccessfully();
      }
    }
  }

  SC_LOG_INFO("Finish unsuccessfully InterpreterStateReadyBeingUnloaded");
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
    SC_LOG_INFO(std::to_string(minLoadTime));
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
    SC_LOG_INFO(std::to_string(maxLoadTime));
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
    SC_LOG_INFO(std::to_string(minUnloadTime));
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
    SC_LOG_INFO(std::to_string(maxUnloadTime));
  }
  return GenerateTime(minUnloadTime, maxUnloadTime);
}

double MobileRobotCoordinationAgent::GenerateTime(int const & min, int const & max)
{
  std::uniform_int_distribution<int> timeDistribution(min, max);
  return timeDistribution(m_randomGenerator);
}