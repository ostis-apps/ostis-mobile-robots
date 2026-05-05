#include "mobile_robot_coordination_agent.hpp"

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
  // Установить для робота состояние "загружается"
  // Найти местоположение робота и из этого местоположения взять свободную коробку и установить через какой-то
  // промежуток времени её в качестве груза робота
  // После этого убрать состояние "загружается" и установить состояние "загружен"


  // убираем состояние "Готов к загрузке"
  ScIterator3Ptr it3 = m_context.CreateIterator3(MobileRobotsKeynodes::concept_ready_being_loaded, ScType::ActualTempPosArc, robotAddr);
  if (it3->Next()){
    m_context.EraseElement(it3->Get(1));
  }
  m_context.GenerateConnector(ScType::ActualTempNegArc, MobileRobotsKeynodes::concept_ready_being_loaded, robotAddr);

  //ставим состояние "Загружается"
  it3 = m_context.CreateIterator3(MobileRobotsKeynodes::concept_robot_is_loading, ScType::ActualTempNegArc, robotAddr);
  if (it3->Next()){
    m_context.EraseElement(it3->Get(1));
  }
  m_context.GenerateConnector(ScType::ActualTempPosArc, MobileRobotsKeynodes::concept_robot_is_loading, robotAddr);

  // Добавить временной промежуток на загрузку

  // Нахождение коробки, перемещение ее на агента робота и добавление состояния "Загружен"
  // При отсутствии коробки робот завершает свою работу
  ScIterator5Ptr it5 = m_context.CreateIterator5(robotAddr, ScType::ConstCommonArc, ScType::Node, 
    MobileRobotsKeynodes::nrel_location, ScType::ActualTempPosArc);
  if (it5->Next()){
    ScIterator5Ptr it5_1 = m_context.CreateIterator5(ScType::Node, ScType::ConstCommonArc, it3->Get(2),
     MobileRobotsKeynodes::nrel_location, ScType::ConstPermPosArc);
    if (it5_1->Next()){
      ScIterator3Ptr it3_1 = m_context.CreateIterator3(MobileRobotsKeynodes::concept_box, ScType::ActualTempPosArc, it5_1->Get(0));
      if(it3_1->Next()){
        m_context.EraseElement(it5_1->Get(4));
        m_context.GenerateConnector(ScType::ActualTempPosArc, MobileRobotsKeynodes::nrel_location, it5_1->Get(1));

        ScIterator3Ptr it3_2 = m_context.CreateIterator3(MobileRobotsKeynodes::concept_box_unloaded, ScType::ActualTempPosArc, robotAddr);
        if (it3_2->Next()){
          m_context.EraseElement(it3_2->Get(1));
        }
        m_context.GenerateConnector(ScType::ActualTempNegArc, MobileRobotsKeynodes::concept_box_unloaded, robotAddr);

        it3_2 = m_context.CreateIterator3(MobileRobotsKeynodes::concept_box_loaded, ScType::ActualTempNegArc, robotAddr);
        if (it3_2->Next()){
          m_context.EraseElement(it3_2->Get(1));
        }
        m_context.GenerateConnector(ScType::ActualTempPosArc, MobileRobotsKeynodes::concept_box_loaded, robotAddr);

        it3 = m_context.CreateIterator3(MobileRobotsKeynodes::concept_robot_is_loading, ScType::ActualTempPosArc, robotAddr);
        if (it3->Next()){
        m_context.EraseElement(it3->Get(1));
        }
        m_context.GenerateConnector(ScType::ActualTempNegArc, MobileRobotsKeynodes::concept_robot_is_loading, robotAddr);
      }
      else{
        it3 = m_context.CreateIterator3(MobileRobotsKeynodes::concept_launched, ScType::ActualTempPosArc, robotAddr);
        if(it3->Next()){
          m_context.EraseElement(it3->Get(1));
          m_context.GenerateConnector(ScType::ActualTempNegArc, MobileRobotsKeynodes::concept_launched, robotAddr);
        }
      }
    }
  }


  return action.FinishSuccessfully();
}

ScResult MobileRobotCoordinationAgent::InterpreterStateReadyBeingUnloaded(ScAction & action, ScAddr const & robotAddr)
{
  // Установить для робота состояние "разгружается"
  // Найти местоположение робота и из этого местоположения взять свободную коробку и установить через какой-то
  // промежуток времени её в качестве груза робота
  // После этого убрать состояние "разгружается" и установить состояние "разгружен"

  // убираем состояние "Готов к разгрузке"
  ScIterator3Ptr it3 = m_context.CreateIterator3(MobileRobotsKeynodes::concept_ready_being_unloaded, ScType::ActualTempPosArc, robotAddr);
  if (it3->Next()){
    m_context.EraseElement(it3->Get(1));
  }
  m_context.GenerateConnector(ScType::ActualTempNegArc, MobileRobotsKeynodes::concept_ready_being_loaded, robotAddr);

  //ставим состояние "Разгружается"
  it3 = m_context.CreateIterator3(MobileRobotsKeynodes::concept_robot_is_unloading, ScType::ActualTempNegArc, robotAddr);
  if (it3->Next()){
    m_context.EraseElement(it3->Get(1));
  }
  m_context.GenerateConnector(ScType::ActualTempPosArc, MobileRobotsKeynodes::concept_robot_is_unloading, robotAddr);

  // Добавить временной промежуток на разгрузку

  // Перемещение коробки с агента-робота в пункт разгрузки (процесс разгрузка)
  // добавление состояния "Разгружен"
  // если в пункте загрузки нет коробок - прекращение работы 
  ScIterator5Ptr it5 = m_context.CreateIterator5(ScType::Node, ScType::ConstCommonArc, robotAddr, 
    MobileRobotsKeynodes::nrel_location, ScType::ActualTempPosArc);
  if (it5->Next()){
    ScIterator3Ptr it3_1 = m_context.CreateIterator3(MobileRobotsKeynodes::concept_box, ScType::ActualTempPosArc, it5->Get(0));
    if(it3_1->Next()){
      m_context.EraseElement(it5->Get(4));
      ScIterator5Ptr it5_1 = m_context.CreateIterator5(robotAddr, ScType::ConstCommonArc, ScType::Node,
     MobileRobotsKeynodes::nrel_location, ScType::ConstPermPosArc);
     if(it5_1->Next()){
      m_context.GenerateConnector(ScType::ActualTempPosArc, it5->Get(0), it5_1->Get(2));

      ScIterator3Ptr it3_2 = m_context.CreateIterator3(MobileRobotsKeynodes::concept_box_loaded, ScType::ActualTempPosArc, robotAddr);
      if (it3_2->Next()){
        m_context.EraseElement(it3_2->Get(1));
      }
      m_context.GenerateConnector(ScType::ActualTempNegArc, MobileRobotsKeynodes::concept_box_loaded, robotAddr);

      it3_2 = m_context.CreateIterator3(MobileRobotsKeynodes::concept_box_unloaded, ScType::ActualTempNegArc, robotAddr);
      if (it3_2->Next()){
        m_context.EraseElement(it3_2->Get(1));
      }
      m_context.GenerateConnector(ScType::ActualTempPosArc, MobileRobotsKeynodes::concept_box_unloaded, robotAddr);

      it3 = m_context.CreateIterator3(MobileRobotsKeynodes::concept_robot_is_unloading, ScType::ActualTempPosArc, robotAddr);
      m_context.EraseElement(it3->Get(1));
      if (it3->Next()){
      }
      m_context.GenerateConnector(ScType::ActualTempNegArc, MobileRobotsKeynodes::concept_robot_is_unloading, robotAddr);

      }
    }
  }
  // тут поменять ScType::ClassNode на ScStructure
  it5 = m_context.CreateIterator5(ScType::Node, ScType::ConstPermPosArc, ScType::Node,
     MobileRobotsKeynodes::rrel_start_point, ScType::ConstPermPosArc);
  if (it5->Next()){
    ScIterator5Ptr it5_1 = m_context.CreateIterator5(ScType::Node, ScType::ConstCommonArc, it5->Get(2),
     MobileRobotsKeynodes::nrel_location, ScType::ConstPermPosArc);
     if(it5_1->Next()){
      if(m_context.CheckConnector(MobileRobotsKeynodes::concept_box, it5_1->Get(0), ScType::ConstPermPosArc))
        return action.FinishSuccessfully();
     }
     it3 = m_context.CreateIterator3(MobileRobotsKeynodes::concept_launched, ScType::ActualTempPosArc, robotAddr);
      if(it3->Next()){
        m_context.EraseElement(it3->Get(1));
        m_context.GenerateConnector(ScType::ActualTempNegArc, MobileRobotsKeynodes::concept_launched, robotAddr);
      }
  }
  return action.FinishSuccessfully();
}

ScResult MobileRobotCoordinationAgent::DoProgram(ScEventChangeMobileRobotState const & event, ScAction & action)
{
  ScAddr const & robotAddr = event.GetArcTargetElement();
  return m_interpreterCallback(action, robotAddr);
}

// убрать излишние операции по нахождению и записывать результаты в переменные типа ScAddr