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
