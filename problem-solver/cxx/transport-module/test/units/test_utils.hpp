#pragma once

#include <sc-memory/test/sc_test.hpp>
#include <sc-builder/scs_loader.hpp>

#include <agents/mobile_robot_coordination_agent.hpp>
#include <agents/mobile_robot_interpretation_agent.hpp>
#include <agents/mobile_robot_analyzer_agent.hpp>
#include <keynodes/keynodes.hpp>

using TransportModuleTest = ScMemoryTest;
std::string const EXAMPLE_MODULE_TEST_FILES_DIR_PATH = "../test-structures/";

void SubscribeAgents(ScAgentContext & context);
void UnsubscribeAgents(ScAgentContext & context);