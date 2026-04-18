"""
This source file is part of an OSTIS project. For the latest info, see http://ostis.net
Distributed under the MIT License
(See accompanying file COPYING.MIT or copy at http://opensource.org/licenses/MIT)
"""

from pathlib import Path

from modules.transport_module.transport_module import TransportModule
from sc_kpm.utils.action_utils import execute_agent
from tests.base_testcase import BaseTestCase

from sc_kpm.identifiers import CommonIdentifiers, ActionStatus

WAIT_TIME = 5


class TransportModuleTestCase(BaseTestCase):
    def setUp(self):
        super().setUp()

    def run_agent(self):
        # Вызов одного из агентов
        pass

    def test_transport_module(self):
        self.load_scs(
            Path(
                self.tests_structures_dir_path,
                "test_example_agent.scs",
            )
        )
        module = TransportModule()
        self.server.add_modules(module)
        with self.server.register_modules():
            self.run_agent()

        self.server.remove_modules(module)
