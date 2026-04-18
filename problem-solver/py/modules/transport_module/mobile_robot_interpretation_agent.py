from sc_client.models import ScAddr
from sc_client.constants import sc_type
from sc_client.constants.common import ScEventType
from sc_kpm import ScAgent, ScModule, ScKeynodes
from sc_kpm.utils import (
    generate_node,
    generate_connector,
    generate_role_relation,
    check_connector,
)
from sc_kpm.utils.action_utils import (
    get_action_arguments,
    generate_action,
    call_action,
    execute_action,
    execute_agent,
    wait_agent,
    finish_action_with_status,
)
from sc_kpm.identifiers import ActionStatus, CommonIdentifiers
from sc_kpm.sc_result import ScResult


class Keynodes:
    ACTION_INTERPRETER_MOBILE_ROBOT = "action_interpreter_mobile_robot"


class MobileRobotInterpretationAgent(ScAgent):
    def __init__(self):
        robot_addr = None # Тут надо для всех роботов из базы знаний зарегистрировать данный агент
        super().__init__(robot_addr, ScEventType.AFTER_GENERATE_INCOMING_ARC)

    def check_initiation_condition(
        self,
        source_addr: ScAddr,
        event_arc_addr: ScAddr,
        robot_addr: ScAddr,
    ) -> bool:
        # Тут надо задать шаблоны и проверять по ним наличие в базе знаний ситуаций, соответствующих командам 
        # агента координации мобильного робота, для дальнейшего вызова on_event
        
        # Подробнее тут https://github.com/ostis-ai/py-sc-client и тут https://github.com/ostis-ai/py-sc-kpm

        return True

    def on_event(
        self,
        source_addr: ScAddr,
        event_arc_addr: ScAddr,
        robot_addr: ScAddr,
    ):
        is_successful = True

        action_class_addr = ScKeynodes.resolve(Keynodes.ACTION_INTERPRETER_MOBILE_ROBOT, sc_type=sc_type.CONST_NODE_CLASS)
        action_addr = generate_action(action_class_addr)

        if not self.check_initiation_condition(source_addr, event_arc_addr, robot_addr):
            finish_action_with_status(action_addr, False)
            return
        
        # Тут должна быть логика обработки команд от агента координации мобильного робота

        # Подробнее тут https://github.com/ostis-ai/py-sc-client и тут https://github.com/ostis-ai/py-sc-kpm

        finish_action_with_status(action_addr, is_successful)
        return ScResult.OK
