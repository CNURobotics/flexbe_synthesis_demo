#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2024 Josh Luzier
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

"""
Define Synthesis_Output.

Created on Wed Feb 28 2024
@author: Josh Luzier
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer

from flexbe_states.log_state import LogState

from flexbe_synthesis_demo_flexbe_states.move_state import MoveState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class Synthesis_OutputSM(Behavior):
    """
    Define Synthesis_Output.

    The state machine generated from the farmer's dillemma specification list.
    To replicate, you will need to specify the user data parameters,
    the input keys into the container, and
    connect the container outputs to the behavior outputs

    """

    def __init__(self, node):
        """Initialize."""
        super().__init__()
        self.name = 'Synthesis_Output'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        LogState.initialize_ros(node)
        MoveState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        """Create behavior."""
        # x:30 y:365, x:130 y:365
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.left = ['farmer', 'wolf', 'goat', 'corn']
        _state_machine.userdata.right = []

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        # x:1182 y:27, x:1222 y:127
        _sm_container_0 = OperatableStateMachine(outcomes=['failed', 'finished'],
                                                 input_keys=['left', 'right'],
                                                 output_keys=['left', 'right'])

        with _sm_container_0:
            # x:35 y:27
            OperatableStateMachine.add('0_move_goat',
                                       MoveState(item='goat', tracker='farmer'),
                                       transitions={'done': '2_move_null', 'canceled': '1_log_failed', 'failed': '1_log_failed'},
                                       autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:25 y:555
            OperatableStateMachine.add('10_move_corn',
                                       MoveState(item='corn', tracker='farmer'),
                                       transitions={'done': '13_move_null', 'canceled': '12_log_failed', 'failed': '12_log_failed'},
                                       autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:335 y:452
            OperatableStateMachine.add('11_log_failed',
                                       LogState(text='Error in the previous move', severity=Logger.REPORT_HINT),
                                       transitions={'done': '14_move_goat'},
                                       autonomy={'done': Autonomy.Off})

            # x:342 y:585
            OperatableStateMachine.add('12_log_failed',
                                       LogState(text='Error in the previous move', severity=Logger.REPORT_HINT),
                                       transitions={'done': '15_move_corn'},
                                       autonomy={'done': Autonomy.Off})

            # x:307 y:674
            OperatableStateMachine.add('13_move_null',
                                       MoveState(item=None, tracker='farmer'),
                                       transitions={'done': '16_move_goat', 'canceled': '17_log_failed', 'failed': '17_log_failed'},
                                       autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:435 y:510
            OperatableStateMachine.add('14_move_goat',
                                       MoveState(item='goat', tracker='farmer'),
                                       transitions={'done': '10_move_corn', 'canceled': '11_log_failed', 'failed': '11_log_failed'},
                                       autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:448 y:596
            OperatableStateMachine.add('15_move_corn',
                                       MoveState(item='corn', tracker='farmer'),
                                       transitions={'done': '13_move_null', 'canceled': '12_log_failed', 'failed': '12_log_failed'},
                                       autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:857 y:515
            OperatableStateMachine.add('16_move_goat',
                                       MoveState(item='goat', tracker='farmer'),
                                       transitions={'done': '19_log_finished', 'canceled': '18_log_failed', 'failed': '18_log_failed'},
                                       autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:717 y:663
            OperatableStateMachine.add('17_log_failed',
                                       LogState(text='Error in the previous move', severity=Logger.REPORT_HINT),
                                       transitions={'done': '20_move_null'},
                                       autonomy={'done': Autonomy.Off})

            # x:1183 y:475
            OperatableStateMachine.add('18_log_failed',
                                       LogState(text='Error in the previous move', severity=Logger.REPORT_HINT),
                                       transitions={'done': '21_move_goat'},
                                       autonomy={'done': Autonomy.Off})

            # x:794 y:268
            OperatableStateMachine.add('19_log_finished',
                                       LogState(text='Everything has crossed the river', severity=Logger.REPORT_HINT),
                                       transitions={'done': 'finished'},
                                       autonomy={'done': Autonomy.Off})

            # x:268 y:8
            OperatableStateMachine.add('1_log_failed',
                                       LogState(text='Error in the previous move', severity=Logger.REPORT_HINT),
                                       transitions={'done': '3_move_goat'},
                                       autonomy={'done': Autonomy.Off})

            # x:910 y:657
            OperatableStateMachine.add('20_move_null',
                                       MoveState(item=None, tracker='farmer'),
                                       transitions={'done': '16_move_goat', 'canceled': '17_log_failed', 'failed': '17_log_failed'},
                                       autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:984 y:320
            OperatableStateMachine.add('21_move_goat',
                                       MoveState(item='goat', tracker='farmer'),
                                       transitions={'done': '19_log_finished', 'canceled': '18_log_failed', 'failed': '18_log_failed'},
                                       autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:30 y:164
            OperatableStateMachine.add('2_move_null',
                                       MoveState(item=None, tracker='farmer'),
                                       transitions={'done': '4_move_wolf', 'canceled': '5_log_failed', 'failed': '5_log_failed'},
                                       autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:398 y:27
            OperatableStateMachine.add('3_move_goat',
                                       MoveState(item='goat', tracker='farmer'),
                                       transitions={'done': '2_move_null', 'canceled': '1_log_failed', 'failed': '1_log_failed'},
                                       autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:31 y:270
            OperatableStateMachine.add('4_move_wolf',
                                       MoveState(item='wolf', tracker='farmer'),
                                       transitions={'done': '7_move_goat', 'canceled': '6_log_failed', 'failed': '6_log_failed'},
                                       autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:317 y:147
            OperatableStateMachine.add('5_log_failed',
                                       LogState(text='Error in the previous move', severity=Logger.REPORT_HINT),
                                       transitions={'done': '8_move_null'},
                                       autonomy={'done': Autonomy.Off})

            # x:325 y:303
            OperatableStateMachine.add('6_log_failed',
                                       LogState(text='Error in the previous move', severity=Logger.REPORT_HINT),
                                       transitions={'done': '9_move_wolf'},
                                       autonomy={'done': Autonomy.Off})

            # x:32 y:428
            OperatableStateMachine.add('7_move_goat',
                                       MoveState(item='goat', tracker='farmer'),
                                       transitions={'done': '10_move_corn', 'canceled': '11_log_failed', 'failed': '11_log_failed'},
                                       autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:429 y:210
            OperatableStateMachine.add('8_move_null',
                                       MoveState(item=None, tracker='farmer'),
                                       transitions={'done': '4_move_wolf', 'canceled': '5_log_failed', 'failed': '5_log_failed'},
                                       autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:434 y:344
            OperatableStateMachine.add('9_move_wolf',
                                       MoveState(item='wolf', tracker='farmer'),
                                       transitions={'done': '7_move_goat', 'canceled': '6_log_failed', 'failed': '6_log_failed'},
                                       autonomy={'done': Autonomy.Off, 'canceled': Autonomy.Off, 'failed': Autonomy.Off},
                                       remapping={'left': 'left', 'right': 'right'})

        with _state_machine:
            # x:30 y:40
            OperatableStateMachine.add('Container',
                                       _sm_container_0,
                                       transitions={'failed': 'failed', 'finished': 'finished'},
                                       autonomy={'failed': Autonomy.Inherit, 'finished': Autonomy.Inherit},
                                       remapping={'left': 'left', 'right': 'right'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
