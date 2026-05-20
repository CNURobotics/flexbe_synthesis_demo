#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2026 Christopher Newport University
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
Define two_rivers_parsed.

Synthesis generated demo

Created on Friday 11-July-2025
@author: David Conner
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_core import initialize_flexbe_core
from flexbe_states.log_state import LogState
from flexbe_river_crossing_flexbe_states.move_state_2r import MoveState2R

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class two_rivers_parsedSM(Behavior):
    """
    Define two_rivers_parsed.

    Synthesis generated demo
    """

    def __init__(self, node):
        """Initialize."""
        super().__init__()
        self.name = 'two_rivers_parsed'

        # parameters of this behavior

        # Initialize ROS node information
        initialize_flexbe_core(node)

        # references to used behaviors

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        """Create state machine."""
        # Root state machine
        # x:30 y:365, x:130 y:365
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.left = ['farmer', 'wolf', 'goat', 'corn']
        _state_machine.userdata.right = []
        _state_machine.userdata.middle = []

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]

        # x:8786 y:981, x:8786 y:823
        _sm_tworivers_0 = OperatableStateMachine(outcomes=['finished', 'failed'],
                                                 input_keys=['left', 'right', 'middle'],
                                                 output_keys=['left', 'right', 'middle'])

        with _sm_tworivers_0:
            # x:236 y:313
            OperatableStateMachine.add('0_move_state',
                                       MoveState2R(item='goat',
                                                   tracker='farmer',
                                                   destination='middle'),
                                       transitions={'done': '2_move_state',
                                                    'canceled': '1_log_failed',
                                                    'failed': '1_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:2954 y:305
            OperatableStateMachine.add('10_log_failed',
                                       LogState(text='Error in the previous move',
                                                severity=Logger.REPORT_HINT),
                                       transitions={'done': '12_move_state'},
                                       autonomy={'done': Autonomy.Off})

            # x:3506 y:477
            OperatableStateMachine.add('11_move_state',
                                       MoveState2R(item='None',
                                                   tracker='farmer',
                                                   destination='middle'),
                                       transitions={'done': '14_move_state',
                                                    'canceled': '13_log_failed',
                                                    'failed': '13_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:3241 y:339
            OperatableStateMachine.add('12_move_state',
                                       MoveState2R(item='wolf',
                                                   tracker='farmer',
                                                   destination='right'),
                                       transitions={'done': '11_move_state',
                                                    'canceled': '10_log_failed',
                                                    'failed': '10_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:3793 y:389
            OperatableStateMachine.add('13_log_failed',
                                       LogState(text='Error in the previous move',
                                                severity=Logger.REPORT_HINT),
                                       transitions={'done': '15_move_state'},
                                       autonomy={'done': Autonomy.Off})

            # x:4345 y:561
            OperatableStateMachine.add('14_move_state',
                                       MoveState2R(item='None',
                                                   tracker='farmer',
                                                   destination='left'),
                                       transitions={'done': '17_move_state',
                                                    'canceled': '16_log_failed',
                                                    'failed': '16_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:4080 y:423
            OperatableStateMachine.add('15_move_state',
                                       MoveState2R(item='None',
                                                   tracker='farmer',
                                                   destination='middle'),
                                       transitions={'done': '14_move_state',
                                                    'canceled': '13_log_failed',
                                                    'failed': '13_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:4632 y:473
            OperatableStateMachine.add('16_log_failed',
                                       LogState(text='Error in the previous move',
                                                severity=Logger.REPORT_HINT),
                                       transitions={'done': '18_move_state'},
                                       autonomy={'done': Autonomy.Off})

            # x:5184 y:645
            OperatableStateMachine.add('17_move_state',
                                       MoveState2R(item='corn',
                                                   tracker='farmer',
                                                   destination='middle'),
                                       transitions={'done': '20_move_state',
                                                    'canceled': '19_log_failed',
                                                    'failed': '19_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:4919 y:507
            OperatableStateMachine.add('18_move_state',
                                       MoveState2R(item='None',
                                                   tracker='farmer',
                                                   destination='left'),
                                       transitions={'done': '17_move_state',
                                                    'canceled': '16_log_failed',
                                                    'failed': '16_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:5471 y:557
            OperatableStateMachine.add('19_log_failed',
                                       LogState(text='Error in the previous move',
                                                severity=Logger.REPORT_HINT),
                                       transitions={'done': '21_move_state'},
                                       autonomy={'done': Autonomy.Off})

            # x:514 y:225
            OperatableStateMachine.add('1_log_failed',
                                       LogState(text='Error in the previous move',
                                                severity=Logger.REPORT_HINT),
                                       transitions={'done': '3_move_state'},
                                       autonomy={'done': Autonomy.Off})

            # x:6023 y:729
            OperatableStateMachine.add('20_move_state',
                                       MoveState2R(item='corn',
                                                   tracker='farmer',
                                                   destination='right'),
                                       transitions={'done': '22_move_state',
                                                    'canceled': '23_log_failed',
                                                    'failed': '23_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:5758 y:591
            OperatableStateMachine.add('21_move_state',
                                       MoveState2R(item='corn',
                                                   tracker='farmer',
                                                   destination='middle'),
                                       transitions={'done': '20_move_state',
                                                    'canceled': '19_log_failed',
                                                    'failed': '19_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:6862 y:813
            OperatableStateMachine.add('22_move_state',
                                       MoveState2R(item='None',
                                                   tracker='farmer',
                                                   destination='middle'),
                                       transitions={'done': '25_move_state',
                                                    'canceled': '24_log_failed',
                                                    'failed': '24_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:6310 y:641
            OperatableStateMachine.add('23_log_failed',
                                       LogState(text='Error in the previous move',
                                                severity=Logger.REPORT_HINT),
                                       transitions={'done': '26_move_state'},
                                       autonomy={'done': Autonomy.Off})

            # x:7149 y:725
            OperatableStateMachine.add('24_log_failed',
                                       LogState(text='Error in the previous move',
                                                severity=Logger.REPORT_HINT),
                                       transitions={'done': '27_move_state'},
                                       autonomy={'done': Autonomy.Off})

            # x:7701 y:897
            OperatableStateMachine.add('25_move_state',
                                       MoveState2R(item='goat',
                                                   tracker='farmer',
                                                   destination='right'),
                                       transitions={'done': '29_log_finished',
                                                    'canceled': '28_log_failed',
                                                    'failed': '28_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:6597 y:675
            OperatableStateMachine.add('26_move_state',
                                       MoveState2R(item='corn',
                                                   tracker='farmer',
                                                   destination='right'),
                                       transitions={'done': '22_move_state',
                                                    'canceled': '23_log_failed',
                                                    'failed': '23_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:7436 y:759
            OperatableStateMachine.add('27_move_state',
                                       MoveState2R(item='None',
                                                   tracker='farmer',
                                                   destination='middle'),
                                       transitions={'done': '25_move_state',
                                                    'canceled': '24_log_failed',
                                                    'failed': '24_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:7988 y:809
            OperatableStateMachine.add('28_log_failed',
                                       LogState(text='Error in the previous move',
                                                severity=Logger.REPORT_HINT),
                                       transitions={'done': '30_move_state'},
                                       autonomy={'done': Autonomy.Off})

            # x:8542 y:981
            OperatableStateMachine.add('29_log_finished',
                                       LogState(text='Everything has crossed to the far right bank',
                                                severity=Logger.REPORT_HINT),
                                       transitions={'done': 'finished'},
                                       autonomy={'done': Autonomy.Off})

            # x:1048 y:397
            OperatableStateMachine.add('2_move_state',
                                       MoveState2R(item='None',
                                                   tracker='farmer',
                                                   destination='left'),
                                       transitions={'done': '5_move_state',
                                                    'canceled': '4_log_failed',
                                                    'failed': '4_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:8275 y:843
            OperatableStateMachine.add('30_move_state',
                                       MoveState2R(item='goat',
                                                   tracker='farmer',
                                                   destination='right'),
                                       transitions={'done': '29_log_finished',
                                                    'canceled': '28_log_failed',
                                                    'failed': '28_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:792 y:259
            OperatableStateMachine.add('3_move_state',
                                       MoveState2R(item='goat',
                                                   tracker='farmer',
                                                   destination='middle'),
                                       transitions={'done': '2_move_state',
                                                    'canceled': '1_log_failed',
                                                    'failed': '1_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:1326 y:525
            OperatableStateMachine.add('4_log_failed',
                                       LogState(text='Error in the previous move',
                                                severity=Logger.REPORT_HINT),
                                       transitions={'done': '6_move_state'},
                                       autonomy={'done': Autonomy.Off})

            # x:1860 y:395
            OperatableStateMachine.add('5_move_state',
                                       MoveState2R(item='wolf',
                                                   tracker='farmer',
                                                   destination='middle'),
                                       transitions={'done': '8_move_state',
                                                    'canceled': '7_log_failed',
                                                    'failed': '7_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:1604 y:517
            OperatableStateMachine.add('6_move_state',
                                       MoveState2R(item='None',
                                                   tracker='farmer',
                                                   destination='left'),
                                       transitions={'done': '5_move_state',
                                                    'canceled': '4_log_failed',
                                                    'failed': '4_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:2138 y:523
            OperatableStateMachine.add('7_log_failed',
                                       LogState(text='Error in the previous move',
                                                severity=Logger.REPORT_HINT),
                                       transitions={'done': '9_move_state'},
                                       autonomy={'done': Autonomy.Off})

            # x:2672 y:393
            OperatableStateMachine.add('8_move_state',
                                       MoveState2R(item='wolf',
                                                   tracker='farmer',
                                                   destination='right'),
                                       transitions={'done': '11_move_state',
                                                    'canceled': '10_log_failed',
                                                    'failed': '10_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

            # x:2416 y:515
            OperatableStateMachine.add('9_move_state',
                                       MoveState2R(item='wolf',
                                                   tracker='farmer',
                                                   destination='middle'),
                                       transitions={'done': '8_move_state',
                                                    'canceled': '7_log_failed',
                                                    'failed': '7_log_failed'},
                                       autonomy={'done': Autonomy.Off,
                                                 'canceled': Autonomy.Off,
                                                 'failed': Autonomy.Off},
                                       remapping={'left': 'left',
                                                  'middle': 'middle',
                                                  'right': 'right'})

        with _state_machine:
            # x:30 y:40
            OperatableStateMachine.add('TwoRivers',
                                       _sm_tworivers_0,
                                       transitions={'finished': 'finished', 'failed': 'failed'},
                                       autonomy={'finished': Autonomy.Inherit,
                                                 'failed': Autonomy.Inherit},
                                       remapping={'left': 'left',
                                                  'right': 'right',
                                                  'middle': 'middle'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
