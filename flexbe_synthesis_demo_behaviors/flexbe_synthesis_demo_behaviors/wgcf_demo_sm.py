#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2023 David Conner
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
Define wgcf_demo.

Created on Sun Oct 15 2023
@author: David Conner
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer

from flexbe_synthesis_demo_flexbe_states.move_state import MoveState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


class wgcf_demoSM(Behavior):
    """
    Define wgcf_demo.

    Hand crafted demo

    """

    def __init__(self, node):
        """Initialize."""
        super().__init__()
        self.name = 'wgcf_demo'

        # parameters of this behavior

        # references to used behaviors
        OperatableStateMachine.initialize_ros(node)
        ConcurrencyContainer.initialize_ros(node)
        PriorityContainer.initialize_ros(node)
        Logger.initialize(node)
        MoveState.initialize_ros(node)

        # Additional initialization code can be added inside the following tags
        # [MANUAL_INIT]

        # [/MANUAL_INIT]

        # Behavior comments:

    def create(self):
        """Create behavior."""
        # x:262 y:31, x:266 y:113
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.left = ['wolf', 'goat', 'corn', 'farmer']
        _state_machine.userdata.right = []

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]

        # [/MANUAL_CREATE]
        # x:1222 y:109, x:626 y:289
        _sm_wgcf_0 = OperatableStateMachine(outcomes=['finished', 'failed'],
                                            input_keys=['left', 'right'],
                                            output_keys=['left', 'right'])

        with _sm_wgcf_0:
            # x:65 y:88
            OperatableStateMachine.add('move_goat_1',
                                       MoveState(item='goat', tracker='farmer'),
                                       transitions={'done': 'move_farmer_1', 'canceled': 'move_goat_1', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.High},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:158 y:238
            OperatableStateMachine.add('move_farmer_1',
                                       MoveState(item=None, tracker='farmer'),
                                       transitions={'done': 'move_wolf_1', 'canceled': 'move_farmer_1', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.High},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:756 y:342
            OperatableStateMachine.add('move_farmer_2',
                                       MoveState(item=None, tracker='farmer'),
                                       transitions={'done': 'move_goat_3', 'canceled': 'move_farmer_2', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.High},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:292 y:572
            OperatableStateMachine.add('move_goat_2',
                                       MoveState(item='goat', tracker='farmer'),
                                       transitions={'done': 'move_corn_1', 'canceled': 'move_goat_2', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.High},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:705 y:129
            OperatableStateMachine.add('move_goat_3',
                                       MoveState(item='goat', tracker='farmer'),
                                       transitions={'done': 'finished', 'canceled': 'move_goat_3', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.High},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:231 y:420
            OperatableStateMachine.add('move_wolf_1',
                                       MoveState(item='wolf', tracker='farmer'),
                                       transitions={'done': 'move_goat_2', 'canceled': 'move_wolf_1', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.High},
                                       remapping={'left': 'left', 'right': 'right'})

            # x:612 y:506
            OperatableStateMachine.add('move_corn_1',
                                       MoveState(item='corn', tracker='farmer'),
                                       transitions={'done': 'move_farmer_2', 'canceled': 'move_corn_1', 'failed': 'failed'},
                                       autonomy={'done': Autonomy.High, 'canceled': Autonomy.High, 'failed': Autonomy.High},
                                       remapping={'left': 'left', 'right': 'right'})

        with _state_machine:
            # x:30 y:40
            OperatableStateMachine.add('WGCF',
                                       _sm_wgcf_0,
                                       transitions={'finished': 'finished', 'failed': 'failed'},
                                       autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
                                       remapping={'left': 'left', 'right': 'right'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]

    # [/MANUAL_FUNC]
