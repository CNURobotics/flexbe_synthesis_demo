#!/usr/bin/env python3
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
Define synthesis_demo.

Default setup for synthesis demonstration.

Created on Wed May 06 2026
@author: Conner
"""


from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_core import initialize_flexbe_core
from flexbe_states.log_state import LogState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class synthesis_demoSM(Behavior):
    """
    Define synthesis_demo.

    Default setup for synthesis demonstration.
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'synthesis_demo'

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
        # finished:x:1275 y:80, failed:x:1275 y:160
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.left = ['wolf', 'goat', 'corn', 'farmer']
        _state_machine.userdata.right = []
        _state_machine.userdata.middle = []

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        # finished:x:1275 y:80, failed:x:1275 y:160
        _sm_rivercrossing_0 = OperatableStateMachine(outcomes=['finished', 'failed'])

        with _sm_rivercrossing_0:
            # x:144 y:74
            OperatableStateMachine.add('Temp',
                                       LogState(text="Temporary state to be replaced by synthesis",
                                                severity=2),
                                       transitions={'done': 'finished'},
                                       autonomy={'done': Autonomy.Off})

        # finished:x:1275 y:80, failed:x:1275 y:160
        _sm_rivercrossing_1 = OperatableStateMachine(outcomes=['finished', 'failed'])

        with _sm_rivercrossing_1:
            # x:284 y:90
            OperatableStateMachine.add('RiverCrossing',
                                       _sm_rivercrossing_0,
                                       transitions={'finished': 'finished', 'failed': 'failed'},
                                       autonomy={'finished': Autonomy.Inherit,
                                                 'failed': Autonomy.Inherit})

        with _state_machine:
            # x:220 y:62
            OperatableStateMachine.add('RiverCrossing',
                                       _sm_rivercrossing_1,
                                       transitions={'finished': 'finished', 'failed': 'failed'},
                                       autonomy={'finished': Autonomy.Inherit,
                                                 'failed': Autonomy.Inherit})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
