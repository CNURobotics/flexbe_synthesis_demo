#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2026 David Conner
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
Define drone_synthesis_enumerated.

Behavior synthesized for the drone synthesis demonstration using enumerated
encoding

Created on Fri Aug 14 2026
@author: David Conner
"""


from drone_demo_flexbe_behaviors.gate_obstacle_avoidance_state import GateObstacleAvoidanceState
from drone_demo_flexbe_behaviors.gate_trajectory_boundary_state import GateTrajectoryBoundaryState
from flexbe_core import Autonomy
from flexbe_core import Behavior
from flexbe_core import ConcurrencyContainer
from flexbe_core import Logger
from flexbe_core import OperatableStateMachine
from flexbe_core import PriorityContainer
from flexbe_core import initialize_flexbe_core
from flexbe_states.log_state import LogState
from flexbe_states.operator_decision_state import OperatorDecisionState
from flexible_drones_flexbe_states.arm_state import ArmState
from flexible_drones_flexbe_states.execute_trajectory_state import ExecuteTrajectoryState
from flexible_drones_flexbe_states.get_trajectory_state import GetTrajectoryState
from flexible_drones_flexbe_states.go_to_state import GoToState
from flexible_drones_flexbe_states.initialize_trajectory_state import InitializeTrajectoryState
from flexible_drones_flexbe_states.land_state import LandState
from flexible_drones_flexbe_states.set_trajectory_obstacles_state import SetTrajectoryObstaclesState
from flexible_drones_flexbe_states.takeoff_state import TakeoffState
from flexible_drones_flexbe_states.upload_trajectory_state import UploadTrajectoryState
from flexible_drones_flexbe_states.visualize_trajectory_state import VisualizeTrajectoryState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class drone_synthesis_enumeratedSM(Behavior):
    """
    Define drone_synthesis_enumerated.

    Behavior synthesized for the drone synthesis demonstration using enumerated
    encoding
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'drone_synthesis_enumerated'

        # parameters of this behavior
        self.add_parameter('drone_name', 'cf1')
        self.add_parameter('gate_radius', '0.9')
        self.add_parameter('gate_distance', '1.5')

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
        # finished:x:1607 y:103, failed:x:1607 y:206
        _state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
        _state_machine.userdata.group_mask = 0
        _state_machine.userdata.takeoff_height = 1.0
        _state_machine.userdata.takeoff_duration_sec = 2.0
        _state_machine.userdata.frame = 0
        _state_machine.userdata.home_position = [0.0, 0.0, 1.0]
        _state_machine.userdata.go_home_duration_sec = 2.0
        _state_machine.userdata.timescale = 1.0
        _state_machine.userdata.reversed = False
        _state_machine.userdata.relative = False
        _state_machine.userdata.landing_height = 0.05
        _state_machine.userdata.landing_duration_sec = 2.0
        _state_machine.userdata.gate_radius = self.gate_radius
        _state_machine.userdata.gate_distance = self.gate_distance

        # Additional creation code can be added inside the following tags
        # [MANUAL_CREATE]


        # [/MANUAL_CREATE]

        # failed:x:10476 y:2668, finished:x:10476 y:3458
        _sm_dronesynthesis_0 = OperatableStateMachine(outcomes=['failed', 'finished'],
                                                      input_keys=['group_mask', 'takeoff_height', 'takeoff_duration_sec', 'frame', 'home_position', 'go_home_duration_sec', 'timescale', 'reversed', 'relative', 'landing_height', 'landing_duration_sec', 'gate_radius', 'gate_distance'])

        with _sm_dronesynthesis_0:
            # x:206 y:321
            OperatableStateMachine.add('1_home_boundary',
                                       GateTrajectoryBoundaryState(gate_id='home',
                                                                   cruise_velocity=0.5,
                                                                   home_position=[0.0, 0.0, 1.0]),
                                       transitions={'done': '3_initialize_trajectory'  # 404 352 412 352 475 352
                                                    , 'failed': '1_home_boundary'  # 309 251 309 383 309 321
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'boundary': 'home'})

            # x:10152 y:3616
            OperatableStateMachine.add('100_log_landing_failed',
                                       LogState(text='Landing failed',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10290 3202 10314 3647 10470 2697
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:9857 y:3580
            OperatableStateMachine.add('101_disarm',
                                       ArmState(arm=False,
                                                service_name=None,
                                                namespace=self.drone_name,
                                                wait_timeout=5.0,
                                                call_timeout=10.0,
                                                request_timeout=0.0),
                                       transitions={'succeeded': '108_log_finished'  # 10074 3555 10057 3622 10170 3489
                                                    , 'failed': '107_log_disarm_failed'  # 10051 3466 10057 3601 10154 3331
                                                    , 'unavailable': '107_log_disarm_failed'  # 10051 3466 10057 3601 10154 3331
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:10122 y:3142
            OperatableStateMachine.add('105_log_recovery_landing_failed',
                                       LogState(text='Recovery landing failed',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10342 2951 10340 3173 10470 2692
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:9857 y:2960
            OperatableStateMachine.add('106_disarm',
                                       ArmState(arm=False,
                                                service_name=None,
                                                namespace=self.drone_name,
                                                wait_timeout=5.0,
                                                call_timeout=10.0,
                                                request_timeout=0.0),
                                       transitions={'succeeded': '113_log_recovery_finished'  # 10059 3008 10057 3002 10142 3015
                                                    , 'failed': '112_log_disarm_failed'  # 10066 2919 10057 2981 10154 2857
                                                    , 'unavailable': '112_log_disarm_failed'  # 10066 2919 10057 2981 10154 2857
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:10154 y:3300
            OperatableStateMachine.add('107_log_disarm_failed',
                                       LogState(text='Drone landed but did not disarm',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10323 3037 10311 3331 10470 2695
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:10170 y:3458
            OperatableStateMachine.add('108_log_finished',
                                       LogState(text='Four-gate flight completed and drone disarmed',
                                                severity=2),
                                       transitions={'done': 'finished'  # 10340 3482 10297 3489 10462 3475
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:1716 y:321
            OperatableStateMachine.add('10_gate4_boundary',
                                       GateTrajectoryBoundaryState(gate_id='C2R',
                                                                   cruise_velocity=0.5,
                                                                   home_position=[0.0, 0.0, 1.0]),
                                       transitions={'done': '12_gate3_boundary'  # 1917 352 1922 352 1992 352
                                                    , 'failed': '10_gate4_boundary'  # 1819 251 1819 383 1819 321
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'boundary': 'gate4'})

            # x:10154 y:2826
            OperatableStateMachine.add('112_log_disarm_failed',
                                       LogState(text='Drone landed but did not disarm',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10350 2779 10311 2857 10470 2688
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:10142 y:2984
            OperatableStateMachine.add('113_log_recovery_finished',
                                       LogState(text='Disarmed after recovery landing',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10357 2865 10324 3015 10470 2690
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:1992 y:321
            OperatableStateMachine.add('12_gate3_boundary',
                                       GateTrajectoryBoundaryState(gate_id='D1',
                                                                   cruise_velocity=0.5,
                                                                   home_position=[0.0, 0.0, 1.0]),
                                       transitions={'done': '14_gate1_boundary'  # 2193 352 2198 352 2268 352
                                                    , 'failed': '12_gate3_boundary'  # 2095 251 2095 383 2095 321
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'boundary': 'gate3'})

            # x:2268 y:321
            OperatableStateMachine.add('14_gate1_boundary',
                                       GateTrajectoryBoundaryState(gate_id='A1',
                                                                   cruise_velocity=0.5,
                                                                   home_position=[0.0, 0.0, 1.0]),
                                       transitions={'done': '16_plan_gate1'  # 2469 352 2474 352 2544 352
                                                    , 'failed': '14_gate1_boundary'  # 2371 251 2371 383 2371 321
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'boundary': 'gate1'})

            # x:2544 y:321
            OperatableStateMachine.add('16_plan_gate1',
                                       GetTrajectoryState(action_topic='/plan_trajectory',
                                                          timeout=180.0,
                                                          timeout_buffer=5.0),
                                       transitions={'done': '18_gate1_gate2_avoidance'  # 2690 462 2744 342 2789 581
                                                    , 'failed': '17_log_plan_gate1_failed'  # 6404 978 2744 363 10145 1593
                                                    , 'timeout': '17_log_plan_gate1_failed'  # 6404 978 2744 363 10145 1593
                                                    },
                                       autonomy={'done': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'trajectory': 'trajectory',
                                                  'start': 'home',
                                                  'target': 'gate1',
                                                  'new_start': 'new_start',
                                                  'trajectory_duration_sec': 'trajectory_duration_sec'})

            # x:10144 y:1562
            OperatableStateMachine.add('17_log_plan_gate1_failed',
                                       LogState(text='Failed to plan home to gate 1 trajectory segment',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10290 2089 10319 1593 10470 2669
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:2789 y:550
            OperatableStateMachine.add('18_gate1_gate2_avoidance',
                                       GateObstacleAvoidanceState(start_gate_id='A1',
                                                                  target_gate_id='B2R'),
                                       transitions={'done': '21_set_gate1_gate2_avoidance'  # 2931 1105 2995 581 3111 1629
                                                    , 'failed': '18_gate1_gate2_avoidance'  # 2892 480 2892 612 2892 550
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'gate_distance': 'gate_distance',
                                                  'avoidance_radius': 'gate_radius',
                                                  'obstacles': 'gate1_gate2_obstacles'})

            # x:3111 y:1598
            OperatableStateMachine.add('21_set_gate1_gate2_avoidance',
                                       SetTrajectoryObstaclesState(obstacles=None,
                                                                   service_name='/set_trajectory_obstacles',
                                                                   namespace=None,
                                                                   wait_timeout=10.0,
                                                                   call_timeout=10.0),
                                       transitions={'succeeded': '25_plan_gate2'  # 3314 2908 3319 1629 3494 4186
                                                    , 'failed': '21_set_gate1_gate2_avoidance'  # 3215 1528 3215 1660 3215 1598
                                                    , 'unavailable': '21_set_gate1_gate2_avoidance'  # 3215 1528 3215 1660 3215 1598
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low},
                                       remapping={'obstacles': 'gate1_gate2_obstacles'})

            # x:3494 y:4155
            OperatableStateMachine.add('25_plan_gate2',
                                       GetTrajectoryState(action_topic='/plan_trajectory',
                                                          timeout=180.0,
                                                          timeout_buffer=5.0),
                                       transitions={'done': '29_gate2_gate3_avoidance'  # 3559 3305 3694 4176 3739 2433
                                                    , 'failed': '28_log_plan_gate2_failed'  # 6879 4080 3694 4197 10145 3963
                                                    , 'timeout': '28_log_plan_gate2_failed'  # 6879 4080 3694 4197 10145 3963
                                                    },
                                       autonomy={'done': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'trajectory': 'trajectory',
                                                  'start': 'gate1',
                                                  'target': 'gate2',
                                                  'new_start': 'new_start',
                                                  'trajectory_duration_sec': 'trajectory_duration_sec'})

            # x:10144 y:3932
            OperatableStateMachine.add('28_log_plan_gate2_failed',
                                       LogState(text='Failed to plan gate 1 to gate 2 trajectory segment',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10290 3375 10319 3963 10470 2702
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:3739 y:2402
            OperatableStateMachine.add('29_gate2_gate3_avoidance',
                                       GateObstacleAvoidanceState(start_gate_id='B2R',
                                                                  target_gate_id='D1'),
                                       transitions={'done': '33_set_gate2_gate3_avoidance'  # 3915 2115 3945 2433 4061 1796
                                                    , 'failed': '29_gate2_gate3_avoidance'  # 3842 2332 3842 2464 3842 2402
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'gate_distance': 'gate_distance',
                                                  'avoidance_radius': 'gate_radius',
                                                  'obstacles': 'gate2_gate3_obstacles'})

            # x:4061 y:1765
            OperatableStateMachine.add('33_set_gate2_gate3_avoidance',
                                       SetTrajectoryObstaclesState(obstacles=None,
                                                                   service_name='/set_trajectory_obstacles',
                                                                   namespace=None,
                                                                   wait_timeout=10.0,
                                                                   call_timeout=10.0),
                                       transitions={'succeeded': '37_plan_gate3'  # 4264 1319 4269 1796 4444 841
                                                    , 'failed': '33_set_gate2_gate3_avoidance'  # 4165 1695 4165 1827 4165 1765
                                                    , 'unavailable': '33_set_gate2_gate3_avoidance'  # 4165 1695 4165 1827 4165 1765
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low},
                                       remapping={'obstacles': 'gate2_gate3_obstacles'})

            # x:4444 y:810
            OperatableStateMachine.add('37_plan_gate3',
                                       GetTrajectoryState(action_topic='/plan_trajectory',
                                                          timeout=180.0,
                                                          timeout_buffer=5.0),
                                       transitions={'done': '41_gate3_gate4_avoidance'  # 4593 940 4644 831 4689 1048
                                                    , 'failed': '40_log_plan_gate3_failed'  # 7354 1301 4644 852 10145 1751
                                                    , 'timeout': '40_log_plan_gate3_failed'  # 7354 1301 4644 852 10145 1751
                                                    },
                                       autonomy={'done': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'trajectory': 'trajectory',
                                                  'start': 'gate2',
                                                  'target': 'gate3',
                                                  'new_start': 'new_start',
                                                  'trajectory_duration_sec': 'trajectory_duration_sec'})

            # x:475 y:321
            OperatableStateMachine.add('3_initialize_trajectory',
                                       InitializeTrajectoryState(trajectory_id=1),
                                       transitions={'done': '4_home_gate1_avoidance'  # 676 352 675 352 757 352
                                                    },
                                       autonomy={'done': Autonomy.Low},
                                       remapping={'trajectory': 'trajectory'})

            # x:10144 y:1720
            OperatableStateMachine.add('40_log_plan_gate3_failed',
                                       LogState(text='Failed to plan gate 2 to gate 3 trajectory segment',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10290 2175 10319 1751 10470 2671
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:4689 y:1017
            OperatableStateMachine.add('41_gate3_gate4_avoidance',
                                       GateObstacleAvoidanceState(start_gate_id='D1',
                                                                  target_gate_id='C2R'),
                                       transitions={'done': '45_set_gate3_gate4_avoidance'  # 4913 1068 4895 1048 5011 1087
                                                    , 'failed': '41_gate3_gate4_avoidance'  # 4792 947 4792 1079 4792 1017
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'gate_distance': 'gate_distance',
                                                  'avoidance_radius': 'gate_radius',
                                                  'obstacles': 'gate3_gate4_obstacles'})

            # x:5011 y:1056
            OperatableStateMachine.add('45_set_gate3_gate4_avoidance',
                                       SetTrajectoryObstaclesState(obstacles=None,
                                                                   service_name='/set_trajectory_obstacles',
                                                                   namespace=None,
                                                                   wait_timeout=10.0,
                                                                   call_timeout=10.0),
                                       transitions={'succeeded': '49_plan_gate4'  # 5267 1144 5219 1087 5394 1201
                                                    , 'failed': '45_set_gate3_gate4_avoidance'  # 5115 986 5115 1118 5115 1056
                                                    , 'unavailable': '45_set_gate3_gate4_avoidance'  # 5115 986 5115 1118 5115 1056
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low},
                                       remapping={'obstacles': 'gate3_gate4_obstacles'})

            # x:5394 y:1170
            OperatableStateMachine.add('49_plan_gate4',
                                       GetTrajectoryState(action_topic='/plan_trajectory',
                                                          timeout=180.0,
                                                          timeout_buffer=5.0),
                                       transitions={'done': '53_gate4_home_avoidance'  # 5536 1327 5594 1191 5639 1462
                                                    , 'failed': '52_log_plan_gate4_failed'  # 7829 1560 5594 1212 10145 1909
                                                    , 'timeout': '52_log_plan_gate4_failed'  # 7829 1560 5594 1212 10145 1909
                                                    },
                                       autonomy={'done': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'trajectory': 'trajectory',
                                                  'start': 'gate3',
                                                  'target': 'gate4',
                                                  'new_start': 'new_start',
                                                  'trajectory_duration_sec': 'trajectory_duration_sec'})

            # x:757 y:321
            OperatableStateMachine.add('4_home_gate1_avoidance',
                                       GateObstacleAvoidanceState(start_gate_id='home',
                                                                  target_gate_id='A1'),
                                       transitions={'done': '6_set_home_gate1_avoidance'  # 978 352 963 352 1072 352
                                                    , 'failed': '4_home_gate1_avoidance'  # 860 251 860 383 860 321
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'gate_distance': 'gate_distance',
                                                  'avoidance_radius': 'gate_radius',
                                                  'obstacles': 'home_gate1_obstacles'})

            # x:10144 y:1878
            OperatableStateMachine.add('52_log_plan_gate4_failed',
                                       LogState(text='Failed to plan gate 3 to gate 4 trajectory segment',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10308 2261 10319 1909 10470 2674
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:5639 y:1431
            OperatableStateMachine.add('53_gate4_home_avoidance',
                                       GateObstacleAvoidanceState(start_gate_id='C2R',
                                                                  target_gate_id='home'),
                                       transitions={'done': '57_set_gate4_home_avoidance'  # 5864 1486 5845 1462 5962 1510
                                                    , 'failed': '53_gate4_home_avoidance'  # 5742 1361 5742 1493 5742 1431
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'gate_distance': 'gate_distance',
                                                  'avoidance_radius': 'gate_radius',
                                                  'obstacles': 'gate4_home_obstacles'})

            # x:5962 y:1479
            OperatableStateMachine.add('57_set_gate4_home_avoidance',
                                       SetTrajectoryObstaclesState(obstacles=None,
                                                                   service_name='/set_trajectory_obstacles',
                                                                   namespace=None,
                                                                   wait_timeout=10.0,
                                                                   call_timeout=10.0),
                                       transitions={'succeeded': '61_plan_home'  # 6218 1557 6171 1510 6345 1604
                                                    , 'failed': '57_set_gate4_home_avoidance'  # 6066 1409 6066 1541 6066 1479
                                                    , 'unavailable': '57_set_gate4_home_avoidance'  # 6066 1409 6066 1541 6066 1479
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low},
                                       remapping={'obstacles': 'gate4_home_obstacles'})

            # x:6345 y:1573
            OperatableStateMachine.add('61_plan_home',
                                       GetTrajectoryState(action_topic='/plan_trajectory',
                                                          timeout=180.0,
                                                          timeout_buffer=5.0),
                                       transitions={'done': '65_visualize_trajectory'  # 6495 1764 6545 1594 6606 1934
                                                    , 'failed': '64_log_plan_home_failed'  # 8305 1841 6545 1615 10144 2067
                                                    , 'timeout': '64_log_plan_home_failed'  # 8305 1841 6545 1615 10144 2067
                                                    },
                                       autonomy={'done': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'trajectory': 'trajectory',
                                                  'start': 'gate4',
                                                  'target': 'home',
                                                  'new_start': 'new_start',
                                                  'trajectory_duration_sec': 'trajectory_duration_sec'})

            # x:10144 y:2036
            OperatableStateMachine.add('64_log_plan_home_failed',
                                       LogState(text='Failed to plan gate 4 to home trajectory segment',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10327 2348 10319 2067 10470 2676
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:6606 y:1903
            OperatableStateMachine.add('65_visualize_trajectory',
                                       VisualizeTrajectoryState(topic='/planned_trajectory',
                                                                frame_id='map',
                                                                offset=[0.0, 0.0, 0.0],
                                                                sample_dt=0.05),
                                       transitions={'done': '69_approve_trajectory'  # 6813 1934 6806 1934 6899 1934
                                                    , 'failed': '65_visualize_trajectory'  # 6706 1833 6706 1965 6706 1903
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'trajectory': 'trajectory'})

            # x:6899 y:1903
            OperatableStateMachine.add('69_approve_trajectory',
                                       OperatorDecisionState(outcomes=['execute', 'cancel'],
                                                             hint='Execute the planned four-gate trajectory?',
                                                             suggestion='execute'),
                                       transitions={'execute': '73_upload_trajectory'  # 7092 2103 7058 1945 7206 2261
                                                    , 'cancel': '72_log_approval_cancelled'  # 8559 2075 7058 1924 10140 2225
                                                    },
                                       autonomy={'execute': Autonomy.High, 'cancel': Autonomy.Full})

            # x:1072 y:321
            OperatableStateMachine.add('6_set_home_gate1_avoidance',
                                       SetTrajectoryObstaclesState(obstacles=None,
                                                                   service_name='/set_trajectory_obstacles',
                                                                   namespace=None,
                                                                   wait_timeout=10.0,
                                                                   call_timeout=10.0),
                                       transitions={'succeeded': '8_gate2_boundary'  # 1321 352 1274 352 1448 352
                                                    , 'failed': '6_set_home_gate1_avoidance'  # 1173 251 1173 383 1173 321
                                                    , 'unavailable': '6_set_home_gate1_avoidance'  # 1173 251 1173 383 1173 321
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low},
                                       remapping={'obstacles': 'home_gate1_obstacles'})

            # x:10140 y:2194
            OperatableStateMachine.add('72_log_approval_cancelled',
                                       LogState(text='Operator cancelled the planned trajectory',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10345 2434 10325 2225 10470 2678
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:7206 y:2230
            OperatableStateMachine.add('73_upload_trajectory',
                                       UploadTrajectoryState(timeout=15.0,
                                                             action_topic=None,
                                                             namespace=self.drone_name),
                                       transitions={'succeeded': '77_arm'  # 7343 3124 7406 2272 7523 3976
                                                    , 'aborted': '76_log_upload_failed'  # 8742 2317 7406 2251 10158 2383
                                                    , 'timeout': '76_log_upload_failed'  # 8742 2317 7406 2251 10158 2383
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'aborted': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'trajectory': 'trajectory',
                                                  'uploaded_pieces': 'uploaded_pieces'})

            # x:10158 y:2352
            OperatableStateMachine.add('76_log_upload_failed',
                                       LogState(text='Failed to upload trajectory',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10349 2520 10309 2383 10470 2681
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:7523 y:3945
            OperatableStateMachine.add('77_arm',
                                       ArmState(arm=True,
                                                service_name=None,
                                                namespace=self.drone_name,
                                                wait_timeout=5.0,
                                                call_timeout=10.0,
                                                request_timeout=0.0),
                                       transitions={'succeeded': '81_takeoff'  # 7593 3512 7723 3966 7772 3058
                                                    , 'failed': '80_log_arm_failed'  # 8905 3896 7723 3987 10166 3805
                                                    , 'unavailable': '80_log_arm_failed'  # 8905 3896 7723 3987 10166 3805
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:10166 y:3774
            OperatableStateMachine.add('80_log_arm_failed',
                                       LogState(text='Failed to arm drone',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10290 3288 10300 3805 10470 2700
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:7772 y:3027
            OperatableStateMachine.add('81_takeoff',
                                       TakeoffState(timeout=15.0,
                                                    action_topic=None,
                                                    namespace=self.drone_name),
                                       transitions={'succeeded': '85_go_home'  # 8117 3072 7972 3069 8342 3069
                                                    , 'aborted': '84_confirm_takeoff_failure'  # 7916 2963 7972 3048 8006 2878
                                                    , 'timeout': '84_confirm_takeoff_failure'  # 7916 2963 7972 3048 8006 2878
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'aborted': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'group_mask': 'group_mask',
                                                  'height': 'takeoff_height',
                                                  'duration_sec': 'takeoff_duration_sec'})

            # x:8006 y:2847
            OperatableStateMachine.add('84_confirm_takeoff_failure',
                                       OperatorDecisionState(outcomes=['continue', 'abort'],
                                                             hint='Confirm the drone is safely airborne before continuing.',
                                                             suggestion='abort'),
                                       transitions={'continue': '85_go_home'  # 8225 2965 8188 2889 8342 3048
                                                    , 'abort': '88_log_takeoff_failed'  # 9133 2705 8188 2868 10158 2541
                                                    },
                                       autonomy={'continue': Autonomy.Low, 'abort': Autonomy.Low})

            # x:8342 y:3027
            OperatableStateMachine.add('85_go_home',
                                       GoToState(timeout=15.0,
                                                 action_topic=None,
                                                 namespace=self.drone_name),
                                       transitions={'succeeded': '90_execute_trajectory'  # 8712 3119 8542 3069 8962 3176
                                                    , 'aborted': '89_confirm_go_home_failure'  # 8505 3117 8542 3048 8592 3186
                                                    , 'timeout': '89_confirm_go_home_failure'  # 8505 3117 8542 3048 8592 3186
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'aborted': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'group_mask': 'group_mask',
                                                  'frame': 'frame',
                                                  'goal': 'home_position',
                                                  'duration_sec': 'go_home_duration_sec'})

            # x:10158 y:2510
            OperatableStateMachine.add('88_log_takeoff_failed',
                                       LogState(text='Takeoff was not confirmed safe by the operator',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10349 2606 10309 2541 10470 2683
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:8592 y:3155
            OperatableStateMachine.add('89_confirm_go_home_failure',
                                       OperatorDecisionState(outcomes=['continue', 'abort'],
                                                             hint='Confirm the drone is safe at the route start before continuing.',
                                                             suggestion='abort'),
                                       transitions={'continue': '90_execute_trajectory'  # 8835 3200 8788 3197 8962 3197
                                                    , 'abort': '93_log_go_home_failed'  # 9429 2938 8788 3176 10150 2699
                                                    },
                                       autonomy={'continue': Autonomy.Low, 'abort': Autonomy.Low})

            # x:1448 y:321
            OperatableStateMachine.add('8_gate2_boundary',
                                       GateTrajectoryBoundaryState(gate_id='B2R',
                                                                   cruise_velocity=0.5,
                                                                   home_position=[0.0, 0.0, 1.0]),
                                       transitions={'done': '10_gate4_boundary'  # 1645 352 1654 352 1716 352
                                                    , 'failed': '8_gate2_boundary'  # 1551 251 1551 383 1551 321
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'boundary': 'gate2'})

            # x:8962 y:3155
            OperatableStateMachine.add('90_execute_trajectory',
                                       ExecuteTrajectoryState(timeout=30.0,
                                                              action_topic=None,
                                                              namespace=self.drone_name,
                                                              trajectory_id=None),
                                       transitions={'succeeded': '95_land'  # 9335 3436 9162 3197 9587 3675
                                                    , 'aborted': '94_log_execution_failed'  # 9174 3158 9162 3176 9265 3139
                                                    , 'timeout': '94_log_execution_failed'  # 9174 3158 9162 3176 9265 3139
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'aborted': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'group_mask': 'group_mask',
                                                  'trajectory': 'trajectory',
                                                  'timescale': 'timescale',
                                                  'reversed': 'reversed',
                                                  'relative': 'relative'})

            # x:10150 y:2668
            OperatableStateMachine.add('93_log_go_home_failed',
                                       LogState(text='Route-start position was not confirmed safe by the operator',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10352 2692 10315 2699 10470 2685
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:9265 y:3108
            OperatableStateMachine.add('94_log_execution_failed',
                                       LogState(text='Trajectory execution failed; attempting recovery landing',
                                                severity=2),
                                       transitions={'done': '99_recover_land'  # 9459 3139 9433 3139 9566 3139
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:9587 y:3644
            OperatableStateMachine.add('95_land',
                                       LandState(timeout=15.0,
                                                 action_topic=None,
                                                 namespace=self.drone_name),
                                       transitions={'succeeded': '101_disarm'  # 9781 3638 9787 3665 9857 3611
                                                    , 'aborted': '100_log_landing_failed'  # 9930 3666 9787 3686 10152 3647
                                                    , 'timeout': '100_log_landing_failed'  # 9930 3666 9787 3686 10152 3647
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'aborted': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'group_mask': 'group_mask',
                                                  'height': 'landing_height',
                                                  'duration_sec': 'landing_duration_sec'})

            # x:9566 y:3108
            OperatableStateMachine.add('99_recover_land',
                                       LandState(timeout=15.0,
                                                 action_topic=None,
                                                 namespace=self.drone_name),
                                       transitions={'succeeded': '106_disarm'  # 9770 3060 9766 3129 9857 2991
                                                    , 'aborted': '105_log_recovery_landing_failed'  # 9904 3161 9766 3150 10123 3173
                                                    , 'timeout': '105_log_recovery_landing_failed'  # 9904 3161 9766 3150 10123 3173
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'aborted': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'group_mask': 'group_mask',
                                                  'height': 'landing_height',
                                                  'duration_sec': 'landing_duration_sec'})

        with _state_machine:
            # x:30 y:40
            OperatableStateMachine.add('DroneSynthesis',
                                       _sm_dronesynthesis_0,
                                       transitions={'failed': 'failed', 'finished': 'finished'},
                                       autonomy={'failed': Autonomy.Inherit,
                                                 'finished': Autonomy.Inherit},
                                       remapping={'group_mask': 'group_mask',
                                                  'takeoff_height': 'takeoff_height',
                                                  'takeoff_duration_sec': 'takeoff_duration_sec',
                                                  'frame': 'frame',
                                                  'home_position': 'home_position',
                                                  'go_home_duration_sec': 'go_home_duration_sec',
                                                  'timescale': 'timescale',
                                                  'reversed': 'reversed',
                                                  'relative': 'relative',
                                                  'landing_height': 'landing_height',
                                                  'landing_duration_sec': 'landing_duration_sec',
                                                  'gate_radius': 'gate_radius',
                                                  'gate_distance': 'gate_distance'})

        return _state_machine

    # Private functions can be added inside the following tags
    # [MANUAL_FUNC]


    # [/MANUAL_FUNC]
