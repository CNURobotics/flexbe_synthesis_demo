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
Define drone_synthesis_one_hot.

Starter behavior for the portable drone synthesis demonstration.

Created on Fri Jul 24 2026
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
from flexible_drones_flexbe_states.set_trajectory_bounds_state import SetTrajectoryBoundsState
from flexible_drones_flexbe_states.set_trajectory_obstacles_state import SetTrajectoryObstaclesState
from flexible_drones_flexbe_states.takeoff_state import TakeoffState
from flexible_drones_flexbe_states.upload_trajectory_state import UploadTrajectoryState
from flexible_drones_flexbe_states.visualize_trajectory_state import VisualizeTrajectoryState

# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]


# [/MANUAL_IMPORT]


class drone_synthesis_one_hotSM(Behavior):
    """
    Define drone_synthesis_one_hot.

    Starter behavior for the portable drone synthesis demonstration.
    """

    def __init__(self, node):
        super().__init__()
        self.name = 'drone_synthesis_one_hot'

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
        # finished:x:2566 y:103, failed:x:2566 y:207
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

        # failed:x:10280 y:60, finished:x:10280 y:320
        _sm_dronesynthesis_0 = OperatableStateMachine(outcomes=['failed', 'finished'],
                                                      input_keys=['group_mask', 'takeoff_height', 'takeoff_duration_sec', 'frame', 'home_position', 'go_home_duration_sec', 'timescale', 'reversed', 'relative', 'landing_height', 'landing_duration_sec', 'gate_radius', 'gate_distance'])

        with _sm_dronesynthesis_0:
            # x:80 y:60
            OperatableStateMachine.add('1_initialize_trajectory',
                                       InitializeTrajectoryState(trajectory_id=1),
                                       transitions={'done': '2_home_gate1_avoidance'  # 290 91 280 91 380 91
                                                    },
                                       autonomy={'done': Autonomy.Low},
                                       remapping={'trajectory': 'trajectory'})

            # x:8480 y:60
            OperatableStateMachine.add('101_confirm_go_home_failure',
                                       OperatorDecisionState(outcomes=['continue', 'abort'],
                                                             hint='Confirm the drone is safe at the route start before continuing.',
                                                             suggestion='abort'),
                                       transitions={'continue': '102_execute_trajectory'  # 8691 94 8683 81 8780 101
                                                    , 'abort': '104_log_go_home_failed'  # 8690 184 8683 101 8780 266
                                                    },
                                       autonomy={'continue': Autonomy.Low, 'abort': Autonomy.Low})

            # x:8780 y:60
            OperatableStateMachine.add('102_execute_trajectory',
                                       ExecuteTrajectoryState(timeout=30.0,
                                                              action_topic=None,
                                                              namespace=self.drone_name,
                                                              trajectory_id=None),
                                       transitions={'succeeded': '107_land'  # 8990 184 8980 101 9080 266
                                                    , 'aborted': '106_log_execution_failed'  # 8990 86 8980 81 9080 91
                                                    , 'timeout': '106_log_execution_failed'  # 8990 86 8980 81 9080 91
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'aborted': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'group_mask': 'group_mask',
                                                  'trajectory': 'trajectory',
                                                  'timescale': 'timescale',
                                                  'reversed': 'reversed',
                                                  'relative': 'relative'})

            # x:8780 y:235
            OperatableStateMachine.add('104_log_go_home_failed',
                                       LogState(text='Route-start position was not confirmed safe by the operator',
                                                severity=2),
                                       transitions={'done': 'failed'  # 9573 193 8952 266 10273 84
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:9080 y:60
            OperatableStateMachine.add('106_log_execution_failed',
                                       LogState(text='Trajectory execution failed; attempting recovery landing',
                                                severity=2),
                                       transitions={'done': '110_recover_land'  # 9277 91 9254 91 9380 91
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:9080 y:235
            OperatableStateMachine.add('107_land',
                                       LandState(timeout=15.0,
                                                 action_topic=None,
                                                 namespace=self.drone_name),
                                       transitions={'succeeded': '112_disarm'  # 9290 261 9280 256 9380 266
                                                    , 'aborted': '111_log_landing_failed'  # 9290 359 9280 276 9380 441
                                                    , 'timeout': '111_log_landing_failed'  # 9290 359 9280 276 9380 441
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'aborted': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'group_mask': 'group_mask',
                                                  'height': 'landing_height',
                                                  'duration_sec': 'landing_duration_sec'})

            # x:9380 y:60
            OperatableStateMachine.add('110_recover_land',
                                       LandState(timeout=15.0,
                                                 action_topic=None,
                                                 namespace=self.drone_name),
                                       transitions={'succeeded': '115_disarm'  # 9590 86 9580 81 9680 91
                                                    , 'aborted': '114_log_recovery_landing_failed'  # 9590 184 9580 101 9680 266
                                                    , 'timeout': '114_log_recovery_landing_failed'  # 9590 184 9580 101 9680 266
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'aborted': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'group_mask': 'group_mask',
                                                  'height': 'landing_height',
                                                  'duration_sec': 'landing_duration_sec'})

            # x:9380 y:410
            OperatableStateMachine.add('111_log_landing_failed',
                                       LogState(text='Landing failed',
                                                severity=2),
                                       transitions={'done': 'failed'  # 9867 302 9540 441 10273 91
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:9380 y:235
            OperatableStateMachine.add('112_disarm',
                                       ArmState(arm=False,
                                                service_name=None,
                                                namespace=self.drone_name,
                                                wait_timeout=5.0,
                                                call_timeout=10.0,
                                                request_timeout=0.0),
                                       transitions={'succeeded': '118_log_finished'  # 9569 446 9580 276 9680 616
                                                    , 'failed': '117_log_disarm_failed'  # 9588 348 9580 256 9680 441
                                                    , 'unavailable': '117_log_disarm_failed'  # 9588 348 9580 256 9680 441
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:9680 y:235
            OperatableStateMachine.add('114_log_recovery_landing_failed',
                                       LogState(text='Recovery landing failed',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10045 200 9897 266 10273 87
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:9680 y:60
            OperatableStateMachine.add('115_disarm',
                                       ArmState(arm=False,
                                                service_name=None,
                                                namespace=self.drone_name,
                                                wait_timeout=5.0,
                                                call_timeout=10.0,
                                                request_timeout=0.0),
                                       transitions={'succeeded': '121_log_recovery_finished'  # 9890 184 9880 101 9980 266
                                                    , 'failed': '120_log_disarm_failed'  # 9890 86 9880 81 9980 91
                                                    , 'unavailable': '120_log_disarm_failed'  # 9890 86 9880 81 9980 91
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:9680 y:410
            OperatableStateMachine.add('117_log_disarm_failed',
                                       LogState(text='Drone landed but did not disarm',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10015 309 9837 441 10273 94
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:9680 y:585
            OperatableStateMachine.add('118_log_finished',
                                       LogState(text='Four-gate flight completed and drone disarmed',
                                                severity=2),
                                       transitions={'done': 'finished'  # 9996 477 9806 616 10266 337
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:980 y:235
            OperatableStateMachine.add('11_set_planner_bounds',
                                       SetTrajectoryBoundsState(x_min=-3.0,
                                                                x_max=3.0,
                                                                y_min=-3.0,
                                                                y_max=3.0,
                                                                z_min=0.48,
                                                                z_max=3.0,
                                                                service_name='/set_trajectory_bounds',
                                                                namespace=None,
                                                                wait_timeout=10.0,
                                                                call_timeout=10.0),
                                       transitions={'succeeded': '8_home_boundary'  # 1082 219 1080 235 1083 122
                                                    , 'failed': '11_set_planner_bounds'  # 1060 161 1080 297 1047 235
                                                    , 'unavailable': '11_set_planner_bounds'  # 1060 161 1080 297 1047 235
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:9980 y:60
            OperatableStateMachine.add('120_log_disarm_failed',
                                       LogState(text='Drone landed but did not disarm',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10166 34 10138 91 10273 61
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:9980 y:235
            OperatableStateMachine.add('121_log_recovery_finished',
                                       LogState(text='Disarmed after recovery landing',
                                                severity=2),
                                       transitions={'done': 'failed'  # 10178 207 10163 266 10273 89
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:1280 y:60
            OperatableStateMachine.add('12_gate4_boundary',
                                       GateTrajectoryBoundaryState(gate_id='C2R',
                                                                   cruise_velocity=0.5,
                                                                   home_position=[0.0, 0.0, 1.0]),
                                       transitions={'done': '16_gate3_boundary'  # 1493 91 1486 91 1580 91
                                                    , 'failed': '15_set_planner_bounds'  # 1401 139 1383 122 1413 235
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'boundary': 'gate4'})

            # x:1280 y:235
            OperatableStateMachine.add('15_set_planner_bounds',
                                       SetTrajectoryBoundsState(x_min=-3.0,
                                                                x_max=3.0,
                                                                y_min=-3.0,
                                                                y_max=3.0,
                                                                z_min=0.48,
                                                                z_max=3.0,
                                                                service_name='/set_trajectory_bounds',
                                                                namespace=None,
                                                                wait_timeout=10.0,
                                                                call_timeout=10.0),
                                       transitions={'succeeded': '12_gate4_boundary'  # 1382 219 1380 235 1383 122
                                                    , 'failed': '15_set_planner_bounds'  # 1360 161 1380 297 1347 235
                                                    , 'unavailable': '15_set_planner_bounds'  # 1360 161 1380 297 1347 235
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:1580 y:60
            OperatableStateMachine.add('16_gate3_boundary',
                                       GateTrajectoryBoundaryState(gate_id='D1',
                                                                   cruise_velocity=0.5,
                                                                   home_position=[0.0, 0.0, 1.0]),
                                       transitions={'done': '20_gate2_boundary'  # 1793 91 1786 91 1880 91
                                                    , 'failed': '19_set_planner_bounds'  # 1701 139 1683 122 1713 235
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'boundary': 'gate3'})

            # x:1580 y:235
            OperatableStateMachine.add('19_set_planner_bounds',
                                       SetTrajectoryBoundsState(x_min=-3.0,
                                                                x_max=3.0,
                                                                y_min=-3.0,
                                                                y_max=3.0,
                                                                z_min=0.48,
                                                                z_max=3.0,
                                                                service_name='/set_trajectory_bounds',
                                                                namespace=None,
                                                                wait_timeout=10.0,
                                                                call_timeout=10.0),
                                       transitions={'succeeded': '16_gate3_boundary'  # 1682 219 1680 235 1683 122
                                                    , 'failed': '19_set_planner_bounds'  # 1660 161 1680 297 1647 235
                                                    , 'unavailable': '19_set_planner_bounds'  # 1660 161 1680 297 1647 235
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:1880 y:60
            OperatableStateMachine.add('20_gate2_boundary',
                                       GateTrajectoryBoundaryState(gate_id='B2R',
                                                                   cruise_velocity=0.5,
                                                                   home_position=[0.0, 0.0, 1.0]),
                                       transitions={'done': '24_gate1_boundary'  # 2093 91 2086 91 2180 91
                                                    , 'failed': '23_set_planner_bounds'  # 2001 139 1983 122 2013 235
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'boundary': 'gate2'})

            # x:1880 y:235
            OperatableStateMachine.add('23_set_planner_bounds',
                                       SetTrajectoryBoundsState(x_min=-3.0,
                                                                x_max=3.0,
                                                                y_min=-3.0,
                                                                y_max=3.0,
                                                                z_min=0.48,
                                                                z_max=3.0,
                                                                service_name='/set_trajectory_bounds',
                                                                namespace=None,
                                                                wait_timeout=10.0,
                                                                call_timeout=10.0),
                                       transitions={'succeeded': '20_gate2_boundary'  # 1982 219 1980 235 1983 122
                                                    , 'failed': '23_set_planner_bounds'  # 1960 161 1980 297 1947 235
                                                    , 'unavailable': '23_set_planner_bounds'  # 1960 161 1980 297 1947 235
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:2180 y:60
            OperatableStateMachine.add('24_gate1_boundary',
                                       GateTrajectoryBoundaryState(gate_id='A1',
                                                                   cruise_velocity=0.5,
                                                                   home_position=[0.0, 0.0, 1.0]),
                                       transitions={'done': '28_plan_gate1'  # 2393 91 2386 91 2480 91
                                                    , 'failed': '27_set_planner_bounds'  # 2301 139 2283 122 2313 235
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'boundary': 'gate1'})

            # x:2180 y:235
            OperatableStateMachine.add('27_set_planner_bounds',
                                       SetTrajectoryBoundsState(x_min=-3.0,
                                                                x_max=3.0,
                                                                y_min=-3.0,
                                                                y_max=3.0,
                                                                z_min=0.48,
                                                                z_max=3.0,
                                                                service_name='/set_trajectory_bounds',
                                                                namespace=None,
                                                                wait_timeout=10.0,
                                                                call_timeout=10.0),
                                       transitions={'succeeded': '24_gate1_boundary'  # 2282 219 2280 235 2283 122
                                                    , 'failed': '27_set_planner_bounds'  # 2260 161 2280 297 2247 235
                                                    , 'unavailable': '27_set_planner_bounds'  # 2260 161 2280 297 2247 235
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:2480 y:60
            OperatableStateMachine.add('28_plan_gate1',
                                       GetTrajectoryState(action_topic='/plan_trajectory',
                                                          timeout=180.0,
                                                          timeout_buffer=5.0),
                                       transitions={'done': '32_gate1_gate2_avoidance'  # 2690 86 2680 81 2780 91
                                                    , 'failed': '31_log_plan_gate1_failed'  # 2690 184 2680 101 2780 266
                                                    , 'timeout': '31_log_plan_gate1_failed'  # 2690 184 2680 101 2780 266
                                                    },
                                       autonomy={'done': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'trajectory': 'trajectory',
                                                  'start': 'home',
                                                  'target': 'gate1',
                                                  'new_start': 'new_start',
                                                  'trajectory_duration_sec': 'trajectory_duration_sec'})

            # x:380 y:60
            OperatableStateMachine.add('2_home_gate1_avoidance',
                                       GateObstacleAvoidanceState(start_gate_id='home',
                                                                  target_gate_id='A1'),
                                       transitions={'done': '4_set_home_gate1_avoidance'  # 593 91 586 91 680 91
                                                    , 'failed': '3_set_planner_bounds'  # 501 139 483 122 513 235
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'gate_distance': 'gate_distance',
                                                  'avoidance_radius': 'gate_radius',
                                                  'obstacles': 'home_gate1_obstacles'})

            # x:2780 y:235
            OperatableStateMachine.add('31_log_plan_gate1_failed',
                                       LogState(text='Failed to plan home to gate 1 trajectory segment',
                                                severity=2),
                                       transitions={'done': 'failed'  # 6574 128 2954 266 10273 63
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:2780 y:60
            OperatableStateMachine.add('32_gate1_gate2_avoidance',
                                       GateObstacleAvoidanceState(start_gate_id='A1',
                                                                  target_gate_id='B2R'),
                                       transitions={'done': '35_set_gate1_gate2_avoidance'  # 2993 91 2986 91 3080 91
                                                    , 'failed': '34_set_planner_bounds'  # 2901 226 2883 122 2913 410
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'gate_distance': 'gate_distance',
                                                  'avoidance_radius': 'gate_radius',
                                                  'obstacles': 'gate1_gate2_obstacles'})

            # x:2780 y:410
            OperatableStateMachine.add('34_set_planner_bounds',
                                       SetTrajectoryBoundsState(x_min=-3.0,
                                                                x_max=3.0,
                                                                y_min=-3.0,
                                                                y_max=3.0,
                                                                z_min=0.48,
                                                                z_max=3.0,
                                                                service_name='/set_trajectory_bounds',
                                                                namespace=None,
                                                                wait_timeout=10.0,
                                                                call_timeout=10.0),
                                       transitions={'succeeded': '32_gate1_gate2_avoidance'  # 2882 306 2880 410 2883 122
                                                    , 'failed': '34_set_planner_bounds'  # 2860 336 2880 472 2847 410
                                                    , 'unavailable': '34_set_planner_bounds'  # 2860 336 2880 472 2847 410
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:3080 y:60
            OperatableStateMachine.add('35_set_gate1_gate2_avoidance',
                                       SetTrajectoryObstaclesState(obstacles=None,
                                                                   service_name='/set_trajectory_obstacles',
                                                                   namespace=None,
                                                                   wait_timeout=10.0,
                                                                   call_timeout=10.0),
                                       transitions={'succeeded': '40_plan_gate2'  # 3294 91 3288 91 3380 91
                                                    , 'failed': '39_set_planner_bounds'  # 3202 139 3184 122 3213 235
                                                    , 'unavailable': '39_set_planner_bounds'  # 3202 139 3184 122 3213 235
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low},
                                       remapping={'obstacles': 'gate1_gate2_obstacles'})

            # x:3080 y:235
            OperatableStateMachine.add('39_set_planner_bounds',
                                       SetTrajectoryBoundsState(x_min=-3.0,
                                                                x_max=3.0,
                                                                y_min=-3.0,
                                                                y_max=3.0,
                                                                z_min=0.48,
                                                                z_max=3.0,
                                                                service_name='/set_trajectory_bounds',
                                                                namespace=None,
                                                                wait_timeout=10.0,
                                                                call_timeout=10.0),
                                       transitions={'succeeded': '35_set_gate1_gate2_avoidance'  # 3182 219 3180 235 3184 122
                                                    , 'failed': '39_set_planner_bounds'  # 3160 161 3180 297 3147 235
                                                    , 'unavailable': '39_set_planner_bounds'  # 3160 161 3180 297 3147 235
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:380 y:235
            OperatableStateMachine.add('3_set_planner_bounds',
                                       SetTrajectoryBoundsState(x_min=-3.0,
                                                                x_max=3.0,
                                                                y_min=-3.0,
                                                                y_max=3.0,
                                                                z_min=0.48,
                                                                z_max=3.0,
                                                                service_name='/set_trajectory_bounds',
                                                                namespace=None,
                                                                wait_timeout=10.0,
                                                                call_timeout=10.0),
                                       transitions={'succeeded': '2_home_gate1_avoidance'  # 482 219 480 235 483 122
                                                    , 'failed': '3_set_planner_bounds'  # 460 161 480 297 447 235
                                                    , 'unavailable': '3_set_planner_bounds'  # 460 161 480 297 447 235
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:3380 y:60
            OperatableStateMachine.add('40_plan_gate2',
                                       GetTrajectoryState(action_topic='/plan_trajectory',
                                                          timeout=180.0,
                                                          timeout_buffer=5.0),
                                       transitions={'done': '44_gate2_gate3_avoidance'  # 3590 86 3580 81 3680 91
                                                    , 'failed': '43_log_plan_gate2_failed'  # 3590 184 3580 101 3680 266
                                                    , 'timeout': '43_log_plan_gate2_failed'  # 3590 184 3580 101 3680 266
                                                    },
                                       autonomy={'done': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'trajectory': 'trajectory',
                                                  'start': 'gate1',
                                                  'target': 'gate2',
                                                  'new_start': 'new_start',
                                                  'trajectory_duration_sec': 'trajectory_duration_sec'})

            # x:3680 y:235
            OperatableStateMachine.add('43_log_plan_gate2_failed',
                                       LogState(text='Failed to plan gate 1 to gate 2 trajectory segment',
                                                severity=2),
                                       transitions={'done': 'failed'  # 7024 136 3854 266 10273 65
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:3680 y:60
            OperatableStateMachine.add('44_gate2_gate3_avoidance',
                                       GateObstacleAvoidanceState(start_gate_id='B2R',
                                                                  target_gate_id='D1'),
                                       transitions={'done': '47_set_gate2_gate3_avoidance'  # 3893 91 3886 91 3980 91
                                                    , 'failed': '46_set_planner_bounds'  # 3801 226 3783 122 3813 410
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'gate_distance': 'gate_distance',
                                                  'avoidance_radius': 'gate_radius',
                                                  'obstacles': 'gate2_gate3_obstacles'})

            # x:3680 y:410
            OperatableStateMachine.add('46_set_planner_bounds',
                                       SetTrajectoryBoundsState(x_min=-3.0,
                                                                x_max=3.0,
                                                                y_min=-3.0,
                                                                y_max=3.0,
                                                                z_min=0.48,
                                                                z_max=3.0,
                                                                service_name='/set_trajectory_bounds',
                                                                namespace=None,
                                                                wait_timeout=10.0,
                                                                call_timeout=10.0),
                                       transitions={'succeeded': '44_gate2_gate3_avoidance'  # 3782 306 3780 410 3783 122
                                                    , 'failed': '46_set_planner_bounds'  # 3760 336 3780 472 3747 410
                                                    , 'unavailable': '46_set_planner_bounds'  # 3760 336 3780 472 3747 410
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:3980 y:60
            OperatableStateMachine.add('47_set_gate2_gate3_avoidance',
                                       SetTrajectoryObstaclesState(obstacles=None,
                                                                   service_name='/set_trajectory_obstacles',
                                                                   namespace=None,
                                                                   wait_timeout=10.0,
                                                                   call_timeout=10.0),
                                       transitions={'succeeded': '52_plan_gate3'  # 4194 91 4188 91 4280 91
                                                    , 'failed': '51_set_planner_bounds'  # 4102 139 4084 122 4113 235
                                                    , 'unavailable': '51_set_planner_bounds'  # 4102 139 4084 122 4113 235
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low},
                                       remapping={'obstacles': 'gate2_gate3_obstacles'})

            # x:680 y:60
            OperatableStateMachine.add('4_set_home_gate1_avoidance',
                                       SetTrajectoryObstaclesState(obstacles=None,
                                                                   service_name='/set_trajectory_obstacles',
                                                                   namespace=None,
                                                                   wait_timeout=10.0,
                                                                   call_timeout=10.0),
                                       transitions={'succeeded': '8_home_boundary'  # 891 91 882 91 980 91
                                                    , 'failed': '7_set_planner_bounds'  # 800 139 781 122 813 235
                                                    , 'unavailable': '7_set_planner_bounds'  # 800 139 781 122 813 235
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low},
                                       remapping={'obstacles': 'home_gate1_obstacles'})

            # x:3980 y:235
            OperatableStateMachine.add('51_set_planner_bounds',
                                       SetTrajectoryBoundsState(x_min=-3.0,
                                                                x_max=3.0,
                                                                y_min=-3.0,
                                                                y_max=3.0,
                                                                z_min=0.48,
                                                                z_max=3.0,
                                                                service_name='/set_trajectory_bounds',
                                                                namespace=None,
                                                                wait_timeout=10.0,
                                                                call_timeout=10.0),
                                       transitions={'succeeded': '47_set_gate2_gate3_avoidance'  # 4082 219 4080 235 4084 122
                                                    , 'failed': '51_set_planner_bounds'  # 4060 161 4080 297 4047 235
                                                    , 'unavailable': '51_set_planner_bounds'  # 4060 161 4080 297 4047 235
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:4280 y:60
            OperatableStateMachine.add('52_plan_gate3',
                                       GetTrajectoryState(action_topic='/plan_trajectory',
                                                          timeout=180.0,
                                                          timeout_buffer=5.0),
                                       transitions={'done': '56_gate3_gate4_avoidance'  # 4490 86 4480 81 4580 91
                                                    , 'failed': '55_log_plan_gate3_failed'  # 4490 184 4480 101 4580 266
                                                    , 'timeout': '55_log_plan_gate3_failed'  # 4490 184 4480 101 4580 266
                                                    },
                                       autonomy={'done': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'trajectory': 'trajectory',
                                                  'start': 'gate2',
                                                  'target': 'gate3',
                                                  'new_start': 'new_start',
                                                  'trajectory_duration_sec': 'trajectory_duration_sec'})

            # x:4580 y:235
            OperatableStateMachine.add('55_log_plan_gate3_failed',
                                       LogState(text='Failed to plan gate 2 to gate 3 trajectory segment',
                                                severity=2),
                                       transitions={'done': 'failed'  # 7474 143 4754 266 10273 68
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:4580 y:60
            OperatableStateMachine.add('56_gate3_gate4_avoidance',
                                       GateObstacleAvoidanceState(start_gate_id='D1',
                                                                  target_gate_id='C2R'),
                                       transitions={'done': '59_set_gate3_gate4_avoidance'  # 4793 91 4786 91 4880 91
                                                    , 'failed': '58_set_planner_bounds'  # 4701 226 4683 122 4713 410
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'gate_distance': 'gate_distance',
                                                  'avoidance_radius': 'gate_radius',
                                                  'obstacles': 'gate3_gate4_obstacles'})

            # x:4580 y:410
            OperatableStateMachine.add('58_set_planner_bounds',
                                       SetTrajectoryBoundsState(x_min=-3.0,
                                                                x_max=3.0,
                                                                y_min=-3.0,
                                                                y_max=3.0,
                                                                z_min=0.48,
                                                                z_max=3.0,
                                                                service_name='/set_trajectory_bounds',
                                                                namespace=None,
                                                                wait_timeout=10.0,
                                                                call_timeout=10.0),
                                       transitions={'succeeded': '56_gate3_gate4_avoidance'  # 4682 306 4680 410 4683 122
                                                    , 'failed': '58_set_planner_bounds'  # 4660 336 4680 472 4647 410
                                                    , 'unavailable': '58_set_planner_bounds'  # 4660 336 4680 472 4647 410
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:4880 y:60
            OperatableStateMachine.add('59_set_gate3_gate4_avoidance',
                                       SetTrajectoryObstaclesState(obstacles=None,
                                                                   service_name='/set_trajectory_obstacles',
                                                                   namespace=None,
                                                                   wait_timeout=10.0,
                                                                   call_timeout=10.0),
                                       transitions={'succeeded': '64_plan_gate4'  # 5094 91 5088 91 5180 91
                                                    , 'failed': '63_set_planner_bounds'  # 5002 139 4984 122 5013 235
                                                    , 'unavailable': '63_set_planner_bounds'  # 5002 139 4984 122 5013 235
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low},
                                       remapping={'obstacles': 'gate3_gate4_obstacles'})

            # x:4880 y:235
            OperatableStateMachine.add('63_set_planner_bounds',
                                       SetTrajectoryBoundsState(x_min=-3.0,
                                                                x_max=3.0,
                                                                y_min=-3.0,
                                                                y_max=3.0,
                                                                z_min=0.48,
                                                                z_max=3.0,
                                                                service_name='/set_trajectory_bounds',
                                                                namespace=None,
                                                                wait_timeout=10.0,
                                                                call_timeout=10.0),
                                       transitions={'succeeded': '59_set_gate3_gate4_avoidance'  # 4982 219 4980 235 4984 122
                                                    , 'failed': '63_set_planner_bounds'  # 4960 161 4980 297 4947 235
                                                    , 'unavailable': '63_set_planner_bounds'  # 4960 161 4980 297 4947 235
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:5180 y:60
            OperatableStateMachine.add('64_plan_gate4',
                                       GetTrajectoryState(action_topic='/plan_trajectory',
                                                          timeout=180.0,
                                                          timeout_buffer=5.0),
                                       transitions={'done': '68_gate4_home_avoidance'  # 5390 86 5380 81 5480 91
                                                    , 'failed': '67_log_plan_gate4_failed'  # 5390 184 5380 101 5480 266
                                                    , 'timeout': '67_log_plan_gate4_failed'  # 5390 184 5380 101 5480 266
                                                    },
                                       autonomy={'done': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'trajectory': 'trajectory',
                                                  'start': 'gate3',
                                                  'target': 'gate4',
                                                  'new_start': 'new_start',
                                                  'trajectory_duration_sec': 'trajectory_duration_sec'})

            # x:5480 y:235
            OperatableStateMachine.add('67_log_plan_gate4_failed',
                                       LogState(text='Failed to plan gate 3 to gate 4 trajectory segment',
                                                severity=2),
                                       transitions={'done': 'failed'  # 7924 150 5654 266 10273 70
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:5480 y:60
            OperatableStateMachine.add('68_gate4_home_avoidance',
                                       GateObstacleAvoidanceState(start_gate_id='C2R',
                                                                  target_gate_id='home'),
                                       transitions={'done': '71_set_gate4_home_avoidance'  # 5693 91 5686 91 5780 91
                                                    , 'failed': '70_set_planner_bounds'  # 5601 226 5583 122 5613 410
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'gate_distance': 'gate_distance',
                                                  'avoidance_radius': 'gate_radius',
                                                  'obstacles': 'gate4_home_obstacles'})

            # x:5480 y:410
            OperatableStateMachine.add('70_set_planner_bounds',
                                       SetTrajectoryBoundsState(x_min=-3.0,
                                                                x_max=3.0,
                                                                y_min=-3.0,
                                                                y_max=3.0,
                                                                z_min=0.48,
                                                                z_max=3.0,
                                                                service_name='/set_trajectory_bounds',
                                                                namespace=None,
                                                                wait_timeout=10.0,
                                                                call_timeout=10.0),
                                       transitions={'succeeded': '68_gate4_home_avoidance'  # 5582 306 5580 410 5583 122
                                                    , 'failed': '70_set_planner_bounds'  # 5560 336 5580 472 5547 410
                                                    , 'unavailable': '70_set_planner_bounds'  # 5560 336 5580 472 5547 410
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:5780 y:60
            OperatableStateMachine.add('71_set_gate4_home_avoidance',
                                       SetTrajectoryObstaclesState(obstacles=None,
                                                                   service_name='/set_trajectory_obstacles',
                                                                   namespace=None,
                                                                   wait_timeout=10.0,
                                                                   call_timeout=10.0),
                                       transitions={'succeeded': '76_plan_home'  # 5994 91 5989 91 6080 91
                                                    , 'failed': '75_set_planner_bounds'  # 5902 139 5884 122 5913 235
                                                    , 'unavailable': '75_set_planner_bounds'  # 5902 139 5884 122 5913 235
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low},
                                       remapping={'obstacles': 'gate4_home_obstacles'})

            # x:5780 y:235
            OperatableStateMachine.add('75_set_planner_bounds',
                                       SetTrajectoryBoundsState(x_min=-3.0,
                                                                x_max=3.0,
                                                                y_min=-3.0,
                                                                y_max=3.0,
                                                                z_min=0.48,
                                                                z_max=3.0,
                                                                service_name='/set_trajectory_bounds',
                                                                namespace=None,
                                                                wait_timeout=10.0,
                                                                call_timeout=10.0),
                                       transitions={'succeeded': '71_set_gate4_home_avoidance'  # 5882 219 5880 235 5884 122
                                                    , 'failed': '75_set_planner_bounds'  # 5860 161 5880 297 5847 235
                                                    , 'unavailable': '75_set_planner_bounds'  # 5860 161 5880 297 5847 235
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:6080 y:60
            OperatableStateMachine.add('76_plan_home',
                                       GetTrajectoryState(action_topic='/plan_trajectory',
                                                          timeout=180.0,
                                                          timeout_buffer=5.0),
                                       transitions={'done': '80_visualize_trajectory'  # 6290 86 6280 81 6380 91
                                                    , 'failed': '79_log_plan_home_failed'  # 6290 184 6280 101 6380 266
                                                    , 'timeout': '79_log_plan_home_failed'  # 6290 184 6280 101 6380 266
                                                    },
                                       autonomy={'done': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'trajectory': 'trajectory',
                                                  'start': 'gate4',
                                                  'target': 'home',
                                                  'new_start': 'new_start',
                                                  'trajectory_duration_sec': 'trajectory_duration_sec'})

            # x:6380 y:235
            OperatableStateMachine.add('79_log_plan_home_failed',
                                       LogState(text='Failed to plan gate 4 to home trajectory segment',
                                                severity=2),
                                       transitions={'done': 'failed'  # 8374 157 6555 266 10273 72
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:680 y:235
            OperatableStateMachine.add('7_set_planner_bounds',
                                       SetTrajectoryBoundsState(x_min=-3.0,
                                                                x_max=3.0,
                                                                y_min=-3.0,
                                                                y_max=3.0,
                                                                z_min=0.48,
                                                                z_max=3.0,
                                                                service_name='/set_trajectory_bounds',
                                                                namespace=None,
                                                                wait_timeout=10.0,
                                                                call_timeout=10.0),
                                       transitions={'succeeded': '4_set_home_gate1_avoidance'  # 781 219 780 235 781 122
                                                    , 'failed': '7_set_planner_bounds'  # 760 161 780 297 747 235
                                                    , 'unavailable': '7_set_planner_bounds'  # 760 161 780 297 747 235
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:6380 y:60
            OperatableStateMachine.add('80_visualize_trajectory',
                                       VisualizeTrajectoryState(topic='/planned_trajectory',
                                                                frame_id='map',
                                                                offset=[0.0, 0.0, 0.0],
                                                                sample_dt=0.05),
                                       transitions={'done': '83_approve_trajectory'  # 6590 91 6580 91 6680 91
                                                    , 'failed': '80_visualize_trajectory'  # 6480 -10 6480 122 6480 60
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'trajectory': 'trajectory'})

            # x:6680 y:60
            OperatableStateMachine.add('83_approve_trajectory',
                                       OperatorDecisionState(outcomes=['execute', 'cancel'],
                                                             hint='Execute the planned four-gate trajectory?',
                                                             suggestion='execute'),
                                       transitions={'execute': '86_upload_trajectory'  # 6870 86 6839 81 6980 91
                                                    , 'cancel': '85_log_approval_cancelled'  # 6870 184 6839 101 6980 266
                                                    },
                                       autonomy={'execute': Autonomy.High, 'cancel': Autonomy.Full})

            # x:6980 y:235
            OperatableStateMachine.add('85_log_approval_cancelled',
                                       LogState(text='Operator cancelled the planned trajectory',
                                                severity=2),
                                       transitions={'done': 'failed'  # 8679 164 7165 266 10273 75
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:6980 y:60
            OperatableStateMachine.add('86_upload_trajectory',
                                       UploadTrajectoryState(timeout=15.0,
                                                             action_topic=None,
                                                             namespace=self.drone_name),
                                       transitions={'succeeded': '89_arm'  # 7190 86 7180 81 7280 91
                                                    , 'aborted': '88_log_upload_failed'  # 7190 184 7180 101 7280 266
                                                    , 'timeout': '88_log_upload_failed'  # 7190 184 7180 101 7280 266
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'aborted': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'trajectory': 'trajectory',
                                                  'uploaded_pieces': 'uploaded_pieces'})

            # x:7280 y:235
            OperatableStateMachine.add('88_log_upload_failed',
                                       LogState(text='Failed to upload trajectory',
                                                severity=2),
                                       transitions={'done': 'failed'  # 8812 172 7431 266 10273 77
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:7280 y:60
            OperatableStateMachine.add('89_arm',
                                       ArmState(arm=True,
                                                service_name=None,
                                                namespace=self.drone_name,
                                                wait_timeout=5.0,
                                                call_timeout=10.0,
                                                request_timeout=0.0),
                                       transitions={'succeeded': '93_takeoff'  # 7490 86 7480 81 7580 91
                                                    , 'failed': '92_log_arm_failed'  # 7490 184 7480 101 7580 266
                                                    , 'unavailable': '92_log_arm_failed'  # 7490 184 7480 101 7580 266
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'failed': Autonomy.Low,
                                                 'unavailable': Autonomy.Low})

            # x:980 y:60
            OperatableStateMachine.add('8_home_boundary',
                                       GateTrajectoryBoundaryState(gate_id='home',
                                                                   cruise_velocity=0.5,
                                                                   home_position=[0.0, 0.0, 1.0]),
                                       transitions={'done': '12_gate4_boundary'  # 1193 91 1186 91 1280 91
                                                    , 'failed': '11_set_planner_bounds'  # 1101 139 1083 122 1113 235
                                                    },
                                       autonomy={'done': Autonomy.Low, 'failed': Autonomy.Low},
                                       remapping={'boundary': 'home'})

            # x:7580 y:235
            OperatableStateMachine.add('92_log_arm_failed',
                                       LogState(text='Failed to arm drone',
                                                severity=2),
                                       transitions={'done': 'failed'  # 8954 179 7714 266 10273 79
                                                    },
                                       autonomy={'done': Autonomy.Low})

            # x:7580 y:60
            OperatableStateMachine.add('93_takeoff',
                                       TakeoffState(timeout=15.0,
                                                    action_topic=None,
                                                    namespace=self.drone_name),
                                       transitions={'succeeded': '97_go_home'  # 7940 88 7780 101 8180 81
                                                    , 'aborted': '96_confirm_takeoff_failure'  # 7790 86 7780 81 7880 91
                                                    , 'timeout': '96_confirm_takeoff_failure'  # 7790 86 7780 81 7880 91
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'aborted': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'group_mask': 'group_mask',
                                                  'height': 'takeoff_height',
                                                  'duration_sec': 'takeoff_duration_sec'})

            # x:7880 y:60
            OperatableStateMachine.add('96_confirm_takeoff_failure',
                                       OperatorDecisionState(outcomes=['continue', 'abort'],
                                                             hint='Confirm the drone is safely airborne before continuing.',
                                                             suggestion='abort'),
                                       transitions={'continue': '97_go_home'  # 8081 94 8062 81 8180 101
                                                    , 'abort': '99_log_takeoff_failed'  # 8081 184 8062 101 8180 266
                                                    },
                                       autonomy={'continue': Autonomy.Low, 'abort': Autonomy.Low})

            # x:8180 y:60
            OperatableStateMachine.add('97_go_home',
                                       GoToState(timeout=15.0,
                                                 action_topic=None,
                                                 namespace=self.drone_name),
                                       transitions={'succeeded': '102_execute_trajectory'  # 8540 88 8380 101 8780 81
                                                    , 'aborted': '101_confirm_go_home_failure'  # 8390 86 8380 81 8480 91
                                                    , 'timeout': '101_confirm_go_home_failure'  # 8390 86 8380 81 8480 91
                                                    },
                                       autonomy={'succeeded': Autonomy.Low,
                                                 'aborted': Autonomy.Low,
                                                 'timeout': Autonomy.Low},
                                       remapping={'group_mask': 'group_mask',
                                                  'frame': 'frame',
                                                  'goal': 'home_position',
                                                  'duration_sec': 'go_home_duration_sec'})

            # x:8180 y:235
            OperatableStateMachine.add('99_log_takeoff_failed',
                                       LogState(text='Takeoff was not confirmed safe by the operator',
                                                severity=2),
                                       transitions={'done': 'failed'  # 9262 186 8331 266 10273 82
                                                    },
                                       autonomy={'done': Autonomy.Low})

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
