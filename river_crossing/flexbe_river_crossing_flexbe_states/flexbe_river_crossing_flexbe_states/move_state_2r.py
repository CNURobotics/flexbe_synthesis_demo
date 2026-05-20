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

"""FlexBE state for two-river crossing with three banks and two independent boats."""

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher

from flexbe_river_crossing_msgs.msg import WGCFStatus

from rclpy.duration import Duration


class MoveState2R(EventState):
    """
    A state that moves items across one river in the two-river WGCF puzzle.

    Three banks (left, middle, right) are connected by two boats.  Each call
    crosses exactly one river segment (left↔middle or middle↔right).

    Parameters
    -- item         string    The item to be moved with the farmer, or None for farmer alone
    -- tracker      string    Character whose bank position determines origin (typically 'farmer')
    -- destination  string    Explicit target bank ('left', 'middle', 'right'), or None to infer

    Userdata inputs
    ># left     object      List of characters on the left bank
    ># middle   object      List of characters on the middle bank
    ># right    object      List of characters on the right bank

    User data outputs
    #> left     object      List of characters on the left bank
    #> middle   object      List of characters on the middle bank
    #> right    object      List of characters on the right bank

    Outcomes
    <= done      completed  Indicates that item will be moved to the destination.
    <= canceled  failure    Indicates that item will be returned to origin (allow trying again)
    <= failed    failure    Indicates complete failure to exit the behavior.

    """

    def __init__(self, item=None, tracker='farmer', destination=None):
        """Initialize state."""
        super().__init__(outcomes=['done', 'canceled', 'failed'],
                         input_keys=['left', 'middle', 'right'],
                         output_keys=['left', 'middle', 'right'])

        ProxyPublisher.initialize(MoveState2R._node)

        # FlexBE serializes Python None as the string 'None' in generated state machines.
        self._item = None if item in (None, 'None') else item

        self._tracker = tracker
        self._topic = '/wgcf_status'
        self._pub = ProxyPublisher({self._topic: WGCFStatus})
        self._wait_time = Duration(seconds=3.0)
        self._origin = None
        # FlexBE serializes Python None as the string 'None' in generated state machines.
        self._target_destination = None if destination in (None, 'None') else destination
        self._destination = None
        self._passengers0 = []  # For boat 0 (left river)
        self._passengers1 = []  # For boat 1 (right river)
        self._return = None
        self._position0 = 0.0
        self._position1 = 0.0
        self._start_time = None

    def execute(self, userdata):
        """Execute state."""
        if self._return is not None:
            return self._return

        elapsed = self._node.get_clock().now().nanoseconds - self._start_time.nanoseconds
        if elapsed > self._wait_time.nanoseconds:
            Logger.loginfo('  landed on the opposite shore!')
            self._return = 'done'
            return self._return
        progress = elapsed / self._wait_time.nanoseconds

        if self._origin == 'left' and self._destination == 'middle':
            self._position0 = progress
            self._position1 = 0.0
            self._passengers0 = [self._tracker] + ([self._item] if self._item else [])
            self._passengers1 = []
        elif self._origin == 'middle' and self._destination == 'right':
            self._position0 = 1.0
            self._position1 = progress
            self._passengers0 = []
            self._passengers1 = [self._tracker] + ([self._item] if self._item else [])
        elif self._origin == 'middle' and self._destination == 'left':
            self._position0 = 1.0 - progress
            self._position1 = 0.0
            self._passengers0 = [self._tracker] + ([self._item] if self._item else [])
            self._passengers1 = []
        elif self._origin == 'right' and self._destination == 'middle':
            self._position0 = 1.0
            self._position1 = 1.0 - progress
            self._passengers0 = []
            self._passengers1 = [self._tracker] + ([self._item] if self._item else [])
        else:
            Logger.logerr(f'{self._name} - unexpected route {self._origin} to {self._destination}!')
            self._return = 'failed'
            return self._return

        self._publish_update(userdata)
        return None

    def on_enter(self, userdata):
        """
        Attempt to move the item from the origin and to the destination.

        Fails if the origin or destination is not in the userdata
        Fails if the item is not in the origin
        Fails if the tracker that tracks turn is in the destination and not the origin
        """
        self._return = None
        self._passengers0.clear()
        self._passengers1.clear()
        self._origin = None
        self._destination = self._target_destination

        try:
            if self._destination is None:
                if not self._set_auto_route(userdata):
                    return
            else:
                # Use destination from parameter and set origin based on the current location of tracker
                if self._tracker in userdata.left:
                    self._origin = 'left'
                elif self._tracker in userdata.middle:
                    self._origin = 'middle'
                elif self._tracker in userdata.right:
                    self._origin = 'right'
                else:
                    Logger.logerr(f'Cannot start new move - {self._tracker} must be on a bank!')
                    self._return = 'failed'
                    return

                if not self._is_valid_route():
                    Logger.logerr(f'Illegal route {self._origin} -> {self._destination}!')
                    self._return = 'failed'
                    return

            # Verify item is present if specified
            if self._item is not None and self._item not in userdata[self._origin]:
                self._return = 'failed'
                Logger.loginfo(f'Cannot move {self._item} - not with {self._tracker} on {self._origin} bank!')
                return

            # Set initial boat positions
            if self._origin == 'left':
                self._position0 = 0.0
                self._position1 = 0.0
                self._passengers0 = [self._tracker] + ([self._item] if self._item else [])
            elif self._origin == 'middle':
                if self._destination == 'right':
                    self._position0 = 1.0
                    self._position1 = 0.0
                    self._passengers1 = [self._tracker] + ([self._item] if self._item else [])
                else:
                    self._position0 = 1.0
                    self._position1 = 0.0
                    self._passengers0 = [self._tracker] + ([self._item] if self._item else [])
            elif self._origin == 'right':
                self._position0 = 1.0
                self._position1 = 1.0
                self._passengers1 = [self._tracker] + ([self._item] if self._item else [])

            # Remove passengers from origin bank
            userdata[self._origin].remove(self._tracker)
            if self._item:
                userdata[self._origin].remove(self._item)

            Logger.loginfo(f'  Left bank : {userdata.left}')
            Logger.loginfo(f'  Middle bank: {userdata.middle}')
            Logger.loginfo(f'  Right bank: {userdata.right}')

            self._publish_update(userdata)
            self._start_time = self._node.get_clock().now()
            Logger.loginfo('     moving ...')

        except Exception as exc:
            Logger.logerr(f'Failed to move {self._item} due to {exc}!')
            self._return = 'failed'
            Logger.logwarn(f'Userdata:\n{userdata}')

    def on_exit(self, userdata):
        """Update on state exit."""
        try:
            if self._start_time is not None:
                # Only run once after on_enter
                Logger.loginfo('Move is done!')
                if self._return is None:
                    self._return = self._manual_transition_requested
                    Logger.logwarn(f"  Using manually requested outcome '{self._return}'")

                active_passengers = []
                if self._origin == 'left' and self._destination == 'middle':
                    active_passengers = self._passengers0
                    self._position0 = 1.0
                    self._position1 = 0.0
                elif self._origin == 'middle' and self._destination == 'right':
                    active_passengers = self._passengers1
                    self._position0 = 1.0
                    self._position1 = 1.0
                elif self._origin == 'middle' and self._destination == 'left':
                    active_passengers = self._passengers0
                    self._position0 = 0.0
                    self._position1 = 0.0
                elif self._origin == 'right' and self._destination == 'middle':
                    active_passengers = self._passengers1
                    self._position0 = 1.0
                    self._position1 = 0.0

                if self._return == 'failed' or self._return == 'canceled':
                    # Return passengers to origin
                    for p in active_passengers:
                        userdata[self._origin].append(p)
                    Logger.logwarn(f'Returned {active_passengers} to {self._origin}!')
                elif self._return == 'done':
                    self._deliver_passengers(userdata, active_passengers)
                    Logger.loginfo(f'Delivered {active_passengers} to {self._destination}!')
                elif self._return is not None:
                    Logger.logerr(f'Unknown outcome={self._return} with {self._origin}, {self._destination}, {active_passengers}!')

                Logger.loginfo(f'Left bank : {userdata.left}')
                Logger.loginfo(f'Middle bank: {userdata.middle}')
                Logger.loginfo(f'Right bank: {userdata.right}')
                self._passengers0.clear()
                self._passengers1.clear()
                self._origin = None
                self._destination = None
                self._start_time = None
                self._publish_update(userdata)

        except Exception as exc:
            Logger.logerr(f'Error processing final move of {self._item} due to {exc}!')
            self._return = 'failed'

        return super().on_exit(userdata)

    def _set_auto_route(self, userdata):
        """
        Select the next adjacent river crossing from current userdata.

        The middle bank can launch either boat.  When carrying an item that is
        on the middle bank, continue toward the right bank; otherwise send the
        tracker back left to preserve the two-river shuttle pattern.
        """
        if self._tracker in userdata.left:
            self._origin = 'left'
            self._destination = 'middle'
        elif self._tracker in userdata.middle:
            self._origin = 'middle'
            if self._item is not None and self._item in userdata.middle:
                self._destination = 'right'
            else:
                self._destination = 'left'
        elif self._tracker in userdata.right:
            self._origin = 'right'
            self._destination = 'middle'
        else:
            Logger.logerr(f'Cannot start new move - {self._tracker} must be on a bank!')
            self._return = 'failed'
            return False

        return True

    def _is_valid_route(self):
        """Return whether the selected route crosses one adjacent river."""
        valid_routes = {
            ('left', 'middle'),
            ('middle', 'left'),
            ('middle', 'right'),
            ('right', 'middle'),
        }
        return (self._origin, self._destination) in valid_routes

    def _deliver_passengers(self, userdata, active_passengers):
        """Move completed passengers to the destination bank."""
        passengers = list(active_passengers)
        conflicts = []

        for passenger in passengers:
            banks = [
                bank for bank in ('left', 'middle', 'right')
                if passenger in userdata[bank]
            ]
            unexpected_banks = [bank for bank in banks if bank != self._origin]
            if unexpected_banks:
                conflicts.append(
                    f"{passenger} already on {', '.join(unexpected_banks)}"
                )

        if conflicts:
            raise RuntimeError(
                'Cannot deliver passengers from inconsistent userdata: '
                + '; '.join(conflicts)
            )

        passengers_to_remove = [
            passenger for passenger in passengers
            if passenger in userdata[self._origin]
        ]
        for passenger in passengers_to_remove:
            userdata[self._origin].remove(passenger)
        userdata[self._destination].extend(passengers)

    def _publish_update(self, userdata):
        """Publish the current two-river visualization status."""
        try:
            msg = WGCFStatus(left=userdata.left[:],
                             middle=userdata.middle[:],
                             right=userdata.right[:],
                             passengers0=self._passengers0[:],
                             passengers1=self._passengers1[:],
                             position0=self._position0,
                             position1=self._position1)
            self._pub.publish(self._topic, msg)
        except Exception:
            Logger.logerr('Failed to publish update!')
