# Copyright 2024 Christopher Newport University CHRISlab 2024
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

"""FlexBE state to move item from  one list to another."""

from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher

from flexbe_synthesis_demo_msgs.msg import WGCFStatus

from rclpy.duration import Duration


class MoveState(EventState):
    """
    A state that moves items from one list to another.

    Used to simulate river crossing the 'wolf, goat, corn' puzzle.

    Parameters
    ----------
    -- item       string    The item to be moved from origin to destination
    -- tracker    string    String value representing who is doing the moving with boat

    Userdata inputs
    ---------------
    ># left     object      list of characters on the left bank
    ># right    object      list of characters on the right bank

    User data outputs
    -----------------
    #> left     object      list of characters on the left bank
    #> right    object      list of characters on the right bank

    Outcomes
    --------
    <= done      completed    Indicates that item will be moved to the destination.
    <= canceled  failure    Indicates that item will be returned to origin (allow trying again)
    <= failed    failure      Indicates complete failure to exit the behavior.

    """

    def __init__(self, item=None, tracker='farmer'):
        """Initialize state."""
        super().__init__(outcomes=['done', 'canceled', 'failed'],
                         input_keys=['left', 'right'],
                         output_keys=['left', 'right'])

        ProxyPublisher.initialize(MoveState._node)

        self._item = item
        self._tracker = tracker
        self._topic = '/wgcf_status'
        self._pub = ProxyPublisher({self._topic: WGCFStatus})
        self._moving_item = None
        self._wait_time = Duration(seconds=5.0)
        self._origin = None
        self._destination = None
        self._passengers = []
        self._return = None
        self._position = 0.0
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
        else:
            if self._destination == 'right':
                self._position = elapsed / self._wait_time.nanoseconds
            else:
                self._position = 1.0 - (elapsed / self._wait_time.nanoseconds)

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
        self._passengers.clear()
        self._origin = None
        self._destination = None

        try:
            if self._tracker in userdata.left:
                if self._item is not None:
                    if self._item not in userdata.left:
                        self._return = 'failed'
                        Logger.loginfo(f'Cannot move {self._item} - not with {self._tracker} on left bank!')
                        return
                self._origin = 'left'
                self._destination = 'right'
                self._position = 0.0

            elif self._tracker in userdata.right:
                if self._item is not None:
                    if self._item not in userdata.right:
                        self._return = 'failed'
                        Logger.loginfo(f'Cannot move {self._item} - not with {self._tracker} on right bank!')
                        return

                self._origin = 'right'
                self._destination = 'left'
                self._position = 1.0  # right bank is positin 1
            else:
                Logger.logerr(f'Cannot start new move - {self._tracker} must be on the boat!')
                self._return = 'failed'
                print(f'Passengers = {self._passengers}')
                print('Userdata:\n', userdata, flush=True)
                return

            self._passengers.append(self._tracker)
            if self._item is not None:
                self._passengers.append(self._item)

            for passenger in self._passengers:
                userdata[self._origin].remove(passenger)

            Logger.loginfo(f'Moving {self._passengers} from {self._origin} to {self._destination} bank ...')
            Logger.loginfo(f'  Left bank : {userdata.left}')
            Logger.loginfo(f'        Boat: {self._passengers}')
            Logger.loginfo(f'  Right bank: {userdata.right}')

            self._publish_update(userdata)
            self._start_time = self._node.get_clock().now()
            Logger.loginfo('     moving ...')

        except Exception as exc:
            Logger.logerr(f'Failed to move {self._item} due to {exc}!')
            self._return = 'failed'
            print('Userdata:\n', userdata, flush=True)
            raise exc

    def on_exit(self, userdata):
        """Update on state exit."""
        try:
            if self._start_time is not None:
                # Only process once if blocked
                Logger.logerr(' Move is done!')
                if self._start_time is None:
                    Logger.logerr('Exit - but start time not initialized')

                if self._return is None:
                    self._return = self._manual_transition_requested
                    print(f"  Using manually requested outcome '{self._return}'", flush=True)

                if self._return == 'failed' or self._return == 'canceled':
                    # Return passengers to origin
                    if self._passengers is not None:
                        for passenger in self._passengers:
                            userdata[self._origin].append(passenger)

                    if self._origin == 'right':
                        self._position = 1.0
                    else:
                        self._position = 0.0
                    Logger.logerr(f'Returned {self._passengers} to {self._origin}!')
                elif self._return == 'done':
                    # End at our desired destination
                    if self._passengers is not None:
                        for passenger in self._passengers:
                            userdata[self._destination].append(passenger)

                    if self._destination == 'right':
                        self._position = 1.0
                    else:
                        self._position = 0.0
                    Logger.logerr(f'Delivered {self._passengers} to {self._destination}!')
                elif self._return is not None:
                    Logger.logerr(f'Unknown outcome={self._return} with {self._origin}, {self._destination}, {self._passengers}!')

                Logger.loginfo(f'Left bank : {userdata.left}')
                Logger.loginfo(f'Right bank: {userdata.right}')
                self._passengers.clear()
                self._origin = None
                self._destination = None
                self._start_time = None
                self._publish_update(userdata)

        except Exception as exc:
            Logger.logerr(f'Error processing final move of {self._item} due to {exc}!')
            self._return = 'failed'

        return super().on_exit(userdata)

    def _publish_update(self, userdata):
        """Publish data."""
        try:
            msg = WGCFStatus(left=userdata.left[:],
                             right=userdata.right[:],
                             passengers=self._passengers[:],
                             position=self._position)
            self._pub.publish(self._topic, msg)
        except Exception:
            Logger.logerr('Failed to publish update!')
