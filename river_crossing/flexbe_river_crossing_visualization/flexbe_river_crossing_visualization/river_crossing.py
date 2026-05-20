#!/usr/bin/env python3

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

"""PyGame based visualization."""

import os
import sys

from flexbe_river_crossing_visualization.image_manager import ImageManager
from flexbe_river_crossing_visualization.sound_manager import SoundManager

import pygame
import pygame.locals

try:
    import rclpy.logging
except Exception:
    rclpy = None

try:
    from flexbe_river_crossing_visualization.msg_handler import MsgHandler
except Exception as exc:
    print(f'Cannot handle ROS 2 connect {exc}')

try:
    # ROS message type
    from flexbe_river_crossing_msgs.msg import WGCFStatus
except Exception:
    print('Unknown ROS 2 type', flush=True)

    class WGCFStatus:
        """Dummy class if running without ROS."""

        left = []
        middle = []
        right = []
        passengers0 = []
        passengers1 = []
        position0 = 0.5
        position1 = 0.5

try:
    from ament_index_python.packages import get_package_share_directory
    # may raise PackageNotFoundError
    base_folder = get_package_share_directory('flexbe_river_crossing_visualization')
    print(f"using '{base_folder}' as the base folder for resources ...")
except Exception:
    base_folder = '.'


def _log_startup_warning(message):
    """Log a startup warning through ROS if available."""
    if rclpy is not None:
        rclpy.logging.get_logger('river_crossing_visualization').warn(message)
    else:
        print(message, flush=True)


class RiverCrossing:
    """Base class for the PyGame-based visualization of synthesis demo."""

    def __init__(self, args):
        """Initialize."""
        pygame.init()  # Initialize the PyGame internals
        self.two_rivers_mode = '--two-rivers' in args

        self.verbose = False
        self.images = ImageManager(folder=os.path.join(base_folder, 'images'),
                                   two_rivers_mode=self.two_rivers_mode)
        self.sounds = SoundManager(folder=os.path.join(base_folder, 'sounds'))
        self.boat0_position = 0.0  # Boat for Left river (left to middle)
        self.boat1_position = 0.0  # Boat for Right river (middle to right)

        # Define some useful values
        self.black, self.white = (0, 0, 0), (255, 255, 255)
        self.font = pygame.font.SysFont(None, 64)

        self.screensize = self.images.background_rect[2], self.images.background_rect[3]
        self.screenrect = pygame.Rect(0, 0, self.screensize[0], self.screensize[1])
        pygame.display.set_caption('Cross the River')
        self.screen = pygame.display.set_mode(self.screensize)
        self.images.convert_images()
        self.all_chars = set(self.images.characters.keys())

        # Initialize some values that will be updated during game play
        self.mouse_position = (0, 0)

        self.running = True
        self.new_game()

        # Set up the thread for handling ROS messages
        try:
            self.msg_handler = MsgHandler('river_crossing', '/wgcf_status', WGCFStatus, args=args)
            self.msg_handler.start()
        except Exception as exc:
            _log_startup_warning(
                'Could not start the ROS interface! Just running as demo '
                f'for now: {type(exc).__name__}: {exc}'
            )
            self.msg_handler = None

    def new_game(self):
        """Set up new game."""
        self.direction = 1.0
        self.left_bank = list(self.images.characters.keys())
        self.left_bank.sort()
        self.right_bank = []
        self.middle_bank = []
        self.passengers0 = []
        self.passengers1 = []
        self.boat0_position = 0.0  # Boat for Left river (left to middle)
        self.boat1_position = 0.0  # Boat for Right river (middle to right)
        self.game_over = False

        print('left:', self.left_bank)
        print('right:', self.right_bank)

    def get_current_state(self):
        """Update the state of visualization."""
        if self.msg_handler is not None:
            msg_data = None
            if not self.msg_handler.is_empty:
                try:
                    msg_data = next(self.msg_handler.data_generator())
                except StopIteration:
                    pass

            if msg_data is not None:
                self.game_over = False  # Clear solved message when behavior sends new data
                self.left_bank = msg_data.left
                self.right_bank = msg_data.right

                self.middle_bank = msg_data.middle

                self.boat0_position = msg_data.position0
                self.boat1_position = msg_data.position1
                if (len(msg_data.passengers0) == 0 and len(self.passengers0) > 0) or \
                        (len(msg_data.passengers1) == 0 and len(self.passengers1) > 0):
                    # just landed
                    self.sounds.pfft.play()

                self.passengers0 = msg_data.passengers0
                self.passengers1 = msg_data.passengers1

        else:
            # Just demo for now
            if self.boat0_position < 1.0:
                self.boat0_position += 0.001
                if self.boat0_position >= 1.0:
                    self.sounds.pfft.play()
                    self.passengers1 = self.passengers0[:]
                    self.passengers0 = []
            else:
                self.boat1_position += 0.001
                if self.boat1_position >= 1.0:
                    self.sounds.pfft.play()
                    self.passengers0 = self.passengers1[:]
                    self.passengers1 = []

    def update(self):
        """Update game state in visualization."""
        if self.handle_events() == pygame.locals.QUIT:
            print('Quitting update loop...', flush=True)
            return

        pygame.mouse.set_visible(True)  # False)

        # redraw background (erases all previous images)
        self.screen.blit(pygame.transform.scale(self.images.background, self.screensize), (0, 0))
        self.get_current_state()

        if not self.game_over and set(self.right_bank) == self.all_chars and not self.passengers0 and not self.passengers1:
            self.game_over = True

        # Draw the left bank
        for character in self.left_bank:
            self.images.draw_character(self.screen, 'left', character)

        # Draw the middle bank IF in two rivers mode
        if self.images.two_rivers_mode:
            for character in self.middle_bank:
                self.images.draw_character(self.screen, 'middle', character)

        # Draw the right bank
        for character in self.right_bank:
            self.images.draw_character(self.screen, 'right', character)

        # Draw the boat
        self.images.draw_boat(self.screen, self.boat0_position, self.boat1_position, self.passengers0, self.passengers1)

        if self.game_over:
            cx = self.screensize[0] // 2
            cy = self.screensize[1] // 2
            line1 = self.font.render('Puzzle solved!', True, self.white)
            line2 = self.font.render('Press SPACE to restart.', True, self.white)
            gap = line1.get_height() // 4
            self.screen.blit(line1, line1.get_rect(center=(cx, cy - line1.get_height() // 2 - gap)))
            self.screen.blit(line2, line2.get_rect(center=(cx, cy + line2.get_height() // 2 + gap)))

        # Standard PyGame stuff
        self.screen.convert_alpha()
        pygame.display.update()

    def handle_events(self):
        """Check latest events since last update, and process accordingly."""
        for event in pygame.event.get():
            if event.type == pygame.locals.QUIT:
                if self.verbose:
                    print('Event: QUIT')
                self.shutdown()
                return pygame.locals.QUIT
            elif event.type == pygame.locals.KEYDOWN:
                # Handle keyboard presses (only a few are of concern to us)
                if self.verbose:
                    print(f'Event: KEYDOWN at {event.key} {pygame.key.name(event.key)}')

                if pygame.key.name(event.key) == 'space' and self.game_over:
                    self.new_game()
            elif event.type == pygame.locals.MOUSEMOTION:
                self.mouse_position = pygame.mouse.get_pos()

    def run(self):
        """Run loop."""
        try:
            while self.running:
                self.update()
        except KeyboardInterrupt:
            if self.msg_handler:
                self.msg_handler.shutdown()

        pygame.display.quit()
        pygame.quit()

    def shutdown(self):
        """Handle shutdown logic."""
        print('Shutdown pygame visualization ...', flush=True)
        self.running = False
        try:
            if self.msg_handler:
                print('Shutdown the msg_handler ...', flush=True)
                self.msg_handler.shutdown()
                self.msg_handler = None
        except Exception:
            pass
        print('Done shutdown.', flush=True)


def main(args=None):
    """Launch the game in demo mode."""
    game = RiverCrossing(args=sys.argv[1:])
    game.run()


if __name__ == '__main__':
    main()
