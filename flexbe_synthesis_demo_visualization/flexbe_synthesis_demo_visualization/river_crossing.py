#!/usr/bin/env python3

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

"""PyGame based visualization."""

import os
import sys

from flexbe_synthesis_demo_visualization.image_manager import ImageManager
from flexbe_synthesis_demo_visualization.sound_manager import SoundManager

import pygame
import pygame.locals

try:
    from flexbe_synthesis_demo_visualization.msg_handler import MsgHandler
except Exception as exc:
    print(f'Cannot handle ROS 2 connect {exc}')

try:
    # ROS message type
    from flexbe_synthesis_demo_msgs.msg import WGCFStatus
except Exception:
    print('Unknown ROS 2 type', flush=True)

    class WGCFStatus:
        """Dummy class if running without ROS."""

        left = []
        right = []
        passengers = []
        position = 0.5

try:
    from ament_index_python.packages import get_package_share_directory
    # may raise PackageNotFoundError
    base_folder = get_package_share_directory('flexbe_synthesis_demo_visualization')
    print(f"using '{base_folder}' as the base folder for resources ...")
except Exception:
    base_folder = '.'


class RiverCrossing:
    """Base class for the PyGame-based visualization of synthesis demo."""

    def __init__(self, args):
        """Initialize."""
        pygame.init()  # Initialize the PyGame internals

        self.verbose = True  # Set to False to hide extra debug prints
        self.images = ImageManager(folder=os.path.join(base_folder, 'images'))
        self.sounds = SoundManager(folder=os.path.join(base_folder, 'sounds'))

        # Define some useful values
        self.black, self.white = (0, 0, 0), (255, 255, 255)

        self.screensize = self.images.background_rect[2], self.images.background_rect[3]
        self.screenrect = pygame.Rect(0, 0, self.screensize[0], self.screensize[1])
        pygame.display.set_caption('Cross the River')
        self.screen = pygame.display.set_mode(self.screensize)
        self.images.convert_images()

        # Initialize some values that will be updated during game play
        self.mouse_position = (0, 0)

        self.running = True
        self.new_game()

        # Set up the thread for handling ROS messages
        try:
            self.msg_handler = MsgHandler('river_crossing', '/wgcf_status', WGCFStatus, args=args)
            self.msg_handler.start()
        except Exception as exc:
            print('Could not start the ROS interface! Just running as demo for now', flush=True)
            print(type(exc), exc, flush=True)
            self.msg_handler = None

    def new_game(self):
        """Set up new game."""
        self.direction = 1.0
        self.left_bank = list(self.images.characters.keys())
        self.left_bank.sort()
        self.right_bank = []
        self.passengers = []
        self.boat_position = 0.0  # Left bank
        self.game_over = False

        print('left:', self.left_bank)
        print('boat:', self.passengers)
        print('right:', self.right_bank)

    def get_current_state(self):
        """Update the state of visualization."""
        if self.msg_handler is not None:
            msg_data = None
            while not self.msg_handler.is_empty:
                msg_data = next(self.msg_handler.data_generator())

            if msg_data is not None:
                # print(f'Got last position={msg_data.position}!', flush=True)
                self.left_bank = msg_data.left
                self.right_bank = msg_data.right
                self.boat_position = msg_data.position
                if len(msg_data.passengers) == 0 and len(self.passengers) > 0:
                    # just landed
                    self.sounds.pfft.play()

                self.passengers = msg_data.passengers

        else:
            # Just demo for now
            self.boat_position += self.direction * 0.001
            if self.boat_position < 0:
                self.sounds.pfft.play()
                self.direction = 1
                self.boat_position = 0.0
            elif self.boat_position > 1.0:
                self.sounds.pfft.play()
                self.direction = -1
                self.boat_position = 1.0

    def update(self):
        """Update game state in visualization."""
        if self.handle_events() == pygame.locals.QUIT:
            print('Quitting update loop...', flush=True)
            return

        pygame.mouse.set_visible(True)  # False)

        # redraw background (erases all previous images)
        self.screen.blit(pygame.transform.scale(self.images.background, self.screensize), (0, 0))
        self.get_current_state()

        # Draw the left bank
        for character in self.left_bank:
            self.images.draw_character(self.screen, 'left', character)

        # Draw the right bank
        for character in self.right_bank:
            self.images.draw_character(self.screen, 'right', character)

        # Draw the boat
        self.images.draw_boat(self.screen, self.boat_position,
                              self.passengers)

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

                # if pygame.key.name(event.key) == 'space' and self.game_over:
                #     self.new_game()
            elif event.type == pygame.locals.MOUSEMOTION:
                self.mouse_position = pygame.mouse.get_pos()
                # print(f'Event: MOUSEMOTION position = {self.mouse_position}')

    def run(self):
        """Run loop."""
        try:
            while self.running:
                self.update()
        except KeyboardInterrupt:
            self.msg_handler.__shutdown__()

        # print('Finished running ...', flush=True)
        pygame.display.quit()
        # print('pygame.quit() ...', flush=True)
        pygame.quit()
        # print('Exit ...', flush=True)

    def game_over(self):
        """Handle end of game."""
        self.game_over = True

    def shutdown(self):
        """Handle shutdown logic."""
        print('Shutdown pygame visualization ...', flush=True)
        self.running = False
        try:
            if self.msg_handler:
                print('Shutdown the msg_handler ...', flush=True)
                self.msg_handler.shutdown()
                del self.msg_handler
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
