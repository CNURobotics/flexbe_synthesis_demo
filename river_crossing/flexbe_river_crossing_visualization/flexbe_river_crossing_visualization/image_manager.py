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

"""Pygame image loading, positioning, and rendering for the river-crossing visualization."""

import logging
import os

import pygame

LOGGER = logging.getLogger(__name__)


def _load_image(path):
    """Load an image asset with a useful error if it cannot be opened."""
    try:
        return pygame.image.load(path)
    except pygame.error as exc:
        LOGGER.error("Failed to load image asset '%s': %s", path, exc)
        raise RuntimeError(f"Failed to load image asset '{path}': {exc}") from exc


class ImageHandler:
    """Class to manage images of visualization."""

    def __init__(self, name,
                 x_offset, y_offset,
                 left_bank_dx, left_bank_dy,
                 middle_bank_dx, middle_bank_dy,
                 right_bank_dx, right_bank_dy,
                 boat_dx, boat_dy,  # offset from center fraction
                 folder='images',
                 ext='png',
                 two_rivers_mode=False):
        """Initialize."""
        self.name = name
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.file_name = name + '.' + ext
        self.image = _load_image(os.path.join(folder, self.file_name))
        self.rect = self.image.get_rect()

        if two_rivers_mode:
            self.bank_offsets = {
                'left': (left_bank_dx, left_bank_dy),
                'middle': (middle_bank_dx, middle_bank_dy),
                'right': (right_bank_dx, right_bank_dy)
            }
        else:
            self.bank_offsets = {
                'left': (left_bank_dx, left_bank_dy),
                'right': (right_bank_dx, right_bank_dy)
            }

        self.boat_offsets = (boat_dx, boat_dy)

    def convert(self):
        """Convert image to PyGame format."""
        self.image.convert()

    def __str__(self):
        """To string."""
        return (f'{self.name} ({self.x_offset}, {self.y_offset}) {self.file_name}\n'
                f'    bank_offsets = ({self.bank_offsets}\n'
                f'    boat offset  = {self.boat_offsets}\n')


class ImageManager:
    """Convenience class to hold images."""

    # Pixel offset from the middle bank's x coordinate to the right-river departure point
    # (far edge of the middle island where boat1 begins its crossing to the right bank).
    DEPARTURE_POINT = 400

    def __init__(self, folder='images', yaml_name=None, two_rivers_mode=False):
        """Initialize."""
        print(f"Using '{folder}' as working directory to find images ...")
        self.two_rivers_mode = two_rivers_mode

        if self.two_rivers_mode:
            self.background = _load_image(os.path.join(folder, 'two_rivers_background.png'))
            self.banks = {'left': (125, 150), 'middle': (375, 165), 'right': (1050, 180)}
            self.bank_vector1 = (self.banks['middle'][0] - self.banks['left'][0],
                                 self.banks['middle'][1] - self.banks['left'][1])
            self.bank_vector2 = (self.banks['right'][0] - self.banks['middle'][0],
                                 self.banks['right'][1] - self.banks['middle'][1])
        else:
            self.background = _load_image(os.path.join(folder, 'river_background.png'))
            self.banks = {'left': (150, 185), 'right': (535, 200)}
            self.bank_vector = (self.banks['right'][0] - self.banks['left'][0],
                                self.banks['right'][1] - self.banks['left'][1])

        self.boat_image = _load_image(os.path.join(folder, 'boat.png'))
        self.background_rect = self.background.get_rect()
        self.boat_rect = self.boat_image.get_rect()

        self.characters = {}
        if yaml_name is None:
            # Default setup
            self.characters['farmer'] = ImageHandler('farmer',
                                                     0, 0,
                                                     -30, -10,  # left bank offset
                                                     275, -10,  # middle bank offset
                                                     120, 30,   # right bank offset
                                                     75, -6,
                                                     folder=folder,
                                                     two_rivers_mode=self.two_rivers_mode)
            self.characters['goat'] = ImageHandler('goat',
                                                   0, 0,
                                                   -87, -30,  # left bank offset
                                                   225, -30,  # middle bank offset
                                                   140, -10,  # right bank offset
                                                   5, -9,
                                                   folder=folder,
                                                   two_rivers_mode=self.two_rivers_mode)
            self.characters['corn'] = ImageHandler('corn',
                                                   0, 0,
                                                   -60, -80,  # left bank offset
                                                   275, -80,  # middle bank offset
                                                   170, -40,  # right bank offset
                                                   12, -2,
                                                   folder=folder,
                                                   two_rivers_mode=self.two_rivers_mode)
            self.characters['wolf'] = ImageHandler('wolf',
                                                   0, 0,
                                                   -40, 45,   # left bank offset
                                                   275, -45,  # middle bank offset
                                                   60, 65,    # right bank offset
                                                   5, -9,
                                                   folder=folder,
                                                   two_rivers_mode=self.two_rivers_mode)

    def convert_images(self):
        """Convert all images to PyGame internal format."""
        self.boat_image.convert()
        for char in self.characters:
            self.characters[char].convert()

    def draw_character(self, screen, bank, character):
        """Draw character on screen."""
        image_handler = self.characters[character]

        char_rect = image_handler.rect.copy()
        char_rect.x = self.banks[bank][0]
        char_rect.y = self.banks[bank][1]
        char_rect.x += image_handler.bank_offsets[bank][0]
        char_rect.y += image_handler.bank_offsets[bank][1]

        screen.blit(image_handler.image, char_rect)

    def draw_boat(self, screen, position0, position1, passengers0, passengers1):
        """Draw boat on screen."""
        if self.two_rivers_mode:
            boat0_pos = (
                int(self.banks['left'][0] + position0 * self.bank_vector1[0]),
                int(self.banks['left'][1] + position0 * self.bank_vector1[1])
            )

            middle_right_x = self.banks['middle'][0] + self.DEPARTURE_POINT
            boat1_pos = (
                int(middle_right_x + position1 * (self.banks['right'][0] - middle_right_x)),
                int(self.banks['middle'][1] + position1 * (self.banks['right'][1] - self.banks['middle'][1]))
            )

            for character in passengers0:
                image_handler = self.characters[character]
                char_rect = image_handler.rect
                char_rect.x = boat0_pos[0] + image_handler.boat_offsets[0]
                char_rect.y = boat0_pos[1] + image_handler.boat_offsets[1]
                screen.blit(image_handler.image, char_rect)

            self.boat_rect.x, self.boat_rect.y = boat0_pos
            screen.blit(self.boat_image, self.boat_rect)

            for character in passengers1:
                image_handler = self.characters[character]
                char_rect = image_handler.rect
                char_rect.x = boat1_pos[0] + image_handler.boat_offsets[0]
                char_rect.y = boat1_pos[1] + image_handler.boat_offsets[1]
                screen.blit(image_handler.image, char_rect)

            self.boat_rect.x = boat1_pos[0]
            self.boat_rect.y = boat1_pos[1]
            screen.blit(self.boat_image, self.boat_rect)
        else:
            # Single river mode
            boat_pos = (
                int(self.banks['left'][0] + position0 * self.bank_vector[0]),
                int(self.banks['left'][1] + position0 * self.bank_vector[1])
            )

            for character in passengers0:
                image_handler = self.characters[character]

                char_rect = image_handler.rect
                char_rect.x = boat_pos[0]
                char_rect.y = boat_pos[1]
                char_rect.x += image_handler.boat_offsets[0]
                char_rect.y += image_handler.boat_offsets[1]

                screen.blit(image_handler.image, char_rect)

            # Draw boat
            self.boat_rect.x = boat_pos[0]
            self.boat_rect.y = boat_pos[1]
            screen.blit(self.boat_image, self.boat_rect)
