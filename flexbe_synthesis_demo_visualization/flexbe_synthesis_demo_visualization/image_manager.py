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

"""Image handler."""

import os

import pygame


class ImageHandler:
    """Class to manage images of visualization."""

    def __init__(self, name,
                 x_offset, y_offset,
                 left_bank_dx, left_bank_dy,
                 right_bank_dx, right_bank_dy,
                 boat_dx, boat_dy,  # offset from center fraction
                 folder='images',
                 ext='png'):
        """Initialize."""
        self.name = name
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.file_name = name + '.' + ext
        self.image = pygame.image.load(os.path.join(folder, self.file_name))
        self.rect = self.image.get_rect()
        self.bank_offsets = {'left': (left_bank_dx, left_bank_dy),
                             'right': (right_bank_dx, right_bank_dy)}

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

    def __init__(self, folder='images', yaml_name=None):
        """Initialize."""
        print(f"Using '{folder}' as working director to find images ...")
        self.background = pygame.image.load(os.path.join(folder, 'river_background.png'))
        self.boat_image = pygame.image.load(os.path.join(folder, 'boat.png'))

        self.background_rect = self.background.get_rect()
        self.boat_rect = self.boat_image.get_rect()

        self.banks = {'left': (150, 185),
                      'right': (535, 200)}
        self.bank_vector = (self.banks['right'][0] - self.banks['left'][0],
                            self.banks['right'][1] - self.banks['left'][1])
        self.characters = {}
        if yaml_name is None:
            # Default setup
            self.characters['farmer'] = ImageHandler('farmer',
                                                     0, 0,
                                                     -30, -10,
                                                     120, 30,
                                                     75, -6,
                                                     folder=folder)
            self.characters['goat'] = ImageHandler('goat',
                                                   0, 0,
                                                   -87, -30,
                                                   140, -10,
                                                   5, -9,
                                                   folder=folder)
            self.characters['corn'] = ImageHandler('corn',
                                                   0, 0,
                                                   -60, -80,
                                                   170, -40,
                                                   12, -2,
                                                   folder=folder)
            self.characters['wolf'] = ImageHandler('wolf',
                                                   0, 0,
                                                   -40, 45,
                                                   60, 65,
                                                   5, -9,
                                                   folder=folder)

    def convert_images(self):
        """Convert all images to PyGame internal format."""
        self.boat_image.convert()
        for char in self.characters:
            print(self.characters[char])
            self.characters[char].convert()

    def draw_character(self, screen, bank, character):
        """Draw character on screen."""
        image_handler = self.characters[character]

        char_rect = image_handler.rect
        char_rect.x = self.banks[bank][0]
        char_rect.y = self.banks[bank][1]
        char_rect.x += image_handler.bank_offsets[bank][0]
        char_rect.y += image_handler.bank_offsets[bank][1]

        screen.blit(image_handler.image, char_rect)

    def draw_boat(self, screen, position, passengers):
        """Draw boat on screen."""
        position = (self.banks['left'][0] + position * self.bank_vector[0],
                    self.banks['left'][1] + position * self.bank_vector[1])

        assert len(passengers) < 3
        for character in passengers:
            image_handler = self.characters[character]

            char_rect = image_handler.rect
            char_rect.x = position[0]
            char_rect.y = position[1]
            char_rect.x += image_handler.boat_offsets[0]
            char_rect.y += image_handler.boat_offsets[1]

            screen.blit(image_handler.image, char_rect)

        self.boat_rect.x = position[0]
        self.boat_rect.y = position[1]
        screen.blit(self.boat_image, self.boat_rect)
