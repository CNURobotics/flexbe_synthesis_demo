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

"""Sound manager."""

import os

import pygame


class SoundManager:
    """Convenience class to hold sounds."""

    def __init__(self, folder='sounds'):
        """Initialize."""
        print(f"Using '{folder}' as working director to find sounds ...")
        pygame.mixer.init(44100, -16, 2, 1024)
        pygame.mixer.music.set_volume(0.8)
        self.pfft = pygame.mixer.Sound(os.path.join(folder, 'pfft.mp3'))
