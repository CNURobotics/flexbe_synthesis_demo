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

"""
Simple script to draw a GraphViz representation of the Mealy automaton.

This simple script reads the specs from Slugs file, and json representation
of the Slugs automaton, and generates a
dot file representation of the Mealy machine.

This is NOT currently installed as a ROS node and
does NOT search package path for files.
"""

import json
import os
import sys


def load_specs(file_path):
    """Load specs from slugsin file."""
    specs = {}
    with open(file_path, 'rt') as fin:
        lines = fin.readlines()
        key = None
        for line in lines:
            line = line.strip()
            if len(line) == 0 or line[0] == '#':
                continue

            if line[0] == '[' and line[-1] == ']':
                key = line
                specs[key] = []
            else:
                specs[key].append(line)
    return specs


def load_automata(file_path):
    """Load explicitStrategy from JSON format."""
    with open(file_path, 'rt') as fin:
        auton = json.load(fin)
    return auton


class State:
    """Define class to hold automata state / GraphViz node data."""

    def __init__(self, name, label, trans):
        """Initialize."""
        self.name = name
        self.label = label
        self.trans = trans
        self.fill = False
        self.width = 0.5

    def __str__(self):
        """To string."""
        return f'{self.__class__.__name__}: {self.name} : {self.label} : {self.trans}'


class EnvState(State):
    """Environment states go first."""

    def __init__(self, name, label, trans):
        """Initialize."""
        super().__init__(name, label, trans)


class SysState(State):
    """System states follow EnvState."""

    def __init__(self, name, label, trans):
        """Initialize."""
        super().__init__(name, label, trans)
        self.width = 0.2
        self.fill = True


class Mealy:
    """Class to hold Mealy state version of explicit strategy."""

    def __init__(self):
        """Initialize."""
        self.env_vars = {}
        self.sys_vars = {}
        self.states = {}
        self.initial_state = None

    def __str__(self):
        """To string."""
        string = 30 * '-' + '\n'
        string += str(self.env_vars) + '\n'
        string += str(self.sys_vars) + '\n'
        string += '\n'
        for name, state in self.states.items():
            string += str(state) + '\n'

        return string

    @staticmethod
    def define_mealy(specs, auton):
        """Define Mealy given specs and automaton."""
        mealy = Mealy()
        for var in specs['[INPUT]']:
            mealy.env_vars[var] = auton['variables'].index(var)
        for var in specs['[OUTPUT]']:
            mealy.sys_vars[var] = auton['variables'].index(var)

        for name, state in auton['nodes'].items():

            # Load environmetal states
            state_vars = [f'{"!" if state["state"][ndx]== 0 else ""}{var}'
                          for var, ndx in mealy.env_vars.items()]

            # Define information for EnvState and SysState using strategy ID
            # Env state transitions to SysState
            env_state = EnvState(f'{name}', state_vars, [f'{name}_sys'])
            # transition to next environment state
            sys_state = SysState(f'{name}_sys',
                                 [var for var, ndx in mealy.sys_vars.items() if state['state'][ndx] == 1],
                                 [f'{tran}' for tran in state['trans']])

            if mealy.initial_state is None:
                mealy.initial_state = env_state

            mealy.states[env_state.name] = env_state
            mealy.states[sys_state.name] = sys_state

            # print(env_state)
            # print(sys_state)

        return mealy

    def to_dot(self, layout='neato', splines=True, initial_state=True, model='subset', rankdir='LR'):
        """
        Write a GraphViz dot specification given Mealy definition.

        @param layout (default: 'neato') https://graphviz.org/docs/layouts/
        @param splines - draw transitions as splines (default: True)
        @param initial_state - draw initial state
        @param model subset, shortpath, ciruit
        """
        dot = 'digraph StateMachine {\n'
        dot += f'   rankdir="{rankdir};"\n'
        dot += f'  layout="{layout}";\n'
        dot += f'  model="{model}"\n;'
        if splines:
            dot += '  splines=true;\n'

        if initial_state:
            dot += '  "Init" [shape=point];\n'

        for state_name, state in self.states.items():
            attributes = []

            if state.fill:
                attributes.append('style=filled')
                attributes.append('fillcolor=black')

            if state.width:
                attributes.append(f'width={state.width}')
                attributes.append(f'height={state.width}')

            state_attributes = ', '.join(attributes)
            if isinstance(state, SysState):
                state_attributes += ', label=""'

            dot += f'  "{state_name}" [{state_attributes}];\n'

        if initial_state:
            # This assumes the initial state is always named 0.
            dot += '  "Init" -> "0" [label="Init", color="blue",style="bold",dir="forward"];\n'

        for state_name, state in self.states.items():
            print(state_name, state)
            if isinstance(state, EnvState):
                assert len(state.trans) == 1, 'Only 1 transition to system state!'

                label = '\\n'.join(state.label)
                for target_state in state.trans:
                    dot += f'  "{state_name}" -> "{target_state}" [label="{label}", color="red",style="bold",dir="forward"];\n'
            elif isinstance(state, SysState):
                assert len(state.label) <= 1, 'Only 1 label from system state!'
                label = state.label[0] if len(state.label) == 1 else ''
                for target_state in state.trans:
                    dot += f'  "{state_name}" -> "{target_state}" [label="{label}", color="darkgreen", style="bold", dir="forward"];\n'
            else:
                raise TypeError(f'Unknown instance type {state.__class__.__name__}')
        dot += '}'
        return dot

    @staticmethod
    def draw_graph(dot, base_file_name):
        """Visualize the graph."""
        try:
            print(f'Drawing graph using {base_file_name} ...')
            from graphviz import Source
            s = Source(dot, format='png')
            s.render(filename=base_file_name, format='png', cleanup=True, view=True)
        except Exception as exc:
            print(exc)
            print('No graphviz; try https://dreampuf.github.io/GraphvizOnline')


if __name__ == '__main__':
    """ Main function."""
    print('Loading spec and strategy data ...')
    folder_base = ''
    file_base = 'Container'
    if len(sys.argv) > 1:
        file_base = sys.argv[1]

    if len(sys.argv) > 2:
        folder_base = sys.argv[2]

    slugs_specs = load_specs(os.path.join(folder_base, file_base + '.slugsin'))
    slugs_auton = load_automata(os.path.join(folder_base, file_base + '.json'))

    # print(slugs_specs)
    # print(30*'=')
    # print(slugs_auton)
    # print(30*'=')

    print('Define the Mealy machine ...')
    mealy = Mealy.define_mealy(slugs_specs, slugs_auton)
    # print(mealy)

    print('Converting to dot format ...')
    dot_str = mealy.to_dot()
    # print(dot_str)
    mealy.draw_graph(dot_str, os.path.join(folder_base, file_base))
    print('Done!')
