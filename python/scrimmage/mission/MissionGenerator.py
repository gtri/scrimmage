"""Generate SCRIMMAGE mission files from a YAML Configuration

@file

@section LICENSE

Copyright (C) 2020 by the Georgia Tech Research Institute (GTRI)

This file is part of SCRIMMAGE.

  SCRIMMAGE is free software: you can redistribute it and/or modify it under
  the terms of the GNU Lesser General Public License as published by the
  Free Software Foundation, either version 3 of the License, or (at your
  option) any later version.

  SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
  License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.

@author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
@author Eric Squires <eric.squires@gtri.gatech.edu>
@date 31 July 2017
@version 0.1.0
@brief Brief file description.
@section DESCRIPTION
A Long description goes here.
"""

import os
import yaml

from jinja2 import FileSystemLoader, Environment

class MissionGenerator():
    def __init__(self, mission_yaml_file):

        with open(os.path.join(mission_yaml_file)) as f:
             self.mission_yaml = yaml.load(f, Loader=yaml.FullLoader)

        template_dirs = [ os.path.join(os.path.dirname(mission_yaml_file), d)
                          for d in self.mission_yaml['template_directories'] ]

        self._file_loader = FileSystemLoader(template_dirs)
        self._env = Environment(loader=self._file_loader)

        # Generate the entity configurations
        entities_xml = [self._render(entity['template'], entity['config'], self._env)
                        for entity in self.mission_yaml['entities']]

        # Generate the mission file with the entity configuration
        try:
            mission_config = self.mission_yaml['mission_file']['config']
        except:
            mission_config = dict()

        mission_config['entities'] = entities_xml

        self.mission = self._render(self.mission_yaml['mission_file']['template'],
                                    mission_config, self._env)

        # Create a mapping from entity ID to type
        self._entity_id_to_type = dict()
        for entity in self.mission_yaml['entities']:
            try:
                entity_type = entity['type']
            except:
                entity_type = None

            try:
                entity_id = int(entity['config']['id'])
                self._entity_id_to_type[entity_id] = entity_type
            except:
                pass

    def entity_id_to_type(self, id):
        return self._entity_id_to_type[id]

    def entity_ids(self):
        ids = []
        for entity in self.mission_yaml['entities']:
            ids.append(entity['config']['id'])
        return ids

    def _render(self, template_name, config, env):
        template = env.get_template(template_name)
        return template.render(config = config)
