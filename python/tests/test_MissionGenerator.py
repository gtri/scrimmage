import os
import unittest
import subprocess
import tempfile

import scrimmage.mission.MissionGenerator as MG

class TestMissionGenerator(unittest.TestCase):
    def test_mission_gen(self):

        script_path = os.path.dirname(os.path.realpath(__file__))
        mission_yaml_path = os.path.join(script_path, "templates/mission.yaml")

        mg = MG.MissionGenerator(mission_yaml_path)
        self.assertTrue(mg.mission)
        self.assertEqual(mg.entity_ids(), [1, 2])

        # Run a scrimmage mission with the mission file
        with tempfile.NamedTemporaryFile('w') as fp:
            fp.write(mg.mission)
            fp.flush()

            cmd = "scrimmage %s" % fp.name
            try:
                subprocess.check_call(cmd.split(), env=os.environ.copy())
            except:
                print("Failed to run scrimmage mission with file:")
                print(mg.mission)

                # Force error
                self.assertTrue(False)

if __name__ == '__main__':
    unittest.main()
