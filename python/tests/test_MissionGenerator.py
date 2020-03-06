import unittest

import scrimmage.mission.MissionGenerator as MG

class TestMissionGenerator(unittest.TestCase):
    def test_mission_gen(self):

        script_path = os.path.dirname(os.path.realpath(__file__))
        mission_yaml_path = os.path.join(script_path, "templates/mission.yaml")

        mg = MG.MissionGenerator(mission_yaml_path)
        print(mg.mission)
        self.assertTrue(False)


if __name__ == '__main__':
    unittest.main()
