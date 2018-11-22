import unittest

from parser.parser import parse_tree


class TestCodeGenerator(unittest.TestCase):

    def test_is_empty_struct(self):
        config = {}
        expected_code = [
            'parameter_namespace_declare(&config.ns, NULL, NULL);',
        ]

        tree = parse_tree(config).to_init_code().split('\n')

        self.assertEqual(tree, expected_code)

    def test_has_one_integer_entry(self):
        config = {'answer': 42}
        expected_code = [
            'parameter_namespace_declare(&config.ns, NULL, NULL);',
            'parameter_integer_declare(&config.answer, &config.ns, "answer");',
        ]

        tree = parse_tree(config).to_init_code().split('\n')

        self.assertEqual(tree, expected_code)

    def test_has_one_namespace_with_entries(self):
        config = {
            'controller': {
                'kp': 10,
                'ki': 0.1,
            }
        }
        expected_code = [
            'parameter_namespace_declare(&config.ns, NULL, NULL);',
            'parameter_namespace_declare(&config.controller.ns, &config.ns, "controller");',
            'parameter_integer_declare(&config.controller.kp, &config.controller.ns, "kp");',
            'parameter_scalar_declare(&config.controller.ki, &config.controller.ns, "ki");',
        ]

        tree = parse_tree(config).to_init_code().split('\n')

        self.assertEqual(tree, expected_code)

    def test_has_depth_three(self):
        config = {
            'robot': {
                'controller': {
                    'kp': 10,
                    'ki': 0.1,
                }
            }
        }
        expected_code = [
            'parameter_namespace_declare(&config.ns, NULL, NULL);',
            'parameter_namespace_declare(&config.robot.ns, &config.ns, "robot");',
            'parameter_namespace_declare(&config.robot.controller.ns, &config.robot.ns, "controller");',
            'parameter_integer_declare(&config.robot.controller.kp, &config.robot.controller.ns, "kp");',
            'parameter_scalar_declare(&config.robot.controller.ki, &config.robot.controller.ns, "ki");',
        ]

        tree = parse_tree(config).to_init_code().split('\n')

        self.assertEqual(tree, expected_code)

    def test_has_depth_max_three(self):
        config = {
            'robot': {
                'controller': {
                    'kp': 10,
                    'ki': 0.1,
                },
                'so': 1337,
            }
        }
        expected_code = [
            'parameter_namespace_declare(&config.ns, NULL, NULL);',
            'parameter_namespace_declare(&config.robot.ns, &config.ns, "robot");',
            'parameter_namespace_declare(&config.robot.controller.ns, &config.robot.ns, "controller");',
            'parameter_integer_declare(&config.robot.controller.kp, &config.robot.controller.ns, "kp");',
            'parameter_scalar_declare(&config.robot.controller.ki, &config.robot.controller.ns, "ki");',
            'parameter_integer_declare(&config.robot.so, &config.robot.ns, "so");',
        ]

        tree = parse_tree(config).to_init_code().split('\n')

        self.assertEqual(tree, expected_code)


if __name__ == '__main__':
    unittest.main()
