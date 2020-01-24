import unittest

from parser.parser import parse_tree


class TestCodeGenerator(unittest.TestCase):
    def test_is_empty_struct(self):
        config = {}
        expected_code = [
            "struct {",
            "    parameter_namespace_t ns;",
            "} config;",
        ]

        tree = parse_tree(config).to_struct().split("\n")

        self.assertEqual(tree, expected_code)

    def test_has_one_entry(self):
        config = {"answer": 42}
        expected_code = [
            "struct {",
            "    parameter_namespace_t ns;",
            "    parameter_t answer;",
            "} config;",
        ]

        tree = parse_tree(config).to_struct().split("\n")

        self.assertEqual(tree, expected_code)

    def test_has_one_namespace_with_entries(self):
        config = {"controller": {"kp": 10, "ki": 0.1,}}
        expected_code = [
            "struct {",
            "    parameter_namespace_t ns;",
            "    struct {",
            "        parameter_namespace_t ns;",
            "        parameter_t kp;",
            "        parameter_t ki;",
            "    } controller;",
            "} config;",
        ]

        tree = parse_tree(config).to_struct().split("\n")

        self.assertEqual(tree, expected_code)

    def test_has_depth_three(self):
        config = {"robot": {"controller": {"kp": 10, "ki": 0.1,}}}
        expected_code = [
            "struct {",
            "    parameter_namespace_t ns;",
            "    struct {",
            "        parameter_namespace_t ns;",
            "        struct {",
            "            parameter_namespace_t ns;",
            "            parameter_t kp;",
            "            parameter_t ki;",
            "        } controller;",
            "    } robot;",
            "} config;",
        ]

        tree = parse_tree(config).to_struct().split("\n")

        self.assertEqual(tree, expected_code)

    def test_has_depth_max_three(self):
        config = {"robot": {"controller": {"kp": 10, "ki": 0.1,}, "so": 1337,}}
        expected_code = [
            "struct {",
            "    parameter_namespace_t ns;",
            "    struct {",
            "        parameter_namespace_t ns;",
            "        struct {",
            "            parameter_namespace_t ns;",
            "            parameter_t kp;",
            "            parameter_t ki;",
            "        } controller;",
            "        parameter_t so;",
            "    } robot;",
            "} config;",
        ]

        tree = parse_tree(config).to_struct().split("\n")

        self.assertEqual(tree, expected_code)

    def test_has_string_entry(self):
        config = {"answer": "42"}
        expected_code = [
            "struct {",
            "    parameter_namespace_t ns;",
            "    parameter_t answer;",
            "    char answer_buffer[16];",
            "} config;",
        ]

        tree = parse_tree(config).to_struct().split("\n")

        self.assertEqual(tree, expected_code)

    def test_replaces_dash_in_entry_name_with_underscore(self):
        config = {"the-answer": 42}
        expected_code = [
            "struct {",
            "    parameter_namespace_t ns;",
            "    parameter_t the_answer;",
            "} config;",
        ]

        tree = parse_tree(config).to_struct().split("\n")

        self.assertEqual(tree, expected_code)


if __name__ == "__main__":
    unittest.main()
