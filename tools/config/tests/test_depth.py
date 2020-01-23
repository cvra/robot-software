import unittest

from parser.parser import depth


class TestDictionaryDepth(unittest.TestCase):
    def test_is_none_for_non_dict(self):
        self.assertEqual(depth(list()), None)
        self.assertEqual(depth(tuple()), None)

    def test_is_zero_for_empty_dict(self):
        self.assertEqual(depth(dict()), 0)

    def test_is_one_for_dict(self):
        self.assertEqual(depth({"answer": 42}), 1)
        self.assertEqual(depth({"answer": 42, "foo": "bar"}), 1)

    def test_is_two_for_nested_dict(self):
        self.assertEqual(depth({"meaning of": {"life": 42}}), 2)
        self.assertEqual(depth({"answer": 42, "foo": {"bar": 2000}}), 2)

    def test_is_two_for_nested_assymetric_dict(self):
        self.assertEqual(depth({"meaning of": {"life": 42}, "foo": "bar"}), 2)


if __name__ == "__main__":
    unittest.main()
