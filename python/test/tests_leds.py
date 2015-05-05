import unittest
from master_board import leds

try:
    import unittest.mock as mock
except ImportError:
    import mock


class LedTestCase(unittest.TestCase):
    def test_leds_mapping(self):
        self.assertEqual(leds.Led.Ready, "ready")
        self.assertEqual(leds.Led.Debug, "debug")
        self.assertEqual(leds.Led.Error, "error")
        self.assertEqual(leds.Led.PowerError, "power_error")
        self.assertEqual(leds.Led.PcError, "pc_error")
        self.assertEqual(leds.Led.BusError, "bus_error")
        self.assertEqual(leds.Led.Yellow[0], "yellow_1")
        self.assertEqual(leds.Led.Yellow[1], "yellow_2")
        self.assertEqual(leds.Led.Green[0], "green_1")
        self.assertEqual(leds.Led.Green[1], "green_2")

    @mock.patch('cvra_rpc.service_call.call')
    def test_set_led(self, call):
        call.return_value = tuple()  # no error
        leds.set_led('host', leds.Led.PcError, True)
        call.assert_any_call('host', 'led_set', ['pc_error', True])

    @mock.patch('cvra_rpc.service_call.call')
    def test_set_many_leds(self, call):
        call.return_value = tuple()  # no error
        leds.set_led('host', [leds.Led.PcError, leds.Led.Yellow], True)

        for i in ['pc_error', 'yellow_1', 'yellow_2']:
            call.assert_any_call('host', 'led_set', [i, True])

    @mock.patch('cvra_rpc.service_call.call')
    def test_set_invalid_led(self, call):
        call.return_value = ("Error: invalid argument value.",)

        with self.assertRaises(RuntimeError):
            leds.set_led('host', leds.Led.PcError, True)



