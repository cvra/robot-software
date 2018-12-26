import unittest
import struct
import serial_datagram
from io import BytesIO

class UARTDatagramEncodeTestCase(unittest.TestCase):
    def test_crc_encapsulation(self):
        """
        Checks that the CRC is correctly encapsulated at the end of the datagram.
        """
        data = b'\x00' * 3
        expected_crc = struct.pack('>I', 4282505490)

        datagram = serial_datagram.encode(data)
        self.assertEqual(expected_crc, datagram[-5:-1], "CRC mismatch")

    def test_end_marker(self):
        """
        Checks that the end marker is added.
        """
        data = b'\x00' * 3
        datagram = serial_datagram.encode(data)
        self.assertEqual(b'\xc0', datagram[-1:], "No end marker")

    def test_end_is_replaced(self):
        """
        Checks that the end byte is escaped.
        """
        data = b'\xC0'  # end marker is 0xC0
        datagram = serial_datagram.encode(data)
        expected = b'\xDB\xDC'  # transposed end

        self.assertEqual(expected, datagram[0:2], "END was not correctly escaped")

    def test_esc_is_escaped(self):
        """
        Checks that the ESC byte is escaped.
        """
        data = b'\xDB' # esc marker
        datagram = serial_datagram.encode(data)
        expected = b'\xDB\xDD'  # transposed esc
        self.assertEqual(expected, datagram[0:2], "ESC was not correctly escaped")

class UARTDatagramDecodeTestCase(unittest.TestCase):
    def test_can_decode_simple_datagram(self):
        """
        Simply tries to decode a complete datagram.
        """
        datagram = b'\x00\x00\x00\xFF\x41\xD9\x12\xC0'
        datagram = bytes(datagram)
        datagram = serial_datagram.decode(datagram)
        self.assertEqual(b'\x00\x00\x00', datagram)

    def test_invalid_crc_raises_exception(self):
        """
        Checks that having a wrong crc raises an exception.
        """
        datagram = b'\x00\x00\x00\xDE\xAD\xDE\xAD\xC0'
        with self.assertRaises(serial_datagram.CRCMismatchError):
            serial_datagram.decode(datagram)

    def test_escape_is_unescaped(self):
        """
        Checks that ESC + ESC_ESC is transformed to ESC.
        """
        datagram = b'\xDB\xDD\xC3\x03\xE4\xD1\xC0'
        datagram = serial_datagram.decode(datagram)
        self.assertEqual(b'\xDB', datagram)

    def test_end_is_unsescaped(self):
        """
        Checks that ESC + ESC_END is transformed to END.
        """
        datagram = serial_datagram.decode(b'\xdb\xdcIf-=\xc0')
        self.assertEqual(b'\xC0', datagram)

    def test_that_weird_sequences_work(self):
        """
        Decode order matters. This can be revealed by making a packet
        containing ESC + ESC_END, which would cause a problem if the replace
        happens in the wrong order.
        """
        data = serial_datagram.ESC + serial_datagram.ESC_END
        datagram = serial_datagram.encode(data)
        self.assertEqual(data, serial_datagram.decode(datagram))

    def test_that_too_short_sequence_raises_exception(self):
        """
        Checks that decoding a datagram smaller than 5 bytes (CRC + END) raises
        an exception.
        """

        with self.assertRaises(serial_datagram.FrameError):
            serial_datagram.decode(b'\x01\x02\x03\x03')


class ReadDatagramTestCase(unittest.TestCase):
    def test_can_read_datagram(self):
        """
        Checks if we can read a whole datagram.
        """
        fd = BytesIO(serial_datagram.encode(b'\x01\x02\x03'))
        dt = serial_datagram.read(fd)
        self.assertEqual(dt, b'\x01\x02\x03')

    def test_what_happens_if_timeout(self):
        """
        Checks that we return None if there was a timeout.
        """
        data = serial_datagram.encode(b'\x01\x02\x03')

        # Introduce a timeout before receiving the last byte
        fd = BytesIO(data[:-1])
        self.assertIsNone(serial_datagram.read(fd))
