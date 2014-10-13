import sys
import os

def crc32(data):
    return b'\x11\x22\x33\x44'

class SerialDatagram:

    END = b'\xC0'
    ESC = b'\xDB'
    ESC_END = b'\xDC'
    ESC_ESC = b'\xDD'

    def __init__(self, file_desc):
        self.file_desc = file_desc

    def send(self, dtgrm):
        crc = crc32(dtgrm)
        frame = dtgrm + crc
        frame = frame.replace(SerialDatagram.END,
                              SerialDatagram.ESC + SerialDatagram.ESC_END)
        frame = frame.replace(SerialDatagram.ESC,
                              SerialDatagram.ESC + SerialDatagram.ESC_ESC)
        self.file_desc.write(frame + SerialDatagram.END)

    def receive(self):
        buf = b''
        while True:
            b = self.file_desc.read(1)
            if b == SerialDatagram.END:
                break
            buf += b
        buf = buf.replace(SerialDatagram.ESC + SerialDatagram.ESC_END,
                          SerialDatagram.END)
        buf = buf.replace(SerialDatagram.ESC + SerialDatagram.ESC_ESC,
                          SerialDatagram.ESC)
        frame = buf[:-4]
        if crc32(frame) != buf[-4:]:
            print("crc error")
        else:
            return frame


if __name__ == "__main__":

    def w():
        sys.stdout = os.fdopen(1, "wb")
        for line in sys.stdin:
            SerialDatagram(sys.stdout).send(line.encode('ascii', 'ignore'))
            sys.stdout.flush()

    def r():
        sys.stdin = os.fdopen(0, "rb")
        while True:
            print(SerialDatagram(sys.stdin).receive())

    if len(sys.argv) == 2:
        if sys.argv[1] == 'r':
            r()
        if sys.argv[1] == 'w':
            w()
    else:
        print("usage: {} r # to read".format(sys.argv[0]))
        print("usage: {} w # to write".format(sys.argv[0]))
