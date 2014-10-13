
all: rcv snd

rcv:
	clang rcv.c ../serial_datagram.c ../dependencies/crc/crc32.c -I.. -I../dependencies/ -o rcv
snd:
	clang snd.c ../serial_datagram.c ../dependencies/crc/crc32.c -I.. -I../dependencies/ -o snd

clean:
	rm -f rcv snd
