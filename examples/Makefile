DEP = ../../


all: rcv snd

rcv:
	clang rcv.c ../serial_datagram.c $(DEP)/crc/crc32.c -I.. -I$(DEP)/ -o rcv
snd:
	clang snd.c ../serial_datagram.c $(DEP)/crc/crc32.c -I.. -I$(DEP)/ -o snd

clean:
	rm -f rcv snd
