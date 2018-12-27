# chibios-syscalls
Better syscalls implementations for ChibiOS

## Usage

Nothing particular is needed: just add this package to your dependencies and it should automatically work.
If you want to use stdout, for example for `printf (3)`, you should define `STDOUT_SD` to the stream you want to use.
Same goes for stdin, using `STDIN_SD`.

One way to do it is to put the following in your `board.h`:
```cpp
#define STDOUT_SD SDU1
#define STDIN_SD SDU1
```

You could also do it in your Makefile by adding the following line:

```
UDEFS = -DSTDOUT_SD=SDU1 -DSTDIN_SD=SDU1
```
