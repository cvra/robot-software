# cmp_mem_access

This library contains memory buffer access functions for the MessagePack library [cmp](https://github.com/camgunz/cmp).

Usage (see [example.c](https://github.com/Stapelzeiger/cmp_mem_access/blob/master/example.c) for a complete version):
```C
cmp_mem_access_t mem;
cmp_ctx_t cmp;
char buffer[128];

// initialize memory access
cmp_mem_access_init(&cmp, &mem, buffer, sizeof(buffer));

// now you can use cmp to write to the buffer
cmp_write_int(&cmp, 42);

// reset to start of buffer
cmp_mem_access_set_pos(&mem, 0);

// read back from buffer
int32_t val;
cmp_read_int(&cmp, &val);
```

The cmp_mem_access functions check buffer bounds and allow you to obtain the number of bytes written to the buffer using `cmp_mem_access_get_pos(cmp_mem_access_t *)`

### installation

Just include the two files

- cmp_mem_access.c
- cmp_mem_access.h

in your build
