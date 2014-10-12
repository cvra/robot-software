# Serial Datagram

Send datagrams over a serial link (byte stream).

### Key features:
 - Detect transmission errors (using CRC-32)
 - "Hot-pluggable": automatically synchronizes with the data-stream by detecting
 the start of a new frame
 - Low overhead
 - No dynamic memory allocation

## Protocol

A datagram frame looks like this:

    [data][CRC32][END]

The CRC32 is the "Ethernet" CRC32 (polynomial 0x104C11DB7, inverted, initialized to 0)
checksum of the (unescaped) data bytes transmitted MSByte first (big-endian).

The data and CRC32 fields are escaped such that an END byte is replaced by
[ESC ESC_END] and an ESC byte is replaced by [ESC ESC_ESC].

| Hex value | Abbreviation | Description
|-----------|--------------|------------
| 0xC0      | END          | Datagram End
| 0xDB      | ESC          | Escape
| 0xDC      | ESC_END      | Transposed End
| 0xDD      | ESC_ESC      | Transposed Escape
