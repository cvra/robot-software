# Flash backed parameter tree storage

This library allows the user to store the configuration tree in the flash memory of a microcontroller for persistence.

It stores the tree in MessagePack format, which allows the saved file to stay compatible when changing the device firmware.
It also contains some basic flash wear levelling logic to spread out the flash writes as much as possible.
The parameter integrity is ensured using a CRC32 to avoid loading invalid values.


## Example usage

See the tests for other examples

```cpp
// Usually this would come from a linker script instead
#define FLASH_ADDR ((void *)0x1234)
#define FLASH_SIZE 512

// Creates the parameter tree containting /foo as parameter
parameter_namespace_t ns;
parameter_namespace_declare(&ns, NULL, NULL);
parameter_integer_declare(&p, &ns, "foo");
parameter_integer_set(&p, 10);

// Save the config to flash
parameter_flash_storage_save(FLASH_ADDR, FLASH_SIZE, &ns);

// Loads it back
parameter_flash_storage_load(&ns, FLASH_ADDR);
```

