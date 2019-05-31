SECTIONS
{
    .shell_commands :
    {
        . = ALIGN(4);
        _shell_commands = .;
        KEEP(*(SORT(.shell_commands.*)))
        KEEP(*(.shell_commands_end))
        . = ALIGN(4);
    } > RODATA_FLASH
}
