SECTIONS
{
    .shell_commands : ALIGN(4)
    {
        . = ALIGN(4);
        __shell_commands_start = .;
        KEEP(*(SORT(.shell_commands*)))
        __shell_commands_end = .;
        . = ALIGN(4);
    } > RODATA_FLASH AT > RODATA_FLASH_LMA
}
