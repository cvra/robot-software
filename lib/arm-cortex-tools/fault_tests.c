#include <stdint.h>
#include <arm-cortex/mpu.h>

void null_pointer_fault(void)
{
    int* addr = NULL;
    *addr = 42; // NULL pointer dereference

    // won't be here
}

void execute_never_fault(void)
{
    static uint32_t buffer[300];

    /* The last false means that memory is non executable. */
    mpu_configure_region(1, &buffer[128], 5, AP_RW_RW, false);

    void (*foo)(void) = (void (*)(void)) & buffer[128];
    foo(); // Jump to non executable memory

    // won't be here
}

void check_register_values(void)
{
    /* Test if the register values in the fault context log are correct.
     * r0 = 0, r1 = 0, r3 = 0 ..., r12 = 12 */
    __asm__ volatile(
        "ldr r0, =0x0 \n\t"
        "ldr r1, =0x1 \n\t"
        "ldr r2, =0x2 \n\t"
        "ldr r3, =0x3 \n\t"
        "ldr r4, =0x4 \n\t"
        "ldr r5, =0x5 \n\t"
        "ldr r6, =0x6 \n\t"
        "ldr r7, =0x7 \n\t"
        "ldr r8, =0x8 \n\t"
        "ldr r9, =0x9 \n\t"
        "ldr r10, =0xa \n\t"
        "ldr r11, =0xb \n\t"
        "ldr r12, =0xc \n\t"
        "blx r0" // jump to address NULL
    );

    // won't be here
}
