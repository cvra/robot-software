#define TRUE    1
#define FALSE   0

.syntax unified
.cpu cortex-m4

#if (CORTEX_USE_FPU == TRUE)
.fpu fpv4-sp-d16
#else
.fpu softvfp
#endif

.thumb
.text

.thumb_func
.globl HardFault_Handler
HardFault_Handler:
    mrs r1, msp
    mrs r2, psp

    // save registers on stack
#if (CORTEX_USE_FPU == TRUE)
    vpush {s16-s31}
#endif
    push {r4-r7, lr}
    push {r8-r11}

    mrs r0, msp
    b fault_handler
    // never return


.thumb_func
.globl MemManage_Handler
MemManage_Handler:
    b HardFault_Handler

.thumb_func
.globl BusFault_Handler
BusFault_Handler:
    b HardFault_Handler

.thumb_func
.globl UsageFault_Handler
UsageFault_Handler:
    b HardFault_Handler
