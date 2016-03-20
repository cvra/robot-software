.syntax unified
.cpu cortex-m0
.fpu softvfp
.thumb
.text

.thumb_func
.globl HardFault_Handler
HardFault_Handler:
    mrs r1, msp
    mrs r2, psp

    // save registers on stack
    push {r4-r7, lr}
    mov r4, r8
    mov r5, r9
    mov r6, r10
    mov r7, r11
    push {r4-r7}

    mrs r0, msp
    bl fault_handler
    // never return
