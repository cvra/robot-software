#include <stdint.h>
#include <stddef.h>
#include <ch.h>
#include "panic_log.h"

#define SCB_CFSR    (*((volatile uint32_t *)0xE000ED28)) // Configurable Fault Status Register
#define SCB_HFSR    (*((volatile uint32_t *)0xE000ED2C)) // HardFault Status Register
#define SCB_MMFAR   (*((volatile uint32_t *)0xE000ED34)) // MemManage Fault Address Register
#define SCB_BFAR    (*((volatile uint32_t *)0xE000ED38)) // BusFault Address Register

// ARMv7-M Architecture Reference Manual, Chapter B1.5.14 Fault behavior, p. 669
// MemManage Status Register, MMFSR
#define MMFSR_IACCVIOL      (1<<0) // Instruction access violation
#define MMFSR_DACCVIOL      (1<<1) // Data access violation
#define MMFSR_MUNSTKERR     (1<<3) // Unstacking error
#define MMFSR_MSTKERR       (1<<4) // Stacking error
#define MMFSR_MLSPERR       (1<<5) // MemManage Fault during FP lazy state preservation
#define MMFSR_MMARVALID     (1<<7) // Memory Manage Address Register address valid flag
// BusFault Status Register, BFSR
#define BFSR_IBUSERR        (1<<0) // Instruction bus error flag
#define BFSR_PRECISERR      (1<<1) // Precise data bus error
#define BFSR_IMPRECISERR    (1<<2) // Imprecise data bus error
#define BFSR_UNSTKERR       (1<<3) // Unstacking error
#define BFSR_STKERR         (1<<4) // Stacking error
#define BFSR_LSPERR         (1<<5) // Bus Fault during FP lazy state preservation
#define BFSR_BFARVALID      (1<<7) // Bus Fault Address Register address valid flag
// UsageFault Status Register, UFSR
#define UFSR_UNDEFINSTR     (1<<0) // The processor attempt to execute an undefined instruction
#define UFSR_INVSTATE       (1<<1) // Invalid combination of EPSR and instruction
#define UFSR_INVPC          (1<<2) // Attempt to load EXC_RETURN into pc illegally
#define UFSR_NOCP           (1<<3) // Attempt to use a coprocessor instruction
#define UFSR_UNALIGNED      (1<<8) // Fault occurs when there is an attempt to make an unaligned memory access
#define UFSR_DIVBYZERO      (1<<9) // Fault occurs when SDIV or DIV instruction is used with a divisor of 0
// HardFault Status Register, HFSR
#define HFSR_VECTTBL        (1<<1)  // Fault occurs because of vector table read on exception processing
#define HFSR_FORCED         (1<<30) // Hard Fault activated when a configurable Fault was received and cannot activate
#define HFSR_DEBUGEVT       (1<<31) // Fault related to debug

struct fault_context {
    // software saved context
    uint32_t r4;
    uint32_t r5;
    uint32_t r6;
    uint32_t r7;
    uint32_t r8;
    uint32_t r9;
    uint32_t r10;
    uint32_t r11;
    uint32_t lr_exc_return; // exception return value

    // hardware saved context
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr; // return address of fault context
    uint32_t pc; // fault address
    uint32_t psr;
};

__attribute__ ((naked))
void HardFault_Handler(void)
{
    __asm__ volatile (
        "mrs r0, MSP            \n\t"
        "mrs r1, PSP            \n\t"
        "tst lr, #4             \n\t"
        "it ne                  \n\t" // check bit 2, if 1 then PSP
        "movne r0, r1           \n\t" // select stack pointer
        "stmdb r0!, {r4-r11,lr} \n\t" // store rest of context on stack
        "push {r0}              \n\t"
        "bl fault_handler       \n\t" // call C fault handler
        "pop {r0}               \n\t"
        "ldmia r0!, {r4-r11,lr} \n\t" // rstore context
        "bx lr                  \n\t" // return
        :::
    );
}

__attribute__ ((naked))
void NMI_Handler(void)
{
    // reboot. I don't have any better idea what to do here.
    NVIC_SystemReset();
}

__attribute__ ((naked))
void MemManage_Handler(void)
{
    HardFault_Handler();
}

__attribute__ ((naked))
void BusFault_Handler(void)
{
    HardFault_Handler();
}

__attribute__ ((naked))
void UsageFault_Handler(void)
{
    HardFault_Handler();
}

void fault_handler(void *sp)
{
    struct fault_context *ctx = (struct fault_context *)sp;

    // get fault name
    const char *isr;
    uint32_t isr_nb = __get_IPSR();
    if (isr_nb < 3 || isr_nb > 6) {
        isr = "unknown fault";
    } else {
        const char *fault_name[] = {"HardFault", "MemManageFault", "BusFault", "UsageFault"};
        isr = fault_name[isr_nb - 3];
    }

    uintptr_t old_sp = (uintptr_t)sp;
    // see ARMv7-M, Chapter B1.5.8 Exception return behavior, p. 652
    if (ctx->lr_exc_return & (1<<4)) {
        old_sp += 0x20; // basic frame
    } else {
        old_sp += 0x68; // extended frame with FPU registers
    }
    if (ctx->psr & (1<<9)) {
        old_sp += 0x04; // correct 8-byte auto alignment
    }

    panic_log_printf("%s at %08x, stack: %08x, called from %08x\n", isr, ctx->pc, old_sp, ctx->lr);
    panic_log_printf("r0-r7:  %08x %08x %08x %08x %08x %08x %08x %08x\n", ctx->r0, ctx->r1, ctx->r2, ctx->r3, ctx->r4, ctx->r5, ctx->r6, ctx->r7);
    panic_log_printf("r8-r12: %08x %08x %08x %08x %08x\n", ctx->r8, ctx->r9, ctx->r10, ctx->r11, ctx->r12);
    panic_log_printf("sp: %08x lr: %08x pc: %08x\n", old_sp, ctx->lr, ctx->pc);

    uint8_t BFSR = (SCB_CFSR >> 8) & 0xff;
    uint8_t MMFSR = (SCB_CFSR) & 0xff;

    panic_log_printf("CFSR: %08x\n", SCB_CFSR);

    if (MMFSR & MMFSR_MMARVALID) {
        panic_log_printf("MemManageFault: 0x%08x\n", SCB_MMFAR);
    }
    if (BFSR & BFSR_BFARVALID) {
        panic_log_printf("BusFault: 0x%08x\n", SCB_BFAR);
    }

    chSysHalt(isr);
}
