#include <ch.h>
#include <stdint.h>

#if (__CORTEX_M != 0x00 && __CORTEX_M != 0x03 && __CORTEX_M != 0x04)
#error "__CORTEX_M version not supported"
#endif

#ifndef ARM_CORTEX_FAULT_FPU_DEBUG
#define ARM_CORTEX_FAULT_FPU_DEBUG 0
#endif

#if (__CORTEX_M == 0x03 || __CORTEX_M == 0x04)
// ARMv7-M Architecture Reference Manual, Chapter B1.5.14 Fault behavior, p. 669
// MemManage Status Register, MMFSR
#define MMFSR_IACCVIOL (1 << 0) // Instruction access violation
#define MMFSR_DACCVIOL (1 << 1) // Data access violation
#define MMFSR_MUNSTKERR (1 << 3) // Unstacking error
#define MMFSR_MSTKERR (1 << 4) // Stacking error
#define MMFSR_MLSPERR (1 << 5) // MemManage Fault during FP lazy state preservation
#define MMFSR_MMARVALID (1 << 7) // Memory Manage Address Register address valid flag
// BusFault Status Register, BFSR
#define BFSR_IBUSERR (1 << 0) // Instruction bus error flag
#define BFSR_PRECISERR (1 << 1) // Precise data bus error
#define BFSR_IMPRECISERR (1 << 2) // Imprecise data bus error
#define BFSR_UNSTKERR (1 << 3) // Unstacking error
#define BFSR_STKERR (1 << 4) // Stacking error
#define BFSR_LSPERR (1 << 5) // Bus Fault during FP lazy state preservation
#define BFSR_BFARVALID (1 << 7) // Bus Fault Address Register address valid flag
// UsageFault Status Register, UFSR
#define UFSR_UNDEFINSTR (1 << 0) // The processor attempt to execute an undefined instruction
#define UFSR_INVSTATE (1 << 1) // Invalid combination of EPSR and instruction
#define UFSR_INVPC (1 << 2) // Attempt to load EXC_RETURN into pc illegally
#define UFSR_NOCP (1 << 3) // Attempt to use a coprocessor instruction
#define UFSR_UNALIGNED (1 << 8) // Fault occurs when there is an attempt to make an unaligned memory access
#define UFSR_DIVBYZERO (1 << 9) // Fault occurs when SDIV or DIV instruction is used with a divisor of 0
// HardFault Status Register, HFSR
#define HFSR_VECTTBL (1 << 1) // Fault occurs because of vector table read on exception processing
#define HFSR_FORCED (1 << 30) // Hard Fault activated when a configurable Fault was received and cannot activate
#define HFSR_DEBUGEVT (1 << 31) // Fault related to debug
#endif

// software saved stack frame
struct arm_soft_frame {
    uint32_t r8;
    uint32_t r9;
    uint32_t r10;
    uint32_t r11;
    uint32_t r4;
    uint32_t r5;
    uint32_t r6;
    uint32_t r7;
    uint32_t lr_irq; // exception return value

#if (__FPU_PRESENT && __FPU_USED)
    uint32_t s16;
    uint32_t s17;
    uint32_t s18;
    uint32_t s19;
    uint32_t s20;
    uint32_t s21;
    uint32_t s22;
    uint32_t s23;
    uint32_t s24;
    uint32_t s25;
    uint32_t s26;
    uint32_t s27;
    uint32_t s28;
    uint32_t s29;
    uint32_t s30;
    uint32_t s31;
#endif
};

// hardware saved stack frame
struct arm_irq_frame {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr; // return address of fault context
    uint32_t pc; // fault address
    uint32_t psr;

#if (__FPU_PRESENT && __FPU_USED)
    uint32_t s0;
    uint32_t s1;
    uint32_t s2;
    uint32_t s3;
    uint32_t s4;
    uint32_t s5;
    uint32_t s6;
    uint32_t s7;
    uint32_t s8;
    uint32_t s9;
    uint32_t s10;
    uint32_t s11;
    uint32_t s12;
    uint32_t s13;
    uint32_t s14;
    uint32_t s15;
    uint32_t fpscr;
    uint32_t reserved;
#endif
};

// must be implemented by user
extern void fault_printf(const char* fmt, ...);

static const char* fault_name(void)
{
#if (__CORTEX_M == 0x00)
    return "HardFault";
#elif (__CORTEX_M == 0x03 || __CORTEX_M == 0x04)
    static const char* faults[] = {"HardFault", "MemManageFault", "BusFault", "UsageFault"};
    uint32_t isr = __get_IPSR();
    if (isr >= 3 && isr <= 6) {
        return faults[isr - 3];
    } else {
        return "unknown";
    }
#endif
}

void fault_handler(struct arm_soft_frame* soft_frame, void* msp, void* psp)
{
    // stack pointer
    void* sp = ((soft_frame->lr_irq & 0x4) == 0) ? msp : psp;
    struct arm_irq_frame* irq_frame = (struct arm_irq_frame*)sp;

    uint32_t fault_stack = (uint32_t)sp + 0x20;
    // see ARMv7-M, Chapter B1.5.8 Exception return behavior, p. 652
    if (!(soft_frame->lr_irq & (1 << 4))) {
        fault_stack += 0x68; // extended frame with FPU registers
    }
    if (irq_frame->psr & (1 << 9)) {
        fault_stack += 0x04; // 8-byte auto alignment
    }

    const char* fault = fault_name();
    fault_printf("%s at %08x, stack: %08x, called from %08x\n", fault,
                 irq_frame->pc, fault_stack, irq_frame->lr);
    fault_printf("r0-r7:  %08x %08x %08x %08x %08x %08x %08x %08x\n",
                 irq_frame->r0, irq_frame->r1, irq_frame->r2, irq_frame->r3,
                 soft_frame->r4, soft_frame->r5, soft_frame->r6, soft_frame->r7);
    fault_printf("r8-r12: %08x %08x %08x %08x %08x\n", soft_frame->r8,
                 soft_frame->r9, soft_frame->r10, soft_frame->r11, irq_frame->r12);
    fault_printf("msp: %08x psp: %08x lr: %08x pc: %08x lr (IRQ): %08x\n",
                 (uint32_t)msp, (uint32_t)psp, irq_frame->lr, irq_frame->pc,
                 soft_frame->lr_irq);

    // FPU state
#if (__FPU_PRESENT && __FPU_USED && ARM_CORTEX_FAULT_FPU_DEBUG)
    fault_printf("FPSCR:   %08x\n", irq_frame->fpscr);
    fault_printf("s0-s7:   %08x %08x %08x %08x %08x %08x %08x %08x\n",
                 irq_frame->s0, irq_frame->s1, irq_frame->s2, irq_frame->s3,
                 irq_frame->s4, irq_frame->s5, irq_frame->s6, irq_frame->s7);
    fault_printf("s8-s15:  %08x %08x %08x %08x %08x %08x %08x %08x\n",
                 irq_frame->s8, irq_frame->s9, irq_frame->s10, irq_frame->s11,
                 irq_frame->s12, irq_frame->s13, irq_frame->s14, irq_frame->s15);
    fault_printf("s16-s23: %08x %08x %08x %08x %08x %08x %08x %08x\n",
                 soft_frame->s16, soft_frame->s17, soft_frame->s18, soft_frame->s19,
                 soft_frame->s20, soft_frame->s21, soft_frame->s22, soft_frame->s23);
    fault_printf("s24-s31: %08x %08x %08x %08x %08x %08x %08x %08x\n",
                 soft_frame->s24, soft_frame->s25, soft_frame->s26, soft_frame->s27,
                 soft_frame->s28, soft_frame->s29, soft_frame->s30, soft_frame->s31);
#endif

    // detailed debug info
#if (__CORTEX_M == 0x03 || __CORTEX_M == 0x04)
    uint16_t UFSR = (SCB->CFSR >> 16) & 0xffff;
    uint8_t BFSR = (SCB->CFSR >> 8) & 0xff;
    uint8_t MMFSR = (SCB->CFSR) & 0xff;

    fault_printf("UFSR: %04x BFSR: %02x MMFSR: %02x\n", UFSR, BFSR, MMFSR);

    if (UFSR & UFSR_UNALIGNED) {
        fault_printf("\"Unaligned memory access\"\n");
    }
    if (UFSR & UFSR_DIVBYZERO) {
        fault_printf("\"Division by zero\"\n");
    }
    if (MMFSR & MMFSR_IACCVIOL) {
        fault_printf("\"Instruction access violation\"\n");
    }
    if (MMFSR & MMFSR_DACCVIOL) {
        fault_printf("\"Data access violation\"\n");
    }
    if (MMFSR & MMFSR_MMARVALID) {
        fault_printf("address: 0x%08x\n", SCB->MMFAR);
    }
    if (BFSR & BFSR_BFARVALID) {
        fault_printf("address: 0x%08x\n", SCB->BFAR);
    }
#endif

    chSysHalt(fault);
}

void fault_init(void)
{
#if (__CORTEX_M == 0x03 || __CORTEX_M == 0x04)
    chSysLock();
    // enable UsageFault, BusFault, MemManageFault
    SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;
    // enable fault on division by zero
    SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk;
    chSysUnlock();
#endif
}
