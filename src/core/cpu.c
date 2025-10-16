/*
 * Copyright 2025 Nuo Shen, Nanjing University
 *
 * This file is part of the UEMU project.
 *
 * The majority of this file is developed by Nuo Shen under the Apache License,
 * Version 2.0. However, certain portions of this file are adapted from the NEMU
 * project (Copyright (c) 2014-2024 Zihao Yu, Nanjing University), which is
 * licensed under the Mulan PSL v2.
 *
 * Therefore, this file as a whole is distributed under both licenses:
 *   - Apache License 2.0 for the original portions by Nuo Shen
 *   - Mulan PSL v2 for the adapted portions from NEMU
 *
 * You may obtain copies of both licenses at:
 *   - Apache License 2.0: http://www.apache.org/licenses/LICENSE-2.0
 *   - Mulan PSL v2:      http://license.coscl.org.cn/MulanPSL2
 *
 * When redistributing this file or a derived work, you must comply with
 * both licenses accordingly.
 */

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "core/cpu.h"
#include "core/decode.h"
#include "core/mem.h"
#include "core/riscv.h"
#include "device/clint.h"
#include "device/goldfish_battery.h"
#include "device/goldfish_rtc.h"
#include "device/uart8250.h"
#include "ui/ui.h"
#include "utils/alarm.h"
#include "utils/logger.h"
#include "utils/slowtimer.h"

void cpu_raise_exception(exception_t cause, uint64_t tval) {
    assert(((uint64_t)cause & INTERRUPT_FLAG) == 0);

    privilege_level_t priv = rv.privilege;
    if (((rv.MEDELEG >> cause) & 1) && (priv == PRIV_U || priv == PRIV_S)) {
        // Shift to S mode
        rv.privilege = PRIV_S;

        rv.decode.npc = cpu_read_csr(CSR_STVEC) & ~3ULL;
        cpu_write_csr(CSR_SEPC, rv.decode.pc);
        cpu_write_csr(CSR_SCAUSE, cause);
        cpu_write_csr(CSR_STVAL, tval);

        uint64_t sstatus = cpu_read_csr(CSR_SSTATUS);
        // Save current SIE to SPIE
        if (sstatus & SSTATUS_SIE)
            sstatus |= SSTATUS_SPIE;
        else
            sstatus &= ~SSTATUS_SPIE;
        // Save PRIV level to SPP
        sstatus &= ~SSTATUS_SPP;
        sstatus |= ((uint64_t)priv << SSTATUS_SPP_SHIFT);
        // Disable S mode interrupt
        sstatus &= ~SSTATUS_SIE;
        // Update
        cpu_write_csr(CSR_SSTATUS, sstatus);
    } else {
        // Shift to M Mode
        rv.privilege = PRIV_M;

        rv.decode.npc = cpu_read_csr(CSR_MTVEC) & ~3ULL;
        cpu_write_csr(CSR_MEPC, rv.decode.pc);
        cpu_write_csr(CSR_MCAUSE, cause);
        cpu_write_csr(CSR_MTVAL, tval);

        uint64_t mstatus = cpu_read_csr(CSR_MSTATUS);
        // Save current MIE to MPIE
        if (mstatus & MSTATUS_MIE)
            mstatus |= MSTATUS_MPIE;
        else
            mstatus &= ~MSTATUS_MPIE;
        // Save PRIV level to MPP
        mstatus &= ~MSTATUS_MPP;
        mstatus |= ((uint64_t)priv << MSTATUS_MPP_SHIFT);
        // Disable M mode interrupt
        mstatus &= ~MSTATUS_MIE;
        // Update
        cpu_write_csr(CSR_MSTATUS, mstatus);
    }

    rv.last_exception = cause;
}

void cpu_raise_intr(uint64_t ip, privilege_level_t priv) {
    pthread_mutex_lock(&rv.csr_lock);
    if (priv == PRIV_M)
        rv.MIP |= ip;
    else if (PRIV_S)
        rv.MIP |= ip & rv.MIDELEG;
    else
        __UNREACHABLE;
    pthread_mutex_unlock(&rv.csr_lock);
}

void cpu_clear_intr(uint64_t ip, privilege_level_t priv) {
    pthread_mutex_lock(&rv.csr_lock);
    if (priv == PRIV_M)
        rv.MIP &= ~ip;
    else if (priv == PRIV_S)
        rv.MIP &= ~(rv.MIDELEG & ip);
    else
        __UNREACHABLE;
    pthread_mutex_unlock(&rv.csr_lock);
}

/**
 * @brief Processes an interruption.
 *
 * This function should only be used during instruction execution.
 *
 * @param intr  The interruption number.
 */
FORCE_INLINE void cpu_process_intr(interrupt_t intr) {
    assert((uint64_t)intr & INTERRUPT_FLAG);

    privilege_level_t priv = rv.privilege;
    uint64_t cause = (uint64_t)intr & ~INTERRUPT_FLAG;
    bool mideleg_flag = (cpu_read_csr(CSR_MIDELEG) >> cause) & 1;

    if (cause == CAUSE_MACHINE_TIMER)
        mideleg_flag = false;

    if (mideleg_flag && (priv == PRIV_U || priv == PRIV_S)) {
        rv.privilege = PRIV_S;
        uint64_t vt_offset = 0;
        uint64_t stvec = cpu_read_csr(CSR_STVEC);
        if (stvec & 1)
            vt_offset = cause << 2;
        cpu_write_csr(CSR_SEPC, rv.PC);
        rv.PC = (stvec & ~3ULL) + vt_offset;
        cpu_write_csr(CSR_SCAUSE, intr);

        uint64_t sstatus = cpu_read_csr(CSR_SSTATUS);
        if (sstatus & SSTATUS_SIE)
            sstatus |= SSTATUS_SPIE;
        else
            sstatus &= ~SSTATUS_SPIE;
        sstatus &= ~SSTATUS_SIE;
        sstatus &= ~SSTATUS_SPP;
        sstatus |= ((uint64_t)priv << SSTATUS_SPP_SHIFT);
        cpu_write_csr(CSR_SSTATUS, sstatus);
    } else {
        rv.privilege = PRIV_M;
        uint64_t vt_offset = 0;
        uint64_t mtvec = cpu_read_csr(CSR_MTVEC);
        if (mtvec & 1)
            vt_offset = cause << 2;
        cpu_write_csr(CSR_MEPC, rv.PC);
        rv.PC = (mtvec & ~3ULL) + vt_offset;
        cpu_write_csr(CSR_MCAUSE, intr);

        uint64_t mstatus = cpu_read_csr(CSR_MSTATUS);
        if (mstatus & MSTATUS_MIE)
            mstatus |= MSTATUS_MPIE;
        else
            mstatus &= ~MSTATUS_MPIE;
        mstatus &= ~MSTATUS_MIE;
        mstatus &= ~MSTATUS_MPP;
        mstatus |= ((uint64_t)priv << MSTATUS_MPP_SHIFT);
        cpu_write_csr(CSR_MSTATUS, mstatus);
    }
}

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define R(i) rv.X[i]
#define F(i) rv.F[i]

typedef enum {
    // 32 bit
    TYPE_I,
    TYPE_U,
    TYPE_S,
    TYPE_J,
    TYPE_R,
    TYPE_B,
    TYPE_R4,

    // 16 bit
    TYPE_CR,
    TYPE_CI,
    TYPE_CSS,
    TYPE_CIW,
    TYPE_CL,
    TYPE_CS,
    TYPE_CA,
    TYPE_CB,
    TYPE_CJ,

    TYPE_N, // none
} inst_type_t;

#define REG_C(x) ((x) + 8)

#define immI()                                                                 \
    do {                                                                       \
        *imm = SEXT(BITS(i, 31, 20), 12);                                      \
    } while (0)
#define immU()                                                                 \
    do {                                                                       \
        *imm = SEXT(BITS(i, 31, 12), 20) << 12;                                \
    } while (0)
#define immS()                                                                 \
    do {                                                                       \
        *imm = (SEXT(BITS(i, 31, 25), 7) << 5) | BITS(i, 11, 7);               \
    } while (0)
#define immJ()                                                                 \
    do {                                                                       \
        *imm = SEXT(BITS(i, 31, 31) << 20 | BITS(i, 19, 12) << 12 |            \
                        BITS(i, 20, 20) << 11 | BITS(i, 30, 21) << 1,          \
                    21);                                                       \
    } while (0)
#define immB()                                                                 \
    do {                                                                       \
        *imm = SEXT(BITS(i, 31, 31) << 12 | BITS(i, 7, 7) << 11 |              \
                        BITS(i, 30, 25) << 5 | BITS(i, 11, 8) << 1,            \
                    13);                                                       \
    } while (0)
#define immCI()                                                                \
    do {                                                                       \
        *imm = SEXT(BITS(i, 12, 12) << 5 | BITS(i, 6, 2), 6);                  \
    } while (0)
#define immCIW()                                                               \
    do {                                                                       \
        *imm = BITS(i, 10, 7) << 6 | BITS(i, 12, 11) << 4 |                    \
               BITS(i, 5, 5) << 3 | BITS(i, 6, 6) << 2;                        \
    } while (0)
#define immCB()                                                                \
    do {                                                                       \
        *imm = SEXT(BITS(i, 12, 12) << 8 | BITS(i, 6, 5) << 6 |                \
                        BITS(i, 2, 2) << 5 | BITS(i, 11, 10) << 3 |            \
                        BITS(i, 4, 3) << 1,                                    \
                    9);                                                        \
    } while (0)
#define immCJ()                                                                \
    do {                                                                       \
        *imm = SEXT(BITS(i, 12, 12) << 11 | BITS(i, 11, 11) << 4 |             \
                        BITS(i, 10, 9) << 8 | BITS(i, 8, 8) << 10 |            \
                        BITS(i, 7, 7) << 6 | BITS(i, 6, 6) << 7 |              \
                        BITS(i, 5, 3) << 1 | BITS(i, 2, 2) << 5,               \
                    12);                                                       \
    } while (0)
#define immCIU()                                                               \
    do {                                                                       \
        *imm = (BITS(i, 12, 12) << 5) | BITS(i, 6, 2);                         \
    } while (0)
#define immCLWSP()                                                             \
    do {                                                                       \
        *imm = BITS(i, 3, 2) << 6 | BITS(i, 12, 12) << 5 | BITS(i, 6, 4) << 2; \
    } while (0)
#define immCLDSP()                                                             \
    do {                                                                       \
        *imm = BITS(i, 4, 2) << 6 | BITS(i, 12, 12) << 5 | BITS(i, 6, 5) << 3; \
    } while (0)
#define immCSWSP()                                                             \
    do {                                                                       \
        *imm = BITS(i, 8, 7) << 6 | BITS(i, 12, 9) << 2;                       \
    } while (0)
#define immCSDSP()                                                             \
    do {                                                                       \
        *imm = BITS(i, 9, 7) << 6 | BITS(i, 12, 10) << 3;                      \
    } while (0)
#define immCLUI()                                                              \
    do {                                                                       \
        *imm = SEXT(BITS(i, 12, 12) << 17 | BITS(i, 6, 2) << 12, 18);          \
    } while (0)
#define immCADDI16SP()                                                         \
    do {                                                                       \
        *imm = SEXT(BITS(i, 12, 12) << 9 | BITS(i, 4, 3) << 7 |                \
                        BITS(i, 5, 5) << 6 | BITS(i, 2, 2) << 5 |              \
                        BITS(i, 6, 6) << 4,                                    \
                    10);                                                       \
    } while (0)
#define immCLW() /* c.lw */                                                    \
    do {                                                                       \
        *imm = (BITS(i, 5, 5) << 6) | (BITS(i, 12, 10) << 3) |                 \
               (BITS(i, 6, 6) << 2);                                           \
    } while (0)
#define immCLD() /* c.fld, c.ld */                                             \
    do {                                                                       \
        *imm = (BITS(i, 6, 5) << 6) | (BITS(i, 12, 10) << 3);                  \
    } while (0)
#define immCSW() /* c.sw */                                                    \
    do {                                                                       \
        *imm = (BITS(i, 5, 5) << 6) | (BITS(i, 12, 10) << 3) |                 \
               (BITS(i, 6, 6) << 2);                                           \
    } while (0)
#define immCSD() /* c.sd, c.fsd */                                             \
    do {                                                                       \
        *imm = (BITS(i, 6, 5) << 6) | (BITS(i, 12, 10) << 3);                  \
    } while (0)
#define immCBS() /* c.srli, c.srai, c.andi */                                  \
    do {                                                                       \
        uint32_t raw = (BITS(i, 12, 12) << 5) | BITS(i, 6, 2);                 \
        if (BITS(i, 11, 10) == 0b10)                                           \
            *imm = SEXT(raw, 6);                                               \
        else                                                                   \
            *imm = raw;                                                        \
    } while (0)

FORCE_INLINE void decode_operand_32(Decode *s, int *rd, int *rs1, int *rs2,
                                    int *rs3, uint64_t *imm, inst_type_t type) {
    uint32_t i = s->inst;
    *rs1 = BITS(i, 19, 15);
    *rs2 = BITS(i, 24, 20);
    *rs3 = BITS(i, 31, 27);
    *rd = BITS(i, 11, 7);
    switch (type) {
        case TYPE_I: immI(); break;
        case TYPE_U: immU(); break;
        case TYPE_S: immS(); break;
        case TYPE_N: break;
        case TYPE_J: immJ(); break;
        case TYPE_R: break;
        case TYPE_B: immB(); break;
        case TYPE_R4: break;
        default: __UNREACHABLE;
    }
}

FORCE_INLINE void decode_operand_16(Decode *s, int *rd, int *rs1, int *rs2,
                                    uint64_t *imm, inst_type_t type) {
    uint16_t i = s->inst & 0xffff;
    uint8_t funct3;
    uint8_t opcode;
    switch (type) {
        case TYPE_CR:
            *rd = BITS(i, 11, 7);
            *rs1 = BITS(i, 11, 7);
            *rs2 = BITS(i, 6, 2);
            break;
        case TYPE_CI:
            *rs1 = *rd = BITS(i, 11, 7);
            funct3 = BITS(i, 15, 13);
            opcode = BITS(i, 1, 0);
            if (opcode == 0b01) {
                if (funct3 ==
                    0b000) { /* C.ADDI (or C.NOP if rd==0 && imm==0) */
                    immCI();
                } else if (funct3 == 0b001) { /* C.ADDIW */
                    immCI();
                } else if (funct3 == 0b010) { /* C.LI */
                    immCI();
                } else if (funct3 == 0b011) {
                    if (*rd == 2) {
                        immCADDI16SP(); /* C.ADDI16SP */
                    } else if (*rd != 0) {
                        immCLUI(); /* C.LUI for rd != 0,2 */
                    } else {
                        immCI();
                    }
                } else {
                    immCI();
                }
            } else if (opcode == 0b10) {
                if (funct3 == 0b000) { /* C.SLLI */
                    immCIU();
                } else if (funct3 == 0b010) { /* C.LWSP */
                    immCLWSP();
                } else if (funct3 == 0b011 ||
                           funct3 == 0b001) { /* C.LDSP C.FLDSP */
                    immCLDSP();
                } else {
                    immCI();
                }
            } else {
                immCI();
            }
            break;
        case TYPE_CSS:
            *rs1 = 2; // sp
            *rs2 = BITS(i, 6, 2);
            funct3 = BITS(i, 15, 13);
            if (funct3 == 0b110) { // C.SWSP
                immCSWSP();
            } else if (funct3 == 0b111 || funct3 == 0b101) { // C.SDSP C.FSDSP
                immCSDSP();
            }
            break;
        case TYPE_CIW:
            *rd = REG_C(BITS(i, 4, 2));
            *rs1 = 2; // sp
            immCIW();
            break;
        case TYPE_CL:
            *rd = REG_C(BITS(i, 4, 2));
            *rs1 = REG_C(BITS(i, 9, 7));
            funct3 = BITS(i, 15, 13);
            if (funct3 == 0b010) {
                immCLW();
            } else if (funct3 == 0b001 || funct3 == 0b011) {
                immCLD();
            } else {
                __UNREACHABLE;
            }
            break;
        case TYPE_CS:
            *rs1 = REG_C(BITS(i, 9, 7));
            *rs2 = REG_C(BITS(i, 4, 2));
            funct3 = BITS(i, 15, 13);
            if (funct3 == 0b110) {
                immCSW();
            } else if (funct3 == 0b101 || funct3 == 0b111) {
                immCSD();
            } else {
                __UNREACHABLE;
            }
            break;
        case TYPE_CA:
            *rd = REG_C(BITS(i, 9, 7));
            *rs1 = REG_C(BITS(i, 9, 7));
            *rs2 = REG_C(BITS(i, 4, 2));
            break;
        case TYPE_CB:
            *rd = *rs1 = REG_C(BITS(i, 9, 7));
            funct3 = BITS(i, 15, 13);
            if (funct3 == 0b100)
                immCBS();
            else
                immCB();
            break;
        case TYPE_CJ: immCJ(); break;
        case TYPE_N: break;
        default: __UNREACHABLE;
    }
}

FORCE_INLINE void _ecall(Decode *s) {
    switch (rv.privilege) {
        case PRIV_M: cpu_raise_exception(CAUSE_MACHINE_ECALL, s->pc); break;
        case PRIV_S: cpu_raise_exception(CAUSE_SUPERVISOR_ECALL, s->pc); break;
        case PRIV_U: cpu_raise_exception(CAUSE_USER_ECALL, s->pc); break;
        default: __UNREACHABLE;
    }
}

FORCE_INLINE void _mret(Decode *s) {
    if (rv.privilege != PRIV_M) {
        cpu_raise_exception(CAUSE_ILLEGAL_INSTRUCTION, s->inst);
        return;
    }
    s->npc = cpu_read_csr(CSR_MEPC);
    uint64_t mstatus = cpu_read_csr(CSR_MSTATUS);

    // Restore PRIV level
    rv.privilege =
        (privilege_level_t)((mstatus & MSTATUS_MPP) >> MSTATUS_MPP_SHIFT);

    if (rv.privilege != PRIV_M)
        mstatus &= ~MSTATUS_MPRV;

    // Restore MIE
    if (mstatus & MSTATUS_MPIE)
        mstatus |= MSTATUS_MIE;
    else
        mstatus &= ~MSTATUS_MIE;

    mstatus |= MSTATUS_MPIE;

    mstatus &= ~MSTATUS_MPP;

    cpu_write_csr(CSR_MSTATUS, mstatus);
}

FORCE_INLINE void _sret(Decode *s) {
    // When TSR=1, this operation is not permitted in S-mode
    if ((rv.privilege == PRIV_S && (rv.MSTATUS & MSTATUS_TSR)) ||
        rv.privilege == PRIV_U) {
        cpu_raise_exception(CAUSE_ILLEGAL_INSTRUCTION, s->inst);
        return;
    }
    s->npc = cpu_read_csr(CSR_SEPC);
    uint64_t sstatus = cpu_read_csr(CSR_SSTATUS);
    rv.privilege =
        (privilege_level_t)((sstatus & SSTATUS_SPP) >> SSTATUS_SPP_SHIFT);

    // Looks weird, but this is what the riscv spec requires
    if (rv.privilege != PRIV_M)
        cpu_write_csr(CSR_MSTATUS, cpu_read_csr(CSR_MSTATUS) & ~MSTATUS_MPRV);

    if (sstatus & SSTATUS_SPIE)
        sstatus |= SSTATUS_SIE;
    else
        sstatus &= ~SSTATUS_SIE;

    sstatus |= SSTATUS_SPIE;

    sstatus &= ~SSTATUS_SPP;

    cpu_write_csr(CSR_SSTATUS, sstatus);
}

FORCE_INLINE void _sfence_vma(Decode *s) {
    // TODO: TLB
    ;
}

#define LOAD(rd, addr, type, sz_type, sz)                                      \
    do {                                                                       \
        type __v = vaddr_read_##sz_type(addr);                                 \
        if (rv.last_exception == CAUSE_EXCEPTION_NONE) {                       \
            R(rd) = (uint64_t)__v;                                             \
        }                                                                      \
    } while (0)

#define LOAD_SEXT(rd, addr, type, sz_type, sz)                                 \
    do {                                                                       \
        type __v = vaddr_read_##sz_type(addr);                                 \
        if (rv.last_exception == CAUSE_EXCEPTION_NONE) {                       \
            R(rd) = SEXT(__v, sz);                                             \
        }                                                                      \
    } while (0)

#define CSR_CHECK_PERM(csr)                                                    \
    do {                                                                       \
        if (((((csr) >> 8) & 0x3) == 3 && rv.privilege < PRIV_M) ||            \
            ((((csr) >> 8) & 0x3) == 1 && rv.privilege < PRIV_S))              \
            cpu_raise_exception(CAUSE_ILLEGAL_INSTRUCTION, rv.decode.pc);      \
    } while (0)

#define FP_INST_PREP()                                                         \
    do {                                                                       \
        assert(softfloat_exceptionFlags == 0);                                 \
        if ((rv.MSTATUS & MSTATUS_FS) == 0)                                    \
            cpu_raise_exception(CAUSE_ILLEGAL_INSTRUCTION, rv.decode.pc);      \
    } while (0)

#define FP_SETUP_RM()                                                          \
    do {                                                                       \
        uint64_t rm = BITS(s->inst, 14, 12);                                   \
        if (rm == FRM_DYN)                                                     \
            rm = rv.FCSR.fields.frm;                                           \
        if (unlikely(rm > FRM_RMM))                                            \
            cpu_raise_exception(CAUSE_ILLEGAL_INSTRUCTION, rv.decode.pc);      \
        else                                                                   \
            softfloat_roundingMode = rm;                                       \
    } while (0)

#define FP_SET_DIRTY()                                                         \
    do {                                                                       \
        rv.MSTATUS |= MSTATUS_SD;                                              \
        rv.MSTATUS |= MSTATUS_FS;                                              \
    } while (0)

#define FP_UPDATE_EXCEPTION_FLAGS()                                            \
    do {                                                                       \
        if (softfloat_exceptionFlags) {                                        \
            FP_SET_DIRTY();                                                    \
            rv.FCSR.fields.fflags |= softfloat_exceptionFlags;                 \
            softfloat_exceptionFlags = 0;                                      \
        }                                                                      \
    } while (0)

#define FP_INST_END()                                                          \
    do {                                                                       \
        FP_SET_DIRTY();                                                        \
        FP_UPDATE_EXCEPTION_FLAGS();                                           \
    } while (0)

#define INSTPAT_INST(s) ((s)->inst)

#define INSTPAT_MATCH(s, name, type, ... /* execute body */)                   \
    {                                                                          \
        int rd = 0, rs1 = 0, rs2 = 0, rs3 = 0;                                 \
        uint64_t imm = 0;                                                      \
        decode_operand_32(s, &rd, &rs1, &rs2, &rs3, &imm,                      \
                          concat(TYPE_, type));                                \
        __VA_ARGS__;                                                           \
    }

static inline void decode_exec_32(Decode *s) {
    // FIXME: function ‘decode_exec’ can never be inlined because it contains a
    // computed goto

    // clang-format off
    INSTPAT_START();

    // RV64I instructions
    INSTPAT("0000000 ????? ????? 000 ????? 01100 11", add    , R, R(rd) = R(rs1) + R(rs2));
    INSTPAT("??????? ????? ????? 000 ????? 00100 11", addi   , I, R(rd) = R(rs1) + imm);
    INSTPAT("??????? ????? ????? 000 ????? 00110 11", addiw  , I, R(rd) = SEXT(BITS(R(rs1) + imm, 31, 0), 32));
    INSTPAT("0000000 ????? ????? 000 ????? 01110 11", addw   , R, R(rd) = SEXT(BITS(R(rs1) + R(rs2), 31, 0), 32));
    INSTPAT("0000000 ????? ????? 111 ????? 01100 11", and    , R, R(rd) = R(rs1) & R(rs2));
    INSTPAT("??????? ????? ????? 111 ????? 00100 11", andi   , I, R(rd) = R(rs1) & imm);
    INSTPAT("??????? ????? ????? ??? ????? 00101 11", auipc  , U, R(rd) = s->pc + imm);
    INSTPAT("??????? ????? ????? 000 ????? 11000 11", beq    , B, if (R(rs1) == R(rs2)) s->npc = s->pc + imm;);
    INSTPAT("??????? ????? ????? 101 ????? 11000 11", bge    , B, if ((int64_t)R(rs1) >= (int64_t)R(rs2)) s->npc = s->pc + imm;);
    INSTPAT("??????? ????? ????? 111 ????? 11000 11", bgeu   , B, if (R(rs1) >= R(rs2)) s->npc = s->pc + imm;);
    INSTPAT("??????? ????? ????? 100 ????? 11000 11", blt    , B, if ((int64_t)R(rs1) < (int64_t)R(rs2)) s->npc = s->pc + imm;);
    INSTPAT("??????? ????? ????? 110 ????? 11000 11", bltu   , B, if (R(rs1) < R(rs2)) s->npc = s->pc + imm;);
    INSTPAT("??????? ????? ????? 001 ????? 11000 11", bne    , B, if (R(rs1) != R(rs2)) s->npc = s->pc + imm;);
    INSTPAT("0000??? ????? 00000 000 00000 00011 11", fence  , I, /* nop */);
    INSTPAT("0000000 00000 00000 001 00000 00011 11", fence.i, I, /* nop */);
    INSTPAT("??????? ????? ????? ??? ????? 11011 11", jal    , J, R(rd) = s->pc + 4; s->npc = s->pc + imm;);
    INSTPAT("??????? ????? ????? 000 ????? 11001 11", jalr   , I, uint64_t t = s->pc + 4; s->npc = (R(rs1) + imm) & ~1ULL; R(rd) = t;);
    INSTPAT("??????? ????? ????? 000 ????? 00000 11", lb     , I, LOAD_SEXT(rd, R(rs1) + imm, uint8_t, b, 8));
    INSTPAT("??????? ????? ????? 100 ????? 00000 11", lbu    , I, LOAD(rd, R(rs1) + imm, uint8_t, b, 8));
    INSTPAT("??????? ????? ????? 011 ????? 00000 11", ld     , I, LOAD(rd, R(rs1) + imm, uint64_t, d, 64));
    INSTPAT("??????? ????? ????? 001 ????? 00000 11", lh     , I, LOAD_SEXT(rd, R(rs1) + imm, uint16_t, s, 16));
    INSTPAT("??????? ????? ????? 101 ????? 00000 11", lhu    , I, LOAD(rd, R(rs1) + imm, uint16_t, s, 16));
    INSTPAT("??????? ????? ????? ??? ????? 01101 11", lui    , U, R(rd) = SEXT(BITS(imm, 31, 12) << 12, 32));
    INSTPAT("??????? ????? ????? 010 ????? 00000 11", lw     , I, LOAD_SEXT(rd, R(rs1) + imm, uint32_t, w, 32));
    INSTPAT("??????? ????? ????? 110 ????? 00000 11", lwu    , I, LOAD(rd, R(rs1) + imm, uint32_t, w, 32));
    INSTPAT("0000000 ????? ????? 110 ????? 01100 11", or     , R, R(rd) = R(rs1) | R(rs2));
    INSTPAT("??????? ????? ????? 110 ????? 00100 11", ori    , I, R(rd) = R(rs1) | imm);
    INSTPAT("??????? ????? ????? 000 ????? 01000 11", sb     , S, vaddr_write_b(R(rs1) + imm, R(rs2)));
    INSTPAT("??????? ????? ????? 011 ????? 01000 11", sd     , S, vaddr_write_d(R(rs1) + imm, R(rs2)));
    INSTPAT("??????? ????? ????? 001 ????? 01000 11", sh     , S, vaddr_write_s(R(rs1) + imm, R(rs2)));
    INSTPAT("0000000 ????? ????? 001 ????? 01100 11", sll    , R, R(rd) = R(rs1) << BITS(R(rs2), 5, 0));
    INSTPAT("000000? ????? ????? 001 ????? 00100 11", slli   , I, R(rd) = R(rs1) << BITS(imm, 5, 0));
    INSTPAT("0000000 ????? ????? 001 ????? 00110 11", slliw  , I, R(rd) = SEXT(BITS(R(rs1), 31, 0) << BITS(imm, 4, 0), 32));
    INSTPAT("0000000 ????? ????? 001 ????? 01110 11", sllw   , R, R(rd) = SEXT((uint32_t)BITS(R(rs1), 31, 0) << (BITS(R(rs2), 4, 0)), 32));
    INSTPAT("0000000 ????? ????? 010 ????? 01100 11", slt    , R, R(rd) = (int64_t)R(rs1) < (int64_t)R(rs2));
    INSTPAT("??????? ????? ????? 010 ????? 00100 11", slti   , I, R(rd) = (int64_t)R(rs1) < (int64_t)imm);
    INSTPAT("??????? ????? ????? 011 ????? 00100 11", sltiu  , I, R(rd) = R(rs1) < imm);
    INSTPAT("0000000 ????? ????? 011 ????? 01100 11", sltu   , R, R(rd) = R(rs1) < R(rs2));
    INSTPAT("0100000 ????? ????? 101 ????? 01100 11", sra    , R, R(rd) = (int64_t)R(rs1) >> BITS(R(rs2), 5, 0));
    INSTPAT("010000? ????? ????? 101 ????? 00100 11", srai   , I, R(rd) = (int64_t)R(rs1) >> BITS(imm, 5, 0));
    INSTPAT("0100000 ????? ????? 101 ????? 00110 11", sraiw  , I, R(rd) = SEXT((int32_t)(BITS(R(rs1), 31, 0)) >> BITS(imm, 4, 0), 32));
    INSTPAT("0100000 ????? ????? 101 ????? 01110 11", sraw   , R, R(rd) = SEXT((int32_t)(BITS(R(rs1), 31, 0)) >> BITS(R(rs2), 4, 0), 32));
    INSTPAT("0000000 ????? ????? 101 ????? 01100 11", srl    , R, R(rd) = R(rs1) >> BITS(R(rs2), 5, 0));
    INSTPAT("000000? ????? ????? 101 ????? 00100 11", srli   , I, R(rd) = R(rs1) >> BITS(imm, 5, 0));
    INSTPAT("0000000 ????? ????? 101 ????? 00110 11", srliw  , I, R(rd) = SEXT(BITS(R(rs1), 31, 0) >> BITS(imm, 4, 0), 32));
    INSTPAT("0000000 ????? ????? 101 ????? 01110 11", srlw   , R, R(rd) = SEXT(BITS(R(rs1), 31, 0) >> BITS(R(rs2), 4, 0), 32));
    INSTPAT("0100000 ????? ????? 000 ????? 01100 11", sub    , R, R(rd) = R(rs1) - R(rs2));
    INSTPAT("0100000 ????? ????? 000 ????? 01110 11", subw   , R, R(rd) = SEXT(BITS(R(rs1) - R(rs2), 31, 0), 32));
    INSTPAT("??????? ????? ????? 010 ????? 01000 11", sw     , S, vaddr_write_w(R(rs1) + imm, BITS(R(rs2), 31, 0)));
    INSTPAT("0000000 ????? ????? 100 ????? 01100 11", xor    , R, R(rd) = R(rs1) ^ R(rs2));
    INSTPAT("??????? ????? ????? 100 ????? 00100 11", xori   , I, R(rd) = R(rs1) ^ imm);
    INSTPAT("??????? ????? ????? 011 ????? 11100 11", csrrc   ,I,
        CSR_CHECK_PERM(imm);
        bool lk = cpu_csr_need_lock(imm);
        if (lk)
            pthread_mutex_lock(&rv.csr_lock);
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            uint64_t t = cpu_read_csr(imm);
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                cpu_write_csr(imm, t & ~R(rs1));
                if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                    R(rd) = t;
            }
        }
        if (lk)
            pthread_mutex_unlock(&rv.csr_lock);
    );
    INSTPAT("??????? ????? ????? 111 ????? 11100 11", csrrci  ,I,
        CSR_CHECK_PERM(imm);
        bool lk = cpu_csr_need_lock(imm);
        if (lk)
            pthread_mutex_lock(&rv.csr_lock);
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            uint64_t zimm = BITS(s->inst, 19, 15);
            uint64_t t = cpu_read_csr(imm);
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                cpu_write_csr(imm, t & ~zimm);
                if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                    R(rd) = t;
            }
        }
        if (lk)
            pthread_mutex_unlock(&rv.csr_lock);
    );
    INSTPAT("??????? ????? ????? 010 ????? 11100 11", csrrs  , I,
        CSR_CHECK_PERM(imm);
        bool lk = cpu_csr_need_lock(imm);
        if (lk)
            pthread_mutex_lock(&rv.csr_lock);
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            uint64_t t = cpu_read_csr(imm);
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                cpu_write_csr(imm, t | R(rs1));
                if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                    R(rd) = t;
            }
        }
        if (lk)
            pthread_mutex_unlock(&rv.csr_lock);
    );
    INSTPAT("??????? ????? ????? 110 ????? 11100 11", csrrsi , I,
        CSR_CHECK_PERM(imm);
        bool lk = cpu_csr_need_lock(imm);
        if (lk)
            pthread_mutex_lock(&rv.csr_lock);
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            uint64_t zimm = BITS(s->inst, 19, 15);
            uint64_t t = cpu_read_csr(imm);
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                cpu_write_csr(imm, t | zimm);
                if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                    R(rd) = t;
            }
        }
        if (lk)
            pthread_mutex_unlock(&rv.csr_lock);
    );
    INSTPAT("??????? ????? ????? 001 ????? 11100 11", csrrw  , I,
        CSR_CHECK_PERM(imm);
        bool lk = cpu_csr_need_lock(imm);
        if (lk)
            pthread_mutex_lock(&rv.csr_lock);
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            uint64_t t = cpu_read_csr(imm);
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                cpu_write_csr(imm, R(rs1));
                if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                    R(rd) = t;
            }
        }
        if (lk)
            pthread_mutex_unlock(&rv.csr_lock);
    );
    INSTPAT("??????? ????? ????? 101 ????? 11100 11", csrrwi , I,
        CSR_CHECK_PERM(imm);
        bool lk = cpu_csr_need_lock(imm);
        if (lk)
            pthread_mutex_lock(&rv.csr_lock);
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            uint64_t zimm = BITS(s->inst, 19, 15);
            uint64_t t = cpu_read_csr(imm);
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                R(rd) = t;
                cpu_write_csr(imm, zimm);
            }
        }
        if (lk)
            pthread_mutex_unlock(&rv.csr_lock);
    );
    INSTPAT("0000000 00001 00000 000 00000 11100 11", ebreak     , N, cpu_raise_exception(CAUSE_BREAKPOINT, s->pc));
    INSTPAT("0000000 00000 00000 000 00000 11100 11", ecall      , N, _ecall(s));
    INSTPAT("0011000 00010 00000 000 00000 11100 11", mret       , N, _mret(s));
    INSTPAT("0001001 ????? ????? 000 00000 11100 11", sfence.vma , R, _sfence_vma(s));
    INSTPAT("0001000 00010 00000 000 00000 11100 11", sret       , N, _sret(s));
    INSTPAT("0001000 00101 00000 000 00000 11100 11", wfi        , N, /* nop */);

    // RV64M instructions
    INSTPAT("0000001 ????? ????? 100 ????? 01100 11", div    , R,
        if (unlikely((int64_t)R(rs2) == 0))
            R(rd) = ~0ULL;
        else if (unlikely((int64_t)R(rs1) == INT64_MIN && (int64_t)R(rs2) == -1))
            R(rd) = (int64_t)R(rs1);
        else
            R(rd) = (int64_t)R(rs1) / (int64_t)R(rs2);
    );
    INSTPAT("0000001 ????? ????? 101 ????? 01100 11", divu   , R,
        if (unlikely(R(rs2) == 0))
            R(rd) = ~0ULL;
        else
            R(rd) = R(rs1) / R(rs2);
    );
    INSTPAT("0000001 ????? ????? 101 ????? 01110 11", divuw  , R,
        uint32_t v1 = BITS(R(rs1), 31, 0);
        uint32_t v2 = BITS(R(rs2), 31, 0);
        if (unlikely(v2 == 0))
            R(rd) = ~0ULL;
        else
            R(rd) = SEXT(v1 / v2, 32);
    );
    INSTPAT("0000001 ????? ????? 100 ????? 01110 11", divw   , R,
        int32_t v1 = (int32_t)BITS(R(rs1), 31, 0);
        int32_t v2 = (int32_t)BITS(R(rs2), 31, 0);
        if (unlikely(v2 == 0))
            R(rd) = ~0ULL;
        else if (unlikely(v1 == INT32_MIN && v2 == -1))
            R(rd) = SEXT(v1, 32);
        else
            R(rd) = SEXT(v1 / v2, 32);
    );
    INSTPAT("0000001 ????? ????? 000 ????? 01100 11", mul    , R, R(rd) = R(rs1) * R(rs2));
    INSTPAT("0000001 ????? ????? 001 ????? 01100 11", mulh   , R, 
        R(rd) = (int64_t)(((__int128_t)(int64_t)R(rs1) * (__int128_t)(int64_t)R(rs2)) >> 64)
    );
    INSTPAT("0000001 ????? ????? 010 ????? 01100 11", mulhsu , R, 
        R(rd) = (int64_t)(((__int128_t)(int64_t)R(rs1) * (__uint128_t)R(rs2)) >> 64)
    );
    INSTPAT("0000001 ????? ????? 011 ????? 01100 11", mulhu  , R,
        R(rd) = (uint64_t)((__uint128_t)R(rs1) * (__uint128_t)R(rs2) >> 64)
    );
    INSTPAT("0000001 ????? ????? 000 ????? 01110 11", mulw   , R, R(rd) = SEXT(BITS(R(rs1) * R(rs2), 31, 0), 32));
    INSTPAT("0000001 ????? ????? 110 ????? 01100 11", rem    , R,
        if (unlikely((int64_t)R(rs2) == 0))
            R(rd) = (int64_t)R(rs1);
        else if (unlikely((int64_t)R(rs1) == INT64_MIN && (int64_t)R(rs2) == -1))
            R(rd) = 0;  // overflow case: remainder is 0
        else
            R(rd) = (int64_t)R(rs1) % (int64_t)R(rs2);
    );
    INSTPAT("0000001 ????? ????? 111 ????? 01100 11", remu   , R,
        if (unlikely(R(rs2) == 0))
            R(rd) = R(rs1);
        else
            R(rd) = R(rs1) % R(rs2);
    );
    INSTPAT("0000001 ????? ????? 111 ????? 01110 11", remuw  , R, 
        uint32_t v1 = BITS(R(rs1), 31, 0);
        uint32_t v2 = BITS(R(rs2), 31, 0);
        if (unlikely(v2 == 0))
            R(rd) = SEXT(v1, 32);
        else
            R(rd) = SEXT(v1 % v2, 32);
    );
    INSTPAT("0000001 ????? ????? 110 ????? 01110 11", remw   , R, 
        int32_t v1 = (int32_t)BITS(R(rs1), 31, 0);
        int32_t v2 = (int32_t)BITS(R(rs2), 31, 0);
        if (unlikely(v2 == 0))
            R(rd) = SEXT(v1, 32);
        else if (unlikely(v1 == INT32_MIN && v2 == -1))
            R(rd) = 0;
        else
            R(rd) = SEXT(v1 % v2, 32);
    );

    // RV64A instructions
    INSTPAT("00010?? 00000 ????? 011 ????? 01011 11", lr.d   , R,
        uint64_t v = vaddr_read_d(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            R(rd) = v;
            rv.reservation_address = R(rs1);
            rv.reservation_valid = true;
        }
    );
    INSTPAT("00010?? 00000 ????? 010 ????? 01011 11", lr.w   , R,
        uint32_t v = vaddr_read_w(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            R(rd) = SEXT(v, 32);
            rv.reservation_address = R(rs1);
            rv.reservation_valid = true;
        }
    );
    INSTPAT("00011?? ????? ????? 011 ????? 01011 11", sc.d   , R,
        if (rv.reservation_valid && rv.reservation_address == R(rs1)) {
            vaddr_write_d(R(rs1), R(rs2));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = 0;
            else
                R(rd) = 1;
        } else {
            R(rd) = 1;
        }
        rv.reservation_valid = false;
    );
    INSTPAT("00011?? ????? ????? 010 ????? 01011 11", sc.w   , R,
        if (rv.reservation_valid && rv.reservation_address == R(rs1)) {
            vaddr_write_w(R(rs1), R(rs2));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = 0;
            else
                R(rd) = 1;
        } else {
            R(rd) = 1;
        }
        rv.reservation_valid = false;
    );
    INSTPAT("00000?? ????? ????? 011 ????? 01011 11", amoadd.d , R,
        uint64_t t = vaddr_read_d(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_d(R(rs1), (int64_t)t + (int64_t)R(rs2));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = t;
        }
    );
    INSTPAT("00000?? ????? ????? 010 ????? 01011 11", amoadd.w , R,
        uint32_t t = vaddr_read_w(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_w(R(rs1), (int32_t)t + (int32_t)R(rs2));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = SEXT(t, 32);
        }
    );
    INSTPAT("01100?? ????? ????? 011 ????? 01011 11", amoand.d , R,
        uint64_t t = vaddr_read_d(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_d(R(rs1), (int64_t)t & (int64_t)R(rs2));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = t;
        }
    );
    INSTPAT("01100?? ????? ????? 010 ????? 01011 11", amoand.w , R,
        uint32_t t = vaddr_read_w(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_w(R(rs1), (int32_t)t & (int32_t)R(rs2));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = SEXT(t, 32);
        }
    );
    INSTPAT("01000?? ????? ????? 011 ????? 01011 11", amoor.d , R,
        uint64_t t = vaddr_read_d(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_d(R(rs1), (int64_t)t | (int64_t)R(rs2));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = t;
        }
    );
    INSTPAT("01000?? ????? ????? 010 ????? 01011 11", amoor.w , R,
        uint32_t t = vaddr_read_w(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_w(R(rs1), (int32_t)t | (int32_t)R(rs2));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = SEXT(t, 32);
        }
    );
    INSTPAT("00100?? ????? ????? 011 ????? 01011 11", amoxor.d , R,
        uint64_t t = vaddr_read_d(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_d(R(rs1), (int64_t)t ^ (int64_t)R(rs2));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = t;
        }
    );
    INSTPAT("00100?? ????? ????? 010 ????? 01011 11", amoxor.w , R,
        uint32_t t = vaddr_read_w(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_w(R(rs1), (int32_t)t ^ (int32_t)R(rs2));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = SEXT(t, 32);
        }
    );
    INSTPAT("10100?? ????? ????? 011 ????? 01011 11", amomax.d , R,
        uint64_t t = vaddr_read_d(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_d(R(rs1), MAX((int64_t)t, (int64_t)R(rs2)));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = t;
        }
    );
    INSTPAT("10100?? ????? ????? 010 ????? 01011 11", amomax.w , R,
        uint32_t t = vaddr_read_w(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_w(R(rs1), MAX((int32_t)t, (int32_t)R(rs2)));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = SEXT(t, 32);
        }
    );
    INSTPAT("11100?? ????? ????? 011 ????? 01011 11", amomaxu.d , R,
        uint64_t t = vaddr_read_d(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_d(R(rs1), MAX(t, R(rs2)));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = t;
        }
    );
    INSTPAT("11100?? ????? ????? 010 ????? 01011 11", amomaxu.w , R,
        uint32_t t = vaddr_read_w(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_w(R(rs1), MAX((uint32_t)t, (uint32_t)R(rs2)));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = SEXT(t, 32);
        }
    );
    INSTPAT("10000?? ????? ????? 011 ????? 01011 11", amomin.d , R,
        uint64_t t = vaddr_read_d(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_d(R(rs1), MIN((int64_t)t, (int64_t)R(rs2)));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = t;
        }
    );
    INSTPAT("10000?? ????? ????? 010 ????? 01011 11", amomin.w , R,
        uint32_t t = vaddr_read_w(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_w(R(rs1), MIN((int32_t)t, (int32_t)R(rs2)));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = SEXT(t, 32);
        }
    );
    INSTPAT("11000?? ????? ????? 011 ????? 01011 11", amominu.d , R,
        uint64_t t = vaddr_read_d(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_d(R(rs1), MIN(t, R(rs2)));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = t;
        }
    );
    INSTPAT("11000?? ????? ????? 010 ????? 01011 11", amominu.w , R,
        uint32_t t = vaddr_read_w(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_w(R(rs1), MIN((uint32_t)t, (uint32_t)R(rs2)));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = SEXT(t, 32);
        }
    );
    INSTPAT("00001?? ????? ????? 011 ????? 01011 11", amoswap.d, R,
        uint64_t t = vaddr_read_d(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_d(R(rs1), R(rs2));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = t;
        }
    );
    INSTPAT("00001?? ????? ????? 010 ????? 01011 11", amoswap.w, R,
        uint32_t t = vaddr_read_w(R(rs1));
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            vaddr_write_w(R(rs1), R(rs2));
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
                R(rd) = SEXT(t, 32);
        }
    );

    // RV64F instructions
    INSTPAT("??????? ????? ????? 010 ????? 00001 11", flw, I,
        uint32_t val = vaddr_read_w(R(rs1) + imm);
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
            fpr_write32(&F(rd), (float32_t){val});
    );
    INSTPAT("??????? ????? ????? 010 ????? 01001 11", fsw, S,
        vaddr_write_w(R(rs1) + imm, (uint32_t)F(rs2).v);
    );
    INSTPAT("0000000 ????? ????? ??? ????? 10100 11", fadd.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                fpr_write32(&F(rd), f32_add(fpr_get_f32(F(rs1)), fpr_get_f32(F(rs2))));
                FP_INST_END();
            }
        }
    );
    INSTPAT("0000100 ????? ????? ??? ????? 10100 11", fsub.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                fpr_write32(&F(rd), f32_sub(fpr_get_f32(F(rs1)), fpr_get_f32(F(rs2))));
                FP_INST_END();
            }
        }
    );
    INSTPAT("0001000 ????? ????? ??? ????? 10100 11", fmul.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                fpr_write32(&F(rd), f32_mul(fpr_get_f32(F(rs1)), fpr_get_f32(F(rs2))));
                FP_INST_END();
            }
        }
    );
    INSTPAT("0001100 ????? ????? ??? ????? 10100 11", fdiv.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                fpr_write32(&F(rd), f32_div(fpr_get_f32(F(rs1)), fpr_get_f32(F(rs2))));
                FP_INST_END();
            }
        }
    );
    INSTPAT("0101100 00000 ????? ??? ????? 10100 11", fsqrt.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                fpr_write32(&F(rd), f32_sqrt(fpr_get_f32(F(rs1))));
                FP_INST_END();
            }
        }
    );
    INSTPAT("0010000 ????? ????? 000 ????? 10100 11", fsgnj.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            float32_t f1 = fpr_get_f32(F(rs1));
            float32_t f2 = fpr_get_f32(F(rs2));
            fpr_write32(&F(rd), (float32_t){(f1.v & ~F32_SIGN) | (f2.v & F32_SIGN)});
            FP_SET_DIRTY();
        }
    );
    INSTPAT("0010000 ????? ????? 001 ????? 10100 11", fsgnjn.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            float32_t f1 = fpr_get_f32(F(rs1));
            float32_t f2 = fpr_get_f32(F(rs2));
            fpr_write32(&F(rd), (float32_t){(f1.v & ~F32_SIGN) | (~f2.v & F32_SIGN)});
            FP_SET_DIRTY();
        }
    );
    INSTPAT("0010000 ????? ????? 010 ????? 10100 11", fsgnjx.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            float32_t f1 = fpr_get_f32(F(rs1));
            float32_t f2 = fpr_get_f32(F(rs2));
            fpr_write32(&F(rd), (float32_t){f1.v ^ (f2.v & F32_SIGN)});
            FP_SET_DIRTY();
        }
    );
    INSTPAT("0010100 ????? ????? 000 ????? 10100 11", fmin.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            if (f32_isSignalingNaN(fpr_get_f32(F(rs1))) || f32_isSignalingNaN(fpr_get_f32(F(rs2))))
                rv.FCSR.fflags.NV = 1;
            bool smaller = f32_lt_quiet(fpr_get_f32(F(rs1)), fpr_get_f32(F(rs2))) ||
                        (f32_eq(fpr_get_f32(F(rs1)), fpr_get_f32(F(rs2))) &&
                            f32_isNegative(fpr_get_f32(F(rs1))));
            if (f32_isNaN(fpr_get_f32(F(rs1))) && f32_isNaN(fpr_get_f32(F(rs2)))) {
                fpr_write32(&F(rd), (float32_t){F32_DEFAULT_NAN});
            } else {
                if (smaller || f32_isNaN(fpr_get_f32(F(rs2))))
                    fpr_write32(&F(rd), fpr_get_f32(F(rs1)));
                else
                    fpr_write32(&F(rd), fpr_get_f32(F(rs2)));
            }
            FP_INST_END();
        }
    );
    INSTPAT("0010100 ????? ????? 001 ????? 10100 11", fmax.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            if (f32_isSignalingNaN(fpr_get_f32(F(rs1))) || f32_isSignalingNaN(fpr_get_f32(F(rs2))))
                rv.FCSR.fflags.NV = 1;
            bool greater = f32_lt_quiet(fpr_get_f32(F(rs2)), fpr_get_f32(F(rs1))) ||
                                (f32_eq(fpr_get_f32(F(rs2)), fpr_get_f32(F(rs1))) &&
                                    f32_isNegative(fpr_get_f32(F(rs2))));
            if (f32_isNaN(fpr_get_f32(F(rs1))) && f32_isNaN(fpr_get_f32(F(rs2)))) {
                fpr_write32(&F(rd), (float32_t){F32_DEFAULT_NAN});
            } else {
                if (greater || f32_isNaN(fpr_get_f32(F(rs2))))
                    fpr_write32(&F(rd), fpr_get_f32(F(rs1)));
                else
                    fpr_write32(&F(rd), fpr_get_f32(F(rs2)));
            }
            FP_INST_END();
        }
    );
    INSTPAT("1100000 00000 ????? ??? ????? 10100 11", fcvt.w.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                R(rd) = (int64_t)f32_to_i32(fpr_get_f32(F(rs1)), softfloat_roundingMode, true);
                FP_INST_END();
            }
        }
    );
    INSTPAT("1100000 00001 ????? ??? ????? 10100 11", fcvt.wu.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                R(rd) = (int64_t)(int32_t)f32_to_ui32(fpr_get_f32(F(rs1)), softfloat_roundingMode, true);
                FP_INST_END();
            }
        }
    );
    INSTPAT("1100000 00010 ????? ??? ????? 10100 11", fcvt.l.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                R(rd) = f32_to_i64(fpr_get_f32(F(rs1)), softfloat_roundingMode, true);
                FP_INST_END();
            }
        }
    );
    INSTPAT("1100000 00011 ????? ??? ????? 10100 11", fcvt.lu.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                R(rd) = f32_to_ui64(fpr_get_f32(F(rs1)), softfloat_roundingMode, true);
                FP_INST_END();
            }
        }
    );
    INSTPAT("1101000 00000 ????? ??? ????? 10100 11", fcvt.s.w, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                fpr_write32(&F(rd), i32_to_f32((int32_t)R(rs1)));
                FP_INST_END();
            }
        }
    );
    INSTPAT("1101000 00001 ????? ??? ????? 10100 11", fcvt.s.wu, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                fpr_write32(&F(rd), ui32_to_f32((int32_t)R(rs1)));
                FP_INST_END();
            }
        }
    );
    INSTPAT("1101000 00010 ????? ??? ????? 10100 11", fcvt.s.l, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                fpr_write32(&F(rd), i64_to_f32(R(rs1)));
                FP_INST_END();
            }
        }
    );
    INSTPAT("1101000 00011 ????? ??? ????? 10100 11", fcvt.s.lu, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                fpr_write32(&F(rd), ui64_to_f32(R(rs1)));
                FP_INST_END();
            }
        }
    );
    INSTPAT("1110000 00000 ????? 000 ????? 10100 11", fmv.x.w, R,
        FP_INST_PREP();
        if (likely(rv.last_exception== CAUSE_EXCEPTION_NONE))
            R(rd) = (int64_t)(int32_t)(uint32_t)F(rs1).v;
    );
    INSTPAT("1111000 00000 ????? 000 ????? 10100 11", fmv.w.x, R,
        FP_INST_PREP();
        if (likely(rv.last_exception== CAUSE_EXCEPTION_NONE)) {
            fpr_write32(&F(rd), (float32_t){(uint32_t)((int32_t)R(rs1))});
            FP_SET_DIRTY();
        }
    );
    INSTPAT("1110000 00000 ????? 001 ????? 10100 11", fclass.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception== CAUSE_EXCEPTION_NONE))
            R(rd) = (int64_t)(int32_t)f32_classify(fpr_get_f32(F(rs1)));
    );
    INSTPAT("1010000 ????? ????? 010 ????? 10100 11", feq.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception== CAUSE_EXCEPTION_NONE)) {
            R(rd) = f32_eq(fpr_get_f32(F(rs1)), fpr_get_f32(F(rs2)));
            FP_UPDATE_EXCEPTION_FLAGS();
        }
    );
    INSTPAT("1010000 ????? ????? 001 ????? 10100 11", flt.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception== CAUSE_EXCEPTION_NONE)) {
            R(rd) = f32_lt(fpr_get_f32(F(rs1)), fpr_get_f32(F(rs2)));
            FP_UPDATE_EXCEPTION_FLAGS();
        }
    );
    INSTPAT("1010000 ????? ????? 000 ????? 10100 11", fle.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception== CAUSE_EXCEPTION_NONE)) {
            R(rd) = f32_le(fpr_get_f32(F(rs1)), fpr_get_f32(F(rs2)));
            FP_UPDATE_EXCEPTION_FLAGS();
        }
    );
    INSTPAT("?????00 ????? ????? ??? ????? 10000 11", fmadd.s, R4,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                fpr_write32(&F(rd), f32_mulAdd(fpr_get_f32(F(rs1)), fpr_get_f32(F(rs2)), fpr_get_f32(F(rs3))));
                FP_INST_END();
            }
        }
    );
    INSTPAT("?????00 ????? ????? ??? ????? 10001 11", fmsub.s, R4,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                fpr_write32(&F(rd), f32_mulAdd(fpr_get_f32(F(rs1)), fpr_get_f32(F(rs2)), f32_neg(fpr_get_f32(F(rs3)))));
                FP_INST_END();
            }
        }
    );
    INSTPAT("?????00 ????? ????? ??? ????? 10010 11", fnmsub.s, R4,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                fpr_write32(&F(rd), f32_mulAdd(f32_neg(fpr_get_f32(F(rs1))), fpr_get_f32(F(rs2)), fpr_get_f32(F(rs3))));
                FP_INST_END();
            }
        }
    );
    INSTPAT("?????00 ????? ????? ??? ????? 10011 11", fnmadd.s, R4,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                fpr_write32(&F(rd), f32_mulAdd(f32_neg(fpr_get_f32(F(rs1))), fpr_get_f32(F(rs2)), f32_neg(fpr_get_f32(F(rs3)))));
                FP_INST_END();
            }
        }
    );

    // RV64D instructions
    INSTPAT("??????? ????? ????? 011 ????? 00001 11", fld, I,
        uint64_t val = vaddr_read_d(R(rs1) + imm);
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
            F(rd) = (float64_t){val};
    );
    INSTPAT("??????? ????? ????? 011 ????? 01001 11", fsd, S, vaddr_write_d(R(rs1) + imm, F(rs2).v));
    INSTPAT("0000001 ????? ????? ??? ????? 10100 11", fadd.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                F(rd) = f64_add(F(rs1), F(rs2));
                FP_INST_END();
            }
        }
    );
    INSTPAT("0000101 ????? ????? ??? ????? 10100 11", fsub.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                F(rd) = f64_sub(F(rs1), F(rs2));
                FP_INST_END();
            }
        }
    );
    INSTPAT("0001001 ????? ????? ??? ????? 10100 11", fmul.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                F(rd) = f64_mul(F(rs1), F(rs2));
                FP_INST_END();
            }
        }
    );
    INSTPAT("0001101 ????? ????? ??? ????? 10100 11", fdiv.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                F(rd) = f64_div(F(rs1), F(rs2));
                FP_INST_END();
            }
        }
    );
    INSTPAT("0101101 00000 ????? ??? ????? 10100 11", fsqrt.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                F(rd) = f64_sqrt(F(rs1));
                FP_INST_END();
            }
        }
    );
    INSTPAT("0010001 ????? ????? 000 ????? 10100 11", fsgnj.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            float64_t f1 = F(rs1), f2 = F(rs2);
            F(rd) = (float64_t){(f1.v & ~F64_SIGN) | (f2.v & F64_SIGN)};
            FP_SET_DIRTY();
        }
    );
    INSTPAT("0010001 ????? ????? 001 ????? 10100 11", fsgnjn.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            float64_t f1 = F(rs1), f2 = F(rs2);
            F(rd) = (float64_t){(f1.v & ~F64_SIGN) | (~f2.v & F64_SIGN)};
            FP_SET_DIRTY();
        }
    );
    INSTPAT("0010001 ????? ????? 010 ????? 10100 11", fsgnjx.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            float64_t f1 = F(rs1), f2 = F(rs2);
            F(rd) = (float64_t){f1.v ^ (f2.v & F64_SIGN)};
            FP_SET_DIRTY();
        }
    );
    INSTPAT("0010101 ????? ????? 000 ????? 10100 11", fmin.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            if (f64_isSignalingNaN(F(rs1)) || f64_isSignalingNaN(F(rs2)))
                rv.FCSR.fflags.NV = 1;
            bool smaller = f64_lt_quiet(F(rs1), F(rs2)) ||
			                   (f64_eq(F(rs1), F(rs2)) &&
                               f64_isNegative(F(rs1)));
            if (f64_isNaN(F(rs1)) && f64_isNaN(F(rs2))) {
                F(rd) = (float64_t){F64_DEFAULT_NAN};
            } else {
                if (smaller || f64_isNaN(F(rs2)))
                    F(rd) = F(rs1);
                else
                    F(rd) = F(rs2);
            }
            FP_INST_END();
        }
    );
    INSTPAT("0010101 ????? ????? 001 ????? 10100 11", fmax.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            if (f64_isSignalingNaN(F(rs1)) || f64_isSignalingNaN(F(rs2)))
                rv.FCSR.fflags.NV = 1;
            bool greater = f64_lt_quiet(F(rs2), F(rs1)) ||
			                   (f64_eq(F(rs2), F(rs1)) && f64_isNegative(F(rs2)));
            if (f64_isNaN(F(rs1)) && f64_isNaN(F(rs2))) {
                F(rd) = (float64_t){F64_DEFAULT_NAN};
            } else {
                if (greater || f64_isNaN(F(rs2)))
                    F(rd) = F(rs1);
                else
                    F(rd) = F(rs2);
            }
            FP_INST_END();
        }
    );
    INSTPAT("1100001 00000 ????? ??? ????? 10100 11", fcvt.w.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                R(rd) = (int64_t)f64_to_i32(F(rs1), softfloat_roundingMode, true);
                FP_INST_END();
            }
        }
    );
    INSTPAT("1100001 00001 ????? ??? ????? 10100 11", fcvt.wu.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                R(rd) = (int64_t)(int32_t)f64_to_ui32(F(rs1), softfloat_roundingMode, true);
                FP_INST_END();
            }
        }
    );
    INSTPAT("1100001 00010 ????? ??? ????? 10100 11", fcvt.l.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                R(rd) = f64_to_i64(F(rs1), softfloat_roundingMode, true);
                FP_INST_END();
            }
        }
    );
    INSTPAT("1100001 00011 ????? ??? ????? 10100 11", fcvt.lu.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                R(rd) = f64_to_ui64(F(rs1), softfloat_roundingMode, true);
                FP_INST_END();
            }
        }
    );
    INSTPAT("1101001 00000 ????? ??? ????? 10100 11", fcvt.d.w, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                F(rd) = i32_to_f64((int32_t)R(rs1));
                FP_INST_END();
            }
        }
    );
    INSTPAT("1101001 00001 ????? ??? ????? 10100 11", fcvt.d.wu, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                F(rd) = ui32_to_f64((int32_t)R(rs1));
                FP_INST_END();
            }
        }
    );
    INSTPAT("1101001 00010 ????? ??? ????? 10100 11", fcvt.d.l, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                F(rd) = i64_to_f64(R(rs1));
                FP_INST_END();
            }
        }
    );
    INSTPAT("1101001 00011 ????? ??? ????? 10100 11", fcvt.d.lu, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                F(rd) = ui64_to_f64(R(rs1));
                FP_INST_END();
            }
        }
    );
    INSTPAT("0100000 00001 ????? ??? ????? 10100 11", fcvt.s.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                fpr_write32(&F(rd), f64_to_f32(F(rs1)));
                FP_INST_END();
            }
        }
    );
    INSTPAT("0100001 00000 ????? ??? ????? 10100 11", fcvt.d.s, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                F(rd) = f32_to_f64(fpr_get_f32(F(rs1)));
                FP_INST_END();
            }
        }
    );
    INSTPAT("1110001 00000 ????? 000 ????? 10100 11", fmv.x.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
            R(rd) = F(rs1).v;
    );
    INSTPAT("1111001 00000 ????? 000 ????? 10100 11", fmv.d.x, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            F(rd) = (float64_t){R(rs1)};
            FP_SET_DIRTY();
        }
    );
    INSTPAT("1110001 00000 ????? 001 ????? 10100 11", fclass.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
            R(rd) = (int64_t)f64_classify(F(rs1));
    );
    INSTPAT("1010001 ????? ????? 010 ????? 10100 11", feq.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            R(rd) = f64_eq(F(rs1), F(rs2));
            FP_UPDATE_EXCEPTION_FLAGS();
        }
    );
    INSTPAT("1010001 ????? ????? 001 ????? 10100 11", flt.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            R(rd) = f64_lt(F(rs1), F(rs2));
            FP_UPDATE_EXCEPTION_FLAGS();
        }
    );
    INSTPAT("1010001 ????? ????? 000 ????? 10100 11", fle.d, R,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            R(rd) = f64_le(F(rs1), F(rs2));
            FP_UPDATE_EXCEPTION_FLAGS();
        }
    );
    INSTPAT("?????01 ????? ????? ??? ????? 10000 11", fmadd.d, R4,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                F(rd) = f64_mulAdd(F(rs1), F(rs2), F(rs3));
                FP_INST_END();
            }
        }
    );
    INSTPAT("?????01 ????? ????? ??? ????? 10001 11", fmsub.d, R4,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
                F(rd) = f64_mulAdd(F(rs1), F(rs2), f64_neg(F(rs3)));
                FP_INST_END();
            }
        }
    );
    INSTPAT("?????01 ????? ????? ??? ????? 10010 11", fnmsub.d, R4,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (rv.last_exception == CAUSE_EXCEPTION_NONE) {
                F(rd) = f64_mulAdd((f64_neg(F(rs1))), F(rs2), F(rs3));
                FP_INST_END();
            }
        }
    );
    INSTPAT("?????01 ????? ????? ??? ????? 10011 11", fnmadd.d, R4,
        FP_INST_PREP();
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
            FP_SETUP_RM();
            if (rv.last_exception == CAUSE_EXCEPTION_NONE) {
                F(rd) = f64_mulAdd((f64_neg(F(rs1))), F(rs2), f64_neg(F(rs3)));
                FP_INST_END();
            }
        }
    );

    // Invalid instructions
    INSTPAT("??????? ????? ????? ??? ????? ????? ??", inv, N, cpu_raise_exception(CAUSE_ILLEGAL_INSTRUCTION, s->inst));

    INSTPAT_END();
    // clang-format on

    R(0) = 0; // reset $zero to 0
}

#undef INSTPAT_MATCH

#define INSTPAT_MATCH(s, name, type, ... /* execute body */)                   \
    {                                                                          \
        int rd = 0, rs1 = 0, rs2 = 0;                                          \
        uint64_t imm = 0;                                                      \
        decode_operand_16(s, &rd, &rs1, &rs2, &imm, concat(TYPE_, type));      \
        __VA_ARGS__;                                                           \
    }

static inline void decode_exec_16(Decode *s) {
    // clang-format off
    INSTPAT_START();

    // RV64C instructions
    INSTPAT("000 ?00000????? 01", c.nop, CI, /* nop */);
    INSTPAT("000 ??????????? 01", c.addi, CI, R(rd) += imm);
    INSTPAT("001 ??????????? 01", c.addiw, CI, R(rd) = SEXT(BITS(R(rd) + imm, 31, 0), 32));
    INSTPAT("010 ??????????? 01", c.li, CI, R(rd) = imm);
    INSTPAT("011 ?00010????? 01", c.addi16sp, CI,
        if (unlikely(imm == 0))
            cpu_raise_exception(CAUSE_ILLEGAL_INSTRUCTION, s->inst);
        else
            R(2) += imm;
    );
    INSTPAT("011 ??????????? 01", c.lui, CI,
        if (unlikely(rd == 2 || imm == 0))
            cpu_raise_exception(CAUSE_ILLEGAL_INSTRUCTION, s->inst);
        else
            R(rd) = imm;
    );
    INSTPAT("100 ?00???????? 01", c.srli, CB, R(rd) >>= imm);
    INSTPAT("100 ?01???????? 01", c.srai, CB, R(rd) = (int64_t)R(rd) >> imm);
    INSTPAT("100 ?10???????? 01", c.andi, CB, R(rd) &= imm);
    INSTPAT("100 011???00??? 01", c.sub, CA, R(rd) -= R(rs2));
    INSTPAT("100 011???01??? 01", c.xor, CA, R(rd) ^= R(rs2));
    INSTPAT("100 011???10??? 01", c.or, CA, R(rd) |= R(rs2));
    INSTPAT("100 011???11??? 01", c.and, CA, R(rd) &= R(rs2));
    INSTPAT("100 111???00??? 01", c.subw, CA, R(rd) = SEXT(BITS(R(rd) - R(rs2), 31, 0), 32));
    INSTPAT("100 111???01??? 01", c.addw, CA, R(rd) = SEXT(BITS(R(rd) + R(rs2), 31, 0), 32));
    INSTPAT("101 ??????????? 01", c.j, CJ, s->npc = s->pc + imm);
    INSTPAT("110 ??????????? 01", c.beqz, CB, if (R(rs1) == 0) s->npc = s->pc + imm;);
    INSTPAT("111 ??????????? 01", c.bnez, CB, if (R(rs1) != 0) s->npc = s->pc + imm;);
    INSTPAT("000 00000000000 00", c.inv, N, cpu_raise_exception(CAUSE_ILLEGAL_INSTRUCTION, s->inst));
    INSTPAT("000 ??????????? 00", c.addi4spn, CIW,
        if (unlikely(imm == 0))
            cpu_raise_exception(CAUSE_ILLEGAL_INSTRUCTION, s->inst);
        else
            R(rd) = R(2) + imm;
    );
    INSTPAT("001 ??????????? 00", c.fld, CL,
        uint64_t val = vaddr_read_d(R(rs1) + imm);
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
            F(rd) = (float64_t){val};
    );
    INSTPAT("010 ??????????? 00", c.lw, CL, LOAD_SEXT(rd, R(rs1) + imm, uint32_t, w, 32));
    INSTPAT("011 ??????????? 00", c.ld, CL, LOAD(rd, R(rs1) + imm, uint64_t, d, 64));
    INSTPAT("101 ??????????? 00", c.fsd, CS, vaddr_write_d(R(rs1) + imm, F(rs2).v));
    INSTPAT("110 ??????????? 00", c.sw, CS, vaddr_write_w(R(rs1) + imm, BITS(R(rs2), 31, 0)));
    INSTPAT("111 ??????????? 00", c.sd, CS, vaddr_write_d(R(rs1) + imm, R(rs2)));
    INSTPAT("000 ??????????? 10", c.slli, CI, R(rd) <<= imm);
    INSTPAT("001 ??????????? 10", c.fldsp, CI,
        uint64_t val = vaddr_read_d(R(2) + imm);
        if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE))
            F(rd) = (float64_t){val};
    );
    INSTPAT("010 ??????????? 10", c.lwsp, CI,
        if (unlikely(rd == 0))
            cpu_raise_exception(CAUSE_ILLEGAL_INSTRUCTION, s->inst);
        else
            LOAD_SEXT(rd, R(2) + imm, uint32_t, w, 32);
    );
    INSTPAT("011 ??????????? 10", c.ldsp, CI,
        if (unlikely(rd == 0))
            cpu_raise_exception(CAUSE_ILLEGAL_INSTRUCTION, s->inst);
        else
            LOAD(rd, R(2) + imm, uint64_t, d, 64);
    );
    INSTPAT("100 0?????00000 10", c.jr, CR,
        if (unlikely(rs1 == 0))
            cpu_raise_exception(CAUSE_ILLEGAL_INSTRUCTION, s->inst);
        else
            s->npc = R(rs1) & ~1ULL;
    );
    INSTPAT("100 0?????????? 10", c.mv, CR,
        if (unlikely(rs2 == 0))
            cpu_raise_exception(CAUSE_ILLEGAL_INSTRUCTION, s->inst);
        else
            R(rd) = R(rs2);
    );
    INSTPAT("100 10000000000 10", c.ebreak, CR, cpu_raise_exception(CAUSE_BREAKPOINT, s->pc));
    INSTPAT("100 1?????00000 10", c.jalr, CR,
        if (unlikely(rs1 == 0)) {
            cpu_raise_exception(CAUSE_ILLEGAL_INSTRUCTION, s->inst);
        } else {
            uint64_t t = s->pc + 2;
            s->npc = R(rs1) & ~1ULL;
            R(1) = t;
        }
    );
    INSTPAT("100 1?????????? 10", c.add, CR, R(rd) += R(rs2));
    INSTPAT("101 ??????????? 10", c.fsdsp, CSS, vaddr_write_d(R(2) + imm, F(rs2).v));
    INSTPAT("110 ??????????? 10", c.swsp, CSS, vaddr_write_w(R(2) + imm, BITS(R(rs2), 31, 0)));
    INSTPAT("111 ??????????? 10", c.sdsp, CSS, vaddr_write_d(R(2) + imm, R(rs2)));

    // Invalid instructions
    INSTPAT("??? ??????????? ??", inv, N, cpu_raise_exception(CAUSE_ILLEGAL_INSTRUCTION, s->inst));

    INSTPAT_END();
    // clang-format on

    R(0) = 0; // reset $zero to 0
}

FORCE_INLINE void cpu_exec_once(Decode *s, uint64_t pc) {
    s->pc = pc;
    size_t len;
    s->inst = vaddr_ifetch(pc, &len);
    if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE)) {
        s->npc = pc + len;
        len == 4 ? decode_exec_32(s) : decode_exec_16(s);
    }
    rv.PC = s->npc;
    rv.MCYCLE++;
    if (likely(rv.last_exception == CAUSE_EXCEPTION_NONE &&
               !rv.suppress_minstret_increase))
        rv.MINSTRET++;
    rv.suppress_minstret_increase = false;
}

static pthread_t cpu_thread;
static pthread_mutex_t cpu_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cpu_cond = PTHREAD_COND_INITIALIZER;
static bool cpu_thread_running = false;

/* Child CPU thread */
static void *cpu_thread_func(void *arg) {
    // Notify the main thread that the child thread has started
    pthread_mutex_lock(&cpu_mutex);
    cpu_thread_running = true;
    pthread_cond_broadcast(&cpu_cond);
    pthread_mutex_unlock(&cpu_mutex);

    uint64_t start = slowtimer_get_microseconds();
    uint64_t inst_cnt = 0;

    for (uint64_t i = 0; i < UINT64_MAX; i++) {
        if (unlikely(rv.shutdown))
            break;
        rv.last_exception = CAUSE_EXCEPTION_NONE;
        if ((i & 255) == 0) {
            interrupt_t intr = rv_get_pending_interrupt();
            if (unlikely(intr != CAUSE_INTERRUPT_NONE))
                cpu_process_intr(intr);
        }
        cpu_exec_once(&rv.decode, rv.PC);
        inst_cnt++;
    }

    uint64_t end = slowtimer_get_microseconds();
    double delta = (double)(end - start) / 1000000.0;
    log_info("Simulation time: %f seconds (%" PRIu64 " microseconds)", delta,
             end - start);
    log_info("Simulation speed: %f insts per second", inst_cnt / delta);

    sleep(1);

    // Notify the main thread again
    pthread_mutex_lock(&cpu_mutex);
    cpu_thread_running = false;
    pthread_cond_broadcast(&cpu_cond);
    pthread_mutex_unlock(&cpu_mutex);

    return NULL;
}

static inline void cpu_thread_start() {
    pthread_mutex_lock(&cpu_mutex);
    if (cpu_thread_running) {
        log_warn("CPU thread already running");
        pthread_mutex_unlock(&cpu_mutex);
        return;
    }

    if (pthread_create(&cpu_thread, NULL, cpu_thread_func, NULL) != 0) {
        pthread_mutex_unlock(&cpu_mutex);
        log_error("pthread_create failed");
        exit(EXIT_FAILURE);
    }

    while (!cpu_thread_running)
        pthread_cond_wait(&cpu_cond, &cpu_mutex);
    pthread_mutex_unlock(&cpu_mutex);
}

void cpu_start() {
    alarm_turn(true);

    pthread_cond_init(&cpu_cond, NULL);
    pthread_mutex_init(&cpu_mutex, NULL);

    cpu_thread_start();

    while (true) {
        pthread_mutex_lock(&cpu_mutex);
        bool running = cpu_thread_running;
        pthread_mutex_unlock(&cpu_mutex);

        if (!running)
            break;

        // Update UI and framebuffer
        ui_update();

        // Update clint
        clint_tick();
        // Update UART
        uart_tick();
        // Update RTC
        rtc_tick();
        // Update battery
        battery_update();
    }

    alarm_turn(false);
    pthread_join(cpu_thread, NULL);

    pthread_mutex_destroy(&cpu_mutex);
    pthread_cond_destroy(&cpu_cond);
}

#define CPU_EXEC_COMMON()                                                      \
    do {                                                                       \
        rv.last_exception = CAUSE_EXCEPTION_NONE;                              \
        interrupt_t intr = rv_get_pending_interrupt();                         \
        if (unlikely(intr != CAUSE_INTERRUPT_NONE))                            \
            cpu_process_intr(intr);                                            \
        cpu_exec_once(&rv.decode, rv.PC);                                      \
    } while (0)

void cpu_step() {
    alarm_turn(true);
    if (!unlikely(rv.shutdown)) {
        clint_tick();
        uart_tick();
        CPU_EXEC_COMMON();
    }
    alarm_turn(false);
}

void cpu_start_archtest() {
    // FIXME: Use a better way to end the test
    uint64_t start = slowtimer_get_microseconds();
    alarm_turn(true);
    for (size_t i = 0; i < SIZE_MAX; i++) {
        if (i % 1000 == 1 && slowtimer_get_microseconds() - start > 4000000)
            break;
        if (rv.shutdown)
            continue;
        clint_tick();
        uart_tick();
        CPU_EXEC_COMMON();
    }
    alarm_turn(false);
}

// clang-format off
static const char *regs[] = {
    "$0", "ra", "sp",  "gp",  "tp", "t0", "t1", "t2",
    "s0", "s1", "a0",  "a1",  "a2", "a3", "a4", "a5",
    "a6", "a7", "s2",  "s3",  "s4", "s5", "s6", "s7",
    "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6"
};
// clang-format on

static const char *csrs[] = {
    [CSR_MVENDORID] = "mvendorid",
    [CSR_MARCHID] = "marchid",
    [CSR_MIMPID] = "mimpid",
    [CSR_MHARTID] = "mhartid",
    [CSR_MSTATUS] = "mstatus",
    [CSR_MISA] = "misa",
    [CSR_MEDELEG] = "medeleg",
    [CSR_MIDELEG] = "mideleg",
    [CSR_MIE] = "mie",
    [CSR_MTVEC] = "mtvec",
    [CSR_MSCRATCH] = "mscratch",
    [CSR_MEPC] = "mepc",
    [CSR_MCAUSE] = "mcause",
    [CSR_MTVAL] = "mtval",
    [CSR_MIP] = "mip",
    [CSR_SSTATUS] = "sstatus",
    [CSR_SIE] = "sie",
    [CSR_STVEC] = "stvec",
    [CSR_SSCRATCH] = "sscratch",
    [CSR_SEPC] = "sepc",
    [CSR_SCAUSE] = "scause",
    [CSR_STVAL] = "stval",
    [CSR_SIP] = "sip",
    [CSR_SATP] = "satp",
};

void cpu_print_registers() {
    for (size_t i = 0; i < NR_GPR; i++)
        printf("%s\t0x%08" PRIx64 "\n", regs[i], R(i));
    printf("%s\t0x%08" PRIx64 "\n", "pc", rv.PC);

    const int implemented_csrs[] = {
        CSR_MVENDORID, CSR_MARCHID, CSR_MIMPID,  CSR_MHARTID,  CSR_MSTATUS,
        CSR_MISA,      CSR_MEDELEG, CSR_MIDELEG, CSR_MIE,      CSR_MTVEC,
        CSR_MSCRATCH,  CSR_MEPC,    CSR_MCAUSE,  CSR_MTVAL,    CSR_MIP,
        CSR_SSTATUS,   CSR_SIE,     CSR_STVEC,   CSR_SSCRATCH, CSR_SEPC,
        CSR_SCAUSE,    CSR_STVAL,   CSR_SIP,     CSR_SATP};
    for (size_t i = 0; i < ARRAY_SIZE(implemented_csrs); i++)
        printf("%s\t0x%08" PRIx64 "\n", csrs[implemented_csrs[i]],
               cpu_read_csr(implemented_csrs[i]));
}
