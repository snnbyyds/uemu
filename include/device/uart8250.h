/* SPDX-License-Identifier: MIT
 *
 * This file is derived from the rv32emu project (MIT licensed).
 * Ported/adapted for uemu — A RISC-V Virtual Platform Emulator.
 *
 * Copyright (c) 2020-2025 National Cheng Kung University, Taiwan
 * Copyright (c) 2025 Nuo Shen, Nanjing University
 *
 * See LICENSE-MIT for the full text of the MIT license.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define UART8250_BASE 0x10000000
#define UART8250_SIZE 0x100
#define UART8250_IRQ 10

enum UART_REG {
    U8250_THR_RBR_DLL = 0,
    U8250_IER_DLH,
    U8250_IIR_FCR,
    U8250_LCR,
    U8250_MCR,
    U8250_LSR,
    U8250_MSR,
    U8250_SR,
};

/* Initialize UART8250 device */
void uart_init();

/* Cleanup UART8250 device */
void uart_destroy();

/* Check for input and update interrupts (called periodically) */
void uart_tick();
