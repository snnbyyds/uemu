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

#include <assert.h>
#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "core/riscv.h"
#include "device/plic.h"
#include "device/uart8250.h"
#include "utils/logger.h"
#include "utils/misc.h"

/* Emulate 8250 (plain, without loopback mode support) */

#define U8250_INTR_THRE 1

static inline int ilog2(int x) {
    int r = 0;
    while (x >>= 1)
        r++;
    return r;
}

typedef struct {
    uint8_t dll, dlh;                    /* divisor (ignored) */
    uint8_t lcr;                         /* UART config */
    uint8_t ier;                         /* interrupt config */
    uint8_t current_intr, pending_intrs; /* interrupt status */
    uint8_t mcr;       /* other output signals, loopback mode (ignored) */
    int in_fd, out_fd; /* I/O handling */
    bool in_ready;
    pthread_mutex_t m;
} u8250_state_t;

static u8250_state_t uart;

static void u8250_update_interrupts(u8250_state_t *uart_state) {
    /* Some interrupts are level-generated. */
    /* TODO: does it also generate an LSR change interrupt? */
    if (uart_state->in_ready)
        uart_state->pending_intrs |= 1;
    else
        uart_state->pending_intrs &= ~1;

    /* Prevent generating any disabled interrupts in the first place */
    uart_state->pending_intrs &= uart_state->ier;

    /* Update current interrupt (higher bits -> more priority) */
    if (uart_state->pending_intrs)
        uart_state->current_intr = ilog2(uart_state->pending_intrs);

    /* Update PLIC IRQ */
    if (uart_state->pending_intrs)
        plic_set_irq(UART8250_IRQ, 1);
    else
        plic_set_irq(UART8250_IRQ, 0);
}

static void u8250_check_ready(u8250_state_t *uart_state) {
    if (uart_state->in_ready)
        return;

    struct pollfd pfd = {uart_state->in_fd, POLLIN, 0};
    poll(&pfd, 1, 0);
    if (pfd.revents & POLLIN)
        uart_state->in_ready = true;
}

static void u8250_handle_out(u8250_state_t *uart_state, uint8_t value) {
    if (write(uart_state->out_fd, &value, 1) < 1)
        log_error("Failed to write UART output: %s", strerror(errno));
}

static uint8_t u8250_handle_in(u8250_state_t *uart_state) {
    uint8_t value = 0;
    u8250_check_ready(uart_state);
    if (!uart_state->in_ready)
        return value;

    if (read(uart_state->in_fd, &value, 1) < 0)
        log_error("Failed to read UART input: %s", strerror(errno));

    uart_state->in_ready = false;

    if (value == 1) { /* start of heading (Ctrl-a) */
        u8250_check_ready(uart_state);
        uint8_t next_char = 0;
        if (read(uart_state->in_fd, &next_char, 1) > 0 &&
            next_char == 120) { /* keyboard x */
            rv_shutdown(0, SHUTDOWN_CAUSE_GUEST_SHUTDOWN);
            exit(EXIT_SUCCESS);
        }
    }

    return value;
}

static uint64_t uart_read(uint64_t addr, size_t n) {
    uint32_t offset = addr & 0x7;
    uint8_t ret = 0;

    pthread_mutex_lock(&uart.m);

    switch (offset) {
        case U8250_THR_RBR_DLL:
            if (uart.lcr & (1 << 7)) /* DLAB */
                ret = uart.dll;
            else
                ret = u8250_handle_in(&uart);
            break;
        case U8250_IER_DLH:
            if (uart.lcr & (1 << 7)) /* DLAB */
                ret = uart.dlh;
            else
                ret = uart.ier;
            break;
        case U8250_IIR_FCR:
            ret = (uart.current_intr << 1) | (uart.pending_intrs ? 0 : 1);
            if (uart.current_intr == U8250_INTR_THRE)
                uart.pending_intrs &= ~(1 << uart.current_intr);
            break;
        case U8250_LCR: ret = uart.lcr; break;
        case U8250_MCR: ret = uart.mcr; break;
        case U8250_LSR:
            /* LSR = no error, TX done & ready */
            ret = 0x60 | (uint8_t)uart.in_ready;
            break;
        case U8250_MSR:
            /* MSR = carrier detect, no ring, data ready, clear to send. */
            ret = 0xb0;
            break;
        default: break;
    }

    u8250_update_interrupts(&uart);
    pthread_mutex_unlock(&uart.m);

    return (uint32_t)(int8_t)ret;
}

static void uart_write(uint64_t addr, uint64_t value, size_t n) {
    uint32_t offset = addr & 0x7;

    pthread_mutex_lock(&uart.m);

    switch (offset) {
        case U8250_THR_RBR_DLL:
            if (uart.lcr & (1 << 7)) { /* DLAB */
                uart.dll = value;
                break;
            }
            u8250_handle_out(&uart, value);
            uart.pending_intrs |= 1 << U8250_INTR_THRE;
            break;
        case U8250_IER_DLH:
            if (uart.lcr & (1 << 7)) { /* DLAB */
                uart.dlh = value;
                break;
            }
            uart.ier = value;
            break;
        case U8250_LCR: uart.lcr = value; break;
        case U8250_MCR: uart.mcr = value; break;
        default: break;
    }

    u8250_update_interrupts(&uart);
    pthread_mutex_unlock(&uart.m);
}

void uart_tick() {
    pthread_mutex_lock(&uart.m);
    u8250_check_ready(&uart);
    u8250_update_interrupts(&uart);
    pthread_mutex_unlock(&uart.m);
}

void uart_init() {
    memset(&uart, 0, sizeof(u8250_state_t));
    pthread_mutex_init(&uart.m, NULL);

    enable_stdin_raw_mode();
    atexit(disable_stdin_raw_mode);

    uart.in_fd = STDIN_FILENO;
    uart.out_fd = STDOUT_FILENO;
    uart.in_ready = false;

    rv_add_device((device_t){
        .name = "UART8250",
        .start = UART8250_BASE,
        .end = UART8250_BASE + UART8250_SIZE - 1ULL,
        .read = uart_read,
        .write = uart_write,
    });
}

void uart_destroy() {
    int rc = pthread_mutex_destroy(&uart.m);
    if (rc)
        log_warn("destroy uart lock failed");
    disable_stdin_raw_mode();
}
