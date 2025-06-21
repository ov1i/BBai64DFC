#include "comms/logger.h"
#include <utils.h>
extern "C" {
#include "ti/drv/uart/UART_stdio.h"
#include <stdarg.h>
#include <string.h>
#include <cstdarg>
#include <cstdio>
}

static char log_buffer[LOGGER_BUFFER_SIZE];
static size_t log_head = 0, log_tail = 0;

void logger_init(void) {
    log_head = log_tail = 0;
}

void logger_log(const char *fmt, ...) {
    char temp[128];
    va_list args;
    va_start(args, fmt);
    uint32 len = vsnprintf(temp, sizeof(temp), fmt, args);
    va_end(args);
    if (len <= 0) return;
    // Ring buffer write
    for (uint32 i = 0; i < len; ++i) {
        log_buffer[log_head] = temp[i];
        log_head = (log_head + 1) % LOGGER_BUFFER_SIZE;
        // Overwrite old data if buffer full
        if (log_head == log_tail) log_tail = (log_tail + 1) % LOGGER_BUFFER_SIZE;
    }
}

uint32 logger_read(char *out, size_t maxlen) {
    size_t count = 0;
    while (log_tail != log_head && count < maxlen - 1) {
        out[count++] = log_buffer[log_tail];
        log_tail = (log_tail + 1) % LOGGER_BUFFER_SIZE;
    }
    out[count] = '\0';
    return count;
}
