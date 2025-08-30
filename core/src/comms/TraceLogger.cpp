extern "C" {
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <FreeRTOS.h>
#include <ti/osal/HwiP.h>
#include <ti/osal/CacheP.h>
#include <ti/osal/DebugP.h>
#include <ti/osal/CacheP.h>
}

#include <data_types.h>

extern "C" uint8_t __TRACEBUF_START;
extern "C" uint8_t __TRACEBUF_END;

static volatile uint8_t *g_base = nullptr;
static size_t            g_cap  = 0;
static volatile size_t   g_head = 0;

static inline void trace_write_raw(const char *data, size_t n)
{
    if (!g_base || g_cap == 0 || n == 0) return;

    uintptr_t key = HwiP_disable();

    size_t head  = g_head;
    size_t first = g_cap - head; if (first > n) first = n;
    memcpy((void*)(g_base + head), data, first);
    size_t rem = n - first;
    if (rem) { memcpy((void*)g_base, data + first, rem); head = rem; }
    else { head += first; if (head == g_cap) head = 0; }
    g_head = head;

    HwiP_restore(key);
    CacheP_wb((void*)g_base, g_cap);   // make visible to A72
}

extern "C" void routeDebugP(void)
{
    // init from linker symbols
    g_base = &__TRACEBUF_START;
    g_cap  = (size_t)(&__TRACEBUF_END - &__TRACEBUF_START);
    g_head = 0;

    memset((void*)g_base, 0, g_cap);
    CacheP_wb((void*)g_base, g_cap);

    const char *hello = "[trace] routeDebugP ready\r\n";
    trace_write_raw(hello, strlen(hello));
}

extern "C" void tracePrintf(const char *fmt, ...)
{
    char line[384];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if ((size_t)n >= sizeof(line)) n = (int)sizeof(line) - 1;
    trace_write_raw(line, (size_t)n);
}