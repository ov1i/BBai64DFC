#ifndef DFC_TRACE_LOGGER_H
#define DFC_TRACE_LOGGER_H

#include <ti/osal/DebugP.h>

#ifdef __cplusplus
extern "C" {
void tracePrintf(const char *fmt, ...);
}
#endif

/* Remap DebugP logging family to the trace buffer */
#ifndef DFC_DEBUGP_ROUTED
#define DFC_DEBUGP_ROUTED 1

#undef DebugP_log0
#undef DebugP_log1
#undef DebugP_log2
#undef DebugP_log3
#undef DebugP_log4
#undef DebugP_log5
#undef DebugP_logZone

#define DebugP_log0(fmt)                               tracePrintf((fmt))
#define DebugP_log1(fmt,a1)                            tracePrintf((fmt),(a1))
#define DebugP_log2(fmt,a1,a2)                         tracePrintf((fmt),(a1),(a2))
#define DebugP_log3(fmt,a1,a2,a3)                      tracePrintf((fmt),(a1),(a2),(a3))
#define DebugP_log4(fmt,a1,a2,a3,a4)                   tracePrintf((fmt),(a1),(a2),(a3),(a4))
#define DebugP_log5(fmt,a1,a2,a3,a4,a5)                tracePrintf((fmt),(a1),(a2),(a3),(a4),(a5))

#define DebugP_logZone(zone, fmt, ...)                 tracePrintf((fmt), ##__VA_ARGS__)

#endif /* DFC_DEBUGP_ROUTED */


#endif // DFC_TRACE_LOGGER_H