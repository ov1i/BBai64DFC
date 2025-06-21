#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stddef.h>
#include <data_types.h>

#ifdef __cplusplus

void logger_init(void);
void logger_log(const char *fmt, ...);
uint32  logger_read(char *out, size_t maxlen);

#endif

#endif // LOGGER_H
