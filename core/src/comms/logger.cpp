#include "comms/logger.h"

extern "C" {
#include <cstdio>
#include <cstdarg>
}

#include <data_types.h>
#include <utils.h>

namespace comms {
    Logger& Logger::instance() {
        static Logger inst;
        return inst;
    }
    
    Logger::Logger() : log_head(0), log_tail(0), rl_instance(nullptr), log_ept(nullptr), linux_rpmsg_addr(0) {
        mutex = xSemaphoreCreateMutex();
    }
    
    void Logger::init() {
        log_head = log_tail = 0;
    }
    
    void Logger::log(const char* fmt, ...) {
        char temp[LOGGER_MSG_MAXLEN];
        va_list args;
        va_start(args, fmt);
        int len = vsnprintf(temp, sizeof(temp), fmt, args);
        va_end(args);
        if (len <= 0) return;
        if (len >= LOGGER_MSG_MAXLEN) len = LOGGER_MSG_MAXLEN - 1;
        
        xSemaphoreTake(mutex, portMAX_DELAY);
        for (int i = 0; i < len; ++i) {
            log_buffer[log_head] = temp[i];
            log_head = (log_head + 1) % LOGGER_BUFFER_SIZE;
            if (log_head == log_tail)
            log_tail = (log_tail + 1) % LOGGER_BUFFER_SIZE; // Overwrite old if full
        }
        log_buffer[log_head] = '\n';
        log_head = (log_head + 1) % LOGGER_BUFFER_SIZE;
        if (log_head == log_tail)
        log_tail = (log_tail + 1) % LOGGER_BUFFER_SIZE;
        xSemaphoreGive(mutex);
    }
    
    size_t Logger::read(char* out, size_t maxlen) {
        xSemaphoreTake(mutex, portMAX_DELAY);
        size_t count = 0;
        while (log_tail != log_head && count < maxlen - 1) {
            out[count++] = log_buffer[log_tail];
            if (log_buffer[log_tail] == '\n') { // Stop at newline (one message per read)
                log_tail = (log_tail + 1) % LOGGER_BUFFER_SIZE;
                break;
            }
            log_tail = (log_tail + 1) % LOGGER_BUFFER_SIZE;
        }
        out[count] = '\0';
        xSemaphoreGive(mutex);
        return count;
    }

    void Logger::set_rpmsg(struct rpmsg_lite_instance* rl, struct rpmsg_lite_endpoint* ep, uint32 linux_addr) {
        rl_instance = rl;
        log_ept = ep;
        linux_rpmsg_addr = linux_addr;
    }

    bool Logger::send_log(const char* msg) {
        if (rl_instance && log_ept && linux_rpmsg_addr) {
            return rpmsg_lite_send(rl_instance, log_ept, linux_rpmsg_addr, msg, strlen(msg)+1, RL_BLOCK) == RL_SUCCESS;
        }
        return false;
    }
    
} // namespace comms
