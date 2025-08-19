#ifndef LOGGER_H
#define LOGGER_H

extern "C" {
#include <cstdarg>
// #include <cstddef>
#include <cstring>
#include "FreeRTOS.h"
#include "semphr.h"
}

#include <data_types.h>
#include <utils.h>

namespace comms {
    class Logger {
    public:
        static Logger& instance();
        void init();
        void log(const char* fmt, ...);
        size_t read(char* out, size_t maxlen);

        void set_rpmsg(struct rpmsg_lite_instance* rl, struct rpmsg_lite_endpoint* ep, uint32 linux_addr);
        bool send_log(const char* msg);
    private:
        Logger();
        char log_buffer[LOGGER_BUFFER_SIZE];
        size_t log_head, log_tail;
        SemaphoreHandle_t mutex;

        struct rpmsg_lite_instance* rl_instance;
        struct rpmsg_lite_endpoint* log_ept;
        uint32 linux_rpmsg_addr;
    };

} // namespace comms

#endif // LOGGER_H
