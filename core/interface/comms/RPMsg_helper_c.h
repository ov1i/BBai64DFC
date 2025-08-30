#ifndef DFC_RPMSG_HELPER_C_H
#define DFC_RPMSG_HELPER_C_H

#include <stdint.h>
#include <stddef.h>
#include <ti/drv/ipc/ipc.h>
#include <data_types.h>
#include <shared_types.h>
#include <utils.h>

typedef struct DFC_t_RPMsg_Wrapper DFC_t_RPMsgWrapper;

void rpmsg_init(void);

// Create/destroy a link object
DFC_t_RPMsgWrapper* rpmsg_createWrapper(void);
void rpmsg_destroyWrapper(DFC_t_RPMsgWrapper* wrapper);

  // Create a local endpoint and announce a service name.
  // - remote_proc: target proc ID
  // - local_name : service name we ANNOUNCE (Note.: Nullptr will not announce anything)
  // - remote_name: service name we RESOLVE on the remote
sint32 rpmsg_open(DFC_t_RPMsgWrapper* wrapper, uint16 remote_proc, const char* local_name, const char* remote_name);

// Triess to resolve remote endpoint by name -> returns: 0 if resolved, -1 if not yet, -2 if no remote_name was set.
sint32 rpmsg_tryResolve(DFC_t_RPMsgWrapper* wrapper, uint32 timeout_us);

 // send(): 0 on success, -1 if remote not ready / error.
sint32 rpmsg_send(DFC_t_RPMsgWrapper* wrapper, const void* buffer, sint32 len);
  // recv(): returns #bytes, or -1 on error/timeout. timeout_us<0 = forever.
sint32 rpmsg_recv(DFC_t_RPMsgWrapper* wrapper, void* buffer, sint32 max_len, sint32 timeout_us);

  // Close endpoint
void rpmsg_close(DFC_t_RPMsgWrapper* wrapper);

  // Status
sint32 rpmsg_isDstReady(const DFC_t_RPMsgWrapper* wrapper);
uint16_t rpmsg_getRemoteProc(const DFC_t_RPMsgWrapper* wrapper);
uint32_t rpmsg_getRemoteEndpt(const DFC_t_RPMsgWrapper* wrapper);
uint32_t rpmsg_getLocalEndpt (const DFC_t_RPMsgWrapper* wrapper);

#endif /* DFC_RPMSG_HELPER_C_H */
