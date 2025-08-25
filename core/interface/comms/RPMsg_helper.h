#ifndef DFC_RPMSG_HELPER_H
#define DFC_RPMSG_HELPER_H

#include "data_types.h"
#include "utils.h"
#include <cstdint>
#include <cstddef>
#include <cstring>

extern "C" {
#include <ti/drv/ipc/ipc.h>
#include <ti/drv/ipc/ipc_rpmsg.h>
#include <ti/drv/ipc/include/rpmessage.h>
}

namespace rpmsg {

class C_RPMsgHelper {
public:
  static void init();

  // Create a local endpoint and announce a service name.
  // - remote_proc: target proc ID
  // - local_name : service name we ANNOUNCE (Note.: Nullptr will not announce anything)
  // - remote_name: service name we RESOLVE on the remote
  bool open(uint16 remote_proc, const char* local_name, const char* remote_name = nullptr);

  // Triess to resolve remote endpoint by name -> returns: 0 if resolved, -1 if not yet, -2 if no remote_name was set.
  sint32 tryResolve(uint32 timeout_us = 0);

  // send(): 0 on success, -1 if remote not ready / error.
  sint32 send(const void* buffer, sint32 len);

  // recv(): returns #bytes, or -1 on error/timeout. timeout_us<0 = forever.
  sint32 recv(void* buffer, sint32 max_len, sint32 timeout_us);

  // Close endpoint
  void close();

  // Status
  bool   isDstReady()   const { return m_remote_endpt != INVALID_ENDPOINT; }
  uint16 getRemoteProc() const { return m_remote_proc; }
  uint32 getRemoteEndpt() const { return m_remote_endpt; }
  uint32 getLocalEndpt()  const { return m_local_endpt; }

  ~C_RPMsgHelper();

private:
  static bool& isInited();

  RPMessage_Handle m_handler{ nullptr };
  uint32           m_local_endpt{ INVALID_ENDPOINT };  // unknown until created
  uint32           m_remote_endpt{ INVALID_ENDPOINT }; // unknown until resolved/received
  uint16           m_remote_proc{ INVALID_REMOTEPROC };
  char             m_remoteName[64]{};          // empty if none
};

} // namespace rpmsg

#endif // DFC_RPMSG_HELPER_H
