#include "comms/rpmsg_helper.hpp"
#include <cstdlib>
#include <cstring>

namespace rpmsg {

bool& C_RPMsgHelper::isInited() {
  static bool state = false;
  return state;
}

void C_RPMsgHelper::init() {
  if (isInited()) return;
  Ipc_InitPrms params;
  Ipc_initPrms_init(&params);
  Ipc_init(&params);
  isInited() = true;
}

bool C_RPMsgHelper::open(uint16 remote_proc, const char* local_name, const char* remote_name) {
  init();

  m_remote_proc = remote_proc;

  RPMessage_CreateParams params;
  RPMessage_CreateParams_init(&params);

  m_handler = RPMessage_create(&params, &m_local_endpt);
  if (!m_handler) return false;

  if (local_name && local_name[0]) {
    RPMessage_announce(static_cast<uint32>(m_remote_proc), static_cast<uint32>(m_local_endpt), local_name);
  }

  if (remote_name && remote_name[0]) {
    std::strncpy(m_remoteName, remote_name, sizeof(m_remoteName) - 1);
    uint32 tmpRENDP = 0xFFFFFFFFu;
    uint32 tmpRPROC = INVALID_ENDPOINT;

    if (RPMessage_getRemoteEndPt(static_cast<uint32>(m_remote_proc), m_remoteName, &tmpRPROC, &tmpRENDP, 1000U) == IPC_SOK) {
      m_remote_proc  = static_cast<uint16>(tmpRPROC);
      m_remote_endpt = tmpRENDP;
    }
  }

  return true;
}

sint32 C_RPMsgHelper::tryResolve(uint32 timeout_us) {
  if (m_remote_endpt != INVALID_ENDPOINT) return 0;
  if (m_remoteName[0] == '\0') return -2;

  uint32 tmpRENDP = 0xFFFFFFFFu;
  uint32 tmpRPROC = INVALID_ENDPOINT;
  if (RPMessage_getRemoteEndPt(static_cast<uint32>(m_remote_proc), m_remoteName, &tmpRPROC, &tmpRENDP, timeout_us) == IPC_SOK) {
    m_remote_proc  = static_cast<uint16>(tmpRPROC);
    m_remote_endpt = tmpRENDP;

    return 0;
  }
  return -1;
}

sint32 C_RPMsgHelper::send(const void* buffer, sint32 len) {
  if (!m_handler || !buffer || len <= 0) return -1;

  if (m_remote_endpt == INVALID_ENDPOINT && m_remoteName[0] != '\0') {
    (void)tryResolve(0);
  }
  
  if (m_remote_endpt == INVALID_ENDPOINT) return -1;

  // RPMessage_send length is uint16 we need to clamp just in case
  uint16 clamp_len = (len > 0xFFFF) ? 0xFFFF : static_cast<uint16>(len);
  sint32 status = RPMessage_send(m_handler, static_cast<uint32>(m_remote_proc), static_cast<uint32>(m_remote_endpt), static_cast<uint32>(m_local_endpt), const_cast<void*>(buffer), clamp_len);
  return (status == IPC_SOK) ? 0 : -1;
}

sint32 C_RPMsgHelper::recv(void* buffer, sint32 max_len, sint32 timeout_us) {
  if (!m_handler || !buffer || max_len <= 0) return -1;

  uint16 len = static_cast<uint16>(max_len);
  uint32 tmpRENDP = 0xFFFFFFFFu;
  uint32 tmpRPROC = INVALID_ENDPOINT;
  uint32 timeout_period = (timeout_us < 0) ? 0xFFFFFFFFu : static_cast<uint32>(timeout_us);

  sint32 status = RPMessage_recv(m_handler, buffer, &len, &tmpRENDP, &tmpRPROC, timeout_period);
  if (status != IPC_SOK) return -1;

  // Learn the peer endpoint (first packet or if the peer changes)
  m_remote_proc  = tmpRPROC;
  m_remote_endpt = tmpRENDP;

  return static_cast<sint32>(len);
}

void C_RPMsgHelper::close() {
  if (m_handler) {
    RPMessage_delete(&m_handler);
    m_handler = nullptr;
  }
  m_local_endpt  = INVALID_ENDPOINT;
  m_remote_endpt = INVALID_ENDPOINT;
  m_remote_proc  = INVALID_REMOTEPROC;
  m_remoteName[0] = '\0';
}

C_RPMsgHelper::~C_RPMsgHelper() {
  close();
}

} // namespace rpmsg
