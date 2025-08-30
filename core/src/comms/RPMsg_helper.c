#include "comms/RPMsg_helper_c.h"

#include <stdlib.h>
#include <string.h>
#include <ti/drv/ipc/ipc.h>

struct DFC_t_RPMsg_Wrapper {
    RPMessage_Handle handle;
    uint32         local_endpt;
    uint32         remote_endpt;
    uint16         remote_proc;
    char           remote_name[64];  /* empty string => no remote to resolve */
};

static uint8 isIPCInited = 0;

void rpmsg_init(void) {
    if(isIPCInited == 0) {
        Ipc_InitPrms params;
        IpcInitPrms_init(0, &params);
        Ipc_init(&params);
        isIPCInited = 1;
    }
}

static void rpmsg_reset(DFC_t_RPMsgWrapper* wrapper) {
    if(!wrapper) return;
    wrapper->handle      = NULL;
    wrapper->local_endpt = INVALID_ENDPOINT;
    wrapper->remote_endpt= INVALID_ENDPOINT;
    wrapper->remote_proc = INVALID_REMOTEPROC;
    wrapper->remote_name[0] = '\0';
}

DFC_t_RPMsgWrapper* rpmsg_createWrapper(void) {
    DFC_t_RPMsgWrapper* wrapper = (DFC_t_RPMsgWrapper*)(malloc(sizeof(DFC_t_RPMsgWrapper)));
    if(!wrapper) return NULL;
    rpmsg_reset(wrapper);

    return wrapper;
}

void rpmsg_destroyWrapper(DFC_t_RPMsgWrapper* wrapper) {
    if(!wrapper) return;
    rpmsg_close(wrapper);
    free(wrapper);
}

sint32 rpmsg_open(DFC_t_RPMsgWrapper* wrapper, uint16 remote_proc, const char* local_name, const char* remote_name) {
    if(!wrapper) return -1;
    rpmsg_init();

    rpmsg_close(wrapper);              // clean any previous handle
    wrapper->remote_proc  = remote_proc;
    wrapper->remote_endpt = INVALID_ENDPOINT;

    RPMessage_Params params;
    RPMessage_init(&params);

    wrapper->handle = RPMessage_create(&params, &wrapper->local_endpt);
    if(!wrapper->handle) {
        rpmsg_reset(wrapper);
        return -1;
    }

    if(local_name && local_name[0]) {
        RPMessage_announce((uint32)wrapper->remote_proc, wrapper->local_endpt, local_name);
    }

    if(remote_name && remote_name[0]) {
        size_t n = sizeof(wrapper->remote_name) - 1;
        strncpy(wrapper->remote_name, remote_name, n);
        wrapper->remote_name[n] = '\0';

        uint32 tmp_endp = INVALID_ENDPOINT;
        uint32 tmp_proc = INVALID_REMOTEPROC;
        if(RPMessage_getRemoteEndPt((uint32)wrapper->remote_proc, wrapper->remote_name, &tmp_proc, &tmp_endp, 1000u) == IPC_SOK) {
            wrapper->remote_proc  = (uint16)tmp_proc;
            wrapper->remote_endpt = tmp_endp;
        }
    } else {
        wrapper->remote_name[0] = '\0';
    }

    return 0;
}

sint32 rpmsg_tryResolve(DFC_t_RPMsgWrapper* wrapper, uint32 timeout_us) {
    if(!wrapper) return -1;
    if(wrapper->remote_endpt != INVALID_ENDPOINT) return 0;
    if(wrapper->remote_name[0] == '\0') return -2;

    uint32 tmp_endp = INVALID_ENDPOINT;
    uint32 tmp_proc = INVALID_REMOTEPROC;
    if(RPMessage_getRemoteEndPt((uint32)wrapper->remote_proc, wrapper->remote_name, &tmp_proc, &tmp_endp, timeout_us) == IPC_SOK) {
        wrapper->remote_proc  = (uint16)tmp_proc;
        wrapper->remote_endpt = tmp_endp;
        return 0;
    }

    return -1;
}

sint32 rpmsg_send(DFC_t_RPMsgWrapper* wrapper, const void* buffer, sint32 len) {
    if(!wrapper || !wrapper->handle || !buffer || len <= 0) return -1;

    if(wrapper->remote_endpt == INVALID_ENDPOINT && wrapper->remote_name[0] != '\0') {
        (void)rpmsg_tryResolve(wrapper, 0);
    }

    if(wrapper->remote_endpt == INVALID_ENDPOINT) return -1;

    // RPMessage_send length is uint16 we need to clamp just in case
    uint16 clamp_len = (len > 0xFFFF) ? 0xFFFFu : (uint16)len;

    sint32 status = RPMessage_send(wrapper->handle, (uint32)wrapper->remote_proc, wrapper->remote_endpt, wrapper->local_endpt, (void*)buffer, clamp_len);
    
    return (status == IPC_SOK) ? 0 : -1;
}

sint32 rpmsg_recv(DFC_t_RPMsgWrapper* wrapper, void* buffer, sint32 max_len, sint32 timeout_us) {
    if(!wrapper || !wrapper->handle || !buffer || max_len <= 0) return -1;

    uint16 len = (uint16)max_len;
    uint32 from_endp = INVALID_ENDPOINT;
    uint32 from_proc = INVALID_REMOTEPROC;
    uint32 to = (timeout_us < 0) ? 0xFFFFFFFFu : (uint32)timeout_us;

    sint32 status = RPMessage_recv(wrapper->handle, buffer, &len, &from_endp, &from_proc, to);
    if(status != IPC_SOK) return -1;

    // Learn the peer endpoint (first packet or if the peer changes)
    wrapper->remote_proc  = (uint16)from_proc;
    wrapper->remote_endpt = from_endp;

    return (sint32)len;
}

void rpmsg_close(DFC_t_RPMsgWrapper* wrapper) {
    if(!wrapper) return;
    if(wrapper->handle) {
        RPMessage_delete(&wrapper->handle);
        wrapper->handle = NULL;
    }

    wrapper->local_endpt  = INVALID_ENDPOINT;
    wrapper->remote_endpt = INVALID_ENDPOINT;
    wrapper->remote_proc  = INVALID_REMOTEPROC;
    wrapper->remote_name[0] = '\0';
}

sint32 rpmsg_isDstReady(const DFC_t_RPMsgWrapper* wrapper) {
     return (wrapper && wrapper->remote_endpt != INVALID_ENDPOINT) ? 1 : 0;
}
uint16 rpmsg_getRemoteProc(const DFC_t_RPMsgWrapper* wrapper) { 
    return wrapper ? wrapper->remote_proc  : INVALID_REMOTEPROC; 
}
uint32 rpmsg_getRemoteEndpt(const DFC_t_RPMsgWrapper* wrapper) { 
    return wrapper ? wrapper->remote_endpt : INVALID_ENDPOINT; 
}
uint32 rpmsg_getLocalEndpt (const DFC_t_RPMsgWrapper* wrapper) { 
    return wrapper ? wrapper->local_endpt  : INVALID_ENDPOINT; 
}
