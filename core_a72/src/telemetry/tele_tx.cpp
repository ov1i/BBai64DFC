#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "common/types/data_types.h"
#include "common/types/shared_types.h"

static int rpmsg_open(const char* svc_name){
  DIR* d = opendir("/sys/class/rpmsg");
  if(!d) return -1;
  struct dirent* e; 
  char path[256], name[128], devnode[128];

  while((e=readdir(d))){
    if(strncmp(e->d_name,"rpmsg",5)!=0) continue;
    snprintf(path,sizeof(path),"/sys/class/rpmsg/%s/name", e->d_name);

    FILE* f = fopen(path,"r");
    if(!f) continue;
    
    if(!fgets(name,sizeof(name),f)){ 
      fclose(f); 
      continue; 
    }
    fclose(f);
    name[strcspn(name,"\r\n")] = 0;
    
    if(strcmp(name, svc_name)==0){
      snprintf(devnode,sizeof(devnode),"/dev/%s", e->d_name);
      int fd = open(devnode, O_RDWR|O_CLOEXEC);
      closedir(d);
      return fd;
    }
  }
  closedir(d);
  return -1;
}

int main(int argc, char** argv) {
    if(argc < 2) {
        fprintf(stderr, "Usage: %s <host_ip> [port=5005] [--rpmsg-name=r5f_to_a72]\n", argv[0]);
        return 1;
    }

    const char* host_ip = argv[1];
    int port = (argc >= 3 && strncmp(argv[2], "--", 2) != 0) ? atoi(argv[2]) : 5005;

    const char* rpmsgName = "r5f_to_a72";
    for(int i = 2; i < argc; i++) {
        if (strncmp(argv[i], "--rpmsg-name=", 12) == 0) {
            rpmsgName = argv[i] + 12;
        }
    }

    int memfd = open("/dev/mem", O_RDONLY | O_SYNC);
    if(memfd < 0) {
        perror("open /dev/mem");
        return 1;
    }

    long pagesz = sysconf(_SC_PAGESIZE);
    if(pagesz <= 0) {
        pagesz = 4096;
    }

    off_t phys = (off_t)telem_addr;
    off_t page_base = phys & ~(off_t)(pagesz - 1);
    off_t page_off  = phys - page_base;
    size_t need     = sizeof(DFC_t_TelemetryPacket);
    size_t map_len  = ((need + page_off + (size_t)pagesz - 1) / (size_t)pagesz) * (size_t)pagesz;

    void* map_base = mmap(nullptr, map_len, PROT_READ, MAP_SHARED, memfd, page_base);
    if(map_base == MAP_FAILED) {
        perror("mmap tele");
        close(memfd);
        return 1;
    }
    volatile DFC_t_TelemetryPacket* pTelemetryPort = (volatile DFC_t_TelemetryPacket*)((uint8*)map_base + page_off);

    int rpfd = rpmsg_open(rpmsgName);
    if(rpfd < 0) {
        fprintf(stderr, "tele_tx: no rpmsg endpoint named '%s'\n", rpmsgName);
        return 1;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock < 0) {
        perror("socket");
        return 1;
    }

    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port   = htons(port);

    if(inet_pton(AF_INET, host_ip, &dst.sin_addr) != 1) {
        fprintf(stderr, "bad ip\n");
        return 1;
    }
    printf("tele_tx: waiting TELEM_READY on '%s' → %s:%d\n", rpmsgName, host_ip, port);

    for (;;) {
        DFC_t_MsgTelemReady telemetryMessage{};
        ssize_t result = read(rpfd, &telemetryMessage, sizeof(telemetryMessage));
        if (result < 0) {
            if(errno == EINTR) {
                continue;
            }
            if(errno == EAGAIN) {
                usleep(1000);
                continue;
            }
            perror("rpmsg read");
            break;
        }

        if((size_t)result < sizeof(telemetryMessage) || telemetryMessage.type != TELEM_READY) {
            continue;
        }

        DFC_t_TelemetryPacket telemetryPacket;
        for (;;) {
            uint32 begin = __atomic_load_n(&pTelemetryPort->seqLock, __ATOMIC_ACQUIRE);
            if(begin & 1u) {
                continue;
            }

            memcpy(&telemetryPacket, (const void*)pTelemetryPort, sizeof(telemetryPacket));
            uint32 end = __atomic_load_n(&pTelemetryPort->seqLock, __ATOMIC_ACQUIRE);
            if(begin == end) {
                break;
            }
        }
        // send payload (datas)
        const uint8* payload = ((const uint8*)&telemetryPacket) + offsetof(DFC_t_TelemetryPacket, timestamp_imu_1);
        size_t plen = sizeof(DFC_t_TelemetryPacket) - offsetof(DFC_t_TelemetryPacket, timestamp_imu_1);
        (void)sendto(sock, payload, plen, 0, (sockaddr*)&dst, sizeof(dst));
    }
    munmap((void*)map_base, map_len);
    close(memfd);
    close(rpfd);
    close(sock);

    return 0;
}
