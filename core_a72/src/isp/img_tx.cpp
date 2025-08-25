#include <arpa/inet.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <poll.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "common/types/data_types.h"
#include "common/types/shared_types.h"

static inline uint64 now_ns() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (uint64)ts.tv_sec * 1000000000ull + (uint64)ts.tv_nsec;
}

static bool getImageHeaderAtomically(uint8 slot, DFC_t_ImageHeader* out, volatile DFC_t_ImageHeader* pImageHeader) {
    while (true) {
        uint32 s0 = __atomic_load_n(&pImageHeader->seqLock, __ATOMIC_ACQUIRE);
        if(s0 & 1u) {
            continue;
        }
        __sync_synchronize();
        memcpy(out, (const void*)pImageHeader, sizeof(*out));
        __sync_synchronize();
        
        uint32 s1 = __atomic_load_n(&pImageHeader->seqLock, __ATOMIC_ACQUIRE);
        if(s0 == s1 && !(s1 & 1u)) {
            return true;
        }
    }
}

static void copySlotAtomically(uint8 slot, uint8* dst, DFC_t_ImageHeader* pOutputImageHeader, volatile DFC_t_ImageHeader* pImageHeader, const uint8* Y) {
    // Copy image then verify header and repeat if necessary
    for (;;) {
        uint32 s0 = __atomic_load_n(&pImageHeader->seqLock, __ATOMIC_ACQUIRE);
        if(s0 & 1u) {
            continue;
        }

        memcpy(dst, Y, getImageDataSize());
        __sync_synchronize();

        DFC_t_ImageHeader tmp;
        memcpy(&tmp, (const void*)pImageHeader, sizeof(tmp));

        uint32 s1 = __atomic_load_n(&pImageHeader->seqLock, __ATOMIC_ACQUIRE);
        if(s0 == s1 && !(s1 & 1u)) {
            *pOutputImageHeader = tmp;
            return;
        }
    }
}

int main(int argc, char** argv) {
    if(argc < 3) {
        fprintf(stderr, "Usage: %s <host_ip> <port> [--max-fps=N]\n", argv[0]);
        return 1;
    }
    const char* host_ip = argv[1];
    int port = atoi(argv[2]);
    int max_fps = 20;

    for(int i = 3; i < argc; i++) {
        if(strncmp(argv[i], "--max-fps=", 10) == 0) {
            max_fps = atoi(argv[i] + 10);
        }
    }
    // open mem location for looking only
    int memfd = open("/dev/mem", O_RDONLY | O_SYNC);
    if(memfd < 0) {
        perror("open /dev/mem");
        return 1;
    }

    size_t map_len = 2 * getImagePortSize();
    long pagesz = sysconf(_SC_PAGESIZE);
    if(pagesz <= 0) {
        pagesz = 4096;
    }

    off_t page_base = (off_t)baseAddr & ~(off_t)(pagesz - 1);
    off_t page_off  = (off_t)baseAddr - page_base;
    size_t map_req  = ((map_len + page_off + (size_t)pagesz - 1) / (size_t)pagesz) * (size_t)pagesz;

    void* map_base = mmap(nullptr, map_req, PROT_READ, MAP_SHARED, memfd, page_base);
    if(map_base == MAP_FAILED) {
        perror("mmap images");
        close(memfd);
        return 1;
    }

    uint8* base = (uint8*)map_base + page_off;
    volatile DFC_t_ImageHeader* H0 = (volatile DFC_t_ImageHeader*)(base + 0);
    const uint8* Y0 = ((const uint8*)H0) + sizeof(DFC_t_ImageHeader);
    volatile DFC_t_ImageHeader* H1 = (volatile DFC_t_ImageHeader*)(base + getImagePortSize());
    const uint8* Y1 = ((const uint8*)H1) + sizeof(DFC_t_ImageHeader);

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if(sock < 0) {
        perror("socket");
        return 1;
    }

    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(port);
    if(inet_pton(AF_INET, host_ip, &dst.sin_addr) != 1) {
        fprintf(stderr, "bad ip\n");
        return 1;
    }
    // buffers for safety, used later
    std::unique_ptr<uint8[]> prevBuf(new uint8[getImageDataSize()]);
    std::unique_ptr<uint8[]> currBuf(new uint8[getImageDataSize()]);
    DFC_t_ImageHeader h0{}, h1{}, hp{}, hc{};

    const size_t MTU = 1400;
    const size_t payload = MTU - sizeof(DFC_t_UDPImgHeader);
    const uint16 chunk_count = (uint16)((getImageDataSize() + payload - 1) / payload);

    uint32 frame_id = 0;
    uint64 last_sent_ns = 0;
    uint64 min_interval_ns = (max_fps > 0) ? (1000000000ull / (uint64)max_fps) : 0;

    printf("img_udp_tx: %ux%u → %s:%d, %u chunks/frame, max_fps=%d\n",
           (unsigned)width, (unsigned)height, host_ip, port,
           (unsigned)chunk_count, max_fps);

    for (;;) {
        uint64 now = now_ns();
        if(min_interval_ns && (now - last_sent_ns) < min_interval_ns) {
            usleep(1000);
            continue;
        }
        // WE decide which is which (latest/current slot) by timestamp
        getImageHeaderAtomically(0, &h0, H0);
        getImageHeaderAtomically(1, &h1, H1);
        const bool s1_is_newer = (h1.ts_ns > h0.ts_ns);
        uint8 curr = s1_is_newer ? 1 : 0;
        uint8 prev = curr ^ 1;

        // Copy both slots atomically into local buffers
        if(curr == 0) {
            copySlotAtomically(0, currBuf.get(), &hc, H0, Y0);
            copySlotAtomically(1, prevBuf.get(), &hp, H1, Y1);
        } else {
            copySlotAtomically(1, currBuf.get(), &hc, H1, Y1);
            copySlotAtomically(0, prevBuf.get(), &hp, H0, Y0);
        }
        
        auto send_one = [&](const uint8* img, const DFC_t_ImageHeader& hh, uint8 id) {
            DFC_t_UDPImgHeader udpHeader{};
            udpHeader.magic = 0x31474D49u;
            udpHeader.frame_id = frame_id;
            udpHeader.ts_ns = hh.ts_ns;
            udpHeader.width = width;
            udpHeader.height = height;
            udpHeader.idx = id;
            udpHeader.chunk_count = chunk_count;

            for(uint16 idx = 0; idx < chunk_count; ++idx) {
                udpHeader.chunk_idx = idx;
                size_t off = (size_t)idx * payload;
                size_t left = (off < getImageDataSize()) ? (getImageDataSize() - off) : 0;
                size_t n = (left < payload) ? left : payload;

                uint8 buf[1600];
                memcpy(buf, &udpHeader, sizeof(udpHeader));
                if(n) {
                    memcpy(buf + sizeof(udpHeader), img + off, n);
                }
                (void)sendto(sock, buf, sizeof(udpHeader) + n, 0, (sockaddr*)&dst, sizeof(dst));
            }
        };

        send_one(prevBuf.get(), hp, 0);
        send_one(currBuf.get(), hc, 1);
        frame_id++;
        last_sent_ns = now_ns();
    }
    munmap(map_base, map_req);
    close(memfd);
    close(sock);
    return 0;
}
