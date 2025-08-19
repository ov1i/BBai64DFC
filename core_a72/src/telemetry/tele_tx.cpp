#include "tele_tx.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <cstring>
#include <iostream>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>


int main() {
    int mem_filedescriptor = open("/dev/mem", O_RDONLY | O_SYNC);
    if (mem_filedescriptor < 0) {
        perror("open /dev/mem");
        return -1;
    }

    TelemetryPacket* telemetryMap = (TelemetryPacket*) mmap(
        NULL,
        shared_mem_size,
        PROT_READ,
        MAP_SHARED,
        mem_filedescriptor,
        shared_mem_addr
    );
    if (telemetryMap == MAP_FAILED) {
        perror("Failed to map shared mem to UDP packet!");
        close(mem_filedescriptor);
        return -1;
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("Failed to create socket!");
        return 1;
    }

    sockaddr_in dest_addr;
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(5005);
    inet_pton(AF_INET, "192.168.1.199", &dest_addr.sin_addr);

    while (true) {
        TelemetryPacket pkt;
        memcpy(&pkt, telemetryMap, sizeof(pkt));

        sendto(sock, &pkt, sizeof(pkt), 0, (sockaddr*)&dest_addr, sizeof(dest_addr));

        usleep(5000);
    }

    close(sock);
    munmap(telemetryMap, shared_mem_size);
    close(mem_filedescriptor);
    return 0;
}
