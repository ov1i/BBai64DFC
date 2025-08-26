#include <arpa/inet.h>
#include <cerrno>
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "common/types/data_types.h"
#include "common/types/shared_types.h"

static int rpmsg_open(const char* svc_name) {
  DIR* d = opendir("/sys/class/rpmsg");
  if (!d) return -1;

  struct dirent* e;
  char path[256];
  char name[128];
  char devnode[128];

  while ((e = readdir(d))) {
    if (strncmp(e->d_name, "rpmsg", 5) != 0) continue;

    snprintf(path, sizeof(path), "/sys/class/rpmsg/%s/name", e->d_name);
    FILE* f = fopen(path, "r");
    if (!f) continue;

    if (!fgets(name, sizeof(name), f)) {
      fclose(f);
      continue;
    }
    fclose(f);

    // strip newline
    name[strcspn(name, "\r\n")] = 0;

    if (strcmp(name, svc_name) == 0) {
      snprintf(devnode, sizeof(devnode), "/dev/%s", e->d_name);
      int fd = open(devnode, O_RDONLY | O_CLOEXEC); // read-only is enough
      closedir(d);
      return fd;
    }
  }
  closedir(d);
  return -1;
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr,
      "Usage: %s <host_ip> [port=5005] [--rpmsg-name=a72_from_r5f]\n", argv[0]);
    return 1;
  }

  const char* host_ip = argv[1];
  int port = 5005;
  if (argc >= 3 && strncmp(argv[2], "--", 2) != 0) {
    port = std::atoi(argv[2]);
  }

  const char* rpmsg_name = "a72_from_r5f";
  for (int i = 2; i < argc; ++i) {
    if (strncmp(argv[i], "--rpmsg-name=", 13) == 0) {
      rpmsg_name = argv[i] + 13;
    }
  }

  // Open UDP socket
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) { perror("socket"); return 1; }

  sockaddr_in dst{};
  dst.sin_family = AF_INET;
  dst.sin_port   = htons(port);

  if (inet_pton(AF_INET, host_ip, &dst.sin_addr) != 1) {
    std::fprintf(stderr, "Invalid IP: %s\n", host_ip);
    return 1;
  }

  // Open RPMsg endpoint
  int rpfd = rpmsg_open(rpmsg_name);
  if (rpfd < 0) {
    std::fprintf(stderr, "telemetry_bridge: no rpmsg endpoint named '%s'\n", rpmsg_name);
    return 1;
  }

  std::printf("telemetry_bridge: RPMsg '%s' → UDP %s:%d\n", rpmsg_name, host_ip, port);

  // Read-forward loop
  // rpmsg chrdevs deliver whole messages per read() so 1024 is a generous buffer
  alignas(8) uint8 buf[1024]; // sizeof(DFC_t_TelemetryPacket) is well under this
  for (;;) {
    ssize_t n = read(rpfd, buf, sizeof(buf));
    if (n < 0) {
      if (errno == EINTR) continue;
      perror("rpmsg read");
      break;
    }
    if (n == 0) {
      // Shouldn't happen on rpmsg we treat as transient
      continue;
    }

    // Basic validation
    if ((size_t)n < sizeof(DFC_t_TelemetryPacket)) {
      // Here we expect the full DFC_t_TelemetryPacket.
      continue;
    }

    const auto* pkt = reinterpret_cast<const DFC_t_TelemetryPacket*>(buf);
    if (pkt->magic != DFC_TELE_MAGIC) {
      // Ignore unrelated payloads on the same endpoint
      continue;
    }
    size_t send_len = sizeof(DFC_t_TelemetryPacket);
    if (pkt->size >= sizeof(DFC_t_TelemetryPacket) && pkt->size <= (size_t)n) {
      send_len = pkt->size;
    }

    // UDP fire-and-forget
    ssize_t s = sendto(sock, buf, send_len, 0, (sockaddr*)&dst, sizeof(dst));
    if (s < 0) {
      // perror("sendto");  // only if debug necessary
    }
  }

  close(rpfd);
  close(sock);
  return 0;
}
