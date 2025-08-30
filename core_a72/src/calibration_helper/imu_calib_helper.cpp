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

    if (!fgets(name, sizeof(name), f)) { fclose(f); continue; }
    fclose(f);

    // strip newline(s)
    name[strcspn(name, "\r\n")] = 0;

    if (strcmp(name, svc_name) == 0) {
      snprintf(devnode, sizeof(devnode), "/dev/%s", e->d_name);
      int fd = open(devnode, O_RDONLY | O_CLOEXEC);
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
      "Usage: %s <host_ip> [port=49998] [--rpmsg-name=a72_from_r5f_calib]\n", argv[0]);
    return 1;
  }

  const char* host_ip = argv[1];
  int port = 49998;
  if (argc >= 3 && strncmp(argv[2], "--", 2) != 0) {
    port = std::atoi(argv[2]);
  }

  const char* rpmsg_name = "a72_from_r5f_calib";
  for (int i = 2; i < argc; ++i) {
    if (strncmp(argv[i], "--rpmsg-name=", 13) == 0) {
      rpmsg_name = argv[i] + 13;
    }
  }

  // UDP socket
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
    std::fprintf(stderr, "calibration_bridge: no rpmsg endpoint named '%s'\n", rpmsg_name);
    return 1;
  }

  std::printf("calibration_bridge: RPMsg '%s' → UDP %s:%d\n", rpmsg_name, host_ip, port);

  // We’ll forward the chunk exactly as the R5F sent it (passthrough).
  // Allocate safely for your max-chunk size
  constexpr size_t MAX_CHUNK_BYTES = sizeof(DFC_t_CalibChunk); // should already include payload[64]
  alignas(8) uint8_t buf[MAX_CHUNK_BYTES];

  // Sanity helpers for validation
  const size_t hdr_min = offsetof(DFC_t_CalibChunk, payload); // bytes up to start of samples

  for (;;) {
    ssize_t n = read(rpfd, buf, sizeof(buf));
    if (n < 0) {
      if (errno == EINTR) continue;
      perror("rpmsg read");
      break;
    }
    if (n == 0) {
      // Shouldn't happen (char dev). Treat as transient.
      continue;
    }
    if ((size_t)n < hdr_min) {
      // too short to be a header
      continue;
    }

    const auto* ch = reinterpret_cast<const DFC_t_CalibChunk*>(buf);
    if (ch->magic != DFC_CALI_MAGIC) {
      // Ignore unrelated messages on the same endpoint
      continue;
    }

    // Expect exactly header + count*samples
    size_t need = hdr_min + (size_t)ch->count * sizeof(DFC_t_CalibSample);
    if ((size_t)n < need) {
      // Truncated read (unexpected), skip
      continue;
    }

    // Forward as-is over UDP
    (void)sendto(sock, buf, need, 0, (sockaddr*)&dst, sizeof(dst));
  }

  close(rpfd);
  close(sock);
  return 0;
}
