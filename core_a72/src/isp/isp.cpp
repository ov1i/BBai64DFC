// isp.cpp — A72 capture → shared DDR (double buffer) + RPMsg doorbell
#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "isp.h"

static volatile sig_atomic_t g_run = 1;
static void on_sigint(int) { g_run = 0; }

static inline uint64 now_ns() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return (uint64)ts.tv_sec * 1000000000ull + (uint64)ts.tv_nsec;
}

static inline void yuyv2y(const uint8 *yuyv, size_t yuyv_bytes, uint8 *yCh,
                          size_t y_bytes) {
  size_t totalSize = yuyv_bytes / 2;
  if (totalSize > y_bytes)
    totalSize = y_bytes;
  for (size_t pxl = 0, i = 0; pxl < totalSize; ++pxl, i += 2)
    yCh[pxl] = yuyv[i];
}

static int RPMsgFallback() {
  DIR *d = opendir("/dev");
  if (!d)
    return -1;
  struct dirent *e;
  char path[256];
  while ((e = readdir(d))) {
    if (strncmp(e->d_name, "rpmsg", 5) == 0) {
      snprintf(path, sizeof(path), "/dev/%s", e->d_name);
      int fd = open(path, O_RDWR | O_CLOEXEC);
      if (fd >= 0) {
        closedir(d);
        return fd;
      }
    }
  }
  closedir(d);
  return -1;
}

int main(int argc, char **argv) {
  const char *imSensorStream = (argc > 1) ? argv[1] : "/dev/video0";
  const char *rpmsgStream = (argc > 2) ? argv[2] : nullptr;

  signal(SIGINT, on_sigint);

  int memfd = -1, vfd = -1, rpfd = -1;
  void *map_base = MAP_FAILED;
  size_t map_len = 0;

  ImageHeader_t *pImageHeader = nullptr;
  ImageHeader_t *pImageHeaderPrev = nullptr;
  uint8 *pImageData = nullptr;
  uint8 *pImageDataPrev = nullptr;

  v4l2_requestbuffers requestBuffer{};
  V4L2Buf_t buffers[kV4L2BufferCount]{};
  v4l2_buf_type bufferType = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  bool ok = false;

  do {
    const off_t baseAddrOffset = (off_t)baseAddr;
    const size_t reqSize = getImagePortSize() * 2;

    memfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (memfd < 0) {
      perror("open /dev/mem");
      break;
    }

    long pagesz = sysconf(_SC_PAGESIZE);
    if (pagesz <= 0)
      pagesz = 4096;
    off_t page_base = baseAddrOffset & ~(off_t)(pagesz - 1);
    off_t page_off = baseAddrOffset - page_base;
    map_len = ((reqSize + page_off + (size_t)pagesz - 1) / (size_t)pagesz) *
              (size_t)pagesz;

    map_base = mmap(nullptr, map_len, PROT_READ | PROT_WRITE, MAP_SHARED, memfd,
                    page_base);
    if (map_base == MAP_FAILED) {
      perror("mmap /dev/mem");
      break;
    }

    uint8 *offsetedAddr = (uint8 *)map_base + page_off;

    pImageHeaderPrev = (ImageHeader_t *)(offsetedAddr + 0);
    pImageDataPrev = (uint8 *)pImageHeaderPrev + sizeof(ImageHeader_t);
    pImageHeader = (ImageHeader_t *)(offsetedAddr + getImagePortSize());
    pImageData = (uint8 *)pImageHeader + sizeof(ImageHeader_t);

    memset(pImageHeaderPrev, 0, sizeof(*pImageHeaderPrev));
    memset(pImageHeader, 0, sizeof(*pImageHeader));
    pImageHeaderPrev->width = pImageHeader->width = width;
    pImageHeaderPrev->height = pImageHeader->height = height;
    pImageHeaderPrev->stride = pImageHeader->stride = width;
    pImageHeaderPrev->size = pImageHeader->size = (uint32)getImageDataSize();
    pImageHeaderPrev->seqLock = pImageHeader->seqLock = 0;
    pImageHeaderPrev->duration = pImageHeader->duration = 0.0;

    if (rpmsgStream) {
      rpfd = open(rpmsgStream, O_RDWR | O_CLOEXEC);
      if (rpfd < 0) {
        perror("open rpmsg_dev"); /* not fatal; keep running */
      }
    } else {
      rpfd = RPMsgFallback();
      if (rpfd < 0)
        fprintf(stderr,
                "Warning: no /dev/rpmsg*; running without doorbells.\n");
    }

    vfd = open(imSensorStream, O_RDWR);
    if (vfd < 0) {
      perror("open video");
      break;
    }

    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(vfd, VIDIOC_S_FMT, &fmt) < 0) {
      perror("VIDIOC_S_FMT");
      break;
    }

    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV) {
      fprintf(stderr, "Driver refused YUYV (0x%08x)\n",
              fmt.fmt.pix.pixelformat);
      break;
    }

    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = fps;
    (void)ioctl(vfd, VIDIOC_S_PARM, &parm);

    requestBuffer = {};
    requestBuffer.count = kV4L2BufferCount;
    requestBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    requestBuffer.memory = V4L2_MEMORY_MMAP;
    if (ioctl(vfd, VIDIOC_REQBUFS, &requestBuffer) < 0) {
      perror("VIDIOC_REQBUFS");
      break;
    }

    if (requestBuffer.count < kV4L2BufferCount) {
      fprintf(stderr, "Not enough V4L2 bufs\n");
      break;
    }

    for (uint8 i = 0; i < kV4L2BufferCount; ++i) {
      v4l2_buffer tempBuff{};
      tempBuff.type = requestBuffer.type;
      tempBuff.memory = requestBuffer.memory;
      tempBuff.index = i;
      if (ioctl(vfd, VIDIOC_QUERYBUF, &tempBuff) < 0) {
        perror("VIDIOC_QUERYBUF");
        break;
      }
      buffers[i].length = tempBuff.length;
      buffers[i].start = mmap(nullptr, tempBuff.length, PROT_READ | PROT_WRITE,
                              MAP_SHARED, vfd, tempBuff.m.offset);

      if (buffers[i].start == MAP_FAILED) {
        perror("mmap v4l2 buf");
        break;
      }
    }
    bool mmap_ok = true;
    for (uint8 i = 0; i < kV4L2BufferCount; ++i) {
      if (!buffers[i].start) {
        mmap_ok = false;
        break;
      }
    }
    if (!mmap_ok)
      break;

    for (uint8 i = 0; i < kV4L2BufferCount; ++i) {
      v4l2_buffer tempBuff{};
      tempBuff.type = requestBuffer.type;
      tempBuff.memory = requestBuffer.memory;
      tempBuff.index = i;

      if (ioctl(vfd, VIDIOC_QBUF, &tempBuff) < 0) {
        perror("VIDIOC_QBUF");
        mmap_ok = false;
        break;
      }
    }
    if (!mmap_ok)
      break;

    if (ioctl(vfd, VIDIOC_STREAMON, &bufferType) < 0) {
      perror("VIDIOC_STREAMON");
      break;
    }

    ok = true;
  } while (0);

  if (!ok) {
    // fall through to cleanup
  } else {
    printf("ISP: %ux%u YUYV→GRAY8 → DDR @ 0x%08llX [prev|curr], RPMsg "
           "doorbells.\n",
           (unsigned)width, (unsigned)height, (unsigned long long)baseAddr);

    uint32 seqLock = 0;
    bool movedCurrent = false;

    while (g_run) {
      pollfd pfd{.fd = vfd, .events = POLLIN};
      int pr = poll(&pfd, 1, 1000);
      if (pr < 0) {
        if (errno == EINTR)
          continue;
        perror("poll");
        break;
      }
      if (pr == 0) {
        fprintf(stderr, "poll timeout\n");
        continue;
      }

      v4l2_buffer tempBuff{};
      tempBuff.type = requestBuffer.type;
      tempBuff.memory = requestBuffer.memory;
      if (ioctl(vfd, VIDIOC_DQBUF, &tempBuff) < 0) {
        if (errno == EINTR || errno == EAGAIN)
          continue;
        perror("VIDIOC_DQBUF");
        break;
      }

      const uint8 *yuyv = (const uint8 *)buffers[tempBuff.index].start;
      const size_t yuyvSize = (size_t)width * (size_t)height * 2;

      const uint64 t_start = now_ns();

      if (movedCurrent) {
        memcpy(pImageDataPrev, pImageData, getImageDataSize());
        memcpy(pImageHeaderPrev, pImageHeader, sizeof(ImageHeader_t));
      }

      pImageHeader->seqLock++;
      __sync_synchronize();

      if (tempBuff.bytesused >= yuyvSize) {
        yuyv2y(yuyv, yuyvSize, pImageData, getImageDataSize());
      } else {
        memset(pImageData, 0, getImageDataSize());
      }

      const uint64 t_end = now_ns();
      pImageHeader->ts_ns = t_end;
      pImageHeader->width = width;
      pImageHeader->height = height;
      pImageHeader->stride = width;
      pImageHeader->size = (uint32)getImageDataSize();
      pImageHeader->duration = (double)(t_end - t_start) / 1e9;

      __sync_synchronize();
      pImageHeader->seqLock++;
      __sync_synchronize();

      movedCurrent = true;

      if (rpfd >= 0) {
        MsgFrameReady_t msg{};
        msg.type = IMREADY;
        msg.padding = 0;
        msg.seqLock = pImageHeader->seqLock;
        msg.ts_ns = pImageHeader->ts_ns;
        ssize_t wr = write(rpfd, &msg, sizeof(msg));
        (void)wr;
      }

      if (ioctl(vfd, VIDIOC_QBUF, &tempBuff) < 0) {
        perror("VIDIOC_QBUF");
        break;
      }
    }
  }

  if (vfd >= 0) {
    (void)ioctl(vfd, VIDIOC_STREAMOFF, &bufferType);
    for (uint8 i = 0; i < kV4L2BufferCount; ++i) {
      if (buffers[i].start)
        munmap(buffers[i].start, buffers[i].length);
    }
    close(vfd);
  }
  if (rpfd >= 0)
    close(rpfd);
  if (map_base != MAP_FAILED)
    munmap(map_base, map_len);
  if (memfd >= 0)
    close(memfd);
  return ok ? 0 : 1;
}
