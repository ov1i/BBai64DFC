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

#include "common/types/data_types.h"
#include "common/types/shared_types.h"

static volatile sig_atomic_t g_run = 1;
static void on_sigint(int){ g_run = 0; }

static inline uint64 now_ns(){
  struct timespec ts; clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return (uint64)ts.tv_sec*1000000000ull + (uint64)ts.tv_nsec;
}

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

static inline void yuyv2y(const uint8* yuyv, size_t yuyv_bytes, uint8* yCh, size_t y_bytes){
  size_t total = yuyv_bytes/2;
  if(total > y_bytes) total = y_bytes;
  for(size_t px=0,i=0; px<total; ++px, i+=2) yCh[px] = yuyv[i];
}

static inline void yuyv2yPadded(const uint8* yuyv, uint32 width, uint32 height, uint32 bytesperline, uint8* yCh, uint32 y_stride) {
  for(uint32 r = 0; r < height; ++r) {
    const uint8* src = yuyv + r * bytesperline;
    uint8* dst = yCh + r * y_stride;
    // copy Y from YUYV
    for(uint32 c = 0, i = 0; c < width; ++c, i += 2) {
      dst[c] = src[i];
    }
  }
}

int main(int argc, char** argv){
  const char* camDev = (argc>1)? argv[1] : "/dev/video0";
  const char* rpmsgName = "a72_to_c66x";
  for(int i=1;i<argc;i++) {
    if(strncmp(argv[i],"--rpmsg-name=",13)==0) {
      rpmsgName = argv[i]+13;
    }
  }
  signal(SIGINT, on_sigint);

  int memfd=-1, vfd=-1, rpfd=-1; 
  void* map_base=MAP_FAILED; 
  size_t map_len=0;
  DFC_t_ImageHeader* pImageHeader[2] = {nullptr,nullptr}; 
  uint8* img[2] = {nullptr,nullptr};
  DFC_t_V4L2Buf buffers[kV4L2BufferCount]{};
  v4l2_buf_type bufType = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  //  map two slots at baseAddr
  do {
    const off_t physBaseAddr = (off_t)baseAddr;
    const size_t reqSize = (size_t)(2*getImagePortSize());
    memfd = open("/dev/mem", O_RDWR|O_SYNC); 

    if(memfd<0) { 
      perror("open /dev/mem"); 
      break;
    }
    long pagesz = sysconf(_SC_PAGESIZE); 
    if(pagesz<=0) {
      pagesz=4096;
    }

    off_t page_base = physBaseAddr & ~(off_t)(pagesz-1);
    off_t page_off  = physBaseAddr - page_base;

    map_len = ((reqSize + page_off + (size_t)pagesz - 1) / (size_t)pagesz) * (size_t)pagesz;
    map_base = mmap(nullptr, map_len, PROT_READ|PROT_WRITE, MAP_SHARED, memfd, page_base);

    if(map_base == MAP_FAILED) { 
      perror("mmap /dev/mem"); 
      break; 
    }

    uint8* base = (uint8*)map_base + page_off;
    pImageHeader[0]=(DFC_t_ImageHeader*)(base+0);
    img[0]=(uint8*)pImageHeader[0]+sizeof(DFC_t_ImageHeader);
    pImageHeader[1]=(DFC_t_ImageHeader*)(base+getImagePortSize());
    img[1]=(uint8*)pImageHeader[1]+sizeof(DFC_t_ImageHeader);

    for(int s=0;s<2;++s) {
      memset(pImageHeader[s],0,sizeof(*pImageHeader[s]));

      pImageHeader[s]->width=width; 
      pImageHeader[s]->height=height; 
      pImageHeader[s]->stride=width;
      pImageHeader[s]->size=(uint32)getImageDataSize(); 
      pImageHeader[s]->seqLock=0;
      pImageHeader[s]->duration=0.0;
    }

    // RPMsg to C66x by name
    rpfd = rpmsg_open(rpmsgName);
    if(rpfd<0) {
      fprintf(stderr,"ISP: WARNING: no rpmsg endpoint named '%s'; running without doorbells.\n", rpmsgName);
    }

    // Open camera
    vfd = open(camDev, O_RDWR); 
    if(vfd<0) { 
      perror("open video"); 
      break; 
    }

    v4l2_format fmt{}; 
    fmt.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width=width; 
    fmt.fmt.pix.height=height;
    fmt.fmt.pix.pixelformat=V4L2_PIX_FMT_YUYV; 
    fmt.fmt.pix.field=V4L2_FIELD_NONE;

    if(ioctl(vfd, VIDIOC_S_FMT, &fmt)<0) { 
      perror("VIDIOC_S_FMT"); 
      break; 
    }

    uint32 cam_w = fmt.fmt.pix.width;
    uint32 cam_h = fmt.fmt.pix.height;
    uint32 yuyv_bpl = fmt.fmt.pix.bytesperline ? fmt.fmt.pix.bytesperline : cam_w * 2;
    bool tightly_packed = (yuyv_bpl == cam_w * 2);

    v4l2_streamparm parm{}; 
    parm.type=V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator=1;
    parm.parm.capture.timeperframe.denominator=fps;
    (void)ioctl(vfd, VIDIOC_S_PARM, &parm);

    v4l2_requestbuffers req{}; 
    req.count=kV4L2BufferCount; 
    req.type=fmt.type; 
    req.memory=V4L2_MEMORY_MMAP;

    if(ioctl(vfd, VIDIOC_REQBUFS, &req)<0) { 
      perror("VIDIOC_REQBUFS");
      break; 
    }

    if(req.count<kV4L2BufferCount) { 
      fprintf(stderr,"Not enough V4L2 bufs\n"); 
      break; 
    }

    for(uint8 i=0;i<kV4L2BufferCount;++i) {
      v4l2_buffer b{}; 
      b.type=req.type; 
      b.memory=req.memory; 
      b.index=i;

      if(ioctl(vfd, VIDIOC_QUERYBUF, &b)<0) { 
        perror("VIDIOC_QUERYBUF"); 
        break; 
      }

      buffers[i].length=b.length;
      buffers[i].start=mmap(nullptr,b.length,PROT_READ|PROT_WRITE,MAP_SHARED,vfd,b.m.offset);
      if(buffers[i].start==MAP_FAILED) { 
        perror("mmap v4l2"); 
        buffers[i].start=nullptr; 
        break; 
      }
    }
    bool ok=true; 
    for(uint8 i=0;i<kV4L2BufferCount;++i) {
      if(!buffers[i].start) {
        ok=false;
      }
    }
    if(!ok) break;

    for(uint8 i=0;i<kV4L2BufferCount;++i) {
      v4l2_buffer b{}; 
      b.type=req.type; 
      b.memory=req.memory; 
      b.index=i;
      if(ioctl(vfd, VIDIOC_QBUF, &b)<0) { 
        perror("VIDIOC_QBUF"); 
        ok=false; 
        break; 
      }
    }
    if(!ok) break;

    if(ioctl(vfd, VIDIOC_STREAMON, &bufType)<0) { 
      perror("VIDIOC_STREAMON"); 
      break; 
    }

    // Run a marathon
    printf("ISP: %ux%u YUYV→GRAY8 → DDR @ 0x%08llX [slot0|slot1], RPMsg to '%s'\n", (unsigned)width,(unsigned)height,(unsigned long long)baseAddr,rpmsgName);

    const size_t yuyvSize = (size_t)width*(size_t)height*2;
    uint8 slot = SLOT_CURR;

    while(g_run) {
      pollfd pfd{};
      pfd.fd=vfd;
      pfd.events=POLLIN;

      int pr=poll(&pfd,1,1000);
      if(pr<0) { 
        if(errno==EINTR) continue; 
        perror("poll"); 
        break; 
      }

      if(pr==0) { 
        fprintf(stderr,"poll timeout\n"); 
        continue; 
      }

      v4l2_buffer b{}; 
      b.type=bufType; 
      b.memory=V4L2_MEMORY_MMAP;

      if(ioctl(vfd, VIDIOC_DQBUF, &b)<0) {
        if(errno==EINTR||errno==EAGAIN) continue;
        perror("VIDIOC_DQBUF"); 
        break;
      }

      const uint64 t0 = now_ns();
      pImageHeader[slot]->seqLock++; 
      __sync_synchronize();

      if (b.bytesused >= (size_t)yuyv_bpl * cam_h) {
        if (tightly_packed) {
          yuyv2y((const uint8*)buffers[b.index].start, (size_t)cam_w * cam_h * 2, img[slot], getImageDataSize());
        } else {
          // row-wise for padded lines
          yuyv2yPadded((const uint8*)buffers[b.index].start, cam_w, cam_h, yuyv_bpl, img[slot], pImageHeader[slot]->stride /* = cam_w */);
        }
      } else {
        memset(img[slot], 0, getImageDataSize());
      }


      const uint64 t1=now_ns();
      pImageHeader[slot]->ts_ns=t1; 
      pImageHeader[slot]->duration=(double)(t1-t0)/1e9;

      __sync_synchronize(); 
      pImageHeader[slot]->seqLock++; 
      __sync_synchronize();

      if(rpfd>=0) {
        DFC_t_MsgFrameReady m{}; 
        m.type=IMREADY;
        m.slot=slot; 
        m.seqLock=pImageHeader[slot]->seqLock; 
        m.ts_ns=pImageHeader[slot]->ts_ns;
        (void)write(rpfd,&m,sizeof(m));
      }

      if(ioctl(vfd, VIDIOC_QBUF, &b)<0) { 
        perror("VIDIOC_QBUF"); 
        break; 
      }

      slot ^= 1;
    }
  } while(0);

  if(vfd>=0) { 
    (void)ioctl(vfd, VIDIOC_STREAMOFF, &bufType); 

    for(uint8 i=0;i<kV4L2BufferCount;++i) {
      if(buffers[i].start) {
        munmap(buffers[i].start,buffers[i].length);
      } 
    }
    
    close(vfd);
  }

  if(rpfd>=0) {
    close(rpfd);
  }

  if(map_base!=MAP_FAILED) {
    munmap(map_base,map_len);
  }
  
  if(memfd>=0) {
    close(memfd);
  }

  return 0;
}
