import cv2, socket, struct, time, math, threading, collections
import numpy as np

IMGF_MAGIC = 0x46474D49
IMGF_HDR   = '<I H H H H I Q H H H H'  # image header

TLMD_MAGIC = 0x444D4C54
# Building Telemetry PACKET see shared_types.h for the layout (little endian used)
TLMD_FMT = (
 '<'        # little-endian
 'I H H I'  # magic, version, size, seq
 'd d d d d d d d d d d d'   # ax..gz (6) + mx..mz (3) + mag_adj[0..2] (3) = 12 doubles
 'B 7x d Q Q'                # mag_rdy + pad, imu_temp, imu_ts_ns, mag_ts_ns
 'd d d Q'                   # baro: p, alt, temp, ts
 'd d d d d d d d d d d d d d d d d d d d d d'  # p[3], v[3], q[4], bg[3], ba[3] = 16 doubles +  (actually 3+3+4+3+3=16)
 'd d d d d d d d d d d d d d d'               # Pdiag[15]
 'Q'                        # ekf_ts_ns
 'd d d d B B 6x'           # RC: thr,roll,pitch,yaw, arm,mode
 'd d d B 7x'               # OF: u, v, quality, valid
 'H H H H 4x'               # motor pwm us[4], pad
)

UDP_IMG_PORT = 5600
UDP_TLM_PORT = 5005

class FrameAssembler:
    def __init__(self):
        self.cur = None
        self.lock = threading.Lock()
        self.last_frame = (None, 0)  # (gray np.ndarray, ts_ns)

    def push_chunk(self, data):
        hdr = struct.unpack(IMGF_HDR, data[:struct.calcsize(IMGF_HDR)])
        magic, ver, w, h, stride, frame_id, ts_ns, chunks, idx, payload, slot = hdr
        if magic != IMGF_MAGIC or ver != 1: return
        payload_bytes = data[struct.calcsize(IMGF_HDR):]
        if len(payload_bytes) != payload: return

        key = (frame_id, w, h, ts_ns, chunks)
        with self.lock:
            if self.cur is None or self.cur['key'] != key:
                self.cur = {
                    'key': key,
                    'buf': bytearray(w*h),
                    'got': [False]*chunks,
                    'w': w, 'h': h, 'ts': ts_ns
                }
            if idx >= chunks: return
            off = idx * 1200
            self.cur['buf'][off:off+payload] = payload_bytes
            self.cur['got'][idx] = True

            if all(self.cur['got']):
                # complete frame
                arr = np.frombuffer(self.cur['buf'], dtype=np.uint8).reshape(self.cur['h'], self.cur['w'])
                self.last_frame = (arr.copy(), self.cur['ts'])
                self.cur = None

    def latest(self):
        with self.lock:
            return self.last_frame

class TelemetryState:
    def __init__(self):
        self.lock = threading.Lock()
        self.last = None

    def update(self, pkt):
        with self.lock:
            self.last = pkt

    def get(self):
        with self.lock:
            return self.last

def recv_images(assembler):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', UDP_IMG_PORT))
    while True:
        data, _ = sock.recvfrom(1400)
        assembler.push_chunk(data)

def recv_tlm(state):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', UDP_TLM_PORT))
    tlm_size = struct.calcsize(TLMD_FMT)
    while True:
        data, _ = sock.recvfrom(2048)
        if len(data) < tlm_size: continue
        fields = struct.unpack(TLMD_FMT, data[:tlm_size])
        if fields[0] != TLMD_MAGIC: continue
        state.update(fields)

def quat_to_euler_wxyz(qw, qx, qy, qz):
    # Yaw-pitch-roll (Z-Y-X) from body->NED
    # yaw
    siny_cosp = 2*(qw*qz + qx*qy)
    cosy_cosp = 1 - 2*(qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    # pitch
    sinp = 2*(qw*qy - qz*qx)
    if abs(sinp) >= 1: pitch = math.copysign(math.pi/2, sinp)
    else: pitch = math.asin(sinp)
    # roll
    sinr_cosp = 2*(qw*qx + qy*qz)
    cosr_cosp = 1 - 2*(qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    return roll, pitch, yaw

def draw_bar(img, x,y,w,h, val, vmin, vmax, label):
    v = (val - vmin)/(vmax-vmin); v = max(0,min(1,v))
    cv2.rectangle(img, (x,y), (x+w,y+h), (200,200,200), 1)
    cv2.rectangle(img, (x,y+h-int(v*h)), (x+w,y+h), (80,220,80), -1)
    cv2.putText(img, f'{label}:{val:.2f}', (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(255,255,255),1)

def spark(img, x,y,w,h, samples, vmin, vmax, color=(50,200,250)):
    if len(samples) < 2: return
    cv2.rectangle(img, (x,y), (x+w,y+h), (60,60,60), 1)
    pts=[]
    n=len(samples); 
    for i,v in enumerate(samples[-w:]):
        t = (v - vmin)/(vmax-vmin); t = max(0,min(1,t))
        px = x+i; py = y+h-int(t*h)
        pts.append((px,py))
    for i in range(1,len(pts)):
        cv2.line(img, pts[i-1], pts[i], color, 1)

def main():
    assembler = FrameAssembler()
    tlm = TelemetryState()

    threading.Thread(target=recv_images, args=(assembler,), daemon=True).start()
    threading.Thread(target=recv_tlm,    args=(tlm,),       daemon=True).start()

    alt_hist = collections.deque(maxlen=300)
    vz_hist  = collections.deque(maxlen=300)

    while True:
        img, ts = assembler.latest()
        hud = np.zeros((480, 360, 3), dtype=np.uint8)

        t = tlm.get()
        if t:
            # unpack selected fields by index
            # indices: [magic,ver,size,seq, ax..gz(6)=4..9, mx..mz=10..12, mag_adj=13..15, mag_rdy=16, imu_temp=18, imu_ts=19, mag_ts=20, baro=21..23, baro_ts=24, p=25..27, v=28..30, q=31..34, bg=35..37, ba=38..40, Pdiag=41..55, ekf_ts=56, rc=57..62, of=63..66, motors=67..70]
            seq = t[3]
            ax,ay,az,gx,gy,gz = t[4:10]
            mx,my,mz = t[10:13]
            baro_alt = t[22]
            pN,pE,pD  = t[25:28]
            vN,vE,vD  = t[28:31]
            qw,qx,qy,qz = t[31:35]
            thr, roll, pitch, yaw_in = t[57:61]
            arm, mode = t[61:63]
            of_u, of_v, of_q, of_valid = t[63:67]
            m0,m1,m2,m3 = t[67:71]

            alt_hist.append(baro_alt)
            vz_hist.append(vD)

            # Drone “icon”: translate by N/E (scaled), rotate by yaw from quat
            r,p,yaw = quat_to_euler_wxyz(qw,qx,qy,qz)
            cx,cy = 180, 240
            scale = 0.3  # px per meter for drawing (tune)
            x = int(cx + pE*scale)
            y = int(cy + -pN*scale)
            L=30
            c = math.cos(yaw); s = math.sin(yaw)
            arm1 = ((int(x - L*s), int(y - L*c)), (int(x + L*s), int(y + L*c)))
            arm2 = ((int(x - L*c), int(y + L*s)), (int(x + L*c), int(y - L*s)))
            cv2.line(hud, arm1[0], arm1[1], (200,200,0), 3)
            cv2.line(hud, arm2[0], arm2[1], (200,200,0), 3)
            cv2.circle(hud, (x,y), 6, (0,255,255), -1)
            cv2.putText(hud, f'Pos NED: [{pN:+.1f},{pE:+.1f},{pD:+.1f}] m', (10,20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,(255,255,255),1)
            cv2.putText(hud, f'Vel NED: [{vN:+.1f},{vE:+.1f},{vD:+.1f}] m/s', (10,40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,(200,200,200),1)

            # Altitude + graph
            draw_bar(hud, 320, 40, 20, 180, -pD, -5, +5, 'Alt')
            spark(hud, 10, 60, 160, 40, list(alt_hist), -5, +5, (0,180,255))
            cv2.putText(hud, 'alt spark', (10,55), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(180,180,180),1)
            spark(hud, 10, 120, 160, 40, list(vz_hist), -3, +3, (0,255,150))
            cv2.putText(hud, 'Vz spark', (10,115), cv2.FONT_HERSHEY_SIMPLEX, 0.4,(180,180,180),1)

            # RC & Motors
            draw_bar(hud, 10, 200, 20, 120, thr, 0, 1, 'THR')
            draw_bar(hud, 40, 200, 20, 120, (roll+1)/2, 0, 1, 'ROLL')
            draw_bar(hud, 70, 200, 20, 120, (pitch+1)/2,0, 1, 'PITCH')
            draw_bar(hud, 100,200, 20, 120, (yaw_in+1)/2,0, 1, 'YAW')
            cv2.putText(hud, f'ARM:{int(arm)} MODE:{int(mode)}', (10,340), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,255),1)

            for i,(pwm,x0) in enumerate(zip([m0,m1,m2,m3],[140,170,200,230])):
                v = (pwm-1000)/1000.0
                draw_bar(hud, x0, 200, 20, 120, v, 0, 1, f'M{i}')

        # Compose big dashboard
        if img is None:
            canvas = np.zeros((480, 640+360, 3), dtype=np.uint8)
            canvas[:,640:,:] = hud
        else:
            rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            # Optical-flow arrow (global)
            if t and of_valid:
                scale=0.02  # tune
                center=(rgb.shape[1]//2, rgb.shape[0]//2)
                tip=(int(center[0]+of_u*scale), int(center[1]+of_v*scale))
                cv2.arrowedLine(rgb, center, tip, (0,255,0), 2, tipLength=0.2)
            canvas = np.zeros((max(480, rgb.shape[0]), 640+360, 3), dtype=np.uint8)
            canvas[:rgb.shape[0], :rgb.shape[1], :] = rgb
            canvas[:hud.shape[0], 640:640+hud.shape[1], :] = hud

        cv2.imshow("Drone GS", canvas)
        if cv2.waitKey(1) == 27: break

if __name__ == '__main__':
    main()
