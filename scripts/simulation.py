import socket, struct, threading, collections, time, math
import numpy as np
import cv2

UDP_IMG_PORT = 5600
UDP_TLM_PORT = 5005
CHUNK_BYTES  = 1200          # image chunk payload size on the wire

# Image packet header
# magic,u16 ver,u16 w,u16 h,u16 stride,u32 frame_id,u64 ts_ns,u16 chunks,u16 idx,u16 payload,u16 slot
IMGF_MAGIC = 0x31474D49
IMGF_HDR   = '<I H H H H I Q H H H H'

TLMD_MAGIC = 0x54454C45

TLMD_FMT = (
    '<'                  # little-endian
    'I H I'              # magic, size, seq
    # MPU9250 (IMU + Mag + adjustment)
    'f f f f f f'        # ax ay az gx gy gz
    'f f f'              # mx my mz
    'f f f'              # mag_adjustment[3]
    'B 7x'               # mag_rdy + pad
    'f'                  # imu_temp
    # BMP280
    'f f f'              # baro_p_hPa, baro_alt_m, baro_temp_C
    # EKF nominal state
    'f f f'              # pN pE pD
    'f f f'              # vN vE vD
    'f f f f'            # qw qx qy qz
    'f f f'              # bgx bgy bgz
    'f f f'              # bax bay baz
    # P diagonal (15)
    'f f f f f f f f f f f f f f f'  # Pdiag[15]
    # RC inputs
    'f f f f'            # thr roll pitch yaw
    'B B 6x'             # arm, mode
    # Optical flow
    'f f f'              # of_u, of_v, of_quality
    'B 7x'               # of_valid
    # Motors
    'H H H H 4x'         # m0..m3
    # PID setpoints (subset, rest left out)
    'f f f'              # pos_sp_N, pos_sp_E, pos_sp_D
    'f f f'              # vel_sp_N, vel_sp_E, vel_sp_D
    'f'                  # yaw_sp
    'B 7x'               # pos_sp_valid
)

TLMD_SIZE = struct.calcsize(TLMD_FMT)

# Index map into the struct.unpack() tuple
FIELD = {
    'magic':0, 'ver':1, 'size':2, 'seq':3,
    # IMU/Mag/Adjust
    'ax':4, 'ay':5, 'az':6, 'gx':7, 'gy':8, 'gz':9,
    'mx':10, 'my':11, 'mz':12, 'mag_adj0':13, 'mag_adj1':14, 'mag_adj2':15,
    'mag_rdy':16, 'imu_temp':17,
    # BMP280
    'baro_p':18, 'baro_alt':19, 'baro_temp':20,
    # EKF state
    'pN':21,'pE':22,'pD':23,'vN':24,'vE':25,'vD':26,
    'qw':27,'qx':28,'qy':29,'qz':30,
    'bgx':31,'bgy':32,'bgz':33,'bax':34,'bay':35,'baz':36,
    # Pdiag 37..52
    # RC
    'rc_thr':53,'rc_roll':54,'rc_pitch':55,'rc_yaw':56,'rc_arm':57,'rc_mode':58,
    # Optical flow
    'of_u':59,'of_v':60,'of_quality':61,'of_valid':62,
    # Motors
    'm0':63,'m1':64,'m2':65,'m3':66,
    # PID
    'sp_pN':67,'sp_pE':68,'sp_pD':69,
    'sp_vN':70,'sp_vE':71,'sp_vD':72,
    'sp_yaw':73,'sp_pos_valid':74,
}
Pdiag_slice = slice(37, 52)


class FrameAssembler:
    """
    Assembles chunked grayscale frames for multiple slots (cameras).
    All slots share the same UDP port, each packet carries 'slot' in the header.
    """
    def __init__(self, chunk_bytes=CHUNK_BYTES, max_slots=4):
        self.chunk_bytes = chunk_bytes
        self.cur  = {}    # (slot, frame_id) -> dict
        self.last = {}    # slot -> (np.uint8 HxW gray, ts_ns)
        self.lock = threading.Lock()

    def push_chunk(self, data: bytes):
        if len(data) < struct.calcsize(IMGF_HDR): return
        hdr = struct.unpack(IMGF_HDR, data[:struct.calcsize(IMGF_HDR)])
        magic, ver, w, h, stride, frame_id, ts_ns, chunks, idx, payload, slot = hdr
        if magic != IMGF_MAGIC or ver != 1: return
        if idx >= chunks: return
        payload_bytes = data[struct.calcsize(IMGF_HDR):]
        if len(payload_bytes) != payload: return

        key = (slot, frame_id)
        with self.lock:
            a = self.cur.get(key)
            if a is None:
                a = {'w':w, 'h':h, 'ts':ts_ns, 'chunks':chunks, 'got':[False]*chunks, 'buf':bytearray(w*h)}
                self.cur[key] = a
            off = idx * self.chunk_bytes
            a['buf'][off:off+payload] = payload_bytes
            a['got'][idx] = True
            if all(a['got']):
                img = np.frombuffer(a['buf'], dtype=np.uint8).reshape(a['h'], a['w'])
                self.last[key[0]] = (img.copy(), a['ts'])
                del self.cur[key]

    def latest_pair(self, slots=(0,1)):
        with self.lock:
            a = self.last.get(slots[0], (None,0))
            b = self.last.get(slots[1], (None,0))
            return a, b

class TelemetryState:
    def __init__(self):
        self.lock = threading.Lock()
        self.t = None
    def update(self, tup):
        with self.lock: self.t = tup
    def get(self): 
        with self.lock: return self.t

def recv_images(asm: FrameAssembler, stop_evt: threading.Event):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', UDP_IMG_PORT))
    sock.settimeout(0.5)
    while not stop_evt.is_set():
        try:
            data, _ = sock.recvfrom(1500)
            asm.push_chunk(data)
        except socket.timeout:
            continue
        except Exception:
            # swallow noisy errors could be an addition
            pass

def recv_tlm(state: TelemetryState, stop_evt: threading.Event):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', UDP_TLM_PORT))
    sock.settimeout(0.5)
    while not stop_evt.is_set():
        try:
            data, _ = sock.recvfrom(4096)
            if len(data) < TLMD_SIZE:
                continue
            tup = struct.unpack(TLMD_FMT, data[:TLMD_SIZE])
            if tup[FIELD['magic']] != TLMD_MAGIC:
                continue
            state.update(tup)
        except socket.timeout:
            continue
        except Exception:
            pass

def label(img, text, x, y, scale=0.5, color=(220,220,220)):
    cv2.putText(img, text, (x,y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1)

def draw_bar(img, x,y,w,h, val, vmin, vmax, title):
    v = (val - vmin)/(vmax-vmin) if vmax!=vmin else 0.0
    v = 0.0 if v<0 else 1.0 if v>1 else v
    cv2.rectangle(img, (x,y), (x+w,y+h), (160,160,160), 1)
    cv2.rectangle(img, (x, y+h-int(v*h)), (x+w, y+h), (80,220,80), -1)
    txt = f'{val:.2f}'
    (tw,th),_=cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX,0.35,1)
    cv2.putText(img, txt, (x+(w-tw)//2, y+h-4), cv2.FONT_HERSHEY_SIMPLEX,0.35,(0,0,0),1)
    (tw,th),_=cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX,0.42,1)
    cv2.putText(img, title, (x+(w-tw)//2, y+h+th+10), cv2.FONT_HERSHEY_SIMPLEX,0.42,(200,200,200),1)

def spark(img, x,y,w,h, samples, vmin, vmax, title=None):
    cv2.rectangle(img, (x,y), (x+w,y+h), (70,70,70), 1)
    if title: cv2.putText(img, title, (x+2,y-4), cv2.FONT_HERSHEY_SIMPLEX,0.38,(180,180,180),1)
    if not samples or len(samples)<2: return
    pts=[]; N=min(w, len(samples)); base=samples[-N:]; rng=(vmax-vmin) or 1.0
    for i,v in enumerate(base):
        t=(v-vmin)/rng; t=0.0 if t<0 else 1.0 if t>1 else t
        pts.append((x+i, y + h - int(t*h)))
    for i in range(1,len(pts)):
        cv2.line(img, pts[i-1], pts[i], (200,200,200), 1)

def draw_drone_3dish(img, cx, cy, yaw, pitch, roll, arm_len=68):
    cv2.ellipse(img, (cx+8, cy+12), (arm_len, int(arm_len*0.35)), 0, 0, 360, (25,25,25), -1)
    c, s = math.cos(yaw), math.sin(yaw)
    t_pitch = 1.0 - 0.25*abs(pitch)
    t_roll  = 1.0 - 0.25*abs(roll)
    w1 = int(9 * t_pitch) + 1
    w2 = int(9 * t_roll ) + 1
    x1a, y1a = int(cx - arm_len*s), int(cy - arm_len*c)
    x1b, y1b = int(cx + arm_len*s), int(cy + arm_len*c)
    x2a, y2a = int(cx - arm_len*c), int(cy + arm_len*s)
    x2b, y2b = int(cx + arm_len*c), int(cy - arm_len*s)
    cv2.line(img, (x1a,y1a), (x1b,y1b), (60,160,255), w1+3)
    cv2.line(img, (x1a,y1a), (x1b,y1b), (130,230,255), w1)
    cv2.line(img, (x2a,y2a), (x2b,y2b), (60,160,255), w2+3)
    cv2.line(img, (x2a,y2a), (x2b,y2b), (130,230,255), w2)
    for (px,py) in [(x1a,y1a),(x1b,y1b),(x2a,y2a),(x2b,y2b)]:
        cv2.circle(img, (px,py), 13, (40,40,40), -1)
        cv2.circle(img, (px,py), 13, (100,100,100), 1)
        cv2.line(img, (px-12,py), (px+12,py), (180,255,255), 2)
        cv2.line(img, (px,py-12), (px,py+12), (180,255,255), 2)
    cv2.circle(img, (cx,cy), 12, (240,240,0), -1)
    cv2.circle(img, (cx,cy), 12, (0,0,0), 1)
    nose = (int(cx + 18*s), int(cy + 18*c))
    cv2.line(img, (cx,cy), nose, (0,255,0), 3)

def quat_to_euler_wxyz(qw,qx,qy,qz):
    # roll (x), pitch (y), yaw (z)
    sinr_cosp = 2*(qw*qx + qy*qz)
    cosr_cosp = 1 - 2*(qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2*(qw*qy - qz*qx)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp)>=1 else math.asin(sinp)
    siny_cosp = 2*(qw*qz + qx*qy)
    cosy_cosp = 1 - 2*(qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def to_rgb_small(gray, w, h):
    if gray is None: return np.zeros((h,w,3), dtype=np.uint8)
    return cv2.resize(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), (w,h), interpolation=cv2.INTER_AREA)


def main():
    asm = FrameAssembler()
    tlm = TelemetryState()
    stop_evt = threading.Event()

    threading.Thread(target=recv_images, args=(asm,stop_evt), daemon=True).start()
    threading.Thread(target=recv_tlm,    args=(tlm,stop_evt), daemon=True).start()

    # histories (for graphs)
    H = {k: collections.deque(maxlen=300) for k in
         ['pN','pE','pD','vN','vE','vD','roll','pitch','yaw',
          'ax','ay','az','gx','gy','gz','mx','my','mz',
          'baro_p','baro_alt','baro_temp']}

    # layout
    IMG_W, IMG_H = 320, 240
    GAP = 6
    LEFT_W = IMG_W*2 + GAP
    HUD_W, HUD_H = 840, 700
    GUTTER = 8

    while True:
        (img0,_), (img1,_) = asm.latest_pair((0,1))
        left0 = to_rgb_small(img0, IMG_W, IMG_H)
        left1 = to_rgb_small(img1, IMG_W, IMG_H)

        hud = np.zeros((HUD_H, HUD_W, 3), dtype=np.uint8)
        t = tlm.get()

        of_valid = False
        if t:
            # pull fields
            pN=t[FIELD['pN']]; pE=t[FIELD['pE']]; pD=t[FIELD['pD']]
            vN=t[FIELD['vN']]; vE=t[FIELD['vE']]; vD=t[FIELD['vD']]
            qw=t[FIELD['qw']]; qx=t[FIELD['qx']]; qy=t[FIELD['qy']]; qz=t[FIELD['qz']]
            ax=t[FIELD['ax']]; ay=t[FIELD['ay']]; az=t[FIELD['az']]
            gx=t[FIELD['gx']]; gy=t[FIELD['gy']]; gz=t[FIELD['gz']]
            mx=t[FIELD['mx']]; my=t[FIELD['my']]; mz=t[FIELD['mz']]
            baro_p=t[FIELD['baro_p']]; baro_alt=t[FIELD['baro_alt']]; baro_temp=t[FIELD['baro_temp']]
            thr=t[FIELD['rc_thr']]; rc_roll=t[FIELD['rc_roll']]; rc_pitch=t[FIELD['rc_pitch']]; rc_yaw=t[FIELD['rc_yaw']]
            arm=t[FIELD['rc_arm']]; mode=t[FIELD['rc_mode']]
            of_u=t[FIELD['of_u']]; of_v=t[FIELD['of_v']]; of_q=t[FIELD['of_quality']]; of_valid=bool(t[FIELD['of_valid']])
            m0=t[FIELD['m0']]; m1=t[FIELD['m1']]; m2=t[FIELD['m2']]; m3=t[FIELD['m3']]
            spN=t[FIELD['sp_pN']]; spE=t[FIELD['sp_pE']]; spD=t[FIELD['sp_pD']]
            spYaw=t[FIELD['sp_yaw']]; spOK=bool(t[FIELD['sp_pos_valid']])

            r,p,yaw = quat_to_euler_wxyz(qw,qx,qy,qz)

            # titles
            label(hud, f'Pos NED: [{pN:+.2f},{pE:+.2f},{pD:+.2f}] m', 10, 24, 0.6, (255,255,255))
            label(hud, f'Vel NED: [{vN:+.2f},{vE:+.2f},{vD:+.2f}] m/s', 10, 46, 0.6, (200,200,200))
            label(hud, f'Setpoint: [{spN:+.1f},{spE:+.1f},{spD:+.1f}]  Yaw_sp:{spYaw:+.2f}  valid:{int(spOK)}', 10, 68, 0.5, (180,220,255))

            # drone + attitude text
            drone_cx, drone_cy = int(HUD_W*0.72), 120
            draw_drone_3dish(hud, drone_cx, drone_cy, yaw, p, r, arm_len=72)
            label(hud, f'RPY [deg]: {math.degrees(r):+5.1f} {math.degrees(p):+5.1f} {math.degrees(yaw):+5.1f}', drone_cx-130, drone_cy+60, 0.5, (200,200,200))

            # Alt bar (up-positive = -pD)
            draw_bar(hud, HUD_W-48, 40, 28, 270, -pD, -10, +10, 'Alt')

            # histories
            for k,val in [('pN',pN),('pE',pE),('pD',pD),('vN',vN),('vE',vE),('vD',vD),
                          ('ax',ax),('ay',ay),('az',az),('gx',gx),('gy',gy),('gz',gz),
                          ('mx',mx),('my',my),('mz',mz),('baro_p',baro_p),('baro_alt',baro_alt),('baro_temp',baro_temp)]:
                H[k].append(val)
            H['roll'].append(r); H['pitch'].append(p); H['yaw'].append(yaw)

            # Graphs
            gx0, gy0 = 10, 92
            gw, gh, gapx, gapy = 200, 52, 18, 12
            # EKF p/v/rpy
            spark(hud, gx0, gy0 + 0*(gh+gapy), gw, gh, H['pN'], -10, 10, 'EKF pN (m)')
            spark(hud, gx0, gy0 + 1*(gh+gapy), gw, gh, H['pE'], -10, 10, 'EKF pE (m)')
            spark(hud, gx0, gy0 + 2*(gh+gapy), gw, gh, H['pD'], -10, 10, 'EKF pD (m)')
            spark(hud, gx0, gy0 + 3*(gh+gapy), gw, gh, H['vN'], -3, 3, 'EKF vN (m/s)')
            spark(hud, gx0, gy0 + 4*(gh+gapy), gw, gh, H['vE'], -3, 3, 'EKF vE (m/s)')
            spark(hud, gx0, gy0 + 5*(gh+gapy), gw, gh, H['vD'], -3, 3, 'EKF vD (m/s)')
            spark(hud, gx0, gy0 + 6*(gh+gapy), gw, gh, H['roll'],  -1.5, 1.5, 'EKF roll (rad)')
            spark(hud, gx0, gy0 + 7*(gh+gapy), gw, gh, H['pitch'], -1.5, 1.5, 'EKF pitch (rad)')
            spark(hud, gx0, gy0 + 8*(gh+gapy), gw, gh, H['yaw'],   -3.2, 3.2, 'EKF yaw (rad)')

            # IMU block
            x1 = gx0 + gw + gapx
            spark(hud, x1, gy0 + 0*(gh+gapy), gw, gh, H['ax'], -0.5, 0.5, 'IMU Acc X')
            spark(hud, x1, gy0 + 1*(gh+gapy), gw, gh, H['ay'], -0.5, 0.5, 'IMU Acc Y')
            spark(hud, x1, gy0 + 2*(gh+gapy), gw, gh, H['az'],  9.5, 10.2,'IMU Acc Z')
            spark(hud, x1, gy0 + 3*(gh+gapy), gw, gh, H['gx'], -0.2, 0.2, 'IMU Gyro X')
            spark(hud, x1, gy0 + 4*(gh+gapy), gw, gh, H['gy'], -0.2, 0.2, 'IMU Gyro Y')
            spark(hud, x1, gy0 + 5*(gh+gapy), gw, gh, H['gz'], -0.2, 0.2, 'IMU Gyro Z')
            spark(hud, x1, gy0 + 6*(gh+gapy), gw, gh, H['mx'], -1.0, 1.0, 'IMU Mag X')
            spark(hud, x1, gy0 + 7*(gh+gapy), gw, gh, H['my'], -1.0, 1.0, 'IMU Mag Y')
            spark(hud, x1, gy0 + 8*(gh+gapy), gw, gh, H['mz'], -1.0, 1.0, 'IMU Mag Z')

            # RC + motors, bottom-right
            bx = HUD_W - 360; yb = HUD_H - 150
            draw_bar(hud, bx,       yb, 26, 110, thr, 0, 1, 'THR')
            draw_bar(hud, bx+36,    yb, 26, 110, (rc_roll +1)/2, 0, 1, 'ROLL')
            draw_bar(hud, bx+72,    yb, 26, 110, (rc_pitch+1)/2, 0, 1, 'PITCH')
            draw_bar(hud, bx+108,   yb, 26, 110, (rc_yaw  +1)/2, 0, 1, 'YAW')
            for i,(pwm,x0m) in enumerate(zip([m0,m1,m2,m3],[bx+200,bx+236,bx+272,bx+308])):
                draw_bar(hud, x0m, yb, 26, 110, (pwm-1000)/1000.0, 0, 1, f'M{i}')
            label(hud, f'ARM:{int(arm)} MODE:{int(mode)}', bx, HUD_H-12, 0.6, (0,255,255))

        # compose main canvas
        H = max(IMG_H, HUD_H)
        W = LEFT_W + GUTTER + HUD_W
        canvas = np.zeros((H, W, 3), dtype=np.uint8)
        canvas[:IMG_H, 0:IMG_W, :] = left0
        canvas[:IMG_H, IMG_W+GAP:IMG_W+GAP+IMG_W, :] = left1
        canvas[:HUD_H, LEFT_W + GUTTER : LEFT_W + GUTTER + HUD_W, :] = hud

        # optical flow arrow
        if t and of_valid:
            start = (LEFT_W//2, IMG_H//2)
            scale = 1.0
            end = (int(start[0] + t[FIELD['of_u']]*scale),
                   int(start[1] + t[FIELD['of_v']]*scale))
            end = (max(0, min(W-1, end[0])), max(0, min(H-1, end[1])))
            cv2.arrowedLine(canvas, start, end, (0,0,0), 8, tipLength=0.15)
            cv2.arrowedLine(canvas, start, end, (0,255,0), 6, tipLength=0.15)
            cv2.circle(canvas, start, 7, (0,0,0), -1)
            cv2.circle(canvas, start, 6, (0,255,0), -1)
            label(canvas, f'OF u,v={t[FIELD["of_u"]]:+.1f},{t[FIELD["of_v"]]:+.1f}  q={t[FIELD["of_quality"]]:.2f}', 8, 20, 0.5, (0,255,0))

        cv2.imshow("Drone GS", canvas)
        k = cv2.waitKey(1)
        if k == 27 or k == ord('q'):
            stop_evt.set()
            break

if __name__ == '__main__':
    main()
