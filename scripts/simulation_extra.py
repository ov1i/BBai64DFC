import socket, struct, threading, collections, math, sys, time
import numpy as np
import time
import cv2

UDP_IMG_PORT = 5600
UDP_TLM_PORT = 5005
CHUNK_BYTES  = 1200  # sender must match

# Image chunk header
IMGF_MAGIC = 0x31474D49
IMGF_HDR   = '<I H H H H I Q H H H H'
TLMD_MAGIC = 0x54454C45

# Telemetry struct 
TLMD_FMT = (
    '<'
    'I H I'               # magic, size, seq
    'f f f f f f'         # ax ay az gx gy gz
    'f f f'               # mx my mz
    'f f f'               # mag_adjustment[3]
    'B 7x'                # mag_rdy + pad[7]
    'f'                   # imu_temp
    'f f f'               # baro_p_hPa, baro_alt_m, baro_temp_C
    'f f f'               # pN pE pD
    'f f f'               # vN vE vD
    'f f f f'             # qw qx qy qz
    'f f f'               # bgx bgy bgz
    'f f f'               # bax bay baz
    'f f f f f f f f f f f f f f f'  # Pdiag[15]
    'f f f f'             # thr roll pitch yaw
    'B B 6x'              # arm mode + pad[6]
    'f f f'               # of_u of_v of_quality
    'B 7x'                # of_valid + pad[7]
    'H H H H 4x'          # m0 m1 m2 m3 + pad[4]
    'f f f'               # pos_sp_N pos_sp_E pos_sp_D
    'f f f'               # vel_sp_N vel_sp_E vel_sp_D
    'f'                   # yaw_sp
    'B 7x'                # pos_sp_valid + pad[7]
)
TLMD_SIZE = struct.calcsize(TLMD_FMT)

FIELD = {
    'magic':0, 'size':1, 'seq':2,
    'ax':3,'ay':4,'az':5,'gx':6,'gy':7,'gz':8,
    'mx':9,'my':10,'mz':11,
    'mag_adj0':12,'mag_adj1':13,'mag_adj2':14,
    'mag_rdy':15,'imu_temp':16,
    'baro_p':17,'baro_alt':18,'baro_temp':19,
    'pN':20,'pE':21,'pD':22,'vN':23,'vE':24,'vD':25,
    'qw':26,'qx':27,'qy':28,'qz':29,
    'bgx':30,'bgy':31,'bgz':32,'bax':33,'bay':34,'baz':35,
    'rc_thr':51,'rc_roll':52,'rc_pitch':53,'rc_yaw':54,
    'rc_arm':55,'rc_mode':56,
    'of_u':57,'of_v':58,'of_quality':59,'of_valid':60,
    'm0':61,'m1':62,'m2':63,'m3':64,
    'sp_pN':65,'sp_pE':66,'sp_pD':67,
    'sp_vN':68,'sp_vE':69,'sp_vD':70,
    'sp_yaw':71,'sp_pos_valid':72,
}

class FrameAssembler:
    def __init__(self, chunk_bytes=CHUNK_BYTES):
        self.chunk_bytes = chunk_bytes
        self.cur = {}
        self.last = {}
        self.lock = threading.Lock()

    def push_chunk(self, data: bytes):
        if len(data) < struct.calcsize(IMGF_HDR): return
        hdr = struct.unpack(IMGF_HDR, data[:struct.calcsize(IMGF_HDR)])
        magic, ver, w, h, stride, frame_id, ts_ns, chunks, idx, payload, slot = hdr
        if magic != IMGF_MAGIC or ver != 1: return
        if idx >= chunks: return
        pay = data[struct.calcsize(IMGF_HDR):]
        if len(pay) != payload: return
        key = (slot, frame_id)
        with self.lock:
            a = self.cur.get(key)
            if a is None:
                a = {'w':w,'h':h,'ts':ts_ns,'chunks':chunks,'got':[False]*chunks,'buf':bytearray(w*h)}
                self.cur[key] = a
            off = idx * self.chunk_bytes
            a['buf'][off:off+payload] = pay
            a['got'][idx] = True
            if all(a['got']):
                img = np.frombuffer(a['buf'], dtype=np.uint8).reshape(a['h'], a['w'])
                self.last[key[0]] = (img.copy(), a['ts'])
                del self.cur[key]

    def latest_pair(self, slots=(0,1)):
        with self.lock:
            A = self.last.get(slots[0], (None,0))
            B = self.last.get(slots[1], (None,0))
            return A, B

class TelemetryState:
    def __init__(self):
        self.lock = threading.Lock()
        self.t = None
    def update(self, tup):
        with self.lock:
            self.t = tup
    def get(self):
        with self.lock:
            return self.t

def recv_images(asm: FrameAssembler, stop_evt: threading.Event):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('0.0.0.0', UDP_IMG_PORT))
    s.settimeout(0.5)
    while not stop_evt.is_set():
        try: data,_ = s.recvfrom(1500); asm.push_chunk(data)
        except socket.timeout: pass
        except Exception: pass

def recv_tlm(state: TelemetryState, stop_evt: threading.Event):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('0.0.0.0', UDP_TLM_PORT))
    s.settimeout(0.5)
    while not stop_evt.is_set():
        try:
            data,_ = s.recvfrom(4096)
            if len(data) < TLMD_SIZE: continue
            tup = struct.unpack(TLMD_FMT, data[:TLMD_SIZE])
            if tup[FIELD['magic']] != TLMD_MAGIC: continue
            state.update(tup)
        except socket.timeout: pass
        except Exception: pass

def label(img, text, x, y, scale=0.5, color=(220,220,220)):
    cv2.putText(img, text, (x,y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1)

def draw_bar(img, x,y,w,h, val, vmin, vmax, title):
    v = (val - vmin)/(vmax-vmin) if vmax!=vmin else 0.0
    v = 0.0 if v<0 else 1.0 if v>1 else v
    cv2.rectangle(img,(x,y),(x+w,y+h),(160,160,160),1)
    cv2.rectangle(img,(x,y+h-int(v*h)),(x+w,y+h),(80,220,80),-1)
    txt=f'{val:.2f}'; (tw,th),_=cv2.getTextSize(txt,cv2.FONT_HERSHEY_SIMPLEX,0.35,1)
    cv2.putText(img,txt,(x+(w-tw)//2,y+h-4),cv2.FONT_HERSHEY_SIMPLEX,0.35,(0,0,0),1)
    (tw,th),_=cv2.getTextSize(title,cv2.FONT_HERSHEY_SIMPLEX,0.42,1)
    cv2.putText(img,title,(x+(w-tw)//2,y+h+th+10),cv2.FONT_HERSHEY_SIMPLEX,0.42,(200,200,200),1)

def spark(img, x,y,w,h, samples, vmin, vmax, title=None):
    cv2.rectangle(img,(x,y),(x+w,y+h),(70,70,70),1)
    if title: cv2.putText(img,title,(x+2,y-4),cv2.FONT_HERSHEY_SIMPLEX,0.38,(180,180,180),1)
    if not samples or len(samples)<2: return
    src = samples if isinstance(samples, list) else list(samples)
    pts=[]; N=min(w,len(src)); base=src[-N:]; rng=(vmax-vmin) or 1.0
    for i,v in enumerate(base):
        t=(v-vmin)/rng; t=0.0 if t<0 else 1.0 if t>1 else t
        pts.append((x+i, y + h - int(t*h)))
    for i in range(1,len(pts)): cv2.line(img, pts[i-1], pts[i], (200,200,200), 1)



def draw_pos_ne_map(img, x, y, size, pN, pE, rng=10.0, label='Pos N/E (m)'):
    # Square map for N/E in [-rng, +rng], N up, E right
    s = size
    cv2.rectangle(img, (x, y), (x + s, y + s), (120, 120, 120), 1)
    # Grid + axes
    for frac in [0.25, 0.5, 0.75]:
        xx = int(x + frac * s); yy = int(y + frac * s)
        cv2.line(img, (xx, y), (xx, y + s), (70, 70, 70), 1)
        cv2.line(img, (x, yy), (x + s, yy), (70, 70, 70), 1)
    cv2.line(img, (x + s//2, y), (x + s//2, y + s), (160, 160, 160), 1)  # E axis
    cv2.line(img, (x, y + s//2), (x + s, y + s//2), (160, 160, 160), 1)  # N axis
    # Clamp/map
    rr = max(1e-3, float(rng))
    e = max(-rr, min(rr, float(pE)))
    n = max(-rr, min(rr, float(pN)))
    px = int(x + (e + rr) / (2*rr) * s)
    py = int(y + (rr - n) / (2*rr) * s)
    # Marker
    cv2.circle(img, (px, py), 8, (0, 0, 0), -1)
    cv2.circle(img, (px, py), 7, (255, 180, 0), -1)
    # Labels
    cv2.putText(img, f'{label}', (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 220, 255), 1)
    cv2.putText(img, f'N:{pN:+.1f}', (x, y + s + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,200,200), 1)
    cv2.putText(img, f'E:{pE:+.1f}', (x + s//2, y + s + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200,200,200), 1)


def draw_param_track(img, x, y, w, h, value, vmin=-1.0, vmax=1.0, label='Param'):
    # Clamp and map value [-1,1] -> screen (top=-1, bottom=+1)
    rng = (vmax - vmin) or 1.0
    t = (value - vmin) / rng
    t = 0.0 if t < 0 else 1.0 if t > 1 else t
    cy = int(y + t * h)
    # Track
    cv2.rectangle(img, (x, y), (x + w, y + h), (140, 140, 140), 1)
    # Ticks at -1, 0, +1
    for frac, txt in [(0.0, f'{vmin:+.1f}'), (0.5, '0'), (1.0, f'{vmax:+.1f}')]:
        yy = int(y + frac * h)
        cv2.line(img, (x-6, yy), (x, yy), (160, 160, 160), 1)
        cv2.putText(img, txt, (x - 44, yy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180, 180, 180), 1)
    # Marker
    cv2.circle(img, (x + w//2, cy), 7, (0, 0, 0), -1)
    cv2.circle(img, (x + w//2, cy), 6, (0, 255, 255), -1)
    # Label + numeric
    cv2.putText(img, f'{label}: {value:+.2f}', (x - 4, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 220, 255), 1)


def draw_param_track_h(img, x, y, w, h, value, vmin=-1.0, vmax=1.0, label='Param H'):
    rng = (vmax - vmin) or 1.0
    t = (value - vmin) / rng
    t = 0.0 if t < 0 else 1.0 if t > 1 else t
    cx = int(x + t * w)
    cv2.rectangle(img, (x, y), (x + w, y + h), (140, 140, 140), 1)
    for frac, txt in [(0.0, f'{vmin:+.1f}'), (0.5, '0'), (1.0, f'{vmax:+.1f}')]:
        xx = int(x + frac * w)
        cv2.line(img, (xx, y+h), (xx, y+h+6), (160, 160, 160), 1)
        cv2.putText(img, txt, (xx-8, y+h+18), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (180,180,180), 1)
    cv2.circle(img, (cx, y + h//2), 7, (0,0,0), -1)
    cv2.circle(img, (cx, y + h//2), 6, (0, 255, 255), -1)
    cv2.putText(img, f'{label}: {value:+.2f}', (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,220,255), 1)
def draw_stick_map(img, x, y, size, u, v, label='Stick XY'):
    # Draw a square map with axes in [-1,1] for (u,v) where v=up is +1
    s = size
    cv2.rectangle(img, (x, y), (x + s, y + s), (120, 120, 120), 1)
    # Grid
    for frac in [0.25, 0.5, 0.75]:
        xx = int(x + frac * s); yy = int(y + frac * s)
        cv2.line(img, (xx, y), (xx, y + s), (70, 70, 70), 1)
        cv2.line(img, (x, yy), (x + s, yy), (70, 70, 70), 1)
    # Axes
    cv2.line(img, (x + s//2, y), (x + s//2, y + s), (160, 160, 160), 1)
    cv2.line(img, (x, y + s//2), (x + s, y + s//2), (160, 160, 160), 1)
    # Clamp and map [-1,1] -> pixel
    uu = max(-1.0, min(1.0, u)); vv = max(-1.0, min(1.0, v))
    px = int(x + (uu + 1) * 0.5 * s)
    py = int(y + (1 - (vv + 1) * 0.5) * s)
    # Marker
    cv2.circle(img, (px, py), 8, (0, 0, 0), -1)
    cv2.circle(img, (px, py), 7, (0, 255, 0), -1)
    cv2.putText(img, f'{label}: ({u:+.2f},{v:+.2f})', (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 220, 255), 1)




def draw_drone_3dish(img, cx, cy, yaw, pitch, roll, arm_len=72, frame='x'):
    # Render a stylized quad. frame: 'x' (arms at ±45°) or 'plus' (arms fore/aft + left/right).
    if frame.lower() == 'x':
        ang_a = yaw + math.pi/4.0
        ang_b = yaw - math.pi/4.0
    else:
        ang_a = yaw
        ang_b = yaw + math.pi/2.0
    cv2.ellipse(img,(cx+8,cy+12),(arm_len,int(arm_len*0.35)),0,0,360,(25,25,25),-1)
    t_a = 1.0 - 0.25*abs(pitch)
    t_b = 1.0 - 0.25*abs(roll)
    w_a = int(9*max(0.6, t_a))+1
    w_b = int(9*max(0.6, t_b))+1
    def arm_ends(angle):
        s, c = math.sin(angle), math.cos(angle)
        return (int(cx - arm_len*s), int(cy - arm_len*c)), (int(cx + arm_len*s), int(cy + arm_len*c))
    (x1a,y1a),(x1b,y1b) = arm_ends(ang_a)
    (x2a,y2a),(x2b,y2b) = arm_ends(ang_b)
    cv2.line(img,(x1a,y1a),(x1b,y1b),(60,160,255),w_a+3)
    cv2.line(img,(x1a,y1a),(x1b,y1b),(130,230,255),w_a)
    cv2.line(img,(x2a,y2a),(x2b,y2b),(60,160,255),w_b+3)
    cv2.line(img,(x2a,y2a),(x2b,y2b),(130,230,255),w_b)
    for (px,py) in [(x1a,y1a),(x1b,y1b),(x2a,y2a),(x2b,y2b)]:
        cv2.circle(img,(px,py),13,(40,40,40),-1)
        cv2.circle(img,(px,py),13,(100,100,100),1)
        cv2.line(img,(px-12,py),(px+12,py),(180,255,255),2)
        cv2.line(img,(px,py-12),(px,py+12),(180,255,255),2)
    cv2.circle(img,(cx,cy),12,(240,240,0),-1)
    cv2.circle(img,(cx,cy),12,(0,0,0),1)
    s, c = math.sin(yaw), math.cos(yaw)
    nose = (int(cx + 18*s), int(cy + 18*c))
    cv2.line(img,(cx,cy),nose,(0,255,0),3)

def quat_to_euler_wxyz(qw,qx,qy,qz):
    sinr_cosp = 2*(qw*qx + qy*qz);  cosr_cosp = 1 - 2*(qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2*(qw*qy - qz*qx); pitch = math.copysign(math.pi/2, sinp) if abs(sinp)>=1 else math.asin(sinp)
    siny_cosp = 2*(qw*qz + qx*qy); cosy_cosp = 1 - 2*(qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def to_rgb_small(gray, w, h):
    if gray is None: return np.zeros((h,w,3), dtype=np.uint8)
    return cv2.resize(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), (w,h), interpolation=cv2.INTER_AREA)

def draw_full_hud_and_canvas(t, H, left0, left1):
    # layout
    IMG_W, IMG_H = 320, 240
    GAP = 6
    LEFT_W = IMG_W*2 + GAP
    HUD_W, HUD_H = 840, 700
    GUTTER = 8
    LEFT_EXTRA = 240

    hud = np.zeros((HUD_H, HUD_W, 3), dtype=np.uint8)

    # read fields
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

    # header texts + Alt bar
    now = time.time()
    # Decide stick XY (roll/pitch) with fallback animation when near-zero inputs
    u_in, v_in = rc_roll, rc_pitch
    if abs(u_in) < 0.02 and abs(v_in) < 0.02:
        u_in = 0.85*math.sin(2*math.pi*0.33*now)
        v_in = 0.85*math.cos(2*math.pi*0.21*now + 0.8)
    # Decide param track (yaw) with fallback
    yaw_in = rc_yaw if abs(rc_yaw) >= 0.02 else 0.85*math.sin(2*math.pi*0.27*now + 1.2)

    # Draw the extra widgets to the LEFT of the Alt bar
    alt_x = HUD_W - 48
    stick_size = 170
    stick_x = alt_x - (stick_size + 80)
    stick_y = 80
    draw_stick_map(hud, stick_x, stick_y, stick_size, u_in, v_in, label='RC XY')
    # Position N/E map under RC XY (bigger)
    pos_size = stick_size
    pos_x = stick_x
    pos_y = stick_y + stick_size + 28
    draw_pos_ne_map(hud, pos_x, pos_y, pos_size, pN, pE, rng=10.0, label='Pos N/E (m)')

    label(hud, f'Pos NED: [{pN:+.2f},{pE:+.2f},{pD:+.2f}] m', 10, 24, 0.6, (255,255,255))
    label(hud, f'Vel NED: [{vN:+.2f},{vE:+.2f},{vD:+.2f}] m/s', 10, 46, 0.6, (200,200,200))
    label(hud, f'Setpoint: [{spN:+.1f},{spE:+.1f},{spD:+.1f}]  Yaw_sp:{spYaw:+.2f}  valid:{int(spOK)}', 10, 68, 0.5, (180,220,255))
    draw_bar(hud, HUD_W-48, 40, 28, 270, -pD, -10, +10, 'Alt')

    # push histories
    for k,val in [('pN',pN),('pE',pE),('pD',pD),('vN',vN),('vE',vE),('vD',vD),
                  ('ax',ax),('ay',ay),('az',az),('gx',gx),('gy',gy),('gz',gz),
                  ('mx',mx),('my',my),('mz',mz),('baro_p',baro_p),('baro_alt',baro_alt),('baro_temp',baro_temp)]:
        H[k].append(val)
    H['roll'].append(r); H['pitch'].append(p); H['yaw'].append(yaw)

    # Graphs: EKF p/v/rpy
    gx0, gy0 = 10, 92; gw, gh, gapx, gapy = 200, 52, 18, 12
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
    spark(hud, x1, gy0 + 2*(gh+gapy), gw, gh, H['az'],  9.5, 10.2, 'IMU Acc Z')
    spark(hud, x1, gy0 + 3*(gh+gapy), gw, gh, H['gx'], -0.2, 0.2, 'IMU Gyro X')
    spark(hud, x1, gy0 + 4*(gh+gapy), gw, gh, H['gy'], -0.2, 0.2, 'IMU Gyro Y')
    spark(hud, x1, gy0 + 5*(gh+gapy), gw, gh, H['gz'], -0.2, 0.2, 'IMU Gyro Z')
    spark(hud, x1, gy0 + 6*(gh+gapy), gw, gh, H['mx'], -1.0, 1.0, 'IMU Mag X')
    spark(hud, x1, gy0 + 7*(gh+gapy), gw, gh, H['my'], -1.0, 1.0, 'IMU Mag Y')
    spark(hud, x1, gy0 + 8*(gh+gapy), gw, gh, H['mz'], -1.0, 1.0, 'IMU Mag Z')

    # RC + motors (nudged up)
    bx = HUD_W - 360; yb = HUD_H - 180
    draw_bar(hud, bx,       yb, 26, 110, thr, 0, 1, 'THR')
    draw_bar(hud, bx+36,    yb, 26, 110, (rc_roll +1)/2, 0, 1, 'ROLL')
    draw_bar(hud, bx+72,    yb, 26, 110, (rc_pitch+1)/2, 0, 1, 'PITCH')
    draw_bar(hud, bx+108,   yb, 26, 110, (rc_yaw  +1)/2, 0, 1, 'YAW')
    for i,(pwm,x0m) in enumerate(zip([m0,m1,m2,m3],[bx+200,bx+236,bx+272,bx+308])):
        draw_bar(hud, x0m, yb, 26, 110, (pwm-1000)/1000.0, 0, 1, f'M{i}')
    label(hud, f'ARM:{int(arm)} MODE:{int(mode)}', bx, HUD_H-12, 0.6, (0,255,255))

    # compose canvas with extra left height for the drone
    left_panel_h = IMG_H + LEFT_EXTRA
    Htot = max(left_panel_h, HUD_H)
    Wtot = LEFT_W + GUTTER + HUD_W
    canvas = np.zeros((Htot, Wtot, 3), dtype=np.uint8)
    canvas[:IMG_H, 0:IMG_W, :] = left0
    canvas[:IMG_H, IMG_W+GAP:IMG_W+GAP+IMG_W, :] = left1
    canvas[:HUD_H, LEFT_W + GUTTER : LEFT_W + GUTTER + HUD_W, :] = hud

    # draw the drone on the LEFT below the two images
    left_drone_cx = LEFT_W // 2
    left_drone_cy = IMG_H + (LEFT_EXTRA//2) + 44
    draw_drone_3dish(canvas, left_drone_cx, left_drone_cy, yaw, p, r, arm_len=78, frame='x')
    label(canvas, f'RPY [deg]: {math.degrees(r):+5.1f} {math.degrees(p):+5.1f} {math.degrees(yaw):+5.1f}',
          left_drone_cx - 140, left_drone_cy + 62, 0.5, (200,200,200))
    yaw_w, yaw_h = 230, 18
    yaw_x = left_drone_cx - yaw_w//2
    yaw_y = left_drone_cy + 140
    draw_param_track_h(canvas, yaw_x, yaw_y, yaw_w, yaw_h, yaw_in, -1.0, 1.0, 'Yaw')

    # optical flow arrow across the two images
    if of_valid:
        start = (LEFT_W//2, IMG_H//2)
        end = (int(start[0] + of_u*1.0), int(start[1] + of_v*1.0))
        end = (max(0,min(Wtot-1,end[0])), max(0,min(Htot-1,end[1])))
        cv2.arrowedLine(canvas, start, end, (0,0,0), 8, tipLength=0.15)
        cv2.arrowedLine(canvas, start, end, (0,255,0), 6, tipLength=0.15)
        cv2.circle(canvas, start, 7, (0,0,0), -1); cv2.circle(canvas, start, 6, (0,255,0), -1)
        label(canvas, f'OF u,v={of_u:+.1f},{of_v:+.1f}  q={of_q:.2f}', 8, 20, 0.5, (0,255,0))

    return canvas

def main():
    asm = FrameAssembler()
    tlm = TelemetryState()
    stop_evt = threading.Event()
    threading.Thread(target=recv_images, args=(asm,stop_evt), daemon=True).start()
    threading.Thread(target=recv_tlm,    args=(tlm,stop_evt), daemon=True).start()

    H = {k: collections.deque(maxlen=300) for k in
         ['pN','pE','pD','vN','vE','vD','roll','pitch','yaw',
          'ax','ay','az','gx','gy','gz','mx','my','mz',
          'baro_p','baro_alt','baro_temp']}

    IMG_W, IMG_H = 320, 240
    GAP = 6; LEFT_W = IMG_W*2 + GAP
    HUD_W, HUD_H = 840, 700; GUTTER = 8
    LEFT_EXTRA = 240

    while True:
        (img0,_), (img1,_) = asm.latest_pair((0,1))
        left0 = to_rgb_small(img0, IMG_W, IMG_H)
        left1 = to_rgb_small(img1, IMG_W, IMG_H)

        t = tlm.get()
        if t:
            canvas = draw_full_hud_and_canvas(t, H, left0, left1)
        else:
            # idle canvas
            Htot = max(IMG_H + LEFT_EXTRA, HUD_H)
            Wtot = LEFT_W + GUTTER + HUD_W
            canvas = np.zeros((Htot, Wtot, 3), dtype=np.uint8)
            canvas[:IMG_H, 0:IMG_W, :] = left0
            canvas[:IMG_H, IMG_W+GAP:IMG_W+GAP+IMG_W, :] = left1

        cv2.imshow("Drone GS", canvas)
        k = cv2.waitKey(1)
        if k == 27 or k == ord('q'):
            stop_evt.set(); break

def demo_preview(save_to=None, steps=240):
    # Histories like in live mode
    H = {k: collections.deque(maxlen=300) for k in
         ['pN','pE','pD','vN','vE','vD','roll','pitch','yaw',
          'ax','ay','az','gx','gy','gz','mx','my','mz',
          'baro_p','baro_alt','baro_temp']}

    # Static images for the left panel
    IMG_W, IMG_H = 320, 240
    yy,xx = np.mgrid[0:IMG_H, 0:IMG_W]
    imgA = ((np.sin((xx+166)/24)+1)*127 + (np.cos((yy-41)/33)+1)*64).astype(np.uint8)
    imgB = ((np.sin((xx-88)/18)+1)*110 + (np.cos((yy+55)/22)+1)*90).astype(np.uint8)
    left0 = cv2.cvtColor(imgA, cv2.COLOR_GRAY2BGR)
    left1 = cv2.cvtColor(imgB, cv2.COLOR_GRAY2BGR)

    # Generate 'steps' synthetic telemetry samples to fill histories
    def pack_tuple(i):
        t = i / float(steps)
        size = struct.calcsize(TLMD_FMT); seq = i+1
        ax,ay,az = 0.05*math.sin(4*t*math.pi), -0.04*math.cos(3*t*math.pi), 9.80 + 0.05*math.sin(2*t*math.pi)
        gx,gy,gz = 0.05*math.sin(2*t*math.pi), 0.04*math.cos(2*t*math.pi), 0.03*math.sin(1.5*t*math.pi)
        mx,my,mz = 0.4*math.sin(t*math.pi), -0.3*math.cos(0.7*t*math.pi), 0.2*math.sin(0.9*t*math.pi)
        magadj=(1.0,1.0,1.0); mag_rdy=1; imu_temp=26.0 + 0.3*math.sin(0.3*t*math.pi)
        baro=(1013.0 + 2.0*math.sin(0.5*t*math.pi), 1.8 + 0.6*math.sin(0.8*t*math.pi), 25.0 + 1.0*math.sin(0.2*t*math.pi))
        pN,pE,pD = 1.0 + 0.8*math.sin(0.6*t*math.pi), 0.5*math.cos(0.4*t*math.pi), -1.5 + 0.3*math.sin(0.9*t*math.pi)
        vN,vE,vD = 0.6*math.cos(0.8*t*math.pi), 0.3*math.sin(0.7*t*math.pi), -0.2 + 0.15*math.cos(0.5*t*math.pi)
        roll,pitch,yaw = 0.25*math.sin(0.7*t*math.pi), -0.18*math.sin(0.9*t*math.pi), 1.2 + 0.4*math.sin(0.6*t*math.pi)
        cr,sr = math.cos(roll/2), math.sin(roll/2)
        cp,sp = math.cos(pitch/2), math.sin(pitch/2)
        cy,sy = math.cos(yaw/2), math.sin(yaw/2)
        qw = cy*cp*cr + sy*sp*sr; qx = cy*cp*sr - sy*sp*cr; qy = sy*cp*sr + cy*sp*cr; qz = sy*cp*cr - cy*sp*sr
        bg=(0.001,0.002,0.003); ba=(0.01,-0.02,0.005)
        Pdiag=[0.1]*15
        rc=(0.6 + 0.3*max(0,math.sin(0.8*t*math.pi)), 0.2*math.sin(0.9*t*math.pi), 0.2*math.sin(0.7*t*math.pi), 0.2*math.sin(0.6*t*math.pi))
        arm=1; mode=3
        of=(220.0*math.cos(0.6*t*math.pi), -150.0*math.sin(0.6*t*math.pi), 0.8)
        of_valid=1
        motors=(1450 + int(60*math.sin(0.9*t*math.pi)), 1500, 1550 + int(40*math.cos(1.1*t*math.pi)), 1480)
        sp=(0.0,0.0,-1.5, 0.0,0.0,0.0, 0.0); sp_valid=1
        vals=[TLMD_MAGIC,size,seq, ax,ay,az,gx,gy,gz, mx,my,mz, *magadj, mag_rdy, imu_temp,
              *baro, pN,pE,pD, vN,vE,vD, qw,qx,qy,qz, *bg, *ba, *Pdiag, *rc, arm, mode,
              *of, of_valid, *motors, *sp, sp_valid]
        return struct.unpack(TLMD_FMT, struct.pack(TLMD_FMT, *vals))

    t_last = None
    for i in range(steps):
        t_last = pack_tuple(i)
        # push into histories by drawing onto a dummy hud
        _ = draw_full_hud_and_canvas(t_last, H, left0, left1)

    # After histories are populated, draw one final canvas with graphs
    canvas = draw_full_hud_and_canvas(t_last, H, left0, left1)
    if save_to:
        cv2.imwrite(save_to, canvas)
    return canvas

if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--demo':
        demo_preview(None, steps=240)
    else:
        main()