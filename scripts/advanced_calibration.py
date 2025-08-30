import socket, struct, time, sys, json
import numpy as np
from pathlib import Path

MAGIC = 0x43414C42
SAMPLE_FMT = "<6f"  # ax ay az mx my mz
HDR_FMT    = "<IHH" # magic, seq, count
HDR_SZ     = struct.calcsize(HDR_FMT)
SAMPLE_SZ  = struct.calcsize(SAMPLE_FMT)

def recv_samples(port=49998, timeout=60.0):
    """Receive UDP chunks until no packets for `timeout` seconds."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", port))
    s.settimeout(timeout)

    acc = []
    mag = []

    last = time.time()
    while True:
        try:
            data, _ = s.recvfrom(65535)
            last = time.time()
            if len(data) < HDR_SZ: continue
            magic, seq, count = struct.unpack_from(HDR_FMT, data, 0)
            if magic != MAGIC: continue
            expected = HDR_SZ + count * SAMPLE_SZ
            if len(data) < expected: continue
            off = HDR_SZ
            for _ in range(count):
                ax, ay, az, mx, my, mz = struct.unpack_from(SAMPLE_FMT, data, off)
                off += SAMPLE_SZ
                acc.append([ax, ay, az])
                mag.append([mx, my, mz])
        except socket.timeout:
            if time.time() - last >= timeout:
                break
    s.close()
    A = np.asarray(acc, dtype=np.float64)
    M = np.asarray(mag, dtype=np.float64)
    return A, M

def fit_ellipsoid(X):
    """
    Fit x^T A x + b^T x + d = 1 (A symmetric).
    Returns (A, b, d). Then bias c, and transform L s.t. L (x - c) is unit sphere.
    """
    X = np.asarray(X, dtype=np.float64)
    x, y, z = X[:,0], X[:,1], X[:,2]
    D = np.column_stack([
        x*x, y*y, z*z, 2*x*y, 2*x*z, 2*y*z,  # symmetric terms
        x, y, z,
        np.ones_like(x)
    ])  # (N, 10)

    # Solve D v = 1 in least squares sense
    rhs = np.ones((X.shape[0], 1))
    v, *_ = np.linalg.lstsq(D, rhs, rcond=None)
    v = v.flatten()

    # Unpack
    A = np.array([[v[0], v[3], v[4]],
                  [v[3], v[1], v[5]],
                  [v[4], v[5], v[2]]], dtype=np.float64)
    b = np.array([v[6], v[7], v[8]], dtype=np.float64)
    d = float(v[9])

    # Center
    c = -0.5 * np.linalg.solve(A, b)

    # Scale factor gamma so that (x-c)^T (A/gamma) (x-c) = 1
    gamma = 1.0 - d + c.T @ A @ c
    W = A / gamma  # shape matrix (SPD if fit OK)

    # L s.t. L^T L = W (Cholesky). Use upper-tri or lower-tri, doesn’t matter for calibration.
    L = np.linalg.cholesky(W)
    return A, b, d, c, L

def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 49998
    print(f"Listening on UDP :{port} ...")
    Aacc, Amag = recv_samples(port=port)
    print(f"Got samples: acc={len(Aacc)}, mag={len(Amag)}")

    # Fit acc
    A_a, b_a, d_a, c_a, L_a = fit_ellipsoid(Aacc)
    # Scale to GRAVITY
    GRAV = 9.80665
    A_acc = L_a * GRAV  # 3x3
    b_acc = c_a         # 3

    # Fit mag
    A_m, b_m, d_m, c_m, L_m = fit_ellipsoid(Amag)
    # multiply by a scalar factor here.
    A_mag = L_m
    b_mag = c_m

    # Save results
    out = {
      "acc": {"M": A_acc.tolist(), "b": b_acc.tolist()},
      "mag": {"M": A_mag.tolist(), "b": b_mag.tolist()},
    }
    Path("calib_results.json").write_text(json.dumps(out, indent=2))
    print("Wrote calib_results.json")

    def fmt9(M):
      return ", ".join(f"{x:.9g}" for x in M.flatten())
    def fmt3(v):
      return ", ".join(f"{x:.9g}" for x in v)

    print("\n// New Params generated (DFC_t_MPU9250_Params)")
    print("{")
    print(f"  /*acc_M*/ {{ {fmt9(A_acc)} }},")
    print(f"  /*acc_b*/ {{ {fmt3(b_acc)} }},")
    print(f"  /*mag_M*/ {{ {fmt9(A_mag)} }},")
    print(f"  /*mag_b*/ {{ {fmt3(b_mag)} }},")
    print("}")

if __name__ == "__main__":
    main()
