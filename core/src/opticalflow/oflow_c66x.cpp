#include "opticalflow/oflow_c66x.h"
#include <string.h>
#include <math.h>

#if defined(__TI_COMPILER_VERSION__) || defined(_TMS320C6600)
  #include <c6x.h>
  #define C66X
#endif


// INDOORS HELPER
static inline float64 quickselect(float64* a, sint32 n, sint32 k) {
  sint32 l=0, r=n-1;
  while(1) {
    float64 p=a[(l+r)/2]; 
    sint32 i=l,j=r;
    while(i<=j) { 
        while(a[i]<p) ++i; 
        while(a[j]>p) --j; 
        if(i<=j) {
            float64 t=a[i];
            a[i]=a[j];
            a[j]=t;
            ++i; --j;
        }
    }
    if(k<=j) r=j; 
    else if(k>=i) l=i; 
    else return a[k];
  }
}

static inline void swp(float64& a, float64& b) { float64 t=a; a=b; b=t; }

static inline int median_index(float64* a, int i, int j, int k) {
  const float64 x=a[i], y=a[j], z=a[k];
  return (x<y) ? ((y<z)? j : (x<z? k : i)) : ((x<z)? i : (y<z? k : j));
}

static inline void insertion_sort(float64* a, int lo, int hi) {
  for(int i=lo+1;i<=hi;++i) {
    float64 v=a[i]; int j=i-1;
    while(j>=lo && a[j]>v) { 
      a[j+1]=a[j]; 
      --j; 
    }
    a[j+1]=v;
  }
}

static inline float64 quickselect_imprv(float64* a, sint32 n, sint32 k) {
  sint32 lo = 0, hi = n - 1;

  for(;;) {
    if(hi - lo <= 16) {                 // tiny partition: finish fast
      insertion_sort(a, lo, hi);
      return a[lo + k];
    }

    // pivot -> median-of-three
    sint32 m = lo + ((hi - lo) >> 1);
    sint32 i1 = lo, i2 = m, i3 = hi;
    if(hi - lo > 64) {
      sint32 s = (hi - lo) / 8;
      i1 = median_index(a, lo,      lo + s,      lo + 2*s);
      i2 = median_index(a, m  - s,  m,           m + s);
      i3 = median_index(a, hi - 2*s,hi - s,      hi);
    }
    sint32 pidx = median_index(a, i1, i2, i3);
    float64 pivot = a[pidx];

    // 3-way partition -> "< pivot | == pivot | > pivot"
    sint32 lt = lo, i = lo, gt = hi;
    while(i <= gt) {
      if(a[i] < pivot)      { swp(a[lt], a[i]); ++lt; ++i; }
      else if (a[i] > pivot) { swp(a[i], a[gt]); --gt; }
      else                   { ++i; }
    }

    // Decide which side contains the k-th (relative to lo)
    if(k < (lt - lo)) {
      hi = lt - 1;
    } else if(k <= (gt - lo)) {
      return pivot;                           // inside the eqiv block
    } else {
      k  = k - (gt - lo) - 1;
      lo = gt + 1;
    }
  }
}
// !INDOORS HELEPR

DFC_t_OFlow_Output of_compute_global_flow(const uint8* prevY, const uint8* currY, sint32 W, sint32 H, sint32 stride, float64 dt_s, const DFC_t_OFlow_Params& params) {
  DFC_t_OFlow_Output out{0,0,0,false};
  
  // guards
  if(!prevY || !currY || W <= 0 || H <= 0 || stride < W) return out;
  if(params.cell_w < 3 || params.cell_h < 3) return out;
  if(dt_s <= 0.0) return out;
  
  const sint32 nx = W / params.cell_w;
  const sint32 ny = H / params.cell_h;

  if (nx < 3 || ny < 3) return out; // we skip 1-cell border

  float64 u_buf[1024], v_buf[1024];
  sint16  sanityBuffer_x[1024], sanityBuffer_y[1024];  // store cell indices for sanity checks
  sint32 n = 0;

  // Process cells (skip 1-cell border)
  for(sint32 gy=1; gy<ny-1; ++gy) {
    const sint32 y0 = gy*params.cell_h;
    const sint32 y1 = y0 + params.cell_h;

    for(sint32 gx=1; gx<nx-1; ++gx) {
      const sint32 x0 = gx*params.cell_w;
      const sint32 x1 = x0 + params.cell_w;

      float64 Gxx=0.0, Gxy=0.0, Gyy=0.0, Gxt=0.0, Gyt=0.0;

      // central differences (Ix = R-L, Iy = D-U, It = curr-prev)
      for(sint32 y=y0+1; y<y1-1; ++y) {
        const uint8* Pu = prevY + (y-1)*stride + x0;
        const uint8* Pm = prevY + (y  )*stride + x0;
        const uint8* Pd = prevY + (y+1)*stride + x0;
        const uint8* Cm = currY + (y  )*stride + x0;

        sint32 x = x0+1;
#ifdef C66X
        // Vectorized 4-pixel steps
        for(; x+3 < x1-1; x += 4) {
          // Load 4 bytes (unsigned) from each row at x-1 / x+1
          // Left/right
          uint32 L4 = _mem4_const((const void*)(Pm + (x-1 - x0)));
          uint32 R4 = _mem4_const((const void*)(Pm + (x+1 - x0)));
          // Up/down at x
          uint32 U4 = _mem4_const((const void*)(Pu + (x   - x0)));
          uint32 D4 = _mem4_const((const void*)(Pd + (x   - x0)));
          // Temporal at x
          uint32 P4 = _mem4_const((const void*)(Pm + (x   - x0)));
          uint32 C4 = _mem4_const((const void*)(Cm + (x   - x0)));

          // Unpack 4xU8 -> 2xU16 (lo/hi halves)
          // Ix = R-L (signed 16-bit) ; Iy = D-U ; It = C-P
          sint32 Ix_lo = _sub2(_unpklu4(R4), _unpklu4(L4));
          sint32 Ix_hi = _sub2(_unpkhu4(R4), _unpkhu4(L4));
          sint32 Iy_lo = _sub2(_unpklu4(D4), _unpklu4(U4));
          sint32 Iy_hi = _sub2(_unpkhu4(D4), _unpkhu4(U4));
          sint32 It_lo = _sub2(_unpklu4(C4), _unpklu4(P4));
          sint32 It_hi = _sub2(_unpkhu4(C4), _unpkhu4(P4));

          // Accumulate normal eq sums: sum(Ix^2), sum(Iy^2), sum(IxIy), sum(IxIt), sum(IyIt)
          Gxx += _dotp2(Ix_lo, Ix_lo) + _dotp2(Ix_hi, Ix_hi);
          Gyy += _dotp2(Iy_lo, Iy_lo) + _dotp2(Iy_hi, Iy_hi);
          Gxy += _dotp2(Ix_lo, Iy_lo) + _dotp2(Ix_hi, Iy_hi);
          Gxt += _dotp2(Ix_lo, It_lo) + _dotp2(Ix_hi, It_hi);
          Gyt += _dotp2(Iy_lo, It_lo) + _dotp2(Iy_hi, It_hi);
        }
#endif
        // Scalar tail
        for(; x < x1-1; ++x) {
          sint32 Ix = (sint32)Pm[x+1 - x0] - (sint32)Pm[x-1 - x0];
          sint32 Iy = (sint32)Pd[x   - x0] - (sint32)Pu[x   - x0];
          sint32 It = (sint32)Cm[x   - x0] - (sint32)Pm[x   - x0];
          Gxx += (float64)Ix*Ix;
          Gyy += (float64)Iy*Iy;
          Gxy += (float64)Ix*Iy;
          Gxt += (float64)Ix*It;
          Gyt += (float64)Iy*It;
        }
      }

      const float64 det = Gxx*Gyy - Gxy*Gxy;
      if(det > (float64)params.texture_det) {
        const float64 inv00 =  Gyy / det;
        const float64 inv01 = -Gxy / det;
        const float64 inv11 =  Gxx / det;
        const float64 u = -(inv00*Gxt + inv01*Gyt);
        const float64 v = -(inv01*Gxt + inv11*Gyt);
        if(n < (sint32)(sizeof(u_buf)/sizeof(u_buf[0]))) {
          u_buf[n] = (float64)(u / dt_s);
          v_buf[n] = (float64)(v / dt_s);

          sanityBuffer_x[n] = (sint16)gx;   // record cell index for sanity checks
          sanityBuffer_y[n] = (sint16)gy;
          ++n;
        }
      }
      if(n >= (sint32)(sizeof(u_buf)/sizeof(u_buf[0]))) break;
    }
    if(n >= (sint32)(sizeof(u_buf)/sizeof(u_buf[0]))) break;
  }

  if(n < params.min_tracks) return out;

  float64* tmp = (float64*)malloc(n*sizeof(float64));
  if (!tmp) return out;

  memcpy(tmp, u_buf, n*sizeof(float64));
  const float64 u_med = quickselect_imprv(tmp, n, n/2);
  memcpy(tmp, v_buf, n*sizeof(float64));
  const float64 v_med = quickselect_imprv(tmp, n, n/2);

  for(sint32 i=0;i<n;i++) { 
    tmp[i] = fabs(u_buf[i]-u_med);
  }
  const float64 mad_u = quickselect_imprv(tmp, n, n/2) + 1e-3;

  for(sint32 i=0;i<n;i++) {
    tmp[i] = fabs(v_buf[i]-v_med);
  }
  const float64 mad_v = quickselect_imprv(tmp, n, n/2) + 1e-3;

  // sanity checks (false movement detection)
  // check near zero cells
  sint32 near_zero = 0;
  for (sint32 i=0; i<n; ++i) {
    float64 mag = hypot(u_buf[i], v_buf[i]);
    if (mag < params.zero_th_pxs) ++near_zero;
  }
  const float64 frac_zero = (float64)near_zero / (float64)n;

  // Quadrant medians using tmp as scratch (limit the mallocs)
  const sint32 midx = nx / 2, midy = ny / 2;

  auto bucket_med = [&](bool left, bool up, float64& um, float64& vm) {
    // U
    sint32 m = 0;
    for(sint32 i=0; i<n; ++i)
      if((sanityBuffer_x[i] < midx) == left && (sanityBuffer_y[i] < midy) == up) tmp[m++] = u_buf[i];
    um = (m>0) ? quickselect_imprv(tmp, m, m/2) : 0.0;
    // V
    m = 0;
    for(sint32 i=0; i<n; ++i)
      if((sanityBuffer_x[i] < midx) == left && (sanityBuffer_y[i] < midy) == up) tmp[m++] = v_buf[i];
    vm = (m>0) ? quickselect_imprv(tmp, m, m/2) : 0.0;
  };

  float64 uUL,vUL,uUR,vUR,uLL,vLL,uLR,vLR;
  bucket_med(true , true , uUL,vUL);
  bucket_med(false, true , uUR,vUR);
  bucket_med(true , false, uLL,vLL);
  bucket_med(false, false, uLR,vLR);

  auto dist = [](float64 ax,float64 ay,float64 bx,float64 by){ return hypot(ax-bx, ay-by); };
  float64 spread = 0.0;
  float64 dists[6] = {
    dist(uUL,vUL,uUR,vUR), dist(uUL,vUL,uLL,vLL), dist(uUL,vUL,uLR,vLR),
    dist(uUR,vUR,uLL,vLL), dist(uUR,vUR,uLR,vLR), dist(uLL,vLL,uLR,vLR)
  };
  
  for(int k=0;k<6;++k) {
    if (dists[k] > spread) {
      spread = dists[k];
    }
  }

  const float64 robust_scale     = hypot(mad_u, mad_v);
  const float64 med_mag          = hypot(u_med,  v_med);
  const bool quadrant_consistent = (spread < params.quad_spread_gain * (robust_scale + 1e-6));
  const bool large_bias          = (med_mag > params.med_th_pxs);
  const bool majority_still_bg   = (frac_zero > params.zero_maj_min);
  const bool suspect             = (!quadrant_consistent) && large_bias && majority_still_bg;

  sint32 inl=0;
  for(sint32 i=0;i<n;i++) {
    if(fabs(u_buf[i]-u_med) < params.mad_gate*mad_u && fabs(v_buf[i]-v_med) < params.mad_gate*mad_v) ++inl;
  }
  out.u_px_per_s = u_med;
  out.v_px_per_s = v_med;
  float64 tempQuality = (float64)inl/(float64)n;
  if(suspect && tempQuality > params.qualityFailed) {
    out.quality = params.qualityFailed; // for flaging false drone movement (false movement detection)
  } else {
    out.quality = tempQuality;
  }
  
  out.valid  = true;

  if(tmp) free(tmp);

  return out;
}
