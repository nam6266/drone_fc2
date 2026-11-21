#pragma once
#include "config.h"

// ============================================================
//  Mixer quad X
//
//            MŨI
//   M4 (CW)  o      o  M1 (CCW)
//              \    /
//               [XX]
//              /    \ .
//   M3 (CCW) o      o  M2 (CW)
//
//  r > 0 : muốn roll tăng (nghiêng phải)  -> giảm motor phải, tăng motor trái
//  p > 0 : muốn pitch tăng (chúi mũi)     -> giảm motor trước, tăng motor sau
//  y > 0 : muốn gz tăng (mũi quay trái)   -> tăng motor CW (phản lực của cánh CW
//                                            đẩy thân quay ngược chiều = CCW)
// ============================================================
inline void mixQuadX(float thr, float r, float p, float y, float out[4]) {
  y *= cfg::MIX_YAW_SIGN;
  out[0] = thr - r - p - y;   // M1 trước-phải CCW
  out[1] = thr - r + p + y;   // M2 sau-phải  CW
  out[2] = thr + r + p - y;   // M3 sau-trái  CCW
  out[3] = thr + r - p + y;   // M4 trước-trái CW

  // Nếu có motor vượt 100 %: hạ đều cả 4 để giữ được chênh lệch (giữ thăng bằng)
  float mx = out[0];
  for (int i = 1; i < 4; i++) if (out[i] > mx) mx = out[i];
  if (mx > 1.0f) for (int i = 0; i < 4; i++) out[i] -= (mx - 1.0f);

  for (int i = 0; i < 4; i++) out[i] = clampf(out[i], 0.0f, 1.0f);
}
