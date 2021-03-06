#include "filters.h"

const iir_cf low_pass8 = {
  .sections = 4,
  .sect_cf  = {
    {    60,     54,     60, -25932,  10924},
    { 16384, -13607,  16384, -24432,  13248},
    { 16384, -19796,  16384, -23291,  15128},
    { 16384, -21305,  16384, -22960,  16072}
  }
};

int16_t iir_TrFII_filter(int16_t sample, uint32_t reset, const iir_cf *cf) {
  static int32_t w[IIR_MAX_SECTIONS][2];
  if (reset) {
    for (uint32_t sect = 0; sect < IIR_MAX_SECTIONS; sect++) {
      w[sect][0] = 0;
      w[sect][1] = 0;
    }
  }
  int16_t x_in = sample;
  for (int32_t sect = cf->sections - 1; sect >= 0; sect--) {
    const iir_sos_cf *sos = &(cf->sect_cf[sect]);
    int32_t y_out  = x_in * sos->b0 + w[sect][0];
    int16_t y_next = IIR_ROUND(y_out);
    w[sect][0] = x_in * sos->b1 + w[sect][1] - y_next * sos->a1;
    w[sect][1] = x_in * sos->b2              - y_next * sos->a2;
    x_in = y_next;
  }
  return x_in;
}

