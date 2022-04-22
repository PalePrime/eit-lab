#ifndef FILTERS_H
#define FILTERS_H

#include "pico/stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IIR_MAX_SECTIONS  4
#define IIR_SHIFT        14

#define IIR_HALF         (1 << (IIR_SHIFT - 1))
#define IIR_ROUND(v)     (int16_t)((v>=0 ? v + IIR_HALF : v + IIR_HALF - 1) >> IIR_SHIFT)

typedef struct iir_sos_cf {
  int16_t b0;
  int16_t b1;
  int16_t b2;
  int16_t a1;
  int16_t a2;
} iir_sos_cf;

typedef struct iir_cf {
  uint32_t   sections;
  iir_sos_cf sect_cf[];
} iir_cf;

extern const iir_cf low_pass8;

int16_t iir_TrFII_filter(int16_t sample, uint32_t reset, const iir_cf *cf);

#ifdef __cplusplus
}
#endif

#endif /*FILTERS_H*/
