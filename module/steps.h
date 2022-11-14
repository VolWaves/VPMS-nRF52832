
#ifndef _STEP_H_
#define _STEP_H_
#include <stdint.h>

int8_t calcStep(int16_t*, uint8_t);

int16_t _setThreshold(int16_t s1, int16_t s2, int16_t s3, int16_t s4);
int16_t _setTimeout(int16_t N1, int16_t N2, int16_t N3);

#endif
