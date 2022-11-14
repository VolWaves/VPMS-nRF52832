
#ifndef _MC3623_H_
#define _MC3623_H_
#include <stdint.h>
#include <stdbool.h>

#define MC3632_READ(x) (x|0xC0)
#define MC3632_WRITE(x) (x|0x40)

typedef struct xyz {
	int8_t x;
	int8_t y;
	int8_t z;
} gsensor_data_t;

bool mc3632_test(void);
void mc3632_init(bool isSleep);
// void mc3632_int_ISR(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);
void mc3632_int_handler(void);
void mc3632_int_scan(void);

extern void gsensor_data_handler(int16_t* data);


#endif
