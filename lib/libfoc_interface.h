#ifndef __LIBFOC_INTERFACE_H__
#define __LIBFOC_INTERFACE_H__
#include "stdint.h"

// --------------驱动函数，需要自行实现------------------

void foc_driver_init(uint8_t pdrv);
void foc_driver_motor_enable(uint8_t pdrv,uint8_t enable);
void foc_driver_set_phase(uint8_t pdrv,uint16_t phase_a, uint16_t phase_b, uint16_t phase_c);
void foc_driver_get_mech_angle(uint8_t pdrv,float *angle);
void foc_driver_debug_printf(uint8_t pdrv,const char *const fmt, ...);
void foc_driver_delay_ms(uint8_t pdrv,uint32_t ms);

#endif
