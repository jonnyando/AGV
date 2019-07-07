#ifndef __DRV8303_H__
#define __DRV8303_H__

#include "stm32f103xb.h"

uint16_t drv_transceive(uint16_t tx_reg);
uint16_t drv_write(uint8_t drv_reg, uint16_t payload);
uint16_t drv_transceive(uint16_t tx_reg);
uint16_t drv_read(uint8_t drv_reg);


#endif // endif __DRV8303_H__
