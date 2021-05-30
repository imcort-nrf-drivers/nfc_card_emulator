#ifndef __CARD_SLOT__
#define __CARD_SLOT__

#include "stdint.h"

// Slot size: 1K
// Page size: 4K

#define STORAGE_START_PAGE    	64
#define STORAGE_SLOT_NUM    		64

#define STORAGE_START_ADDR 		4 * 1024 * STORAGE_START_PAGE
#define STORAGE_STOP_ADDR 		(STORAGE_START_ADDR + STORAGE_SLOT_NUM * 1024 - 1)


uint32_t card_slot_begin(void);
uint32_t card_slot_write(uint16_t slot_num, const uint8_t * src, uint16_t len);
uint32_t card_slot_read(uint16_t slot_num, uint8_t * dest, uint16_t len);

#endif
