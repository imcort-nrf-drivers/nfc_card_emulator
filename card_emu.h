#ifndef __CARD_EMU_H
#define __CARD_EMU_H

#include "stdint.h"

void card_emu_begin(void);
void ntag215_card_change(uint8_t slot);
void ntag215_current_slot_init(void);

#endif
