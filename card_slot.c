#include "card_slot.h"

#include "app_error.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_nvmc.h"

#include "nrf_log.h"

static volatile bool flash_is_busy = false;
static uint8_t page_buffer[4096];
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = STORAGE_START_ADDR,
    .end_addr   = STORAGE_STOP_ADDR,
};

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
						flash_is_busy = false;
        } break;

        default:
            break;
    }
}

uint32_t card_slot_begin(void)
{
		uint32_t  err_code;
	
		/* Set up FSTORAGE */
		nrf_fstorage_api_t *p_fs_api;
		p_fs_api = &nrf_fstorage_nvmc;
		err_code = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
	
		return err_code;

}

uint32_t card_slot_write(uint16_t slot_num, const uint8_t * src, uint16_t len)
{
		uint32_t  err_code;
	
		if(slot_num > STORAGE_SLOT_NUM)
		{
				NRF_LOG_INFO("Slot number %d not exist", slot_num);
				return false;
		}
	
		if(len > 1024)
		{
				NRF_LOG_INFO("Slot length %d too long", len);
				return false;
		}
		
	  //Find this slot in which page
		uint32_t page_addr = STORAGE_START_ADDR + (slot_num / 4) * 4096;
		NRF_LOG_INFO("Writing slot number %d in page address 0x%x", slot_num, page_addr);
	
		//Read entire page to buffer
		err_code = nrf_fstorage_read(&fstorage, page_addr, page_buffer, 4096);
		APP_ERROR_CHECK(err_code);
		
		//Erase entire page
		flash_is_busy = true;
		err_code = nrf_fstorage_erase(&fstorage, page_addr, 1, NULL);
		APP_ERROR_CHECK(err_code);
		
		while(flash_is_busy) __WFE();
		
		//Writing this slot into buffer
		memcpy(page_buffer + (slot_num % 4) * 1024, src, len);
	
		//Writing buffer into page
		err_code = nrf_fstorage_write(&fstorage, page_addr, page_buffer, 4096, NULL);
		
		return err_code;

}

uint32_t card_slot_read(uint16_t slot_num, uint8_t * dest, uint16_t len)
{
		uint32_t  err_code;
	
		if(slot_num > STORAGE_SLOT_NUM)
		{
				NRF_LOG_INFO("Slot number %d not exist", slot_num);
				return false;
		}
	
		if(len > 1024)
		{
				NRF_LOG_INFO("Slot length %d too long", len);
				return false;
		}
		
		//Read slot to dest
		uint32_t slot_addr = STORAGE_START_ADDR + slot_num * 1024;
		err_code = nrf_fstorage_read(&fstorage, slot_addr, dest, len);
		
		return err_code;

}
