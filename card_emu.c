#include "card_emu.h"

#include "hal_nfc_t2t.h"
#include "nrf_log.h"
#include "nrf_delay.h"

#include "ntag215_keys.h"

#include "nrf_fstorage.h"
#include "nrf_fstorage_nvmc.h"

#define NFC_CMD_READ           0x30
#define NFC_CMD_WRITE          0xA2
#define NFC_CMD_GET_VERSION    0x60
#define NFC_CMD_READ_SIG       0x3c
#define NFC_CMD_PWD_AUTH       0x1B
#define NFC_CMD_FAST_READ      0x3A

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
        } break;

        default:
            break;
    }
}

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0x3e000,
    .end_addr   = 0x3ffff,
};

static uint8_t ntag215_memory[135 * 4];

static uint8_t NTAG215_Version[8] = {
    0x00, 0x04, 0x04, 0x02, 0x01, 0x00, 0x11, 0x03
};

static uint8_t NTAG215_Signature[32];

static uint8_t NTAG215_PwdOK[2] = {
    0x80, 0x80, 
};

static void nfc_received_process(const uint8_t * p_data, size_t data_length)
{
		//NRF_LOG_INFO("NFC COMMAND %d", p_data[0]);
		
		uint8_t command = p_data[0];
		uint8_t block_num = p_data[1];
	
		//NRF_LOG_DEBUG("")
		
		switch (command)
		{
			case NFC_CMD_READ:
				NRF_LOG_INFO("NFC Read Block %d", block_num);
				if (block_num < 135)
					hal_nfc_send(&ntag215_memory[block_num*4], 16);
				else
					hal_send_ack_nack(0x0);
				break;
			case NFC_CMD_WRITE:
				NRF_LOG_INFO("NFC Write Block %d: %x, %x, %x, %x", block_num, p_data[2], p_data[3], p_data[4], p_data[5]);
				if (data_length == 6) 
				{
						for (int i=0;i<4;i++)
						{
								if(block_num == 133)
									continue;
								if( (block_num*4+i > 9)  )
									ntag215_memory[block_num*4+i] = p_data[i + 2];
						}
						hal_send_ack_nack(0xA);
				} else {
				
					hal_send_ack_nack(0x0);
					
				}
				break;
			case NFC_CMD_GET_VERSION:
				NRF_LOG_INFO("NFC Get Version");
				hal_nfc_send(NTAG215_Version, 8);
				break;
			
			case NFC_CMD_READ_SIG:
				NRF_LOG_INFO("NFC Read Signature");
				hal_nfc_send(NTAG215_Signature, 32);
				break;
			
			case NFC_CMD_FAST_READ:
				NRF_LOG_INFO("NFC Fast Read %x to %x", block_num, p_data[2]);
				hal_nfc_send(&ntag215_memory[block_num*4], (p_data[2] - block_num + 1)*4);
				break;
			
			case NFC_CMD_PWD_AUTH:
				NRF_LOG_INFO("NFC Password: %x %x %x %x", p_data[1], p_data[2], p_data[3], p_data[4]);
				hal_nfc_send(NTAG215_PwdOK, 2); //Always accept password
				break;
			
			default:
				NRF_LOG_INFO("NFC CMD %x", p_data[0]);
				break;	
		
		}

}

static void nfc_callback(void * p_context, hal_nfc_event_t event, const uint8_t * p_data, size_t data_length)
{
    (void)p_context;
	
    switch (event)
    {
        case HAL_NFC_EVENT_FIELD_ON:
						NRF_LOG_INFO("NFC EVENT ON");
						//drv_oled_on();
            break;
        case HAL_NFC_EVENT_FIELD_OFF:
						NRF_LOG_INFO("NFC EVENT OFF");
            break;
				case HAL_NFC_EVENT_COMMAND:
						//NRF_LOG_INFO("NFC Command Received: %x", p_data[0]);
						nfc_received_process(p_data, data_length);
            break;
				
				case HAL_NFC_EVENT_DATA_TRANSMITTED:
						//NRF_LOG_INFO("NFC EVENT Data Transmitted");
						break;
				
        default:
						NRF_LOG_INFO("Unknown NFC EVENT %d, CMD %x", event, p_data[0]);
            break;
    }
		
}

void ntag215_memory_format(uint8_t* memory, uint8_t* uid)
{
		uint8_t current_sum;
	
		memset(memory, 0, 135 * 4);
	
		// Page 00h
		memory[0 * 4 + 0] = uid[0];
		memory[0 * 4 + 1] = uid[1];
		memory[0 * 4 + 2] = uid[2];
		current_sum = 0x88 ^ uid[0] ^ uid[1] ^ uid[2];
		memory[0 * 4 + 3] = current_sum;
	
		// Page 01h
		memory[1 * 4 + 0] = uid[3];
		memory[1 * 4 + 1] = uid[4];
		memory[1 * 4 + 2] = uid[5];
		memory[1 * 4 + 3] = uid[6];
		current_sum = uid[3] ^ uid[4] ^ uid[5] ^ uid[6];
	
		// Page 02h
		memory[2 * 4 + 0] = current_sum;
		memory[2 * 4 + 1] = 0x48;
	
		// Page 03h
		memory[3 * 4 + 0] = 0xe1;
		memory[3 * 4 + 1] = 0x10;
		memory[3 * 4 + 2] = 0x3e;
		// Page 04h
		memory[4 * 4 + 0] = 0x03;
		memory[4 * 4 + 2] = 0xfe;
		// Page 83h (131)
		memory[131 * 4 + 0] = 0x04;
		memory[131 * 4 + 3] = 0xff;

}

void card_emu_begin(void)
{
		uint32_t  err_code;

    /* Set up NFC */
    err_code = hal_nfc_setup(nfc_callback, NULL);
    APP_ERROR_CHECK(err_code);
	
		err_code = hal_nfc_start();
		APP_ERROR_CHECK(err_code);
	
		uint8_t uid[7];
		memcpy(uid, NTAG215_UID_Signature + 0, 7);
		ntag215_memory_format(ntag215_memory, uid);
		hal_nfc_parameter_set(HAL_NFC_PARAM_ID_NFCID1, uid, 7);
	
		memcpy(NTAG215_Signature, NTAG215_UID_Signature + 7, 32);
	
		NRF_LOG_HEXDUMP_INFO(NTAG215_Signature, 32);

}
