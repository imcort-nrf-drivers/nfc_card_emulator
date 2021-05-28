#include "card_emu.h"

#include "hal_nfc_t2t.h"
#include "nrf_log.h"
#include "nrf_delay.h"

#define NFC_CMD_READ           0x30
#define NFC_CMD_WRITE          0xA2
#define NFC_CMD_GET_VERSION    0x60
#define NFC_CMD_READ_SIG       0x3c
#define NFC_CMD_PWD_AUTH       0x1B
#define NFC_CMD_FAST_READ      0x3A

static uint8_t ntag215_memory[135 * 4] = {
	
	0x04,0xe8,0x92,0xf6,0x5a,0x57,0x49,0x80,0xc4,0x48,0x0f,0xe0,0xf1,0x10,0xff,0xee,0xa5,0x00,0x00,0x00,
  0xea,0xc4,0x91,0x3e,0x01,0xa3,0xdd,0xf8,0x53,0xae,0xe0,0xbf,0x52,0xfe,0x32,0x31,0x3a,0x15,0x1f,0xd6,
  0x68,0xb1,0x2c,0xd0,0xa7,0x10,0x62,0xed,0x24,0x22,0x41,0x4c,0xe9,0xe9,0x1a,0xd0,0x3b,0x86,0x65,0x0f,
  0xdc,0x9c,0xa7,0xa1,0xa7,0x1f,0xc5,0x8d,0x37,0xe7,0x3e,0x78,0x5e,0x48,0x08,0x75,0x38,0xbd,0xe4,0x3b,
  0x79,0x6c,0x73,0x9f,0x01,0x81,0x00,0x01,0x00,0x44,0x05,0x02,0x30,0x30,0x31,0x30,0xa8,0x42,0x4d,0x2d,
  0x8f,0x5d,0xd0,0x04,0x86,0xb4,0x54,0x3c,0xc5,0xf8,0x4d,0x1e,0xc4,0x5a,0xe2,0x6e,0x1b,0x77,0x5c,0xcd,
  0xe5,0x19,0x99,0xaa,0xc3,0x23,0x0e,0x95,0xe2,0x86,0xc6,0x2b,0x2d,0xba,0x73,0xc5,0x39,0xb2,0x0f,0x15,
  0x86,0x4b,0x8d,0xcf,0xc7,0x2d,0x5f,0xb0,0x49,0x95,0xc1,0xef,0xab,0x84,0xb2,0xcb,0x8f,0x20,0xe1,0x4d,
  0xec,0x0e,0xca,0x62,0x29,0x84,0x4d,0x73,0xf3,0x74,0x90,0x1e,0x12,0x70,0x6f,0xe7,0x02,0x32,0x68,0xc4,
  0xbb,0xae,0x35,0xdc,0x13,0xad,0xda,0x7a,0xa7,0x7c,0x01,0x1c,0x32,0x6c,0x5f,0x4f,0x64,0x05,0xcf,0xc4,
  0x11,0xf5,0x90,0xff,0xa2,0x82,0x44,0xf8,0x8e,0xae,0x04,0x72,0x2d,0xda,0x72,0x0e,0x1e,0xeb,0x68,0xe7,
  0x5a,0xd6,0xd4,0x0b,0x26,0xe6,0xc1,0xad,0xc6,0xb0,0x4e,0xcb,0xbd,0x52,0x5e,0x70,0x1f,0x7d,0xf3,0x42,
  0x56,0x1e,0xca,0x9c,0x52,0xcb,0xbe,0x40,0x64,0x31,0x5d,0x7c,0x25,0xc2,0xd3,0xf6,0x07,0x31,0x83,0xc3,
  0x9c,0x65,0xf8,0x6e,0xd6,0xa4,0x65,0x59,0x81,0xc0,0x6e,0x6c,0x95,0xa5,0x0e,0xd2,0xa3,0xc3,0x82,0xc2,
  0x99,0xf5,0x45,0xe2,0xb3,0x49,0x71,0xd1,0xd6,0x66,0xda,0x28,0xbf,0x58,0x82,0xf0,0x6f,0x6c,0x89,0x4a,
  0xde,0xd4,0x64,0x49,0xd7,0x3b,0x31,0x5a,0x95,0x4e,0x8e,0x1c,0x7b,0x5b,0x7a,0xec,0xd0,0xdd,0xca,0x00,
  0x6d,0x6d,0x51,0x2c,0x68,0xf4,0x4f,0x16,0x77,0x78,0xcd,0x80,0xd0,0xb2,0x42,0x75,0x1e,0x59,0x9b,0xb3,
  0xd8,0x75,0xa6,0x9b,0xdd,0x0a,0x46,0xcc,0x98,0x57,0xfa,0xbc,0x5e,0x6b,0xe7,0x45,0x4e,0x19,0xee,0x58,
  0x1a,0xc2,0x20,0xa8,0x42,0x21,0x7b,0xa3,0x39,0x92,0x05,0x11,0xbd,0x8c,0x7e,0x45,0x49,0xb3,0x50,0xde,
  0x76,0x0f,0x3e,0xc9,0x97,0x26,0x9c,0x6a,0x1d,0x76,0x27,0x61,0xa6,0xcb,0xf2,0x2b,0xb5,0x6e,0x03,0x93,
  0x6d,0x90,0x9c,0xae,0xae,0x1b,0xa9,0xc5,0xa5,0x6d,0xe5,0xf9,0xec,0xee,0x8d,0x35,0x11,0x44,0x8f,0x15,
  0xe5,0x2e,0x9a,0x1e,0xca,0x0b,0xdb,0xa7,0x53,0x65,0x93,0x05,0xa1,0xf8,0x26,0x2e,0x96,0x90,0xd7,0x9c,
  0xed,0xd0,0xca,0xf0,0xfc,0x86,0x67,0x42,0x59,0x6c,0xf8,0x25,0xfc,0x59,0xf3,0x42,0xb9,0x46,0x19,0x00,
  0x3a,0x66,0xc4,0xc1,0x15,0x04,0xe1,0x2e,0x04,0xdd,0xcf,0x48,0xf5,0x85,0x30,0x4c,0xf2,0x86,0x03,0xd8,
  0xe6,0x03,0x09,0x39,0xed,0x85,0xa0,0xe5,0xa3,0x8d,0xa9,0xff,0x7c,0xc3,0xf9,0x46,0x2f,0xad,0x85,0xa7,
  0xd7,0x76,0x8c,0x67,0x4d,0xc3,0xe0,0x47,0xff,0x5c,0x79,0x1d,0x94,0x1b,0x26,0x7b,0xe3,0xbe,0x95,0xcc,
  0x01,0x00,0x0f,0xbd,0x00,0x00,0x00,0x04,0x5f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00

};

static uint8_t NTAG215_Version[8] = {
    0x00, 0x04, 0x04, 0x02, 0x01, 0x00, 0x11, 0x03
};

static uint8_t NTAG215_Signature[32] = {
    0xdd, 0xd0, 0xa9, 0x67,
};

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
				NRF_LOG_INFO("NFC Write Block %d", block_num);
				if (data_length == 6) 
				{
						for (int i=0;i<4;i++)
						{
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
				NRF_LOG_INFO("NFC Fast Read %d to %d", block_num, p_data[2]);
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
						NRF_LOG_INFO("NFC Command Received: %x", p_data[0]);
						nfc_received_process(p_data, data_length);
            break;
				
				case HAL_NFC_EVENT_DATA_TRANSMITTED:
						NRF_LOG_INFO("NFC EVENT Data Transmitted");
						break;
				
        default:
						NRF_LOG_INFO("NFC EVENT %d, CMD %x", event, p_data[0]);
            break;
    }
		
}

void card_emu_begin(void)
{
		uint32_t  err_code;

    /* Set up NFC */
    err_code = hal_nfc_setup(nfc_callback, NULL);
    APP_ERROR_CHECK(err_code);
	
		err_code = hal_nfc_start();
		APP_ERROR_CHECK(err_code);
	
		//uint8_t uid[8] = {0xc0,0x58,0xc8,0xb5};
	
		hal_nfc_parameter_set(HAL_NFC_PARAM_ID_NFCID1, ntag215_memory, 7);

}
