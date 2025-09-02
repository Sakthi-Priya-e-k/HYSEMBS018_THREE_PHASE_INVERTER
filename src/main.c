#include "at32f403a_407_board.h"
#include "at32f403a_407_clock.h"
#include "at32_emac.h"
#include "netconf.h"
#include "tcp_server.h"
#include "string.h"
#include "lwip/tcp.h"
#include "lwip/opt.h"
#include <stdint.h>
#include <stdbool.h>
#include "spi_flash.h"
#include "ip_addr.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

/**
 **************************************************************************
 * @file     spi_flash.c
 * @brief    spi_flash source code
 **************************************************************************
 *                       Copyright notice & Disclaimer
 *
 * The software Board Support Package (BSP) that is made available to
 * download from Artery official website is the copyrighted work of Artery.
 * Artery authorizes customers to use, copy, and distribute the BSP
 * software and its related documentation for the purpose of design and
 * development in conjunction with Artery microcontrollers. Use of the
 * software is governed by this copyright notice and the following disclaimer.
 *
 * THIS SOFTWARE IS PROVIDED ON "AS IS" BASIS WITHOUT WARRANTIES,
 * GUARANTEES OR REPRESENTATIONS OF ANY KIND. ARTERY EXPRESSLY DISCLAIMS,
 * TO THE FULLEST EXTENT PERMITTED BY LAW, ALL EXPRESS, IMPLIED OR
 * STATUTORY OR OTHER WARRANTIES, GUARANTEES OR REPRESENTATIONS,
 * INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE, OR NON-INFRINGEMENT.
 *
 **************************************************************************
 */

/** @addtogroup AT32F407_periph_examples
 * @{
 */

/** @addtogroup 407_SPI_w25q_flash
 * @{
 */
uint8_t spiflash_sector_buf[SPIF_SECTOR_SIZE];

//RMS MACROS
#define NUM_SAMPLES 100  // Number of samples for RMS calculation
#define V_REF 3.3        // Reference voltage in volts
#define ADC_RES 4095     // 12-bit ADC resolution

#define DELAY                            100
#define FAST                             1
#define SLOW                             4

#define NUM_VALUES 6  // Number of values to receive
#define BUFFER_SIZE 100  // Define the maximum size of the string buffer

char receive_buffer[BUFFER_SIZE];  // Buffer to store the received string
uint8_t buffer_index = 0;           // Index to track the buffer position
uint8_t values_received = 0;        // Counter for the number of values received
int values[NUM_VALUES];             // Array to store the 6 values
int user_number = 0;                // Variable to store received number

uint8_t g_speed = FAST;
volatile uint32_t local_time = 0;
__IO uint32_t systick_index = 0;

// The values should be integers (uint8_t) as expected by Arduino
uint8_t start_marker = 255;   // Always use 255 as the start marker
uint8_t frequency = 50;       // Value either 50 or 60
uint8_t phase1_amp = 50;      // Value between 0-100
uint8_t phase2_amp = 25;      // Value between 0-100
uint8_t phase3_amp = 75;      // Value between 0-100

/**
 * @brief  spi configuration.
 * @param  none
 * @retval none
 */
void spiflash_init(void) {
	gpio_init_type gpio_initstructure;
	spi_init_type spi_init_struct;

	crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);

	/* software cs, pe12 as a general io to control flash cs */
	gpio_initstructure.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_initstructure.gpio_pull = GPIO_PULL_UP;
	gpio_initstructure.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_initstructure.gpio_pins = GPIO_PINS_4;
	gpio_init(GPIOE, &gpio_initstructure);

	/* configure the SCK pin */
	gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
	gpio_initstructure.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_initstructure.gpio_mode = GPIO_MODE_MUX;
	gpio_initstructure.gpio_pins = GPIO_PINS_2;
	gpio_initstructure.gpio_pull = GPIO_PULL_NONE;
	gpio_init(GPIOE, &gpio_initstructure);

	/* configure the MISO pin */
	gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
	gpio_initstructure.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_initstructure.gpio_mode = GPIO_MODE_INPUT;
	gpio_initstructure.gpio_pins = GPIO_PINS_5;
	gpio_initstructure.gpio_pull = GPIO_PULL_NONE;
	gpio_init(GPIOE, &gpio_initstructure);

	/* configure the MOSI pin */
	gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
	gpio_initstructure.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_initstructure.gpio_mode = GPIO_MODE_MUX;
	gpio_initstructure.gpio_pins = GPIO_PINS_6;
	gpio_initstructure.gpio_pull = GPIO_PULL_NONE;
	gpio_init(GPIOE, &gpio_initstructure);

	FLASH_CS_HIGH();
	crm_periph_clock_enable(CRM_SPI4_PERIPH_CLOCK, TRUE);
	spi_default_para_init(&spi_init_struct);
	spi_init_struct.transmission_mode = SPI_TRANSMIT_FULL_DUPLEX;
	spi_init_struct.master_slave_mode = SPI_MODE_MASTER;
	spi_init_struct.mclk_freq_division = SPI_MCLK_DIV_8;
	spi_init_struct.first_bit_transmission = SPI_FIRST_BIT_MSB;
	spi_init_struct.frame_bit_num = SPI_FRAME_8BIT;
	spi_init_struct.clock_polarity = SPI_CLOCK_POLARITY_HIGH;
	spi_init_struct.clock_phase = SPI_CLOCK_PHASE_2EDGE;
	spi_init_struct.cs_mode_selection = SPI_CS_SOFTWARE_MODE;
	spi_init(SPI4, &spi_init_struct);

	spi_enable(SPI4, TRUE);
}

/**
 * @brief  write data to flash
 * @param  pbuffer: the pointer for data buffer
 * @param  write_addr: the address where the data is written
 * @param  length: buffer length
 * @retval none
 */
void spiflash_write(uint8_t *pbuffer, uint32_t write_addr, uint32_t length) {
	uint32_t sector_pos;
	uint16_t sector_offset;
	uint16_t sector_remain;
	uint16_t index;
	uint8_t *spiflash_buf;
	spiflash_buf = spiflash_sector_buf;

	/* sector address */
	sector_pos = write_addr / SPIF_SECTOR_SIZE;

	/* address offset in a sector */
	sector_offset = write_addr % SPIF_SECTOR_SIZE;

	/* the remain in a sector */
	sector_remain = SPIF_SECTOR_SIZE - sector_offset;
	if (length <= sector_remain) {
		/* smaller than a sector size */
		sector_remain = length;
	}

//printf("Starting write operation...\r\n");
//printf("Initial write address: 0x%X, length: %d\r\n", write_addr, length);

	while (1) {
		/* read a sector */
//printf("Reading sector at address: 0x%X\r\n", sector_pos * SPIF_SECTOR_SIZE);
		spiflash_read(spiflash_buf, sector_pos * SPIF_SECTOR_SIZE,
		SPIF_SECTOR_SIZE);

//printf("Sector read complete. First 16 bytes of buffer: ");
		for (index = 0; index < 16; index++)
			printf("0x%X ", spiflash_buf[index]);
//printf("\r\n");

		/* validate the read erea */
		for (index = 0; index < sector_remain; index++) {
			if (spiflash_buf[sector_offset + index] != 0xFF) {
				/* there are some data not equal 0xff, so this secotr needs erased */
//printf("Sector not erased at offset: %d\r\n", sector_offset + index);
				break;
			}
		}
		if (index < sector_remain) {
			/* erase the sector */
//printf("Erasing sector at address: 0x%X\r\n", sector_pos * SPIF_SECTOR_SIZE);
			spiflash_sector_erase(sector_pos);

			/* copy the write data */
//printf("Copying data to sector buffer...\r\n");
			for (index = 0; index < sector_remain; index++) {
				spiflash_buf[index + sector_offset] = pbuffer[index];
			}

//printf("Writing sector to address: 0x%X\r\n", sector_pos * SPIF_SECTOR_SIZE);
			spiflash_write_nocheck(spiflash_buf, sector_pos * SPIF_SECTOR_SIZE,
			SPIF_SECTOR_SIZE); /* program the sector */
//printf("Sector write complete.\r\n");
		} else {
			/* write directly in the erased area */
//printf("Writing directly to erased area: address 0x%X, length %d\r\n", write_addr, sector_remain);
			spiflash_write_nocheck(pbuffer, write_addr, sector_remain);
//printf("Direct write complete.\r\n");
		}
		if (length == sector_remain) {
			/* write end */
//printf("Write operation completed successfully.\r\n");
			break;
		} else {
			/* go on writing */
//printf("Continuing to the next sector...\r\n");
			sector_pos++;
			sector_offset = 0;

			pbuffer += sector_remain;
			write_addr += sector_remain;
			length -= sector_remain;
			if (length > SPIF_SECTOR_SIZE) {
				/* could not write the remain data in the next sector */
				sector_remain = SPIF_SECTOR_SIZE;
			} else {
				/* could write the remain data in the next sector */
				sector_remain = length;
			}
		}
	}
//printf("Exiting spiflash_write function.\r\n");
}

/**
 * @brief  read data from flash
 * @param  pbuffer: the pointer for data buffer
 * @param  read_addr: the address where the data is read
 * @param  length: buffer length
 * @retval none
 */
void spiflash_read(uint8_t *pbuffer, uint32_t read_addr, uint32_t length) {
//printf("spiflash_read: Starting read operation...\r\n");
//printf("Read address: 0x%X, Length: %d\r\n", read_addr, length);
	FLASH_CS_LOW();
//printf("CS pulled LOW.\r\n");
	spi_byte_write(SPIF_READDATA); /* send instruction */
//printf("Sent READ command: 0x%X\r\n", SPIF_READDATA);
	spi_byte_write((uint8_t) ((read_addr) >> 16)); /* send 24-bit address */
	spi_byte_write((uint8_t) ((read_addr) >> 8));
	spi_byte_write((uint8_t) read_addr);
//printf("Sent 24-bit address: 0x%X\r\n", read_addr);

// Read data into buffer
//printf("Reading %d bytes...\r\n", length);
	spi_bytes_read(pbuffer, length);
	for (uint32_t i = 0; i < length; i++) {
		if (i < 16) // Limit debug output to first 16 bytes for readability
				{
//printf("Byte[%d]: 0x%X\r\n", i, pbuffer[i]);
		}
	}
	FLASH_CS_HIGH();
//printf("CS pulled HIGH. Read operation complete.\r\n");
}

/**
 * @brief  erase a sector data
 * @param  erase_addr: sector address to erase
 * @retval none
 */
void spiflash_sector_erase(uint32_t erase_addr) {
	erase_addr *= SPIF_SECTOR_SIZE; /* translate sector address to byte address */
	spiflash_write_enable();
	spiflash_wait_busy();
	FLASH_CS_LOW();
	spi_byte_write(SPIF_SECTORERASE);
	spi_byte_write((uint8_t) ((erase_addr) >> 16));
	spi_byte_write((uint8_t) ((erase_addr) >> 8));
	spi_byte_write((uint8_t) erase_addr);
	FLASH_CS_HIGH();
	spiflash_wait_busy();
}

/**
 * @brief  write data without check
 * @param  pbuffer: the pointer for data buffer
 * @param  write_addr: the address where the data is written
 * @param  length: buffer length
 * @retval none
 */
void spiflash_write_nocheck(uint8_t *pbuffer, uint32_t write_addr,
		uint32_t length) {
	uint16_t page_remain;

	/* remain bytes in a page */
	page_remain = SPIF_PAGE_SIZE - write_addr % SPIF_PAGE_SIZE;
	if (length <= page_remain) {
		/* smaller than a page size */
		page_remain = length;
	}
	while (1) {
		spiflash_page_write(pbuffer, write_addr, page_remain);
		if (length == page_remain) {
			/* all data are programmed */
			break;
		} else {
			/* length > page_remain */
			pbuffer += page_remain;
			write_addr += page_remain;

			/* the remain bytes to be programmed */
			length -= page_remain;
			if (length > SPIF_PAGE_SIZE) {
				/* can be programmed a page at a time */
				page_remain = SPIF_PAGE_SIZE;
			} else {
				/* smaller than a page size */
				page_remain = length;
			}
		}
	}
}

/**
 * @brief  write a page data
 * @param  pbuffer: the pointer for data buffer
 * @param  write_addr: the address where the data is written
 * @param  length: buffer length
 * @retval none
 */
void spiflash_page_write(uint8_t *pbuffer, uint32_t write_addr, uint32_t length) {
	if ((0 < length) && (length <= SPIF_PAGE_SIZE)) {
		/* set write enable */
		spiflash_write_enable();

		FLASH_CS_LOW();

		/* send instruction */
		spi_byte_write(SPIF_PAGEPROGRAM);

		/* send 24-bit address */
		spi_byte_write((uint8_t) ((write_addr) >> 16));
		spi_byte_write((uint8_t) ((write_addr) >> 8));
		spi_byte_write((uint8_t) write_addr);
		spi_bytes_write(pbuffer, length);

		FLASH_CS_HIGH();

		/* wait for program end */
		spiflash_wait_busy();
	}
}

/**
 * @brief  write data continuously
 * @param  pbuffer: the pointer for data buffer
 * @param  length: buffer length
 * @retval none
 */
void spi_bytes_write(uint8_t *pbuffer, uint32_t length) {
	volatile uint8_t dummy_data;

#if defined(SPI_TRANS_DMA)
  dma_init_type dma_init_struct;
  dma_reset(DMA2_CHANNEL3);
  dma_reset(DMA2_CHANNEL4);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = length;
  dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  dma_init_struct.memory_base_addr = (uint32_t)&dummy_data;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  dma_init_struct.memory_inc_enable = FALSE;
  dma_init_struct.peripheral_base_addr = (uint32_t)(&SPI4->dt);
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_VERY_HIGH;
  dma_init_struct.loop_mode_enable = FALSE;
  dma_init(DMA2_CHANNEL3, &dma_init_struct);

  dma_init_struct.buffer_size = length;
  dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  dma_init_struct.memory_base_addr = (uint32_t)pbuffer;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)(&SPI4->dt);
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_VERY_HIGH;
  dma_init_struct.loop_mode_enable = FALSE;
  dma_init(DMA2_CHANNEL4, &dma_init_struct);

  spi_i2s_dma_transmitter_enable(SPI4, TRUE);
  spi_i2s_dma_receiver_enable(SPI4, TRUE);

  dma_channel_enable(DMA2_CHANNEL3, TRUE);
  dma_channel_enable(DMA2_CHANNEL4, TRUE);

  while(dma_flag_get(DMA2_FDT3_FLAG) == RESET);
  dma_flag_clear(DMA2_FDT3_FLAG);

  dma_channel_enable(DMA2_CHANNEL3, FALSE);
  dma_channel_enable(DMA2_CHANNEL4, FALSE);

  spi_i2s_dma_transmitter_enable(SPI4, FALSE);
  spi_i2s_dma_receiver_enable(SPI4, FALSE);
#else
	while (length--) {
		while (spi_i2s_flag_get(SPI4, SPI_I2S_TDBE_FLAG) == RESET)
			;
		spi_i2s_data_transmit(SPI4, *pbuffer);
		while (spi_i2s_flag_get(SPI4, SPI_I2S_RDBF_FLAG) == RESET)
			;
		dummy_data = spi_i2s_data_receive(SPI4);
		pbuffer++;
	}
#endif
}

/**
 * @brief  read data continuously
 * @param  pbuffer: buffer to save data
 * @param  length: buffer length
 * @retval none
 */
void spi_bytes_read(uint8_t *pbuffer, uint32_t length) {
	uint8_t write_value = FLASH_SPI_DUMMY_BYTE;

	printf("spi_bytes_read: Starting read operation for %d bytes...\r\n",
			length);

#if defined(SPI_TRANS_DMA)
  printf("Using DMA for SPI read.\r\n");
  dma_init_type dma_init_struct;
  dma_reset(DMA2_CHANNEL3);
  dma_reset(DMA2_CHANNEL4);
  dma_default_para_init(&dma_init_struct);
  dma_init_struct.buffer_size = length;
  dma_init_struct.direction = DMA_DIR_MEMORY_TO_PERIPHERAL;
  dma_init_struct.memory_base_addr = (uint32_t)&write_value;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  dma_init_struct.memory_inc_enable = FALSE;
  dma_init_struct.peripheral_base_addr = (uint32_t)(&SPI4->dt);
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_VERY_HIGH;
  dma_init_struct.loop_mode_enable = FALSE;
  dma_init(DMA2_CHANNEL3, &dma_init_struct);

  dma_init_struct.buffer_size = length;
  dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
  dma_init_struct.memory_base_addr = (uint32_t)pbuffer;
  dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_BYTE;
  dma_init_struct.memory_inc_enable = TRUE;
  dma_init_struct.peripheral_base_addr = (uint32_t)(&SPI4->dt);
  dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_BYTE;
  dma_init_struct.peripheral_inc_enable = FALSE;
  dma_init_struct.priority = DMA_PRIORITY_VERY_HIGH;
  dma_init_struct.loop_mode_enable = FALSE;
  dma_init(DMA2_CHANNEL4, &dma_init_struct);

  printf("Starting DMA transfer...\r\n");
  spi_i2s_dma_transmitter_enable(SPI4, TRUE);
  spi_i2s_dma_receiver_enable(SPI4, TRUE);
  dma_channel_enable(DMA2_CHANNEL3, TRUE);
  dma_channel_enable(DMA2_CHANNEL4, TRUE);

  while(dma_flag_get(DMA2_FDT4_FLAG) == RESET)
  {
 printf("Waiting for DMA transfer to complete...\r\n");
  }
  dma_flag_clear(DMA2_FDT4_FLAG);

  dma_channel_enable(DMA2_CHANNEL3, FALSE);
  dma_channel_enable(DMA2_CHANNEL4, FALSE);

  spi_i2s_dma_transmitter_enable(SPI4, FALSE);
  spi_i2s_dma_receiver_enable(SPI4, FALSE);

  printf("DMA transfer complete.\r\n");
#else
//printf("Using polling for SPI read.\r\n");
	while (length--) {
//printf("Sending dummy byte: 0x%X\r\n", write_value);
		while (spi_i2s_flag_get(SPI4, SPI_I2S_TDBE_FLAG) == RESET)
			;
		spi_i2s_data_transmit(SPI4, write_value);
		while (spi_i2s_flag_get(SPI4, SPI_I2S_RDBF_FLAG) == RESET)
			;
		*pbuffer = spi_i2s_data_receive(SPI4);
		pbuffer++;
	}
//printf("Polling-based SPI read complete.\r\n");
#endif
}

/**
 * @brief  wait program done
 * @param  none
 * @retval none
 */
void spiflash_wait_busy(void) {
	while ((spiflash_read_sr1() & 0x01) == 0x01)
		;
}

/**
 * @brief  read sr1 register
 * @param  none
 * @retval none
 */
uint8_t spiflash_read_sr1(void) {
	uint8_t breadbyte = 0;
	FLASH_CS_LOW();
	spi_byte_write(SPIF_READSTATUSREG1);
	breadbyte = (uint8_t) spi_byte_read();
	FLASH_CS_HIGH();
	return (breadbyte);
}

/**
 * @brief  enable write operation
 * @param  none
 * @retval none
 */
void spiflash_write_enable(void) {
	FLASH_CS_LOW();
	spi_byte_write(SPIF_WRITEENABLE);
	FLASH_CS_HIGH();
}

/**
 * @brief  read device id
 * @param  none
 * @retval device id
 */
uint16_t spiflash_read_id(void) {
	uint16_t wreceivedata = 0;
	FLASH_CS_LOW();
	spi_byte_write(SPIF_MANUFACTDEVICEID);
	spi_byte_write(0x00);
	spi_byte_write(0x00);
	spi_byte_write(0x00);
	wreceivedata |= spi_byte_read() << 8;
	wreceivedata |= spi_byte_read();
	FLASH_CS_HIGH();
	return wreceivedata;
}

/**
 * @brief  write a byte to flash
 * @param  data: data to write
 * @retval flash return data
 */
uint8_t spi_byte_write(uint8_t data) {
	uint8_t brxbuff;
	spi_i2s_dma_transmitter_enable(SPI4, FALSE);
	spi_i2s_dma_receiver_enable(SPI4, FALSE);
	spi_i2s_data_transmit(SPI4, data);
	while (spi_i2s_flag_get(SPI4, SPI_I2S_RDBF_FLAG) == RESET)
		;
	brxbuff = spi_i2s_data_receive(SPI4);
	while (spi_i2s_flag_get(SPI4, SPI_I2S_BF_FLAG) != RESET)
		;
	return brxbuff;
}

/**
 * @brief  read a byte to flash
 * @param  none
 * @retval flash return data
 */
uint8_t spi_byte_read(void) {
	return (spi_byte_write(FLASH_SPI_DUMMY_BYTE));
}

/**
 * @}
 */

/**
 * @}
 */

// Define LCD pin connections
#define LCD_RS GPIO_PINS_10   // PC10 -> Register Select
#define LCD_E  GPIO_PINS_11   // PC11 -> Enable
#define LCD_RW  GPIO_PINS_12   // PC12 -> Read/Write

//REVERSE POLARITY PINS
#define DCP0_PIN GPIO_PINS_13 //PE13 -> DCP0 PIN
#define DCP1_PIN GPIO_PINS_14 //PE14 -> DCP1 PIN

//SWITCH BUZZER PIN
#define SW_BUZZ_PIN GPIO_PINS_12 //PD12 -> SWITCH BUZZER PIN

//SURGE PROTECTION PINS
#define DCS_PIN GPIO_PINS_3 //PE3 -> DCS PIN
volatile uint8_t surge_flag = 0; //flag variable for dc input surge

volatile uint8_t adc_interrupt_enabled = 1; //flag variable to enable adc readings

//NEUTRAL DISCONNECTION PINS
#define NEUTRAL_PIN GPIO_PINS_1 //PE1 -> NEUTRAL DISCONNECTION PIN
volatile uint8_t neutral_disconnect_flag = 0; //flag variable for neutral disconnect

//OUTPUT SURGE PROTECTION PINS
#define ACS_PIN GPIO_PINS_0 //PE0 -> ACS PIN
volatile uint8_t output_surge_flag = 0; //flag variable for output ac surge

uint16_t reg_value; //modbus register

#define INITIAL_DUTY_CYCLE 0
#define DUTY_CYCLE_STEP   5
#define MAX_DUTY_CYCLE   65535
#define MIN_DUTY_CYCLE   0
#define TIMER_RESOLUTION   65535

//BUZZER PIN
#define BUZZER_PIN GPIO_PINS_11 //PE11 -> BUZZER PIN

//EMERGENCY SWITCH
#define EMER_SWITCH GPIO_PINS_12 //PE12 -> EMERGENCY SWITCH

//START SWITCH
#define START_SWITCH GPIO_PINS_15 //PE15 -> START SWITCH

// Add a global or static system state variable
static int system_state = 0; // 0: Normal, 1: Recovering from under-voltage, 2: Recovering from over-voltage

/* Global variable to track the last ADC update time */
volatile uint32_t adc_update_timer = 0;

// Add a variable to track the last time the tasks were executed
static uint32_t last_update_time = 0;

//RELAY PINS
#define RELAY_0 GPIO_PINS_7  //PE7 -> RELAY 0 PIN
#define RELAY_1 GPIO_PINS_8  //PE8 -> RELAY 1 PIN
#define RELAY_2 GPIO_PINS_9  //PE9 -> RELAY 2 PIN

// Global flag to track neutral disconnect state
volatile uint8_t neutral_disconnect_detected = 0;

//SPI FLASH VARIABLE
#define FLASH_TEST_ADDR                  0x1000
#define BUF_SIZE                         0x256

uint8_t tx_buffer[BUF_SIZE];  //SPI TX BUFFER VARIABLE
uint8_t rx_buffer[BUF_SIZE];  //SPI RX BUFFER VARIABLE
volatile error_status transfer_status = ERROR;  //SPI ERROR STATUS VARIABLE

/* Modbus Function Codes */
#define MODBUS_FUNC_READ_HOLDING_REGISTERS 0x03
#define MODBUS_FUNC_WRITE_SINGLE_REGISTER  0x06

/* Modbus TCP Registers and Addressing */
#define MODBUS_TCP_PORT                    502
#define MODBUS_BUFFER_SIZE                 256
#define MODBUS_HOLDING_REG_START           0x0000
#define MODBUS_FREQ_CONTROL_REGISTER       0x0001
#define MODBUS_REQD_VOLTAGE_SET_REGISTER   0x0002  // Register for setting required voltage
#define MODBUS_MAX_VOLTAGE     0x0003  // Register for setting maximum voltage
#define MODBUS_MIN_VOLTAGE                 0x0004  // Register for setting minimum voltage
//#define MODBUS_TIMER2_DUTY_REGISTER      0x0002  // Register for Timer 2 duty cycle
//#define MODBUS_TIMER5_DUTY_REGISTER      0x0003  // Register for Timer 5 duty cycle
#define CURRENT_TRANSFORMER_SELECT_RATIO   0x0005  // Register for current transformer
#define MAX_OUTPUT_CURRENT                 0x0006  // Register for setting maximum current
#define IGBT_FAN_ON_OFF	                   0x0007  // Register for setting maximum current
#define MODBUS_FREQ_50HZ                   50
#define MODBUS_FREQ_60HZ                   60
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION  0x01
#define MODBUS_EXCEPTION_ILLEGAL_ADDRESS   0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA      0x03
#define MODBUS_HOLDING_REG_COUNT           8
#define MODBUS_TRANSACTION_ID_HIGH_BYTE    0
#define MODBUS_TRANSACTION_ID_LOW_BYTE     1
#define MODBUS_PROTOCOL_ID_HIGH_BYTE       2
#define MODBUS_PROTOCOL_ID_LOW_BYTE        3
#define MODBUS_LENGTH_HIGH_BYTE            4
#define MODBUS_LENGTH_LOW_BYTE             5
#define MODBUS_UNIT_IDENTIFIER             6
#define MODBUS_FUNCTION_CODE               7
#define MODBUS_START_ADDR_HIGH_BYTE        8
#define MODBUS_START_ADDR_LOW_BYTE         9
#define MODBUS_REG_COUNT_HIGH_BYTE         10
#define MODBUS_REG_COUNT_LOW_BYTE          11
#define TCP_KEEPALIVE_INTERVAL             10000  // 10 seconds

struct tcp_pcb *modbus_pcb;

/* Modbus TCP frame structure */
typedef struct {
	uint16_t transaction_id;
	uint16_t protocol_id;
	uint16_t length;
	uint8_t unit_id;
	uint8_t function_code;
	uint8_t data[MODBUS_BUFFER_SIZE];
} modbus_tcp_frame_t;

//timer configuration variables
tmr_output_config_type tmr_oc_init_structure;
crm_clocks_freq_type crm_clocks_freq_struct = { 0 };

//timer variables
uint16_t ccr1_val = 333;
uint16_t ccr2_val = 333;
uint16_t ccr3_val = 333;
uint16_t ccr4_val = 333;

/* Add new variables for duty cycle control */
uint16_t timer2_duty = 0;  // Default 0% duty cycle
uint16_t timer5_duty = 0;  // Default 0% duty cycle

uint16_t current_transformer = 1; //ct choose ratio 1000, 500, 100

uint16_t max_current = 1000; //set max current variable
uint16_t min_current = 100;  //set min current variable

uint16_t igbt_fan = 1; //by default the igbt fan will be always on

uint16_t required_voltage = 0; //set required voltage
uint16_t maximum_voltage = 0;  //set maximum voltage
uint16_t minimum_voltage = 0;  //set minimum voltage

volatile uint16_t modbus_set_voltage = 240; // By default the required voltage value is set
volatile uint8_t modbus_value_updated = 0; // Flag to track if value is updated

volatile uint16_t modbus_min_voltage = 0; // By default maximum voltage is set
volatile uint16_t modbus_max_voltage = 480; // By default minimum voltage is set

//int frequency; //variable to set the required frequency
float pwm1_adjust, pwm2_adjust, pwm3_adjust; //duty cycle adjust based on the
//feedback voltage
float sinewave_frequency; //frequency to be send to the arduino
//to set sine wave frequency
int DC_INPUT_VOLTAGE;
uint16_t new_value; //mapped function variable
uint16_t adc_value[11] = { 0 }; //variable to store the adc readings
uint16_t mapped_value[11] = { 0 }; //variable to map and store the adc values

char buffer[64]; //Buffer to hold the dc input voltage to display on LCD
int buffer_length; //associated with the lcd display functions

uint8_t dcp0_value, dcp1_value; //variables for reverse polarity check;

uint8_t dcs_value = 1; //surge detection variables

uint8_t neutral_value = 1; //neutral disconnection variables

uint8_t acs_value = 1; //surge detection variables

uint8_t relay_0_value, relay_1_value, relay_2_value; //relay state store variables

//frequency value set in spi flash
uint32_t frequency_address = 0x1004; //variable to store frequency value in address in spi flash
uint8_t flash_frequency; //writing variable to store frequency in spi flash
uint8_t starting_frequency = 50; //reading variable to read frequency from spi flash

//required voltage set in spi flash
uint32_t required_voltage_address = 0x1008; //variable to store reqd voltage
//value in address in spi flash
uint16_t write_voltage = 300; //writing variable to store reqd voltage in spi flash
uint8_t req_volt_write[2]; //writing array variable to store reqd voltage in spi flash in 8 bit
uint8_t read_voltage[2]; //reading array variable to read reqd voltage from spi flash in 8 bit
uint16_t output_voltage; //reading variable to read reqd voltage from spi flash

//MAXIMUM VOLTAGE SET IN SPI FLASH
uint32_t max_voltage_address = 0x1012; //variable to store max voltage value
//in address in spi flash
uint16_t write_max_voltage = 400; //writing variable to store max voltage in spi flash
uint8_t max_volt_write[2]; //writing array variable to store max voltage in spi flash in 8 bit
uint8_t max_volt_read[2]; //reading array variable to read max voltage from spi flash in 8 bit
uint16_t output_max_voltage; //reading variable to read max voltage from spi flash

//MINIMUM VOLTAGE SET IN SPI FLASH
uint32_t min_voltage_address = 0x1016; //variable to store min voltage value
//in address in spi flash
uint16_t write_min_voltage = 0; //writing variable to store min voltage in spi flash
uint8_t min_volt_write[2]; //writing array variable to store min voltage in spi flash in 8 bit
uint8_t min_volt_read[2]; //reading array variable to read min voltage from spi flash in 8 bit
uint16_t output_min_voltage; //reading variable to read min voltage from spi flash

//CURRENT TRANSFORMER SET IN SPI FLASH
uint32_t current_transformer_address = 0x1020; //variable to store current transformer ratio
//in address in spi flash
uint8_t current_transformer_write = 1; //writing variable to store ct value in spi flash
uint8_t current_transformer_read; //reading variable to read ct value from spi flash

//MAXIMUM CURRENT SET IN SPI FLASH
uint32_t max_current_address = 0x1024; //variable to store max current value
//in address in spi flash
uint16_t req_max_current = 700; //writing variable to store max current value in spi flash
uint8_t req_max_current_write[2]; //writing array variable to store max current value in spi flash in 8 bit
uint8_t req_max_current_read[2]; //reading array variable to read max current value from spi flash in 8 bit
uint16_t output_max_current; //reading variable to read max current value from spi flash

uint8_t switch_press = 1; //variable to indicate whether the button is pressed

uint16_t timer_period = 0;
uint16_t channel1_pulse = 0;

uint16_t timer_period_2 = 0;
uint16_t channel2_pulse = 0;

uint16_t prescaler_value = 0;
uint16_t prescaler_value_2 = 0;
uint16_t prescaler_value_3 = 0;
uint16_t prescaler_value_4 = 0;
uint16_t prescaler_value_6 = 0;

void crm_configuration(void);

/* ADC variables */
//TO READ THE ADC VALUES
__IO uint16_t adc1_ordinary_valuetab[11] = { 0 };
__IO uint16_t vmor_flag_index = 0;
volatile uint8_t dma_transfer_complete = 0;

/* Modbus Holding Registers */
uint16_t modbus_holding_registers[MODBUS_HOLDING_REG_COUNT] = { 0 };

/* General configuration */

/*#define DELAY 100
 #define FAST 1
 #define SLOW 4
 uint8_t g_speed = FAST;
 volatile uint32_t local_time = 0; */

/* TCP connection state */
static uint8_t tcp_connection_state = 0;

/* Function Prototypes */
void user_delay_ms(uint32_t ms);
void LCD_GPIO_Init(void);
void LCD_SendCommand(uint8_t command);
void LCD_SendData(uint8_t data);
void LCD_Init(void);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
void LCD_WriteChar(char c);
void LCD_WriteString(char *str);
void __delay_ms(uint32_t ms);
static err_t modbus_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t modbus_receive(void *arg, struct tcp_pcb *tpcb, struct pbuf *p,
		err_t err);
static err_t modbus_poll(void *arg, struct tcp_pcb *tpcb);
static void modbus_error(void *arg, err_t err);
//void set_required_voltage(void);
void set_timer4_frequency(uint8_t frequency);
void set_timer2_duty_cycle(uint16_t duty_cycle);
void set_timer5_duty_cycle(uint16_t duty_cycle);
void set_timer10_duty_cycle(uint16_t duty_cycle);
void current_transformer_select(uint16_t value, uint16_t *mapped_current);
static void process_modbus_request(struct tcp_pcb *tpcb, uint8_t *data,
		uint16_t len);
static void send_modbus_response(struct tcp_pcb *tpcb,
		modbus_tcp_frame_t *request, uint8_t *data, uint16_t len);
void modbus_server_init(void);
static void gpio_config(void);
static void dma_config(void);
static void adc_config(void);
//void process_modbus_request(uint8_t *data, uint16_t length, struct tcp_pcb *pcb);
//void send_modbus_response(uint8_t *data, uint16_t length, struct tcp_pcb *pcb);
void update_modbus_registers_with_adc(void);
void adc_periodic_update(volatile uint32_t localtime);
uint16_t map_adc_to_value(uint16_t adc_value); //DC INPUT VOLTAGE
uint16_t three_phase_voltage_map(float adc_value[3], uint16_t output_values[3]); //AC OUTPUT VOLTAGE
void reverse_polarity_gpio_init(void);
void reverse_polarity_check(void);
void exint_surge_pin_config(void);
void input_surge_check(void);
void neutral_disconnection(void);
void voltage_in_range_check(void);
void output_voltage_in_range_check(void);
void feedback_voltage(void);
void output_current_in_range_check(void);

//USER DEFINED DELAY FUNCTION
void __delay_ms(uint32_t ms) {
	volatile uint32_t i, j;
	for (i = 0; i < ms; i++) {
		for (j = 0; j < 1000; j++) {
			__NOP();  // No operation instruction
		}
	}
}

// Debug function to verify GPIO states
void debug_gpio_pins(void) {
	uint16_t pb_state = GPIOE->odt;
	uint16_t pd_state = GPIOD->odt;
}

// LCD Initialization and GPIO setup
void LCD_GPIO_Init(void) {
	gpio_init_type gpio_init_struct;

	crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOD_PERIPH_CLOCK, TRUE);

// Configure PC10 (RS) ,PC11 (E) and PC12 (RW) as output
	gpio_default_para_init(&gpio_init_struct);
	gpio_init_struct.gpio_pins = LCD_RS | LCD_E | LCD_RW;
	gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
	gpio_init(GPIOC, &gpio_init_struct);

// Configure PD0 to PD7 as output for data lines
	gpio_init_struct.gpio_pins = GPIO_PINS_0 | GPIO_PINS_1 | GPIO_PINS_2

	| GPIO_PINS_3 | GPIO_PINS_4 | GPIO_PINS_5 | GPIO_PINS_6 | GPIO_PINS_7
			| GPIO_PINS_8 | GPIO_PINS_9 | GPIO_PINS_10;
	gpio_init(GPIOD, &gpio_init_struct);

	gpio_bits_reset(GPIOD, SW_BUZZ_PIN);

	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_init_struct.gpio_pins = SW_BUZZ_PIN;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init(GPIOD, &gpio_init_struct);

// Initialize pins to low
	gpio_bits_reset(GPIOC, LCD_RS | LCD_E);
//gpio_port_write(GPIOD, 0x00);

	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_MODERATE;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_init_struct.gpio_pins = BUZZER_PIN;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init(GPIOE, &gpio_init_struct);

	debug_gpio_pins();
}

//SEND COMMAND FUNCTION OF THE LCD
void LCD_SendCommand(uint8_t command) {
	gpio_bits_reset(GPIOC, LCD_RS);  // RS = 0 for command

	// Manually set each data bit to correct pins
	gpio_bits_write(GPIOD, GPIO_PINS_0, (command & (1 << 0)) ? SET : RESET);
	gpio_bits_write(GPIOD, GPIO_PINS_1, (command & (1 << 1)) ? SET : RESET);
	gpio_bits_write(GPIOD, GPIO_PINS_2, (command & (1 << 2)) ? SET : RESET);
	gpio_bits_write(GPIOD, GPIO_PINS_3, (command & (1 << 3)) ? SET : RESET);
	gpio_bits_write(GPIOD, GPIO_PINS_4, (command & (1 << 4)) ? SET : RESET);
	gpio_bits_write(GPIOD, GPIO_PINS_5, (command & (1 << 5)) ? SET : RESET);
	gpio_bits_write(GPIOD, GPIO_PINS_7, (command & (1 << 6)) ? SET : RESET); // D6 → GPIO_PINS_7
	gpio_bits_write(GPIOD, GPIO_PINS_6, (command & (1 << 7)) ? SET : RESET); // D7 → GPIO_PINS_6

	gpio_bits_set(GPIOC, LCD_E);     // E = 1
	user_delay_ms(1);
	gpio_bits_reset(GPIOC, LCD_E);   // E = 0
	user_delay_ms(5);  // Delay for command processing
}

//SEND DATA FUNCTION OF THE LCD
void LCD_SendData(uint8_t data) {
	gpio_bits_set(GPIOC, LCD_RS);    // RS = 1 for data

	// Manually set each data bit to correct pins
	gpio_bits_write(GPIOD, GPIO_PINS_0, (data & (1 << 0)) ? SET : RESET);
	gpio_bits_write(GPIOD, GPIO_PINS_1, (data & (1 << 1)) ? SET : RESET);
	gpio_bits_write(GPIOD, GPIO_PINS_2, (data & (1 << 2)) ? SET : RESET);
	gpio_bits_write(GPIOD, GPIO_PINS_3, (data & (1 << 3)) ? SET : RESET);
	gpio_bits_write(GPIOD, GPIO_PINS_4, (data & (1 << 4)) ? SET : RESET);
	gpio_bits_write(GPIOD, GPIO_PINS_5, (data & (1 << 5)) ? SET : RESET);
	gpio_bits_write(GPIOD, GPIO_PINS_7, (data & (1 << 6)) ? SET : RESET); // D6 → GPIO_PINS_7
	gpio_bits_write(GPIOD, GPIO_PINS_6, (data & (1 << 7)) ? SET : RESET); // D7 → GPIO_PINS_6

	gpio_bits_set(GPIOC, LCD_E);     // E = 1
	user_delay_ms(1);
	gpio_bits_reset(GPIOC, LCD_E);   // E = 0
	user_delay_ms(5);  // Delay for data processing
}

//LCD INITIALIZE FUNCTION
void LCD_Init(void) {
	LCD_GPIO_Init();  // Initialize GPIO for LCD
	user_delay_ms(20);

// Function Set: 8-bit mode, 2 lines, 5x8 dots
	LCD_SendCommand(0x30);  // 8-bit mode
	user_delay_ms(5);
	LCD_SendCommand(0x30);  // 8-bit mode
	user_delay_ms(5);
	LCD_SendCommand(0x30);  // 8-bit mode
	user_delay_ms(5);

	LCD_SendCommand(0x38);
	LCD_SendCommand(0x0C);
	LCD_SendCommand(0x06);
	LCD_SendCommand(0x01);
	user_delay_ms(2);
}

//LCD CLEAR DISPLAY FUNCTION
void LCD_Clear(void) {
	LCD_SendCommand(0x01);  // Clear display command
	user_delay_ms(2);  // Clearing takes longer
}

//LCD SET CURSOR FUNCTION
void LCD_SetCursor(uint8_t row, uint8_t col) {
	uint8_t address;

	switch (row) {
	case 1:
		address = 0x00 + col;
		break;
	case 2:
		address = 0x40 + col;
		break;
	case 3:
		address = 0x14 + col;
		break;
	case 4:
		address = 0x54 + col;
		break;
	default:
		address = 0x00 + col;
		break;  // Default to row 1
	}

	LCD_SendCommand(0x80 | address);  // Set DDRAM address
}

// Write a single character to the LCD
void LCD_WriteChar(char c) {
	LCD_SendData(c);
}

// Write a string of characters to the LCD
void LCD_WriteString(char *str) {
	while (*str) {
		LCD_WriteChar(*str++);
	}
}

//USART PIN SETUP FUNCTION
void usart_configuration(void) {
	gpio_init_type gpio_init_struct;

	/* Enable USART3 and GPIO clock */
	crm_periph_clock_enable(CRM_USART6_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);

	gpio_default_para_init(&gpio_init_struct);

	/* Configure the USART3 TX pin (PB10) */
	gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_init_struct.gpio_mode = GPIO_MODE_MUX;
	gpio_init_struct.gpio_pins = GPIO_PINS_6;
	gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
	gpio_init(GPIOC, &gpio_init_struct);

	/* Configure the USART3 RX pin (PB11) */
	gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
	gpio_init_struct.gpio_pins = GPIO_PINS_7;
	gpio_init(GPIOC, &gpio_init_struct);

	/* Configure USART3 */
	usart_init(USART6, 115200, USART_DATA_8BITS, USART_STOP_1_BIT);
	usart_transmitter_enable(USART6, TRUE);
	usart_receiver_enable(USART6, TRUE);
	usart_enable(USART6, TRUE);
}

//PD11 pin is set as output pin to generate sine wave in arduino
void gpio_pins_configuration(void)
{
	gpio_init_type gpio_initstructure;
	gpio_default_para_init(&gpio_initstructure);

	gpio_bits_reset(GPIOD,GPIO_PINS_11);

	gpio_initstructure.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_initstructure.gpio_pull = GPIO_PULL_NONE;
	gpio_initstructure.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_initstructure.gpio_pins = GPIO_PINS_11;
	gpio_init(GPIOD, &gpio_initstructure);

	gpio_initstructure.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_initstructure.gpio_pull = GPIO_PULL_NONE;
	gpio_initstructure.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_initstructure.gpio_pins = GPIO_PINS_5;
	gpio_init(GPIOC, &gpio_initstructure);
}

void process_received_data()
{
	receive_buffer[buffer_index] = '\0';

	for (uint8_t i = 0; i < buffer_index; i++)
	{
		while (usart_flag_get(USART6, USART_TDC_FLAG) == RESET);
		usart_data_transmit(USART6, receive_buffer[i]);
	}
	buffer_index = 0;
}

/**
 * @brief  Function to send a single byte via UART
 */
void uart_transmit_byte(uint8_t byte)
{
	while (usart_flag_get(USART6, USART_TDBE_FLAG) == RESET);
	usart_data_transmit(USART6, byte);
}

/**
 * @brief  Function to send a string via UART
 */
void uart_transmit_string(const char *str)
{
	while (*str)
	{
		while (usart_flag_get(USART6, USART_TDBE_FLAG) == RESET);
		usart_data_transmit(USART6, (uint16_t) *str++);
	}
}

/**
 * @brief  Function to transmit a number via UART
 */
void uart_transmit_number(int number)
{
	char buffer[20];
	snprintf(buffer, sizeof(buffer), "%d", number);
	uart_transmit_string(buffer);
}

/**
 * @brief  Function to send data to Arduino in the expected format
 * @note   Sends raw bytes (not ASCII/string) in the format Arduino expects
 */
void send_data_to_arduino(uint8_t start_marker, uint8_t frequency, 
		uint8_t phase1_amp, uint8_t phase2_amp, uint8_t phase3_amp)
{
	// Send each byte individually without any separators
	uart_transmit_byte(start_marker);  // Start marker (255)
	uart_transmit_byte(frequency);     // Frequency value
	uart_transmit_byte(phase1_amp);    // Phase 1 amplitude
	uart_transmit_byte(phase2_amp);    // Phase 2 amplitude
	uart_transmit_byte(phase3_amp);    // Phase 3 amplitude

	// Debug output to UART or other debug interface
	printf("Sent data to Arduino: %d %d %d %d %d\r\n",
	       start_marker, frequency, phase1_amp, phase2_amp, phase3_amp);
}

/* TCP connection state functions */
uint8_t is_tcp_connected(void) 
{
	return tcp_connection_state;
}

void set_tcp_connection_state(uint8_t state) 
{
	tcp_connection_state = state;
}

/* GPIO CONFIGURATION FUNCTION FOR ADC AND TIMER FOR FRQ 10KHz DUTY 50 */
static void gpio_config(void) 
{
	gpio_init_type gpio_initstructure;
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);
	crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);

	gpio_default_para_init(&gpio_initstructure);
	gpio_initstructure.gpio_mode = GPIO_MODE_ANALOG;
	gpio_initstructure.gpio_pins = GPIO_PINS_0 | GPIO_PINS_3 |GPIO_PINS_4 | GPIO_PINS_5
			| GPIO_PINS_6 | GPIO_PINS_7;
	gpio_init(GPIOA, &gpio_initstructure);

	/* configure the IN10 pin */
	gpio_initstructure.gpio_mode = GPIO_MODE_ANALOG;
	gpio_initstructure.gpio_pins = GPIO_PINS_0;
	gpio_init(GPIOC, &gpio_initstructure);

	/* configure the IN12 pin */
	gpio_initstructure.gpio_mode = GPIO_MODE_ANALOG;
	gpio_initstructure.gpio_pins = GPIO_PINS_2;
	gpio_init(GPIOC, &gpio_initstructure);

	/* configure the IN13 pin */
	gpio_initstructure.gpio_mode = GPIO_MODE_ANALOG;
	gpio_initstructure.gpio_pins = GPIO_PINS_3;
	gpio_init(GPIOC, &gpio_initstructure);

	/* configure the IN14 pin */
	gpio_initstructure.gpio_mode = GPIO_MODE_ANALOG;
	gpio_initstructure.gpio_pins = GPIO_PINS_4;
	gpio_init(GPIOC, &gpio_initstructure);

	/* configure the IN9 pin */
	gpio_initstructure.gpio_mode = GPIO_MODE_ANALOG;
	gpio_initstructure.gpio_pins = GPIO_PINS_1;
	gpio_init(GPIOB, &gpio_initstructure);

	gpio_initstructure.gpio_pins = GPIO_PINS_0;
	gpio_initstructure.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_initstructure.gpio_pull = GPIO_PULL_NONE;
	gpio_initstructure.gpio_mode = GPIO_MODE_MUX;
	gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;

	gpio_init(GPIOB, &gpio_initstructure);
}

//CLOCK CONFIGURATION FUNCTION FOR TIMER AND GPIO PORT
void crm_configuration(void) 
{
	/* tmr3 clock enable */
	crm_periph_clock_enable(CRM_TMR3_PERIPH_CLOCK, TRUE);

	/* gpio clock enable */
	crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK, TRUE);

	crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK, TRUE);

	crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK, TRUE);
}

//MODBUS SERVER INITIALIZATION FUNCTION
void modbus_server_init(void) 
{
	err_t err;
	printf("Initializing Modbus server");

	gpio_config();

	modbus_pcb = tcp_new();
	if (modbus_pcb != NULL) {
		err = tcp_bind(modbus_pcb, IP_ADDR_ANY, MODBUS_TCP_PORT);
		if (err == ERR_OK) {
			modbus_pcb = tcp_listen(modbus_pcb);
			tcp_accept(modbus_pcb, modbus_accept);
			printf("Modbus server listening on port 502");
		} else {
			printf("Failed to bind Modbus server");
		}
	} else {
		printf("Failed to create new TCP PCB");
	}
}

//DMA CONFIGURATION FUNCTION FOR ADC
static void dma_config(void) 
{
	dma_init_type dma_init_struct;
	crm_periph_clock_enable(CRM_DMA1_PERIPH_CLOCK, TRUE);
	dma_reset(DMA1_CHANNEL1);
	dma_default_para_init(&dma_init_struct);
	dma_init_struct.buffer_size = 11;
	dma_init_struct.direction = DMA_DIR_PERIPHERAL_TO_MEMORY;
	dma_init_struct.memory_base_addr = (uint32_t) adc1_ordinary_valuetab;
	dma_init_struct.memory_data_width = DMA_MEMORY_DATA_WIDTH_HALFWORD;
	dma_init_struct.memory_inc_enable = TRUE;
	dma_init_struct.peripheral_base_addr = (uint32_t) &(ADC1->odt);
	dma_init_struct.peripheral_data_width = DMA_PERIPHERAL_DATA_WIDTH_HALFWORD;
	dma_init_struct.peripheral_inc_enable = FALSE;
	dma_init_struct.priority = DMA_PRIORITY_HIGH;
	dma_init_struct.loop_mode_enable = TRUE;
	dma_init(DMA1_CHANNEL1, &dma_init_struct);

	dma_channel_enable(DMA1_CHANNEL1, TRUE);
}

/**
 * @brief  adc configuration.
 * @param  none
 * @retval none
 */
//ADC CONFIGURATION FUNCTION TO GET ADC VALUES
static void adc_config(void) 
{
	adc_base_config_type adc_base_struct;
	crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK, TRUE);
	crm_adc_clock_div_set(CRM_ADC_DIV_6);
	nvic_irq_enable(ADC1_2_IRQn, 4, 0);

	/* select combine mode */
	adc_combine_mode_select(ADC_INDEPENDENT_MODE);
	adc_base_default_para_init(&adc_base_struct);
	adc_base_struct.sequence_mode = TRUE;
	adc_base_struct.repeat_mode = FALSE;
	adc_base_struct.data_align = ADC_RIGHT_ALIGNMENT;
	adc_base_struct.ordinary_channel_length = 11;
	adc_base_config(ADC1, &adc_base_struct);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_0, 1, ADC_SAMPLETIME_239_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_3, 2, ADC_SAMPLETIME_239_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_4, 3, ADC_SAMPLETIME_239_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_5, 4, ADC_SAMPLETIME_239_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_6, 5, ADC_SAMPLETIME_239_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_7, 6, ADC_SAMPLETIME_239_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_9, 7, ADC_SAMPLETIME_239_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_10, 8, ADC_SAMPLETIME_239_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_12, 9, ADC_SAMPLETIME_239_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_13, 10, ADC_SAMPLETIME_239_5);
	adc_ordinary_channel_set(ADC1, ADC_CHANNEL_14, 11, ADC_SAMPLETIME_239_5);
	adc_ordinary_conversion_trigger_set(ADC1, ADC12_ORDINARY_TRIG_SOFTWARE,
			TRUE);
	adc_dma_mode_enable(ADC1, TRUE);
	adc_voltage_monitor_enable(ADC1, ADC_VMONITOR_SINGLE_ORDINARY);
	adc_voltage_monitor_threshold_value_set(ADC1, 0xBBB, 0xAAA);
	adc_voltage_monitor_single_channel_select(ADC1, ADC_CHANNEL_5);
	adc_interrupt_enable(ADC1, ADC_VMOR_INT, TRUE);

	adc_enable(ADC1, TRUE);
	adc_calibration_init(ADC1);
	while (adc_calibration_init_status_get(ADC1))
		;
	adc_calibration_start(ADC1);
	while (adc_calibration_status_get(ADC1))
		;
}

// Define indices for each ADC channel in the adc1_ordinary_valuetab array
#define DC_VOLTAGE_INDEX         0  // PA0 - Channel 0
#define IGBT_TEMP1_INDEX         1  // PA3 - Channel 3
#define AC_VOLTAGE_U_INDEX       2  // PA4 - Channel 4
#define AC_VOLTAGE_V_INDEX       3  // PA5 - Channel 5
#define AC_VOLTAGE_W_INDEX       4  // PA6 - Channel 6
#define IGBT_TEMP2_INDEX         5  // PA7 - Channel 7
#define AC_CURRENT_U_INDEX       6  // PB1 - Channel 9
#define AC_CURRENT_V_INDEX       7  // PC0 - Channel 10
#define AC_CURRENT_W_INDEX       8  // PC2 - Channel 12
#define IGBT_TEMP4_INDEX         9  // PC3 - Channel 13
#define IGBT_TEMP3_INDEX         10 // PC4 - Channel 14

uint32_t map_value(uint32_t value, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max)
{
  return out_min + (value - in_min) * (out_max - out_min) / (in_max - in_min);
}

// Function to map temperature ADC values to degrees Celsius
uint16_t map_temperature(uint16_t adc_value) {
    // Assuming linear relationship between ADC value and temperature
    // Adjust these values based on your temperature sensor's characteristics
    return map_value(adc_value, 0, 4095, 25, 150);
}

// Function to process and map IGBT temperature readings
void update_igbt_temperatures(uint16_t mapped_temps[4]) {
    // Map the temperature ADC values to actual temperatures
    mapped_temps[0] = map_temperature(adc1_ordinary_valuetab[IGBT_TEMP1_INDEX]); // IGBT Temp 1 (PA3)
    mapped_temps[1] = map_temperature(adc1_ordinary_valuetab[IGBT_TEMP2_INDEX]); // IGBT Temp 2 (PA7)
    mapped_temps[2] = map_temperature(adc1_ordinary_valuetab[IGBT_TEMP3_INDEX]); // IGBT Temp 3 (PC4)
    mapped_temps[3] = map_temperature(adc1_ordinary_valuetab[IGBT_TEMP4_INDEX]); // IGBT Temp 4 (PC3)

    // Print the temperature values for debugging
    printf("IGBT Temperature 1: %d°C\r\n", mapped_temps[0]);
    printf("IGBT Temperature 2: %d°C\r\n", mapped_temps[1]);
    printf("IGBT Temperature 3: %d°C\r\n", mapped_temps[2]);
    printf("IGBT Temperature 4: %d°C\r\n", mapped_temps[3]);
}

// Updated function to compute RMS values with correct channel indices
float compute_rms(uint8_t channel_index) {
    static int sample_index = 0;
    static float sum = 0.0;
    static float rms_value = 0.0;  // Store last computed RMS

    if (sample_index < NUM_SAMPLES) {  // Take one sample per function call
        adc_ordinary_software_trigger_enable(ADC1, TRUE);  // Start ADC

        uint16_t adc_value = adc1_ordinary_valuetab[channel_index];  // Read ADC value
        float voltage = (adc_value * V_REF) / ADC_RES; // Convert ADC value to voltage
        sum += voltage * voltage;  // Square and accumulate

        printf("Sample %d for channel index %d: sum = %f\n", sample_index, 
				channel_index, sum);

        sample_index++;  // Increment sample index
    }

    if (sample_index == NUM_SAMPLES) { // Compute RMS after collecting all samples
        rms_value = sqrt(sum / NUM_SAMPLES);
        printf("Computed RMS for channel index %d: %f\n", channel_index, 
				rms_value);

        // Reset for next cycle
        sample_index = 0;
        sum = 0.0;
    }

    return rms_value;  // Return the last computed RMS value
}

//VOLTAGE MAP FUNCTION FOR AC VOLTAGE (U,V,W)
uint16_t three_phase_voltage_map(float voltage_value[3],
		uint16_t output_values[3]) {
	for (int i = 0; i < 3; i++) {
// Mapping 0.0V - 3.3V to 0 - 480
		output_values[i] = 0
				+ ((voltage_value[i] - 0.0) * (480 - 0) / (3.3 - 0.0));

// Ensure the values stay within bounds
		if (output_values[i] < 0)
			output_values[i] = 0;
		if (output_values[i] > 480)
			output_values[i] = 480;
	}
}

void update_mapped_voltages()
{
	uint16_t adc_values[3] = {adc1_ordinary_valuetab[AC_VOLTAGE_U_INDEX],
			adc1_ordinary_valuetab[AC_VOLTAGE_V_INDEX],
			adc1_ordinary_valuetab[AC_VOLTAGE_W_INDEX]};

	printf("Raw ADC Values: Phase U: %d, Phase V: %d, Phase W: %d\r\n",
			adc_values[0],adc_values[1],adc_values[2]);

	float voltage_values[3];

	for(int i=0; i<3; i++)
	{
		voltage_values[i] = (adc_values[i]*V_REF)/ADC_RES;
		printf("Voltage %d: %.2fV\r\n",i,voltage_values[i]);
	}

	uint16_t mapped_three_phase[3];
	three_phase_voltage_map(voltage_values, mapped_three_phase);

	printf("Mapped Voltages Phase U: %d, Phase V: %d, Phase W: %d\r\n",
			mapped_three_phase[0],mapped_three_phase[1],
			mapped_three_phase[2]);

	for(int i=0; i<3; i++)
	{
		mapped_value[i+1] = mapped_three_phase[i];
		modbus_holding_registers[i+1] = mapped_value[i+1];
	}
}

// Updated function to get average IGBT temperature
uint16_t get_average_igbt_temperature() {
    uint16_t igbt_temps[4];
    update_igbt_temperatures(igbt_temps);

    uint32_t sum = igbt_temps[0] + igbt_temps[1] + igbt_temps[2] + igbt_temps[3];
    uint16_t avg_temp = (uint16_t)(sum / 4);

    printf("Average IGBT Temperature: %d°C\r\n", avg_temp);
    return avg_temp;
}

//MAPPING FUNCTION TO MAP DC VOLTAGE
uint16_t map_adc_to_value(uint16_t dc_value)   //DC INPUT VOLTAGE
{
	uint16_t new_value;

// Map the ADC value from 0-4095 to 300-700
	new_value = ((dc_value * 400) / 4095) + 300;

	mapped_value[0] = new_value;

	return mapped_value[0];
}

// Updated function to update Modbus registers with all ADC data
void update_modbus_registers_with_adc(void) {
    // DC Voltage from PA0
    adc_value[0] = adc1_ordinary_valuetab[DC_VOLTAGE_INDEX];
    mapped_value[0] = map_adc_to_value(adc_value[0]);
    modbus_holding_registers[0] = mapped_value[0];

    // Get raw ADC values for each phase
    uint16_t adc_values[3] = {
       adc1_ordinary_valuetab[AC_VOLTAGE_U_INDEX],  // Phase U
       adc1_ordinary_valuetab[AC_VOLTAGE_V_INDEX],  // Phase V
       adc1_ordinary_valuetab[AC_VOLTAGE_W_INDEX]   // Phase W
    };

    // Print raw ADC values for debugging
    /*printf("Raw ADC Values: Phase U: %d, Phase V: %d, Phase W: %d\r\n",
               adc_values[0], adc_values[1], adc_values[2]);

    // Convert to voltage (0-3.3V range assuming 12-bit ADC)
    float voltage_values[3];
    for (int i = 0; i < 3; i++) {
       voltage_values[i] = (adc_values[i] * V_REF) / ADC_RES;
       printf("Voltage %d: %.2f V\r\n", i, voltage_values[i]);
    }

    // Map voltages to the desired range
    uint16_t mapped_three_phase[3];
    three_phase_voltage_map(voltage_values, mapped_three_phase);

    // Debug output for mapped values
    printf("Mapped Voltages: Phase U: %d, Phase V: %d, Phase W: %d\r\n",
         mapped_three_phase[0], mapped_three_phase[1], mapped_three_phase[2]);*/

    // AC Voltage from PA4, PA5, PA6
    update_mapped_voltages();

    // Update Modbus registers
    /*for (int i = 0; i < 3; i++) {
       mapped_value[i + 1] = mapped_three_phase[i];
       modbus_holding_registers[i + 1] = mapped_value[i + 1];
    }*/

    // AC Current from PB1, PC0, PC2
    uint16_t mapped_current[3]; // Array to hold the mapped current values for each phase

    // Map current transformer values based on the selected ratio
    uint16_t current_raw[3] = {
        adc1_ordinary_valuetab[AC_CURRENT_U_INDEX],
        adc1_ordinary_valuetab[AC_CURRENT_V_INDEX],
        adc1_ordinary_valuetab[AC_CURRENT_W_INDEX]
    };

    // Use the current transformer ratio to map the raw values
    current_transformer_select(current_transformer, mapped_current);

    // Update Modbus registers with the mapped current values
    for (int i = 0; i < 3; i++) {
        mapped_value[i + 4] = mapped_current[i];
        modbus_holding_registers[i + 4] = mapped_current[i];
    }

    // IGBT Temperatures from PA3, PA7, PC4, PC3
    // Store average temperature in register 7
    mapped_value[7] = get_average_igbt_temperature();
    modbus_holding_registers[7] = mapped_value[7];

    // Optional: Store individual IGBT temperatures in additional registers
    //uint16_t igbt_temps[4];
    //update_igbt_temperatures(igbt_temps);

    // Assuming registers 12-15 are available for individual IGBT temperatures
    //for (int i = 0; i < 4; i++) {
       //modbus_holding_registers[12 + i] = igbt_temps[i];
    //}
}

/* Function to periodically update ADC values */
void adc_periodic_update(volatile uint32_t localtime) {
// Check if 1000 ms (1 second) has elapsed since the last ADC update
	if (localtime - adc_update_timer >= 1000 || localtime < adc_update_timer) {
		adc_update_timer = localtime; // Update the timer
		update_modbus_registers_with_adc(); // Perform ADC reading and Modbus register update
	}
}

/* Accept callback */
static err_t modbus_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
	LWIP_UNUSED_ARG(arg);
	LWIP_UNUSED_ARG(err);

	printf("New connection accepted");

	/* Set keep-alive */
	newpcb->keep_idle = TCP_KEEPALIVE_INTERVAL;

	/* Set callbacks */
	tcp_recv(newpcb, modbus_receive);
	tcp_poll(newpcb, modbus_poll, 4);
	tcp_err(newpcb, modbus_error);

	return ERR_OK;
}

/* Poll callback */
static err_t modbus_poll(void *arg, struct tcp_pcb *tpcb) {
	LWIP_UNUSED_ARG(arg);

	if (tpcb->state != ESTABLISHED) {
		tcp_close(tpcb);
		return ERR_ABRT;
	}
	return ERR_OK;
}

/* Error callback */
static void modbus_error(void *arg, err_t err) {
	LWIP_UNUSED_ARG(arg);
	LWIP_UNUSED_ARG(err);
	printf("TCP error occurred");
}

/* Receive callback */
static err_t modbus_receive(void *arg, struct tcp_pcb *tpcb, struct pbuf *p,
		err_t err) {
	LWIP_UNUSED_ARG(arg);

	if (p == NULL) {
		printf("Connection closed");
		tcp_close(tpcb);
		return ERR_OK;
	}

	if (err != ERR_OK) {
		pbuf_free(p);
		return err;
	}

	process_modbus_request(tpcb, (uint8_t*) p->payload, p->len);
	tcp_recved(tpcb, p->tot_len);
	pbuf_free(p);
	return ERR_OK;
}

/* Send Modbus response */
static void send_modbus_response(struct tcp_pcb *tpcb,
		modbus_tcp_frame_t *request, uint8_t *data, uint16_t len) {
	struct pbuf *p;
	modbus_tcp_frame_t response;

	response.transaction_id = request->transaction_id;
	response.protocol_id = request->protocol_id;
	response.length = len + 2;
	response.unit_id = request->unit_id;
	response.function_code = request->function_code;
	memcpy(response.data, data, len);

	p = pbuf_alloc(PBUF_TRANSPORT, len + 8, PBUF_RAM);
	if (p != NULL) {
		memcpy(p->payload, &response, len + 8);
		tcp_write(tpcb, p->payload, p->len, TCP_WRITE_FLAG_COPY);
		tcp_output(tpcb);
		pbuf_free(p);
	}
}

/* Process Modbus request */
static void process_modbus_request(struct tcp_pcb *tpcb, uint8_t *data,
		uint16_t len) {
	if (len < 12) {  // Minimum Modbus TCP frame length
		printf("Invalid frame length\r\n");
		return;
	}

// Extract MBAP header fields
	uint16_t transaction_id = (data[0] << 8) | data[1];
	uint16_t protocol_id = (data[2] << 8) | data[3];
	uint16_t length = (data[4] << 8) | data[5];
	uint8_t unit_id = data[6];
	uint8_t function_code = data[7];

// Prepare response buffer
	uint8_t response[256];
	uint16_t response_len = 0;

// Copy MBAP header to response
	response[0] = data[0];  // Transaction ID High
	response[1] = data[1];  // Transaction ID Low
	response[2] = 0x00;     // Protocol ID High
	response[3] = 0x00;     // Protocol ID Low
	response[6] = unit_id;  // Unit ID
	response[7] = function_code;  // Function Code

	switch (function_code) {
	case MODBUS_FUNC_READ_HOLDING_REGISTERS: {
		uint16_t start_addr = (data[8] << 8) | data[9];
		uint16_t reg_count = (data[10] << 8) | data[11];

		printf("Read Holding Registers - Start Addr: %d, Count: %d\r\n",
				start_addr, reg_count);

// Check if the requested address and count are valid
		if (start_addr + reg_count <= MODBUS_HOLDING_REG_COUNT) {

			response[8] = reg_count * 2;  // Byte count
			for (int i = 0; i < reg_count; i++) {
				reg_value = modbus_holding_registers[start_addr + i];
				response[9 + i * 2] = (reg_value >> 8) & 0xFF;
				response[10 + i * 2] = reg_value & 0xFF;
// Handle special registers
				switch (start_addr + i) {
				case MODBUS_FREQ_CONTROL_REGISTER:
					reg_value = starting_frequency;  // Current frequency
					break;
				case MODBUS_REQD_VOLTAGE_SET_REGISTER:
					reg_value = required_voltage;
					break;
				case MODBUS_MAX_VOLTAGE:
					reg_value = maximum_voltage;
					break;
				case MODBUS_MIN_VOLTAGE:
					reg_value = minimum_voltage;
					break;
				case CURRENT_TRANSFORMER_SELECT_RATIO:
					reg_value = current_transformer;
					break;
				case MAX_OUTPUT_CURRENT:
					reg_value = max_current;
					break;
				case IGBT_FAN_ON_OFF:
					reg_value = igbt_fan;
					break;
				default:
					reg_value = modbus_holding_registers[start_addr + i];
					break;
				}
			}
			response_len = 9 + (reg_count * 2);
			response[4] = ((response_len - 6) >> 8) & 0xFF;
			response[5] = (response_len - 6) & 0xFF;
		}
		break;
	}
	case MODBUS_FUNC_WRITE_SINGLE_REGISTER: {
		uint16_t reg_addr = (data[8] << 8) | data[9];
		uint16_t reg_value = (data[10] << 8) | data[11];

		printf("Write Single Register - Address: %d, Value: %d\r\n", reg_addr,
				reg_value);

		switch (reg_addr) {
		case MODBUS_FREQ_CONTROL_REGISTER:
			if (reg_value == MODBUS_FREQ_50HZ || reg_value == MODBUS_FREQ_60HZ) {
				flash_frequency = reg_value;
				spiflash_write(&flash_frequency, frequency_address, 1);

				spiflash_read(&starting_frequency, frequency_address, 1);
				set_timer4_frequency(starting_frequency);
				memcpy(response + 8, data + 8, 4);
				response_len = 12;
				response[4] = 0x00;
				response[5] = 0x06;
			} else {
				set_timer4_frequency(starting_frequency);
				response[7] |= 0x80;
				response[8] = MODBUS_EXCEPTION_ILLEGAL_DATA;
				response_len = 9;
				response[4] = 0x00;
				response[5] = 0x03;
			}
			break;
		case MODBUS_REQD_VOLTAGE_SET_REGISTER:
			if ((reg_value >= 0) && (reg_value <= 480)) {
				write_voltage = reg_value;  // Update global variable
				modbus_value_updated = 1;       // Set update flag
				req_volt_write[0] = (write_voltage >> 8) & 0xFF;
				req_volt_write[1] = write_voltage & 0xFF;

				spiflash_write(req_volt_write, required_voltage_address, 2);

				spiflash_read(read_voltage, required_voltage_address, 2);

				output_voltage = (read_voltage[0] << 8) | read_voltage[1];
				//output_voltage = 240;
				printf("Voltage %d\r\n", output_voltage);
				phase1_amp = ((output_voltage *100)/480)+0;
				phase2_amp = ((output_voltage *100)/480)+0;
				phase3_amp = ((output_voltage *100)/480)+0;
				//set_timer2_duty_cycle(output_voltage);
				//set_timer5_duty_cycle(output_voltage);
				//set_timer10_duty_cycle(output_voltage);
				memcpy(response + 8, data + 8, 4);
				response_len = 12;
				response[4] = 0x00;
				response[5] = 0x06;
			} else {
				response[7] |= 0x80;
				response[8] = MODBUS_EXCEPTION_ILLEGAL_DATA;
				response_len = 9;
				response[4] = 0x00;
				response[5] = 0x03;
			}
			break;
		case MODBUS_MAX_VOLTAGE:
			if ((reg_value >= 0) && (reg_value <= 480)) {
				write_max_voltage = reg_value;  // Update global variable

				max_volt_write[0] = (write_max_voltage >> 8) & 0xFF;
				max_volt_write[1] = write_max_voltage & 0xFF;

				spiflash_write(max_volt_write, max_voltage_address, 2);
				spiflash_read(max_volt_read, max_voltage_address, 2);

				output_max_voltage = (max_volt_read[0] << 8) | max_volt_read[1];
				output_voltage_in_range_check();
				memcpy(response + 8, data + 8, 4);
				response_len = 12;
				response[4] = 0x00;
				response[5] = 0x06;
			} else {
				response[7] |= 0x80;
				response[8] = MODBUS_EXCEPTION_ILLEGAL_DATA;
				response_len = 9;
				response[4] = 0x00;
				response[5] = 0x03;
			}
			break;
		case MODBUS_MIN_VOLTAGE:
			if ((reg_value >= 0) && (reg_value <= 480)) {
				write_min_voltage = reg_value;  // Update global variable

				min_volt_write[0] = (write_min_voltage >> 8) & 0xFF;
				min_volt_write[1] = write_min_voltage & 0xFF;

				spiflash_write(min_volt_write, min_voltage_address, 2);
				spiflash_read(min_volt_read, min_voltage_address, 2);

				output_min_voltage = (min_volt_read[0] << 8) | min_volt_read[1];
				output_voltage_in_range_check();
				memcpy(response + 8, data + 8, 4);
				response_len = 12;
				response[4] = 0x00;
				response[5] = 0x06;
			} else {
				response[7] |= 0x80;
				response[8] = MODBUS_EXCEPTION_ILLEGAL_DATA;
				response_len = 9;
				response[4] = 0x00;
				response[5] = 0x03;
			}
			break;
		case CURRENT_TRANSFORMER_SELECT_RATIO:
			printf("Writing to Current Transformer Register: %d\n", reg_value);
			if (reg_value == 1 || reg_value == 2 || reg_value == 3) {
				current_transformer_write = reg_value;
				spiflash_write(&current_transformer_write,
						current_transformer_address, 1);
				spiflash_read(&current_transformer_read,
						current_transformer_address, 1);
				printf("Value in modbus: %d\r\n", reg_value);
				uint16_t mapped_current[3] = { 0 }; // Array to store mapped current values
				current_transformer_select(current_transformer_read,
						mapped_current); // Pass both arguments
// Update the modbus holding registers with the mapped current values
				for (int i = 0; i < 3; i++) {
					modbus_holding_registers[i + 9] = mapped_current[i]; // Assuming registers start from 9 for current values
				}
				memcpy(response + 8, data + 8, 4);
				response_len = 12;
				response[4] = 0x00;
				response[5] = 0x06;
			} else {
				response[7] |= 0x80;
				response[8] = MODBUS_EXCEPTION_ILLEGAL_DATA;
				response_len = 9;
				response[4] = 0x00;
				response[5] = 0x03;
			}
			break;
		case MAX_OUTPUT_CURRENT:
			if (reg_value <= 1000) {
				req_max_current = reg_value;
				req_max_current_write[0] = (req_max_current >> 8) & 0xFF;
				req_max_current_write[1] = req_max_current & 0xFF;

				spiflash_write(req_max_current_write, max_current_address, 2);

				spiflash_read(req_max_current_read, max_current_address, 2);

				output_max_current = (req_max_current_read[0] << 8)
						| req_max_current_read[1];
				output_current_in_range_check();
				memcpy(response + 8, data + 8, 4);
				response_len = 12;
				response[4] = 0x00;
				response[5] = 0x06;
			} else {
				response[7] |= 0x80;
				response[8] = MODBUS_EXCEPTION_ILLEGAL_DATA;
				response_len = 9;
				response[4] = 0x00;
				response[5] = 0x03;
			}
			break;
		case IGBT_FAN_ON_OFF:
			if(reg_value == 0 || reg_value == 1)
			{
				uint8_t fan_state = (uint8_t)reg_value;
				//spiflash_write(&fan_state, igbt_fan_address, 1);

				// Read back to confirm
				//uint8_t fan_state_read;
				//spiflash_read(&fan_state_read, igbt_fan_address, 1);

				if (fan_state == 1)
				{
				    // Turn ON the fan - Set the GPIO pin high
				    gpio_bits_set(GPIOC, GPIO_PINS_5);
				    printf("IGBT Fan turned ON\r\n");
				}
				else
				{
				    // Turn OFF the fan - Set the GPIO pin low
					gpio_bits_reset(GPIOC, GPIO_PINS_5);
				    printf("IGBT Fan turned OFF\r\n");
				}
				modbus_holding_registers[IGBT_FAN_ON_OFF] = fan_state;

				// Prepare the standard response
				memcpy(response + 8, data + 8, 4);
				response_len = 12;
				response[4] = 0x00;
				response[5] = 0x06;
			}
			else
			{
				// Invalid value (not 0 or 1)
				response[7] |= 0x80;
				response[8] = MODBUS_EXCEPTION_ILLEGAL_DATA;
				response_len = 9;
				response[4] = 0x00;
				response[5] = 0x03;
			}
			break;
		default:
			response[7] |= 0x80;
			response[8] = MODBUS_EXCEPTION_ILLEGAL_ADDRESS;
			response_len = 9;
			response[4] = 0x00;
			response[5] = 0x03;
			break;
		}
		break;
	}
	default:
		printf("Unsupported function code: 0x%02X\r\n", function_code);
		response[7] |= 0x80;
		response[8] = 0x01;
		response_len = 9;
		response[4] = 0x00;
		response[5] = 0x03;
		break;
	}
// Send response
	if (response_len > 0) {
		struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, response_len, PBUF_RAM);
		if (p != NULL) {
			memcpy(p->payload, response, response_len);
			tcp_write(tpcb, p->payload, response_len, TCP_WRITE_FLAG_COPY);
			tcp_output(tpcb);
			pbuf_free(p);
			printf("Response sent successfully\r\n");
		} else {
			printf("Failed to allocate response buffer\r\n");
		}
	}
}

//RELAY 3 ON FUNCTION
void relay_3_on(uint16_t voltage) {
	if (((mapped_value[1] >= (modbus_set_voltage - 20))
			&& (mapped_value[1] <= (modbus_set_voltage + 20)))
			&& ((mapped_value[2] >= (modbus_set_voltage - 20))
					&& (mapped_value[2] <= (modbus_set_voltage + 20)))
			&& ((mapped_value[3] >= (modbus_set_voltage - 20))
					&& (mapped_value[3] <= (modbus_set_voltage + 20)))) {
		gpio_bits_set(GPIOE, RELAY_2);
	}
}

//TIMER 4 FREQUENCY SET FUNCTION IN PB6 (50/60 Hz)
void set_timer4_frequency(uint8_t frequency) {
	uint16_t prescaler_value;

// Turn on buzzer and keep it on if the frequency is invalid
	if (frequency != 50 && frequency != 60) {
		gpio_bits_set(GPIOE, BUZZER_PIN);  // Turn on buzzer
		printf("Invalid frequency: %d\r\n", frequency); // Indicate invalid frequency
		return;  // Exit function without configuring the timer
	}

// If the frequency is valid, turn off the buzzer
//gpio_bits_reset(GPIOD, SW_BUZZ_PIN);  // Turn off buzzer

	if (frequency == 50) {
//uart_transmit_number(frequency);
		sinewave_frequency = (double) frequency;
	} else if (frequency == 60) {
//uart_transmit_number(frequency);
		sinewave_frequency = (double) frequency;
	} else {
		printf("Invalid frequency: %d\r\n", frequency);
		return;
	}
}

/* Function to update duty cycle for Timer 2 for PB10 */
/*void set_timer2_duty_cycle(uint16_t required_voltage) {

	if (required_voltage > 480)
		required_voltage = 480;

	if (required_voltage < 0)
		required_voltage = 0;

	uint16_t duty_cycle = (required_voltage * 100) / 480;

	uint16_t new_ccr = (duty_cycle * 665) / 100;

	/* compute the value to be set in arr regiter to generate signal frequency at 37.5 khz for Timer 2 */
	/*prescaler_value_4 = (uint16_t) (system_core_clock / 24000000) - 1;

	tmr_base_init(TMR2, 665, prescaler_value_4);
	tmr_cnt_dir_set(TMR2, TMR_COUNT_UP);

	tmr_output_config_type tmr_oc_init_structure;
	tmr_output_default_para_init(&tmr_oc_init_structure);

	tmr_oc_init_structure.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A; // Changed to MODE_A
	tmr_oc_init_structure.oc_output_state = TRUE;
	tmr_oc_init_structure.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH; // Changed to ACTIVE_HIGH
	tmr_oc_init_structure.oc_idle_state = FALSE;

	tmr_output_channel_config(TMR2, TMR_SELECT_CHANNEL_3,
			&tmr_oc_init_structure);
	tmr_channel_value_set(TMR2, TMR_SELECT_CHANNEL_3, new_ccr);
	tmr_output_channel_buffer_enable(TMR2, TMR_SELECT_CHANNEL_3, TRUE);

	/* output enable tmr4 */
	//tmr_output_enable(TMR2, TRUE);

	/* enable tmr4 */
	//tmr_counter_enable(TMR2, TRUE);

	//printf("Current Voltage: %d, Required Voltage: %d, Duty Cycle: %d\r\n",
			//mapped_value[1], required_voltage, duty_cycle);
//}

/* Function to update duty cycle for Timer 5 for PA3 */
/*void set_timer5_duty_cycle(uint16_t required_voltage) {

	if (required_voltage > 480)
		required_voltage = 480;

	if (required_voltage < 0)
		required_voltage = 0;

	uint16_t duty_cycle = (required_voltage * 100) / 480;

	uint16_t new_ccr = (duty_cycle * 665) / 100;

	/* compute the value to be set in arr regiter to generate signal frequency at 37.5 khz for Timer 5 */
	/*prescaler_value_3 = (uint16_t) (system_core_clock / 24000000) - 1;

	tmr_base_init(TMR5, 665, prescaler_value_3);
	tmr_cnt_dir_set(TMR5, TMR_COUNT_UP);

	tmr_output_config_type tmr_oc_init_structure;
	tmr_output_default_para_init(&tmr_oc_init_structure);

	tmr_oc_init_structure.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A; // Changed to MODE_A
	tmr_oc_init_structure.oc_output_state = TRUE;
	tmr_oc_init_structure.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH; // Changed to ACTIVE_HIGH
	tmr_oc_init_structure.oc_idle_state = FALSE;

	tmr_output_channel_config(TMR5, TMR_SELECT_CHANNEL_4,
			&tmr_oc_init_structure);
	tmr_channel_value_set(TMR5, TMR_SELECT_CHANNEL_4, new_ccr);
	tmr_output_channel_buffer_enable(TMR5, TMR_SELECT_CHANNEL_4, TRUE);

	/* output enable tmr4 */
	//tmr_output_enable(TMR5, TRUE);

	/* enable tmr4 */
	//tmr_counter_enable(TMR5, TRUE);

	//printf("Current Voltage: %d, Required Voltage: %d, Duty Cycle: %d\r\n",
			//mapped_value[2], required_voltage, duty_cycle);
//}

/* Function to update duty cycle for Timer 10 for PB8 */
/*void set_timer10_duty_cycle(uint16_t required_voltage) {

	if (required_voltage > 480)
		required_voltage = 480;

	if (required_voltage < 0)
		required_voltage = 0;

	uint16_t duty_cycle = (required_voltage * 100) / 480;

	uint16_t new_ccr = (duty_cycle * 665) / 100;

	/* compute the value to be set in arr regiter to generate signal frequency at 37.5 khz for Timer 2 */
	/*prescaler_value_6 = (uint16_t) (system_core_clock / 24000000) - 1;

	tmr_base_init(TMR10, 665, prescaler_value_6);
	tmr_cnt_dir_set(TMR10, TMR_COUNT_UP);

	tmr_output_config_type tmr_oc_init_structure;
	tmr_output_default_para_init(&tmr_oc_init_structure);

	tmr_oc_init_structure.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_A; // Changed to MODE_A
	tmr_oc_init_structure.oc_output_state = TRUE;
	tmr_oc_init_structure.oc_polarity = TMR_OUTPUT_ACTIVE_HIGH; // Changed to ACTIVE_HIGH
	tmr_oc_init_structure.oc_idle_state = FALSE;

	tmr_output_channel_config(TMR10, TMR_SELECT_CHANNEL_1,
			&tmr_oc_init_structure);
	tmr_channel_value_set(TMR10, TMR_SELECT_CHANNEL_1, new_ccr);
	tmr_output_channel_buffer_enable(TMR10, TMR_SELECT_CHANNEL_1, TRUE);

	/* output enable tmr4 */
	//tmr_output_enable(TMR10, TRUE);

	/* enable tmr4 */
	//tmr_counter_enable(TMR10, TRUE);

	//printf("Current Voltage: %d, Required Voltage: %d, Duty Cycle: %d\r\n",
			//mapped_value[3], required_voltage, duty_cycle);
//}

//FUNCTION TO SELECT THE CT RATIO
void current_transformer_select(uint16_t value, uint16_t *mapped_current) {
	int i;
	uint16_t adc_max = 4095; // ADC range for 12-bit ADC

	printf("Value in modbus: %d\r\n", reg_value);

	printf("ADC Values: Phase U: %d, Phase V: %d, Phase W: %d\n",
			adc1_ordinary_valuetab[4], adc1_ordinary_valuetab[5],
			adc1_ordinary_valuetab[6]);

	printf("Current Transformer Ratio: %d\n", value);

	spiflash_read(&current_transformer_read, current_transformer_address, 1);

// Add validation or type casting if needed
	uint16_t ratio = value; // or use a method to extract the correct value

	printf("Interpreted Transformer Ratio: %d\n", ratio);

// Map the ADC value to the corresponding current range based on the ratio
	switch (current_transformer_read) {
	case 1:  // 1000/5 ratio
		for (i = 0; i < 3; i++) {
			printf("ADC[%d]: %d, Mapped Current: %d\n", i,
					adc1_ordinary_valuetab[i + 4],
					(adc1_ordinary_valuetab[i + 4] * 1000) / 4095);
			mapped_current[i] = (adc1_ordinary_valuetab[i + 4] * 1000) / 4095;
		}
		break;

	case 2:  // 500/5 ratio
		for (i = 0; i < 3; i++) {
			printf("ADC[%d]: %d, Mapped Current: %d\n", i,
					adc1_ordinary_valuetab[i + 4],
					(adc1_ordinary_valuetab[i + 4] * 500) / 4095);
			mapped_current[i] = (adc1_ordinary_valuetab[i + 4] * 500) / 4095;
		}
		break;

	case 3:  // 100/5 ratio
		for (i = 0; i < 3; i++) {
			printf("ADC[%d]: %d, Mapped Current: %d\n", i,
					adc1_ordinary_valuetab[i + 4],
					(adc1_ordinary_valuetab[i + 4] * 100) / 4095);
			mapped_current[i] = (adc1_ordinary_valuetab[i + 4] * 100) / 4095;
		}
		break;

	default:
		for (i = 0; i < 3; i++) {
			mapped_current[i] = 0;  // Invalid ratio, set current to 0
		}
		break;
	}
// Print the final mapped current values
	for (i = 0; i < 3; i++) {
		printf("Mapped Current[%d]: %d\n", i, mapped_current[i]); // Debugging line
	}
}

//REVERSE POLARITY GPIO INITIALIZATION FUNCTION
void reverse_polarity_gpio_init(void) //reverse polarity, buzzer and relay pins initialization
{
	gpio_init_type gpio_initstructure;
	crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, TRUE);

	gpio_initstructure.gpio_pull = GPIO_PULL_DOWN;
	gpio_initstructure.gpio_pins = GPIO_PINS_13 | GPIO_PINS_14;

	gpio_initstructure.gpio_mode = GPIO_MODE_OUTPUT;
	gpio_initstructure.gpio_pins = GPIO_PINS_7 | GPIO_PINS_8 | GPIO_PINS_9
			| GPIO_PINS_11;
	gpio_initstructure.gpio_pull = GPIO_PULL_NONE;
	gpio_initstructure.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
	gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
	gpio_init(GPIOE, &gpio_initstructure);
}

//REVERSE POLARITY CHECK FUNCTION
void reverse_polarity_check(void) {
// Read the values of DCP0 and DCP1 pins
	dcp0_value = gpio_input_data_bit_read(GPIOE, DCP0_PIN);  // Read DCP0 pin
	dcp1_value = gpio_input_data_bit_read(GPIOE, DCP1_PIN);  // Read DCP1 pin

	dcp0_value = (GPIOE->idt & GPIO_PINS_13) ? 1 : 0;
	dcp1_value = (GPIOE->idt & GPIO_PINS_14) ? 1 : 0;
	printf("DCP0: %d, DCP1: %d\n", dcp0_value, dcp1_value);

// Check the polarity based on the values of DCP0 and DCP1
	if (dcp0_value == 0 && dcp1_value == 1) {
// Correct polarity
		printf("Correct Polarity\n");
//gpio_bits_reset(GPIOE, BUZZER_PIN);
		LCD_WriteString("Correct Polarity");
		user_delay_ms(1000);
		LCD_Clear();
	} else if (dcp0_value == 1 && dcp1_value == 0) {
// Reverse polarity
		printf("Reverse Polarity\n");
		gpio_bits_set(GPIOE, BUZZER_PIN);
		LCD_WriteString("Reverse Polarity");
		user_delay_ms(1000);
//LCD_Clear();

		__disable_irq();

		gpio_bits_reset(GPIOE, RELAY_0);
		gpio_bits_reset(GPIOE, RELAY_1);
		gpio_bits_reset(GPIOE, RELAY_2);
		while (1) {
			dcp0_value = gpio_input_data_bit_read(GPIOC, DCP0_PIN); // Read DCP0 pin
			dcp1_value = gpio_input_data_bit_read(GPIOC, DCP1_PIN); // Read DCP1 pin

			dcp0_value = (GPIOC->idt & GPIO_PINS_13) ? 1 : 0;
			dcp1_value = (GPIOC->idt & GPIO_PINS_14) ? 1 : 0;

			if (dcp0_value == 0 && dcp1_value == 1) {
// Correct polarity
				printf("Correct Polarity\n");
				gpio_bits_reset(GPIOE, BUZZER_PIN);
				LCD_Clear();
				LCD_WriteString("Polarity Corrected");
				user_delay_ms(1000);
				__enable_irq();
				break;
			}
		}
	} else if (dcp0_value == 1 && dcp1_value == 1) {
// Not connected
		printf("Not Connected\n");
		LCD_WriteString("Not Connected");
		user_delay_ms(1000);
		__disable_irq();

		gpio_bits_reset(GPIOE, RELAY_0);
		gpio_bits_reset(GPIOE, RELAY_1);
		gpio_bits_reset(GPIOE, RELAY_2);
		while (1) {
			dcp0_value = gpio_input_data_bit_read(GPIOC, DCP0_PIN); // Read DCP0 pin
			dcp1_value = gpio_input_data_bit_read(GPIOC, DCP1_PIN); // Read DCP1 pin

			dcp0_value = (GPIOC->idt & GPIO_PINS_13) ? 1 : 0;
			dcp1_value = (GPIOC->idt & GPIO_PINS_14) ? 1 : 0;

			if (dcp0_value == 0 && dcp1_value == 1) {
// Correct polarity
				printf("Correct Polarity\n");
				gpio_bits_reset(GPIOE, BUZZER_PIN);
				LCD_Clear();
				LCD_WriteString("Polarity Corrected");
				user_delay_ms(1000);
				__enable_irq();
				break;
			}
		}
	} else {
		printf("Error: Unexpected Pin States\n");
	}
}

//DC VOLTAGE CHECK FUNCTION
void voltage_in_range_check(void) {
	if (mapped_value[0] >= 350 && mapped_value[0] <= 700) {
		LCD_Clear();
		LCD_SetCursor(1, 0);
//gpio_bits_reset(GPIOD, SW_BUZZ_PIN);

		sprintf(buffer, "DCV :%d", mapped_value[0]);
		LCD_WriteString(buffer);

		if (at32_button_press() == USER_BUTTON) {
			switch_press = 1;
//gpio_bits_set(GPIOD, GPIO_PINS_9);
			gpio_bits_set(GPIOD, SW_BUZZ_PIN);
			user_delay_ms(50);
			gpio_bits_reset(GPIOD, SW_BUZZ_PIN);
			LCD_Clear();
			LCD_WriteString("System Starting");
			user_delay_ms(1000);

			gpio_bits_set(GPIOE, RELAY_0);
			user_delay_ms(2000);
			gpio_bits_set(GPIOE, RELAY_1);
			spiflash_read(read_voltage, required_voltage_address, 2);

			output_voltage = (read_voltage[0] << 8) | read_voltage[1];

			//set_timer2_duty_cycle(output_voltage);
			//set_timer5_duty_cycle(output_voltage);
			//set_timer10_duty_cycle(output_voltage);
			user_delay_ms(2000);
			relay_3_on(modbus_set_voltage);
		}
	} else if (mapped_value[0] < 350) {
		switch_press = 0;
		//set_timer2_duty_cycle(0);
		//set_timer5_duty_cycle(0);
		//set_timer10_duty_cycle(0);
		LCD_Clear();
		LCD_SetCursor(1, 0);
		sprintf(buffer, "DCV :%d", mapped_value[0]);
		LCD_WriteString(buffer);

		LCD_SetCursor(2, 0);
		LCD_WriteString("UNDER VOLTAGE");
//gpio_bits_set(GPIOD, GPIO_PINS_10);
		at32_led_off(LED6);
		at32_led_on(LED7);
		gpio_bits_set(GPIOE, BUZZER_PIN);
		//set_timer2_duty_cycle(0);
		//set_timer5_duty_cycle(0);
		//set_timer10_duty_cycle(0);
		gpio_bits_reset(GPIOE, RELAY_0);
		gpio_bits_reset(GPIOE, RELAY_1);
		gpio_bits_reset(GPIOE, RELAY_2);

		LCD_SetCursor(3, 0);
		LCD_WriteString("PRESS RESET SWITCH");

		LCD_SetCursor(4, 0);
		LCD_WriteString("SYSTEM OFF");
	} else if (mapped_value[0] > 700) {
		switch_press = 0;
		//set_timer2_duty_cycle(0);
		//set_timer5_duty_cycle(0);
		//set_timer10_duty_cycle(0);
		LCD_Clear();
		LCD_SetCursor(1, 0);
		sprintf(buffer, "DCV :%d", mapped_value[0]);
		LCD_WriteString(buffer);

		LCD_SetCursor(2, 0);
		LCD_WriteString("OVER VOLTAGE");
//gpio_bits_set(GPIOD, GPIO_PINS_10);
		at32_led_off(LED6);
		at32_led_on(LED7);
		gpio_bits_set(GPIOE, BUZZER_PIN);
		//set_timer2_duty_cycle(0);
		//set_timer5_duty_cycle(0);
		//set_timer10_duty_cycle(0);
		gpio_bits_reset(GPIOE, RELAY_0);
		gpio_bits_reset(GPIOE, RELAY_1);
		gpio_bits_reset(GPIOE, RELAY_2);

		LCD_SetCursor(3, 0);
		LCD_WriteString("PRESS RESET SWITCH");

		LCD_SetCursor(4, 0);
		LCD_WriteString("SYSTEM OFF");
	}
}

//AC VOLTAGE CHECK FUNCTION
void output_voltage_in_range_check(void) {
	static bool voltage_fault = false; // Add this flag to track if there was an over/under voltage condition

	spiflash_read(max_volt_read, max_voltage_address, 2);
	output_max_voltage = (max_volt_read[0] << 8) | max_volt_read[1];

	spiflash_read(min_volt_read, min_voltage_address, 2);
	output_min_voltage = (min_volt_read[0] << 8) | min_volt_read[1];

// Check if any voltage is out of range
	bool voltage_out_of_range = (mapped_value[1] < output_min_voltage
			|| mapped_value[1] > output_max_voltage)
			|| (mapped_value[2] < output_min_voltage
					|| mapped_value[2] > output_max_voltage)
			|| (mapped_value[3] < output_min_voltage
					|| mapped_value[3] > output_max_voltage);

	if (voltage_out_of_range) {
		switch_press = 0;

		voltage_fault = true;  // Set fault flag

//gpio_bits_set(GPIOD, GPIO_PINS_10);
		at32_led_off(LED6);
		at32_led_on(LED7);

// Turn off all relays
		gpio_bits_reset(GPIOE, RELAY_0);
		gpio_bits_reset(GPIOE, RELAY_1);
		gpio_bits_reset(GPIOE, RELAY_2);

		LCD_Clear();
		LCD_SetCursor(1, 0);
		sprintf(buffer, "AC U:%d V:%d W:%d", mapped_value[1], mapped_value[2],
				mapped_value[3]);
		LCD_WriteString(buffer);

		LCD_SetCursor(2, 0);
		if (mapped_value[1] > output_max_voltage
				|| mapped_value[2] > output_max_voltage
				|| mapped_value[3] > output_max_voltage) {
			LCD_WriteString("OUTPUT OVERVOLTAGE");
		} else {
			LCD_WriteString("OUTPUT UNDERVOLTAGE");
		}

		LCD_SetCursor(3, 0);
		LCD_WriteString("PRESS RESET SWITCH");
		gpio_bits_set(GPIOD, SW_BUZZ_PIN);

		LCD_SetCursor(4, 0);
		LCD_WriteString("SYSTEM OFF");
	} else if (!voltage_out_of_range) {
		gpio_bits_set(GPIOD, GPIO_PINS_9);
// Only display normal operation message if no fault is active
		sprintf(buffer, "AC U:%d V:%d W:%d", mapped_value[1], mapped_value[2],
				mapped_value[3]);
		LCD_SetCursor(2, 0);
		LCD_WriteString(buffer);
		at32_led_on(LED6);
		at32_led_off(LED7);
		spiflash_read(read_voltage, required_voltage_address, 2);
		output_voltage = (read_voltage[0] << 8) | read_voltage[1];

		if (switch_press == 1) {

			if ((mapped_value[1] >= (output_voltage - 20)
					&& mapped_value[1] <= (output_voltage + 20))
					&& (mapped_value[2] >= (output_voltage - 20)
							&& mapped_value[2] <= (output_voltage + 20))
					&& (mapped_value[3] >= (output_voltage - 20)
							&& mapped_value[3] <= (output_voltage + 20))) {
				gpio_bits_set(GPIOE, RELAY_2);
			}
		}
	}

// Check for reset button press
	if (at32_button_press() == USER_BUTTON && voltage_fault) {
		switch_press = 1;
		LCD_Clear();
		LCD_WriteString("    System Resumed");
		user_delay_ms(1000);

// Only reset the fault flag and turn on relays if voltage is in range
		if (!voltage_out_of_range) {
			voltage_fault = false;  // Clear fault flag
//gpio_bits_set(GPIOD, GPIO_PINS_9);
			at32_led_on(LED6);
			at32_led_off(LED7);

// Sequential relay activation
			gpio_bits_set(GPIOE, RELAY_0);
			user_delay_ms(2000);
			gpio_bits_set(GPIOE, RELAY_1);
			user_delay_ms(2000);

// Check if voltage is within required range before setting RELAY_2
			spiflash_read(read_voltage, required_voltage_address, 2);
			output_voltage = (read_voltage[0] << 8) | read_voltage[1];

			if ((mapped_value[1] >= (output_voltage - 20)
					&& mapped_value[1] <= (output_voltage + 20))
					&& (mapped_value[2] >= (output_voltage - 20)
							&& mapped_value[2] <= (output_voltage + 20))
					&& (mapped_value[3] >= (output_voltage - 20)
							&& mapped_value[3] <= (output_voltage + 20))) {
				gpio_bits_set(GPIOE, RELAY_2);
			}
		}
	}
}

//AC FEEDBACK VOLTAGE FUNCTION
void feedback_voltage(void) {
// Read target voltage from flash
	spiflash_read(read_voltage, required_voltage_address, 2);
	output_voltage = (read_voltage[0] << 8) | read_voltage[1];

	spiflash_read(&starting_frequency, frequency_address, 1);

// Define limits
	uint16_t lower_limit = output_voltage - 20;  // 180V
	uint16_t upper_limit = output_voltage + 20;  // 220V

	// Define min/max amplitude values
	uint16_t min_amplitude = 0;  // Adjust based on your system
	uint16_t max_amplitude = 100; // Adjust based on your system

// Check and correct each channel independently
// Channel 1
	if (mapped_value[1] < lower_limit) {
// If below 180V, increase duty cycle
		phase1_amp = phase1_amp + 1;
		if (phase1_amp > max_amplitude) 
			phase1_amp = max_amplitude;
		printf("Feedback Voltage\r\n");
		printf("Phase1 low (%d) - increasing to: %d\r\n",
				mapped_value[1], phase1_amp);
		//set_timer2_duty_cycle(lower_limit);
	} else if (mapped_value[1] > upper_limit) {
// If above 220V, decrease duty cycle
		phase1_amp = phase1_amp - 1;
		if (phase1_amp < min_amplitude) 
			phase1_amp = min_amplitude;
		printf("Phase1 high (%d) - decreasing to: %d\r\n",
		        		mapped_value[1], phase1_amp);
		//set_timer2_duty_cycle(upper_limit);
	}

// Channel 2
	if (mapped_value[2] < lower_limit) {
		phase2_amp = phase2_amp + 1;
		if (phase2_amp > max_amplitude) 
			phase2_amp = max_amplitude;
		printf("Phase2 low (%d) - increasing to: %d\r\n",
				mapped_value[2], phase2_amp);
		//set_timer5_duty_cycle(lower_limit);
	} else if (mapped_value[2] > upper_limit) {
		phase2_amp = phase2_amp - 1;
		if (phase2_amp < min_amplitude) 
			phase2_amp = min_amplitude;
		printf("Phase2 high (%d) - decreasing to: %d\r\n",
		        		mapped_value[2], phase2_amp);
		//set_timer5_duty_cycle(upper_limit);
	}

// Channel 3
	if (mapped_value[3] < lower_limit) {
		phase3_amp = phase3_amp + 1;
		if (phase3_amp > max_amplitude) 
			phase3_amp = max_amplitude;
		printf("Phase3 low (%d) - increasing to: %d\r\n",
		        		mapped_value[3], phase3_amp);
		//set_timer10_duty_cycle(lower_limit);
	} else if (mapped_value[3] > upper_limit) {
		phase3_amp = phase3_amp - 1;
		if (phase3_amp < min_amplitude) 
			phase3_amp = min_amplitude;
		printf("Phase3 high (%d) - decreasing to: %d\r\n",
		        		mapped_value[3], phase3_amp);
		//set_timer10_duty_cycle(upper_limit);
	}
	// Safety condition: If all phases are at max amplitude but voltage is still too low
	// OR if all phases are at min amplitude but voltage is still too high
	if ((phase1_amp >= max_amplitude && phase2_amp >= max_amplitude 
		&& phase3_amp >= max_amplitude &&
	    (mapped_value[1] < lower_limit || mapped_value[2] < lower_limit 
				|| mapped_value[3] < lower_limit)) |
		|(phase1_amp <= min_amplitude && phase2_amp <= min_amplitude 
				&& phase3_amp <= min_amplitude 
				&& (mapped_value[1] > upper_limit 
						|| mapped_value[2] > upper_limit 
						|| mapped_value[3] > upper_limit))) {

	     // Disconnect relay as a safety measure
	     gpio_bits_reset(GPIOE, RELAY_2);
	     printf("SAFETY: Voltage outside limits despite max/min amplitude - RELAY2 disconnected\r\n");
	 }
	send_data_to_arduino(start_marker, starting_frequency,
			phase1_amp, phase2_amp, phase3_amp);
}

//AC CURRENT CHECK FUNCTION
void output_current_in_range_check(void) {
	sprintf(buffer, "Iu:%d Iv:%d Iw:%d", mapped_value[4], mapped_value[5],
			mapped_value[6]);

	LCD_SetCursor(3, 0);

	LCD_WriteString(buffer);

//user_delay_ms(100);
	spiflash_read(&current_transformer_read, current_transformer_address, 1);

	spiflash_read(req_max_current_read, max_current_address, 2);

	output_max_current = (req_max_current_read[0] << 8)
			| req_max_current_read[1];

	if (current_transformer_read == 1) {
		if (output_max_current > 1000) {
			switch_press = 0;
			LCD_SetCursor(4, 0);
			LCD_WriteString("Current exceeds");
			gpio_bits_set(GPIOE, BUZZER_PIN);

//gpio_bits_reset(GPIOD, GPIO_PINS_9);
//gpio_bits_set(GPIOD, GPIO_PINS_10);
			at32_led_off(LED6);
			at32_led_on(LED7);
			gpio_bits_reset(GPIOE, RELAY_0);
			gpio_bits_reset(GPIOE, RELAY_1);
			gpio_bits_reset(GPIOE, RELAY_2);
		}
	}
	if (current_transformer_read == 2) {
		if (output_max_current > 500) {
			switch_press = 0;
			LCD_SetCursor(4, 0);
			LCD_WriteString("Current exceeds");
			gpio_bits_set(GPIOE, BUZZER_PIN);

//gpio_bits_reset(GPIOD, GPIO_PINS_9);
//gpio_bits_set(GPIOD, GPIO_PINS_10);
			at32_led_off(LED6);
			at32_led_on(LED7);
			gpio_bits_reset(GPIOE, RELAY_0);
			gpio_bits_reset(GPIOE, RELAY_1);
			gpio_bits_reset(GPIOE, RELAY_2);
		}
	}
	if (current_transformer_read == 3) {
		if (output_max_current > 100) {
			switch_press = 0;
			LCD_SetCursor(4, 0);
			LCD_WriteString("Current exceeds");
			gpio_bits_set(GPIOE, BUZZER_PIN);

//gpio_bits_reset(GPIOD, GPIO_PINS_9);
//gpio_bits_set(GPIOD, GPIO_PINS_10);
			at32_led_off(LED6);
			at32_led_on(LED7);
			gpio_bits_reset(GPIOE, RELAY_0);
			gpio_bits_reset(GPIOE, RELAY_1);
			gpio_bits_reset(GPIOE, RELAY_2);
		}
	}
}

//SURGE INTERRUPT CONFIGURATION
void exint_surge_pin_config(void) {
	gpio_init_type gpio_init_struct;
	exint_init_type exint_init_struct;

	gpio_default_para_init(&gpio_init_struct);
	gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
	gpio_init_struct.gpio_pins = GPIO_PINS_3;
	gpio_init_struct.gpio_pull = GPIO_PULL_UP;
	gpio_init(GPIOE, &gpio_init_struct);

// Configure GPIOE_PIN3 as input and associate it with EXTI line
	gpio_exint_line_config(GPIO_PORT_SOURCE_GPIOE, GPIO_PINS_SOURCE3); // Assuming DCS_PIN = PE3

// Initialize EXTI line for rising edge detection
	exint_default_para_init(&exint_init_struct);
	exint_init_struct.line_enable = TRUE;
	exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
	exint_init_struct.line_select = EXINT_LINE_3; // DCS_PIN is on EXTI line 3
	exint_init_struct.line_polarity = EXINT_TRIGGER_FALLING_EDGE; // Trigger on rising edge (spike detection)
	exint_init(&exint_init_struct);

	exint_interrupt_enable(EXINT_LINE_3, TRUE);

// Configure NVIC for EXTI line 3
	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);
	nvic_irq_enable(EXINT3_IRQn, 0, 0);
}

//DC SURGE CHECK FUNCTION
void input_surge_check(void) {
// Read the values of DCS pin
	dcs_value = gpio_input_data_bit_read(GPIOE, DCS_PIN);  // Read DCS pin
	dcs_value = (GPIOE->idt & GPIO_PINS_3) ? 1 : 0;
	printf("SURGE_VALUE: %d\r\n", dcs_value);
	if (dcs_value == 0) {
// Surge Detected
		printf(" Surge Detected\n");
		LCD_Clear();
		LCD_WriteString("Surge Detected");
//gpio_bits_reset(GPIOD, GPIO_PINS_9);
//gpio_bits_set(GPIOD, GPIO_PINS_10);
		at32_led_off(LED6);
		at32_led_on(LED7);
		gpio_bits_reset(GPIOE, RELAY_0);
		printf("RELAY_0 IS OFF\n");
		gpio_bits_reset(GPIOE, RELAY_1);
		printf("RELAY_1 IS OFF\n");
		gpio_bits_reset(GPIOE, RELAY_2);
		printf("RELAY_2 IS OFF\n");
	}
	relay_0_value = gpio_input_data_bit_read(GPIOE, RELAY_0); // Read RELAY_0 pin
	relay_1_value = gpio_input_data_bit_read(GPIOE, RELAY_1); // Read RELAY_1 pin
	relay_2_value = gpio_input_data_bit_read(GPIOE, RELAY_2); // Read RELAy_2 pin

	relay_0_value = (GPIOE->idt & GPIO_PINS_7) ? 1 : 0;
	relay_1_value = (GPIOE->idt & GPIO_PINS_8) ? 1 : 0;
	relay_2_value = (GPIOE->idt & GPIO_PINS_9) ? 1 : 0;
	printf("RELAY_0: %d, RELAY_1: %d, RELAY_2: %d\n", relay_0_value,
			relay_1_value, relay_2_value);
}

//SURGE INTERRUPT HANDLER FUNCTION
void EXINT3_IRQHandler(void) {
// Check if EXTI interrupt is triggered by line 3
	if (exint_interrupt_flag_get(EXINT_LINE_3) != RESET) {
		printf("INTERRUPT TRIGGERED");

// Disable ADC interrupt immediately
		nvic_irq_disable(ADC1_2_IRQn);
		adc_interrupt_enabled = 0;
// Handle surge detection
		surge_flag = 1;
		input_surge_check();
// Clear the interrupt flag
		exint_flag_clear(EXINT_LINE_3);
	}
}

//NEUTRAL INTERRUPT CONFIGURATION
void exint_neutral_pin_config(void) {
	gpio_init_type gpio_init_struct;
	exint_init_type exint_init_struct;

	/* configure the EXINT1 */
	gpio_default_para_init(&gpio_init_struct);
	gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
	gpio_init_struct.gpio_pins = GPIO_PINS_1;
	gpio_init_struct.gpio_pull = GPIO_PULL_UP;
	gpio_init(GPIOE, &gpio_init_struct);

	gpio_exint_line_config(GPIO_PORT_SOURCE_GPIOE, GPIO_PINS_SOURCE1);

	exint_default_para_init(&exint_init_struct);
	exint_init_struct.line_enable = TRUE;
	exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
	exint_init_struct.line_select = EXINT_LINE_1;
	exint_init_struct.line_polarity = EXINT_TRIGGER_FALLING_EDGE;
	exint_init(&exint_init_struct);

	exint_interrupt_enable(EXINT_LINE_1, TRUE);

	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

	nvic_irq_enable(EXINT1_IRQn, 1, 0);
}

//NEUTRAL DISCONNECTION FUNCTION
void neutral_disconnection(void) {
// Read the values of DCS pin
	neutral_value = gpio_input_data_bit_read(GPIOE, NEUTRAL_PIN); // Read NEUTRAL pin

	neutral_value = (GPIOE->idt & GPIO_PINS_1) ? 1 : 0;
	printf("NEUTRAL_VALUE: %d\r\n", neutral_value);
	if (neutral_value == 0 && mapped_value[4] > 300 && mapped_value[5] > 300
			&& mapped_value[6] > 300) {
// Surge Detected
		printf("Neutral Disconnected\n");
		LCD_Clear();
		LCD_WriteString("Neutral Disconnected");
//gpio_bits_reset(GPIOD, GPIO_PINS_9);
//gpio_bits_set(GPIOD, GPIO_PINS_10);
		at32_led_off(LED6);
		at32_led_on(LED7);
		gpio_bits_reset(GPIOE, RELAY_0);
		printf("RELAY_0 IS OFF\n");
		gpio_bits_reset(GPIOE, RELAY_1);
		printf("RELAY_1 IS OFF\n");
		gpio_bits_reset(GPIOE, RELAY_2);
		printf("RELAY_2 IS OFF\n");

// Disable ADC interrupt
		nvic_irq_disable(ADC1_2_IRQn);
		adc_interrupt_enabled = 0;
	}
	relay_0_value = gpio_input_data_bit_read(GPIOE, RELAY_0); // Read RELAY_0 pin
	relay_1_value = gpio_input_data_bit_read(GPIOE, RELAY_1); // Read RELAY_1 pin
	relay_2_value = gpio_input_data_bit_read(GPIOE, RELAY_2); // Read RELAy_2 pin

	relay_0_value = (GPIOE->idt & GPIO_PINS_7) ? 1 : 0;
	relay_1_value = (GPIOE->idt & GPIO_PINS_8) ? 1 : 0;
	relay_2_value = (GPIOE->idt & GPIO_PINS_9) ? 1 : 0;
	printf("RELAY_0: %d, RELAY_1: %d, RELAY_2: %d\n", relay_0_value,
			relay_1_value, relay_2_value);
}

//NEUTRAL INTERRUPT HANDLER FUNCTION
void EXINT1_IRQHandler(void) {
// Check if EXTI interrupt is triggered by line 3
	if (exint_interrupt_flag_get(EXINT_LINE_1) != RESET) {
		printf("INTERRUPT TRIGGERED");
		neutral_disconnect_flag = 1;
		neutral_disconnection();
// Clear the interrupt flag
		exint_flag_clear(EXINT_LINE_1);
	}
}

//AC SURGE PIN CONFIGURATION
void exint_ac_surge_pin_config(void) {
	gpio_init_type gpio_init_struct;
	exint_init_type exint_init_struct;

	/* configure the EXINT0 */
	gpio_default_para_init(&gpio_init_struct);
	gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
	gpio_init_struct.gpio_pins = GPIO_PINS_0;
	gpio_init_struct.gpio_pull = GPIO_PULL_UP;
	gpio_init(GPIOE, &gpio_init_struct);

	gpio_exint_line_config(GPIO_PORT_SOURCE_GPIOE, GPIO_PINS_SOURCE0);

	exint_default_para_init(&exint_init_struct);
	exint_init_struct.line_enable = TRUE;
	exint_init_struct.line_mode = EXINT_LINE_INTERRUPT;
	exint_init_struct.line_select = EXINT_LINE_0;
	exint_init_struct.line_polarity = EXINT_TRIGGER_RISING_EDGE;
	exint_init(&exint_init_struct);

	exint_interrupt_enable(EXINT_LINE_0, TRUE);

	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4);

	nvic_irq_enable(EXINT0_IRQn, 2, 0);
}

//OUTPUT SURGE FUNCTION
void output_surge(void) {
// Read the values of DCS pin
	acs_value = gpio_input_data_bit_read(GPIOE, ACS_PIN); // Read NEUTRAL pin

	acs_value = (GPIOE->idt & GPIO_PINS_0) ? 1 : 0;
	printf("ACS_VALUE: %d\r\n", acs_value);
	if (acs_value == 1) {
// Surge Detected
		printf("AC Surge Detected\n");
		LCD_Clear();
		LCD_WriteString("AC Surge Detected");
//gpio_bits_reset(GPIOD, GPIO_PINS_9);
//gpio_bits_set(GPIOD, GPIO_PINS_10);
		at32_led_off(LED6);
		at32_led_on(LED7);
		gpio_bits_reset(GPIOE, RELAY_0);
		printf("RELAY_0 IS OFF\n");
		gpio_bits_reset(GPIOE, RELAY_1);
		printf("RELAY_1 IS OFF\n");
		gpio_bits_reset(GPIOE, RELAY_2);
		printf("RELAY_2 IS OFF\n");
	}
	relay_0_value = gpio_input_data_bit_read(GPIOE, RELAY_0); // Read RELAY_0 pin
	relay_1_value = gpio_input_data_bit_read(GPIOE, RELAY_1); // Read RELAY_1 pin
	relay_2_value = gpio_input_data_bit_read(GPIOE, RELAY_2); // Read RELAy_2 pin

	relay_0_value = (GPIOE->idt & GPIO_PINS_7) ? 1 : 0;
	relay_1_value = (GPIOE->idt & GPIO_PINS_8) ? 1 : 0;
	relay_2_value = (GPIOE->idt & GPIO_PINS_9) ? 1 : 0;
	printf("RELAY_0: %d, RELAY_1: %d, RELAY_2: %d\n", relay_0_value,
			relay_1_value, relay_2_value);
}

//AC SURGE INTERRUPT HANDLER
void EXINT0_IRQHandler(void) {
// Check if EXTI interrupt is triggered by line 3
	if (exint_interrupt_flag_get(EXINT_LINE_0) != RESET) {
		printf("INTERRUPT TRIGGERED");

// Disable ADC interrupt immediately
		nvic_irq_disable(ADC1_2_IRQn);
		adc_interrupt_enabled = 0;
// Handle surge detection
		output_surge_flag = 1;
		output_surge();

// Clear the interrupt flag
		exint_flag_clear(EXINT_LINE_0);
	}
}

/* Main function */
void main(void) {
	error_status status;

	__IO uint32_t index = 0;
	__IO uint32_t flash_id_index = 0;
	nvic_priority_group_config(NVIC_PRIORITY_GROUP_4); //for measuring adc in 7 channels we uncommented it
			/* System Initialization */
	system_clock_config();
	/* peripheral clocks configuration */
	crm_configuration();

	at32_board_init();
	at32_led_on(LED5);
	at32_led_off(LED6);
	at32_led_off(LED7);
	uart_print_init(115200);
	custom_uart_print_init(115200);
	gpio_pins_configuration();
	spiflash_init();

	printf("System initialized");
	LCD_Init();  // Initialize the LCD
	LCD_Clear();

	printf("System Core Clock Frequency: %lu Hz\n", system_core_clock);
	crm_clocks_freq_get(&crm_clocks_freq_struct);
	printf("APB1 Frequency: %lu Hz\n", crm_clocks_freq_struct.apb1_freq);
	printf("APB2 Frequency: %lu Hz\n", crm_clocks_freq_struct.apb2_freq);
	printf("AHB Frequency: %lu Hz\n", crm_clocks_freq_struct.ahb_freq);

	do {
		status = emac_system_init();
		if (status == ERROR) {
			printf("EMAC init failed, retrying...");
			user_delay_ms(500);
		}
	} while (status == ERROR);

	uint16_t phy_id_low = 0;  // Lower 16 bits of PHY ID (Register 2)
	uint16_t phy_id_high = 0; // Upper 16 bits of PHY ID (Register 3)
	uint32_t phy_id = 0;      // Full 32-bit PHY ID

// Read Register 2 (PHY ID lower 16 bits)
	if (emac_phy_register_read(PHY_ADDRESS, 0x02, &phy_id_low) != SUCCESS) {
		printf("Failed to read PHY ID (Register 2)\n");
	}

// Read Register 3 (PHY ID upper 16 bits)
	if (emac_phy_register_read(PHY_ADDRESS, 0x03, &phy_id_high) != SUCCESS) {
		printf("Failed to read PHY ID (Register 3)\n");
	}

// Combine the two 16-bit values to form the 32-bit PHY ID
	phy_id = phy_id_high | (phy_id_low << 16);

// Print the PHY ID
	printf("PHY ID: 0x%08X\n", phy_id);

// Check if the PHY ID matches LAN8720 (example)
	if (phy_id == 0x0181B8A0) 
	{
		printf("Detected DM9162 PHY\n");
	} 
	else if(phy_id == 0x0007C0F1)
	{
		printf("Detected LAN8720 PHY\n");
	}
	else 
	{
		printf("Unknown PHY detected\n");
	}

	printf("EMAC initialized");

	user_delay_ms(1000);
	tcpip_stack_init();

	printf("TCP/IP stack initialized");

	user_delay_ms(1000);
	modbus_server_init();

	printf("System Initialized, TCP Server Running on Port 502\r\n");

	/* compute the value to be set in arr regiter to generate signal frequency at 10 khz */
	prescaler_value = (uint16_t) (system_core_clock / 6896550) - 1;

	gpio_config();
	dma_config();
	adc_config();

	uint8_t button_value = (GPIOE->idt & GPIO_PINS_10) ? 1 : 0;
	printf("button_value: %d\n", button_value);

	LCD_SetCursor(2, 0);  // First line, first column
	LCD_WriteString("MAX CONTROLS PVT LTD");  // Alternate method
	LCD_SendCommand(0x0C);
	at32_led_on(LED6);

	user_delay_ms(3000);
	LCD_Clear();

	reverse_polarity_gpio_init();
//reverse_polarity_check();

	exint_surge_pin_config();

	exint_neutral_pin_config();

	exint_ac_surge_pin_config();

	adc_ordinary_software_trigger_enable(ADC1, TRUE);

	/* tmr3 time base configuration */
	tmr_base_init(TMR3, 665, prescaler_value);
	tmr_cnt_dir_set(TMR3, TMR_COUNT_UP);
//tmr_clock_source_div_set(TMR3, TMR_CLOCK_DIV1);

//tmr_clock_source_div_set(TMR2, TMR_CLOCK_DIV1);
	spiflash_read(&starting_frequency, frequency_address, 1);

	set_timer4_frequency(starting_frequency);

	//read the required voltage from spi
	spiflash_read(read_voltage, required_voltage_address, 2);

	output_voltage = (read_voltage[0] << 8) | read_voltage[1];

	phase1_amp = ((output_voltage * 100)/480)+0;
	phase2_amp = ((output_voltage * 100)/480)+0;
	phase3_amp = ((output_voltage * 100)/480)+0;

	tmr_output_default_para_init(&tmr_oc_init_structure);
	tmr_oc_init_structure.oc_mode = TMR_OUTPUT_CONTROL_PWM_MODE_B;
	tmr_oc_init_structure.oc_output_state = TRUE;
	tmr_oc_init_structure.oc_polarity = TMR_OUTPUT_ACTIVE_LOW;
	tmr_oc_init_structure.oc_idle_state = TRUE;
	tmr_oc_init_structure.occ_output_state = TRUE;
	tmr_oc_init_structure.occ_polarity = TMR_OUTPUT_ACTIVE_HIGH;
	tmr_oc_init_structure.occ_idle_state = FALSE;

	tmr_output_channel_config(TMR3, TMR_SELECT_CHANNEL_3,
			&tmr_oc_init_structure);
	tmr_channel_value_set(TMR3, TMR_SELECT_CHANNEL_3, ccr1_val);
	tmr_output_channel_buffer_enable(TMR3, TMR_SELECT_CHANNEL_3, TRUE);

	uint32_t debug_counter = 0;
	static uint16_t min_values[4] = { 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF };
	static uint16_t max_values[4] = { 0, 0, 0, 0 };

	tmr_period_buffer_enable(TMR3, TRUE);

	/* tmr enable counter */
	tmr_counter_enable(TMR3, TRUE);

	flash_id_index = spiflash_read_id();
	printf("id: 0x%x\r\n", flash_id_index);
	if ((flash_id_index != W25Q128) && (flash_id_index != W25Q80)
			&& (flash_id_index != W25Q16) && (flash_id_index != W25Q32)
			&& (flash_id_index != W25Q64)) {
		printf("flash id check error!\r\n");
		for (index = 0; index < 50; index++) {
			at32_led_toggle(LED2);
			at32_led_toggle(LED3);
			delay_ms(200);
		}
	} else {
		printf("flash id check success! id: %x\r\n", flash_id_index);
	}

// Write a specific value to a specific address
	uint32_t write_address = 0x1000;  // Choose your desired address
	uint8_t value_to_write = 240;    // Choose your desired value

// Write the value
	printf("Write operation started.");
	spiflash_write(&value_to_write, write_address, 1);
	printf("Write operation complete");

// Optionally, read back and verify
	uint8_t read_back;
	spiflash_read(&read_back, write_address, 1);

	if (read_back == value_to_write) {
		at32_led_on(LED2);
		at32_led_on(LED3);
		at32_led_on(LED4);
		printf("Write successful! Value: %d\r\n", read_back);
		printf("Write successful! Value: %d\r\n", starting_frequency);

	} else {
		at32_led_off(LED2);
		at32_led_off(LED3);
		at32_led_off(LED4);
		printf("Write failed. Read back: %d\r\n", read_back);
	}

	send_data_to_arduino(start_marker,starting_frequency,phase1_amp,
							phase2_amp, phase3_amp);

	/* Main Loop */
	while (1) {
		lwip_rx_loop_handler();
		lwip_periodic_handle(local_time);

		adc_ordinary_software_trigger_enable(ADC1, TRUE);

		adc_periodic_update(local_time);  // Handle ADC periodic update

		system_state = 1; //for checking igbt board purpose temporary

//gpio_bits_set(GPIOD, GPIO_PINS_8);
//gpio_bits_reset(GPIOD, GPIO_PINS_10);

		/*if (system_state == 0) {
		 adc_interrupt_enabled = 0;
		 if (mapped_value[0] >= 350 && mapped_value[0] <= 700) {
		 LCD_Clear();
		 LCD_SetCursor(1, 0);
		 gpio_bits_reset(GPIOE, BUZZER_PIN);

		 //gpio_bits_set(GPIOD, GPIO_PINS_9);

		 sprintf(buffer, "DCV :%d", mapped_value[0]);
		 LCD_WriteString(buffer);

		 LCD_SetCursor(1, 12);
		 LCD_WriteString("POL: C");

		 LCD_SetCursor(2, 0);
		 LCD_WriteString("PRESS START SWITCH");

		 delay_ms(500);

		 if (at32_start_button_press() == USER_BUTTON) {
		 gpio_bits_set(GPIOD, SW_BUZZ_PIN);
		 user_delay_ms(50);
		 gpio_bits_reset(GPIOD, SW_BUZZ_PIN);
		 LCD_Clear();
		 LCD_WriteString("System Starting");
		 user_delay_ms(1000);

		 gpio_bits_set(GPIOE, RELAY_0);
		 user_delay_ms(2000);
		 gpio_bits_set(GPIOE, RELAY_1);
		 spiflash_read(read_voltage, required_voltage_address, 2);

		 output_voltage = (read_voltage[0] << 8) | read_voltage[1];
		 set_timer2_duty_cycle(output_voltage);
		 set_timer5_duty_cycle(output_voltage);
		 set_timer10_duty_cycle(output_voltage);
		 user_delay_ms(2000);
		 if (((mapped_value[1] >= (output_voltage - 20))
		 && (mapped_value[1] <= (output_voltage + 20)))
		 && ((mapped_value[2] >= (output_voltage - 20))
		 && (mapped_value[2] <= (output_voltage + 20)))
		 && ((mapped_value[3] >= (output_voltage - 20))
		 && (mapped_value[3] <= (output_voltage + 20)))) {
		 gpio_bits_set(GPIOE, RELAY_2);
		 }

		 system_state = 1; // Reset state after handling
		 adc_interrupt_enabled = 1;
		 }
		 } else if (mapped_value[0] < 350) {
		 LCD_Clear();
		 LCD_SetCursor(1, 0);
		 sprintf(buffer, "DCV :%d", mapped_value[0]);
		 LCD_WriteString(buffer);

		 LCD_SetCursor(2, 0);
		 LCD_WriteString("UNDER VOLTAGE");
		 gpio_bits_set(GPIOE, BUZZER_PIN);
		 //gpio_bits_set(GPIOD, GPIO_PINS_10);
		 at32_led_off(LED6);
		 at32_led_on(LED7);
		 set_timer2_duty_cycle(0);
		 set_timer5_duty_cycle(0);
		 set_timer10_duty_cycle(0);
		 gpio_bits_reset(GPIOE, RELAY_0);
		 gpio_bits_reset(GPIOE, RELAY_1);
		 gpio_bits_reset(GPIOE, RELAY_2);

		 LCD_SetCursor(3, 0);
		 LCD_WriteString("PRESS RESET SWITCH");

		 LCD_SetCursor(4, 0);
		 LCD_WriteString("SYSTEM OFF");
		 } else if (mapped_value[0] > 700) {
		 LCD_Clear();
		 LCD_SetCursor(1, 0);
		 sprintf(buffer, "DCV :%d", mapped_value[0]);
		 LCD_WriteString(buffer);

		 LCD_SetCursor(2, 0);
		 LCD_WriteString("OVER VOLTAGE");
		 gpio_bits_set(GPIOE, BUZZER_PIN);
		 //gpio_bits_set(GPIOD, GPIO_PINS_10);
		 at32_led_off(LED6);
		 at32_led_on(LED7);
		 set_timer2_duty_cycle(0);
		 set_timer5_duty_cycle(0);
		 set_timer10_duty_cycle(0);
		 gpio_bits_reset(GPIOE, RELAY_0);
		 gpio_bits_reset(GPIOE, RELAY_1);
		 gpio_bits_reset(GPIOE, RELAY_2);

		 LCD_SetCursor(3, 0);
		 LCD_WriteString("PRESS RESET SWITCH");

		 LCD_SetCursor(4, 0);
		 LCD_WriteString("SYSTEM OFF");
		 }
		 }*/

		while (surge_flag == 1) {
			gpio_bits_set(GPIOD, GPIO_PINS_11);
// Halt all critical operations
			switch_press = 0;

			//set_timer2_duty_cycle(0);
			//set_timer5_duty_cycle(0);
			//set_timer10_duty_cycle(0);
			LCD_Clear();
			LCD_WriteString("SURGE DETECTED");
			LCD_SetCursor(2, 0);
			LCD_WriteString("Press reset switch");
			user_delay_ms(250);
//gpio_bits_set(GPIOD, GPIO_PINS_10);
			at32_led_off(LED6);
			at32_led_on(LED7);
			gpio_bits_reset(GPIOE, RELAY_0);
			gpio_bits_reset(GPIOE, RELAY_1);
			gpio_bits_reset(GPIOE, RELAY_2);

			if (at32_button_press() == USER_BUTTON) {
// Reset system state
				switch_press = 1;
				gpio_bits_reset(GPIOD, GPIO_PINS_11);

//gpio_bits_reset(GPIOD, GPIO_PINS_10);
//gpio_bits_set(GPIOD, GPIO_PINS_9);
				gpio_bits_set(GPIOD, SW_BUZZ_PIN);
				user_delay_ms(50);
				gpio_bits_reset(GPIOD, SW_BUZZ_PIN);
				surge_flag = 0;

// Re-enable ADC interrupt
				nvic_irq_enable(ADC1_2_IRQn, 3, 0);
				adc_interrupt_enabled = 1;

				LCD_Clear();
				LCD_WriteString("System Resumed");
				user_delay_ms(1000);
				at32_led_on(LED6);
				at32_led_off(LED7);
				gpio_bits_set(GPIOE, RELAY_0);
				user_delay_ms(2000);
				gpio_bits_set(GPIOE, RELAY_1);
				spiflash_read(read_voltage, required_voltage_address, 2);

				output_voltage = (read_voltage[0] << 8) | read_voltage[1];
				//set_timer2_duty_cycle(output_voltage);
				//set_timer5_duty_cycle(output_voltage);
				//set_timer10_duty_cycle(output_voltage);
				user_delay_ms(2000);
				if (((mapped_value[1] >= (output_voltage - 20))
						&& (mapped_value[1] <= (output_voltage + 20)))
						&& ((mapped_value[2] >= (output_voltage - 20))
								&& (mapped_value[2] <= (output_voltage + 20)))
						&& ((mapped_value[3] >= (output_voltage - 20))
								&& (mapped_value[3] <= (output_voltage + 20)))) {
					gpio_bits_set(GPIOE, RELAY_2);
				}
				break;
			}
		}
		while (neutral_disconnect_flag == 1) {
// Halt all critical operations
			if (mapped_value[4] > 300 && mapped_value[5] > 300
					&& mapped_value[6] > 300) {
				gpio_bits_set(GPIOD,GPIO_PINS_11);
				switch_press = 0;

				//set_timer2_duty_cycle(0);
				//set_timer5_duty_cycle(0);
				//set_timer10_duty_cycle(0);
				LCD_Clear();
				LCD_WriteString("NEUTRAL DISCONNECTED");
				LCD_SetCursor(2, 0);
				LCD_WriteString("Press reset switch");
				user_delay_ms(250);
				at32_led_off(LED6);
				at32_led_on(LED7);
//gpio_bits_set(GPIOD, GPIO_PINS_10);
				gpio_bits_reset(GPIOE, RELAY_0);
				gpio_bits_reset(GPIOE, RELAY_1);
				gpio_bits_reset(GPIOE, RELAY_2);

				if (at32_button_press() == USER_BUTTON) {
// Reset system state
					switch_press = 1;
					gpio_bits_reset(GPIOD,GPIO_PINS_11);

//gpio_bits_reset(GPIOD, GPIO_PINS_10);
//gpio_bits_set(GPIOD, GPIO_PINS_9);
					gpio_bits_set(GPIOD, SW_BUZZ_PIN);
					user_delay_ms(50);
					gpio_bits_reset(GPIOD, SW_BUZZ_PIN);
					neutral_disconnect_flag = 0;

// Re-enable ADC interrupt
					nvic_irq_enable(ADC1_2_IRQn, 3, 0);
					adc_interrupt_enabled = 1;

					LCD_Clear();
					LCD_WriteString("System Resumed");
					user_delay_ms(1000);
					at32_led_on(LED6);
					at32_led_off(LED7);
					gpio_bits_set(GPIOE, RELAY_0);
					user_delay_ms(2000);
					gpio_bits_set(GPIOE, RELAY_1);
					spiflash_read(read_voltage, required_voltage_address, 2);

					output_voltage = (read_voltage[0] << 8) | read_voltage[1];
					//set_timer2_duty_cycle(output_voltage);
					//set_timer5_duty_cycle(output_voltage);
					//set_timer10_duty_cycle(output_voltage);
					user_delay_ms(2000);
					if (((mapped_value[1] >= (output_voltage - 20))
							&& (mapped_value[1] <= (output_voltage + 20)))
							&& ((mapped_value[2] >= (output_voltage - 20))
									&& (mapped_value[2] <= (output_voltage + 20)))
							&& ((mapped_value[3] >= (output_voltage - 20))
									&& (mapped_value[3] <= (output_voltage + 20)))) {
						gpio_bits_set(GPIOE, RELAY_2);
					}
					break;
				}
			} else {
// If voltages are below threshold, just clear the flag and continue normal operation
				neutral_disconnect_flag = 0;
				break;
			}
		}
		while (output_surge_flag == 1) {
			gpio_bits_set(GPIOD, GPIO_PINS_11);
// Halt all critical operations
			switch_press = 0;

			//set_timer2_duty_cycle(0);
			//set_timer5_duty_cycle(0);
			//set_timer10_duty_cycle(0);
			LCD_Clear();
			LCD_WriteString("AC SURGE DETECTED");
			LCD_SetCursor(2, 0);
			LCD_WriteString("Press reset switch");
			user_delay_ms(250);
			at32_led_off(LED6);
			at32_led_on(LED7);
//gpio_bits_set(GPIOD, GPIO_PINS_10);
			gpio_bits_reset(GPIOE, RELAY_0);
			gpio_bits_reset(GPIOE, RELAY_1);
			gpio_bits_reset(GPIOE, RELAY_2);

			if (at32_button_press() == USER_BUTTON) {
// Reset system state
				switch_press = 1;
				gpio_bits_reset(GPIOD,GPIO_PINS_11);

//gpio_bits_reset(GPIOD, GPIO_PINS_10);
//gpio_bits_set(GPIOD, GPIO_PINS_9);
				gpio_bits_set(GPIOD, SW_BUZZ_PIN);
				user_delay_ms(50);
				gpio_bits_reset(GPIOD, SW_BUZZ_PIN);
				output_surge_flag = 0;

// Re-enable ADC interrupt
				nvic_irq_enable(ADC1_2_IRQn, 3, 0);
				adc_interrupt_enabled = 1;

				LCD_Clear();
				LCD_WriteString("System Resumed");
				user_delay_ms(1000);
				at32_led_on(LED6);
				at32_led_off(LED7);
				gpio_bits_set(GPIOE, RELAY_0);
				user_delay_ms(2000);
				gpio_bits_set(GPIOE, RELAY_1);
				spiflash_read(read_voltage, required_voltage_address, 2);

				output_voltage = (read_voltage[0] << 8) | read_voltage[1];
				//set_timer2_duty_cycle(output_voltage);
				//set_timer5_duty_cycle(output_voltage);
				//set_timer10_duty_cycle(output_voltage);
				user_delay_ms(2000);
				if (((mapped_value[1] >= (output_voltage - 20))
						&& (mapped_value[1] <= (output_voltage + 20)))
						&& ((mapped_value[2] >= (output_voltage - 20))
								&& (mapped_value[2] <= (output_voltage + 20)))
						&& ((mapped_value[3] >= (output_voltage - 20))
								&& (mapped_value[3] <= (output_voltage + 20)))) {
					gpio_bits_set(GPIOE, RELAY_2);
				}
				break;
			}
		}
		if (adc_interrupt_enabled) {
			//gpio_bits_reset(GPIOD, GPIO_PINS_11);
			lwip_rx_loop_handler();
			lwip_periodic_handle(local_time);

			adc_periodic_update(local_time);  // Handle ADC periodic update

			if (local_time - last_update_time >= 1000) {
// 1000ms has passed, perform the tasks
// Update the voltages and currents
				voltage_in_range_check();
				output_voltage_in_range_check();
				output_current_in_range_check();
				feedback_voltage();
				//send_data_to_arduino(start_marker,starting_frequency,phase1_amp,
						//phase2_amp, phase3_amp);
// Update the last update time
				last_update_time = local_time;
			}
			//gpio_bits_reset(GPIOD, GPIO_PINS_11);
			//send_data_to_arduino(start_marker,starting_frequency,phase1_amp,
									//phase2_amp, phase3_amp);
			adc_ordinary_software_trigger_enable(ADC1, TRUE);
			if (at32_emergency_button_press() == USER_BUTTON) {
				gpio_bits_set(GPIOD, GPIO_PINS_11);
				gpio_bits_set(GPIOD, SW_BUZZ_PIN);
				user_delay_ms(50);
				gpio_bits_reset(GPIOD, SW_BUZZ_PIN);
				adc_interrupt_enabled = 0;
				gpio_bits_reset(GPIOE, RELAY_0);
				gpio_bits_reset(GPIOE, RELAY_1);
				gpio_bits_reset(GPIOE, RELAY_2);
				system_state = 0;
			}
		}
	}
}
