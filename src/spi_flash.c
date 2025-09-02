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

#include "spi_flash.h"
#include <stdio.h>

/** @addtogroup AT32F407_periph_examples
  * @{
  */

/** @addtogroup 407_SPI_w25q_flash
  * @{
  */

uint8_t spiflash_sector_buf[SPIF_SECTOR_SIZE];

/**
  * @brief  spi configuration.
  * @param  none
  * @retval none
  */
void spiflash_init(void)
{
  gpio_init_type gpio_initstructure;
  spi_init_type spi_init_struct;

  crm_periph_clock_enable(CRM_GPIOE_PERIPH_CLOCK, TRUE);
  crm_periph_clock_enable(CRM_DMA2_PERIPH_CLOCK, TRUE);

  /* software cs, pe12 as a general io to control flash cs */
  gpio_initstructure.gpio_out_type       = GPIO_OUTPUT_PUSH_PULL;
  gpio_initstructure.gpio_pull           = GPIO_PULL_UP;
  gpio_initstructure.gpio_mode           = GPIO_MODE_OUTPUT;
  gpio_initstructure.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
  gpio_initstructure.gpio_pins           = GPIO_PINS_4;
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
  gpio_initstructure.gpio_out_type  = GPIO_OUTPUT_PUSH_PULL;
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
void spiflash_write(uint8_t *pbuffer, uint32_t write_addr, uint32_t length)
{
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
  if(length <= sector_remain)
  {
    /* smaller than a sector size */
    sector_remain = length;
  }

  //printf("Starting write operation...\r\n");
  //printf("Initial write address: 0x%X, length: %d\r\n", write_addr, length);

  while(1)
  {
    /* read a sector */
	//printf("Reading sector at address: 0x%X\r\n", sector_pos * SPIF_SECTOR_SIZE);
    spiflash_read(spiflash_buf, sector_pos * SPIF_SECTOR_SIZE, SPIF_SECTOR_SIZE);

    //printf("Sector read complete. First 16 bytes of buffer: ");
    for (index = 0; index < 16; index++) printf("0x%X ", spiflash_buf[index]);
    //printf("\r\n");

    /* validate the read erea */
    for(index = 0; index < sector_remain; index++)
    {
      if(spiflash_buf[sector_offset + index] != 0xFF)
      {
        /* there are some data not equal 0xff, so this secotr needs erased */
    	//printf("Sector not erased at offset: %d\r\n", sector_offset + index);
        break;
      }
    }
    if(index < sector_remain)
    {
      /* erase the sector */
      //printf("Erasing sector at address: 0x%X\r\n", sector_pos * SPIF_SECTOR_SIZE);
      spiflash_sector_erase(sector_pos);

      /* copy the write data */
      //printf("Copying data to sector buffer...\r\n");
      for(index = 0; index < sector_remain; index++)
      {
        spiflash_buf[index + sector_offset] = pbuffer[index];
      }

      //printf("Writing sector to address: 0x%X\r\n", sector_pos * SPIF_SECTOR_SIZE);
      spiflash_write_nocheck(spiflash_buf, sector_pos * SPIF_SECTOR_SIZE, SPIF_SECTOR_SIZE); /* program the sector */
      //printf("Sector write complete.\r\n");
    }
    else
    {
      /* write directly in the erased area */
      //printf("Writing directly to erased area: address 0x%X, length %d\r\n", write_addr, sector_remain);
      spiflash_write_nocheck(pbuffer, write_addr, sector_remain);
      //printf("Direct write complete.\r\n");
    }
    if(length == sector_remain)
    {
      /* write end */
      //printf("Write operation completed successfully.\r\n");
      break;
    }
    else
    {
      /* go on writing */
      //printf("Continuing to the next sector...\r\n");
      sector_pos++;
      sector_offset = 0;

      pbuffer += sector_remain;
      write_addr += sector_remain;
      length -= sector_remain;
      if(length > SPIF_SECTOR_SIZE)
      {
        /* could not write the remain data in the next sector */
        sector_remain = SPIF_SECTOR_SIZE;
      }
      else
      {
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
void spiflash_read(uint8_t *pbuffer, uint32_t read_addr, uint32_t length)
{
  //printf("spiflash_read: Starting read operation...\r\n");
  //printf("Read address: 0x%X, Length: %d\r\n", read_addr, length);
  FLASH_CS_LOW();
  //printf("CS pulled LOW.\r\n");
  spi_byte_write(SPIF_READDATA); /* send instruction */
  //printf("Sent READ command: 0x%X\r\n", SPIF_READDATA);
  spi_byte_write((uint8_t)((read_addr) >> 16)); /* send 24-bit address */
  spi_byte_write((uint8_t)((read_addr) >> 8));
  spi_byte_write((uint8_t)read_addr);
  //printf("Sent 24-bit address: 0x%X\r\n", read_addr);

  // Read data into buffer
  //printf("Reading %d bytes...\r\n", length);
  spi_bytes_read(pbuffer, length);
  for (uint32_t i = 0; i < length; i++)
  {
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
void spiflash_sector_erase(uint32_t erase_addr)
{
  erase_addr *= SPIF_SECTOR_SIZE; /* translate sector address to byte address */
  spiflash_write_enable();
  spiflash_wait_busy();
  FLASH_CS_LOW();
  spi_byte_write(SPIF_SECTORERASE);
  spi_byte_write((uint8_t)((erase_addr) >> 16));
  spi_byte_write((uint8_t)((erase_addr) >> 8));
  spi_byte_write((uint8_t)erase_addr);
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
void spiflash_write_nocheck(uint8_t *pbuffer, uint32_t write_addr, uint32_t length)
{
  uint16_t page_remain;

  /* remain bytes in a page */
  page_remain = SPIF_PAGE_SIZE - write_addr % SPIF_PAGE_SIZE;
  if(length <= page_remain)
  {
    /* smaller than a page size */
    page_remain = length;
  }
  while(1)
  {
    spiflash_page_write(pbuffer, write_addr, page_remain);
    if(length == page_remain)
    {
      /* all data are programmed */
      break;
    }
    else
    {
      /* length > page_remain */
      pbuffer += page_remain;
      write_addr += page_remain;

      /* the remain bytes to be programmed */
      length -= page_remain;
      if(length > SPIF_PAGE_SIZE)
      {
        /* can be programmed a page at a time */
        page_remain = SPIF_PAGE_SIZE;
      }
      else
      {
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
void spiflash_page_write(uint8_t *pbuffer, uint32_t write_addr, uint32_t length)
{
  if((0 < length) && (length <= SPIF_PAGE_SIZE))
  {
    /* set write enable */
    spiflash_write_enable();

    FLASH_CS_LOW();

    /* send instruction */
    spi_byte_write(SPIF_PAGEPROGRAM);

    /* send 24-bit address */
    spi_byte_write((uint8_t)((write_addr) >> 16));
    spi_byte_write((uint8_t)((write_addr) >> 8));
    spi_byte_write((uint8_t)write_addr);
    spi_bytes_write(pbuffer,length);

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
void spi_bytes_write(uint8_t *pbuffer, uint32_t length)
{
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
  while(length--)
  {
    while(spi_i2s_flag_get(SPI4, SPI_I2S_TDBE_FLAG) == RESET);
    spi_i2s_data_transmit(SPI4, *pbuffer);
    while(spi_i2s_flag_get(SPI4, SPI_I2S_RDBF_FLAG) == RESET);
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
void spi_bytes_read(uint8_t *pbuffer, uint32_t length)
{
  uint8_t write_value = FLASH_SPI_DUMMY_BYTE;

  printf("spi_bytes_read: Starting read operation for %d bytes...\r\n", length);

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
  while(length--)
  {
	//printf("Sending dummy byte: 0x%X\r\n", write_value);
    while(spi_i2s_flag_get(SPI4, SPI_I2S_TDBE_FLAG) == RESET);
    spi_i2s_data_transmit(SPI4, write_value);
    while(spi_i2s_flag_get(SPI4, SPI_I2S_RDBF_FLAG) == RESET);
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
void spiflash_wait_busy(void)
{
  while((spiflash_read_sr1() & 0x01) == 0x01);
}

/**
  * @brief  read sr1 register
  * @param  none
  * @retval none
  */
uint8_t spiflash_read_sr1(void)
{
  uint8_t breadbyte = 0;
  FLASH_CS_LOW();
  spi_byte_write(SPIF_READSTATUSREG1);
  breadbyte = (uint8_t)spi_byte_read();
  FLASH_CS_HIGH();
  return (breadbyte);
}

/**
  * @brief  enable write operation
  * @param  none
  * @retval none
  */
void spiflash_write_enable(void)
{
  FLASH_CS_LOW();
  spi_byte_write(SPIF_WRITEENABLE);
  FLASH_CS_HIGH();
}

/**
  * @brief  read device id
  * @param  none
  * @retval device id
  */
uint16_t spiflash_read_id(void)
{
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
uint8_t spi_byte_write(uint8_t data)
{
  uint8_t brxbuff;
  spi_i2s_dma_transmitter_enable(SPI4, FALSE);
  spi_i2s_dma_receiver_enable(SPI4, FALSE);
  spi_i2s_data_transmit(SPI4, data);
  while(spi_i2s_flag_get(SPI4, SPI_I2S_RDBF_FLAG) == RESET);
  brxbuff = spi_i2s_data_receive(SPI4);
  while(spi_i2s_flag_get(SPI4, SPI_I2S_BF_FLAG) != RESET);
  return brxbuff;
}

/**
  * @brief  read a byte to flash
  * @param  none
  * @retval flash return data
  */
uint8_t spi_byte_read(void)
{
  return (spi_byte_write(FLASH_SPI_DUMMY_BYTE));
}

/**
  * @}
  */

/**
  * @}
  */

