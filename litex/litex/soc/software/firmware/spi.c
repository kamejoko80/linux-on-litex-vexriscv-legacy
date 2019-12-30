#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <console.h>
#include <string.h>
#include <uart.h>
#include <system.h>
#include <id.h>
#include <irq.h>
#include <crc.h>

#include <generated/csr.h>
#include <generated/mem.h>

#include "spi.h"

#ifdef SPI_MASTER_BASE

/* Function prototype */
void spi_irq (void);

void spi_init(void)
{

#if 1 /* SD card interface */
    /*
     * sclk = wb_clk /((SPI_DIV + 1) x 2)
     *
     */
    SPI_DIV   = 0x00;  // fclk/2 ~ 24MHz
    SPI_CTRL  = (1 << TX_NEG);
    // SPI_CTRL |= (1 << IE);
    SPI_CTRL |= 0x08;  // 8 byte transfer
    SPI_SS    = 0x00;  // cns inactive

#endif


#if 0 /* Test spi slave loopback */
    /*
     * sclk = wb_clk /((SPI_DIV + 1) x 2)
     *
     */
    SPI_DIV   = 0x03;  // fclk/8 ~ 6MHz
    SPI_CTRL  = (1 << TX_NEG);
    // SPI_CTRL |= (1 << IE);
    SPI_CTRL |= 0x08;  // 8 byte transfer
    SPI_SS    = 0x00;  // cns inactive

#endif

#if 0 /* Test ADC128S102 */
    /*
     * sclk = wb_clk /((SPI_DIV + 1) x 2)
     *
     */
    SPI_DIV   = 0x01;  // fclk/4 ~ 12MHz
    SPI_CTRL  = (1 << ASS) | (1 << TX_NEG);
    // SPI_CTRL |= (1 << IE);
    SPI_CTRL |= 0x10;  // 16 byte transfer
    SPI_SS    = 0x01;  // cns = 0

    //printf("SPI_DIV  = %X\r\n", SPI_DIV);
    //printf("SPI_CTRL = %X\r\n", SPI_CTRL);
    //printf("SPI_SS   = %X\r\n", SPI_SS);
#endif
}

uint16_t spi_byte_transfer(uint8_t byte)
{
    SPI_TX0   = byte;
    SPI_CTRL |= (1 << GO_BSY);
    while(SPI_CTRL & (1 << GO_BSY));
    return SPI_RX0;
}

void delay(uint32_t n);
void delay(uint32_t n)
{
    for(int i=0; i<n; i++)
        for(int j=0; j<100; j++);
}

void spi_csn_active(void)
{
    SPI_SS = 0x01;
    //delay(10);
}

void spi_csn_inactive(void)
{
    SPI_SS = 0x00;
}

uint16_t spi_adc_read(uint8_t chanel)
{
    SPI_TX0   = (chanel << 11);
    SPI_CTRL |= (1 << GO_BSY);
    while(SPI_CTRL & (1 << GO_BSY));
    return SPI_RX0;
}

void spi_irq (void)
{
    uint32_t dummy;
    printf("spi master irq\n");
    dummy = SPI_CTRL;
    dummy = dummy;
}

#endif /* SPI_MASTER_BASE */
