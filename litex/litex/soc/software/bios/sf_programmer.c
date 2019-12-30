#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <crc.h>
#include <uart.h>
#include <console.h>
#include <generated/csr.h>
#include <generated/mem.h>
#include "fatfs.h"
#include "boot.h"

#ifdef CSR_SPI_MASTER_0_BASE
void spi0_init(void)
{
    /* IE=0, IPOL=0, CPHA=0, CPOL=0 */
    spi_master_0_config_write(0x00);
}

void spi0_csn_active(void)
{
    spi_master_0_csn_write(0);
}

void spi0_csn_inactive(void)
{
    spi_master_0_csn_write(1);
}

void spi0_byte_transfer(uint8_t txbyte, uint8_t *rxbuff)
{
    spi_master_0_tx_data_write(txbyte);
    spi_master_0_start_write(1);
    while(!spi_master_0_done_read());

    if(rxbuff != NULL)
    {
        *rxbuff = spi_master_0_rx_data_0_read();
    }
}

void sf_read_rdid(void)
{
    uint8_t mid;
    uint8_t did_low, did_high;

    spi0_csn_active();

    /* Read DID command */
    spi0_byte_transfer(0x9F, NULL);
    spi0_byte_transfer(0x00, &mid);
    spi0_byte_transfer(0x00, &did_high);
    spi0_byte_transfer(0x00, &did_low);

    spi0_csn_inactive();

    printf("Manufacture ID = %X\n", mid);
    printf("Device ID High = %X\n", did_high);
    printf("Device ID Low  = %X\n", did_low);
}

void sf_read_manufacturer(void)
{
    uint8_t mid;
    uint8_t did;

    spi0_csn_active();

    /* Read DID command */
    spi0_byte_transfer(0x90, NULL);
    spi0_byte_transfer(0x00, NULL); // Dummy
    spi0_byte_transfer(0x00, NULL); // Dummy
    spi0_byte_transfer(0x00, NULL); // ADDR

    spi0_byte_transfer(0x00, &mid);
    spi0_byte_transfer(0x00, &did);

    spi0_csn_inactive();

    printf("Manufacture ID = %X\n", mid);
    printf("Device ID      = %X\n", did);
}
#endif

#ifdef CSR_SPI_MASTER_1_BASE
void spi1_init(void)
{
    /* IE=0, IPOL=0, CPHA=0, CPOL=0 */
    spi_master_1_config_write(0x00);
}

void spi1_csn_active(void)
{
    spi_master_1_csn_write(0);
}

void spi1_csn_inactive(void)
{
    spi_master_1_csn_write(1);
}

void spi1_byte_transfer(uint8_t txbyte, uint8_t *rxbuff, uint8_t len)
{
    spi_master_1_tx_data_write(txbyte);
    spi_master_1_start_write(1);
    while(!spi_master_1_done_read());

    if((rxbuff != NULL) && (len >=4))
    {
        *rxbuff++ = spi_master_1_rx_data_0_read();
        *rxbuff++ = spi_master_1_rx_data_1_read();
        *rxbuff++ = spi_master_1_rx_data_2_read();
        *rxbuff++ = spi_master_1_rx_data_3_read();
    }
}
#endif

void main_app (void)
{
   printf("SPI flash programmer\n");

   spi0_init();
   sf_read_rdid();
   sf_read_manufacturer();

   return;
}
