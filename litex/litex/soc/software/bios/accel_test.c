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
#include "spi.h"

#define DATA_INTEGRITY_CHECK

#define CPU_FREQ (100000000)
#define AXIS_NUM (510) // should be 3*n

uint16_t buff[512];
volatile uint16_t sample_num;
volatile bool g_data_available = false;

#ifdef DATA_INTEGRITY_CHECK
volatile uint32_t g_crc_32 = 0;
volatile char g_response = 0;

void HAL_Delay(uint32_t n)
{
    uint32_t i, j;
    for(i = 0; i < n; i++)
    {
        for(j = 0; j < CPU_FREQ/1000; j++);
    }
}

void get_crc32_from_accel_simulator(void)
{
    uint8_t *p = (uint8_t *)&g_crc_32;
    uint8_t i;

    /* Receive CRC32 check number from accel simulator */
    if(mbx_rcv_len_read() == 4)
    {
        for(i = 0; i < 4; i++)
        {
            p[i] = mbx_rcv_din_read();
            mbx_rcv_rd_write(1);
        }
    }

    /* To make sure the FIFO is empty */
    while(mbx_rcv_len_read())
    {
        mbx_rcv_rd_write(1);
    }
}

#endif

#ifdef CSR_MBX_SND_BASE

char *msg = "hello accel simulator\n";

void mbx_send_msg(uint8_t *buff, uint8_t len)
{
    uint8_t i;

    /* Send data */
    for(i = 0; i < len; i++)
    {
        mbx_snd_dout_write(buff[i]);
    }

    /* Notify to the receiver */
    mbx_snd_int_write(1);
}

void mbx_message(void)
{
    mbx_send_msg((uint8_t *)msg, strlen(msg));
}

#endif

uint16_t accel_read_reg(uint8_t addr)
{
    uint8_t reg;

    spi_csn_active();
    spi_byte_transfer(0x0B);
    spi_byte_transfer(addr);
    reg = spi_byte_transfer(0x00);
    spi_csn_inactive();

    return reg;
}

void accel_write_reg(uint8_t addr, uint8_t value)
{
    spi_csn_active();
    spi_byte_transfer(0x0A);
    spi_byte_transfer(addr);
    spi_byte_transfer(value);
    spi_csn_inactive();
}

void read_sample_num(void)
{
    sample_num = accel_read_reg(13);
    sample_num = sample_num << 8;
    sample_num |= accel_read_reg(12);
}

void dump_sample(uint16_t sample)
{
    if(sample == 0)
    {
        printf("0000 ");
    }
    else if (0x10 > sample)  // 1 digit
    {
        printf("000%X ", sample);
    }
    else if (0x100 > sample) // 2 digits
    {
        printf("00%X ", sample);
    }
    else if (0x1000 > sample) // 3 digits
    {
        printf("0%X ", sample);
    }
    else
    {
        printf("%X ", sample);
    }
}

void dump_fifo_data(void)
{
    uint32_t j;

    for(j = 0; j < AXIS_NUM; j++)
    {
        dump_sample(buff[j]);

        if(((j + 1) % 3)==0 && j != 0)
        {
            printf("\n");
        }
    }

#ifdef DATA_INTEGRITY_CHECK
    uint32_t crc_32;

    get_crc32_from_accel_simulator();

    crc_32 = crc32((const unsigned char *)buff, AXIS_NUM*2);

    if(crc_32 != g_crc_32)
    {
        printf("crc check error!\n");
        printf("simulator crc = %X\n", g_crc_32);
        printf("test crc      = %X\n", crc_32);
    }
    else
    {
        printf("crc check ok!\n");
    }
#endif

    printf("\n");
}

void accel_read_fifo(uint32_t size)
{
    uint32_t j;
    uint16_t low_byte, high_byte;
    //static uint16_t k = 0;
    for(j = 0; j < size; j++)
    {
        spi_csn_active();
        spi_byte_transfer(0x0D);
        low_byte = spi_byte_transfer(0x00);
        high_byte = spi_byte_transfer(0x00);
        spi_csn_inactive();

        buff[j] = (high_byte << 8) | low_byte;
        //printf("%4X ", buff[j]);

        //if((j % 3)==0 && j != 0)
        //{
        //    printf("\n");
        //}
    }

    //k++;

    //if(k==3)
    //{
      // POWER_CTL    = Stop measure
    //   accel_write_reg(45, 0x00);
    //}
}

void gpio_irq(void)
{
    read_sample_num();

    if(sample_num >= AXIS_NUM)
    {
      accel_read_fifo(AXIS_NUM);
      g_data_available = true;
    }
}

void accel_test(void)
{
    printf("Accel test program start\n");

    printf("ID             = %X\n", accel_read_reg(0));
    printf("FIFO_ENTRIES_L = %X\n", accel_read_reg(12));
    printf("FIFO_ENTRIES_H = %X\n", accel_read_reg(13));
    printf("POWR_CTRL      = %X\n", accel_read_reg(45));

    accel_write_reg(31, 0x52); // SOFT_RESET   = Soft reset (0x52)

#ifdef DATA_INTEGRITY_CHECK
    /* Send sample num to accel simulator */
    //while(g_response != 'O')
    //{
    //    sample_num = AXIS_NUM;
    //    mbx_send_msg((uint8_t *)&sample_num, 2);
    //    HAL_Delay(500);
    //}
#endif

    /* Set mode fifo streamming */
    printf("Setting FIFO stream mode\n");

    if(AXIS_NUM > 255)
    {
       accel_write_reg(40, 0x0A);           // FIFO_CONTROL = Stream mode
    }
    else
    {
        accel_write_reg(40, 0x02);          // FIFO_CONTROL = Stream mode
    }

    accel_write_reg(41, (uint8_t)AXIS_NUM); // FIFO_SAMPLES = 60 (should be = 3N)
    accel_write_reg(44, 3);                 // FILTER_CTL   => ODR = 0 (25 Hz)
    accel_write_reg(42, 0x84);              // INTMAP1 with FIFO_WATERMARK interrupt (active low)
    accel_write_reg(43, 0x84);              // INTMAP2 with FIFO_WATERMARK interrupt (active low)
    accel_write_reg(45, 0x02);              // POWER_CTL    = start measure

    while(readchar_nonblock() == 0)
    {
        /* Wait for data available */
        while(!g_data_available && readchar_nonblock() == 0);
        dump_fifo_data();
        g_data_available = false;
    }

    accel_write_reg(45, 0x00);     // POWER_CTL    = Stop measure

    printf("Accel test program end\n");
}

#ifdef MBX_RCV_INTERRUPT
void mbx_rcv_irq (void)
{

#ifdef DATA_INTEGRITY_CHECK
    /* Check response from accel simulator */
    if(mbx_rcv_len_read() == 1)
    {
        g_response = (char)mbx_rcv_din_read();
        mbx_rcv_rd_write(1);
    }

    /* To make sure the FIFO is empty */
    while(mbx_rcv_len_read())
    {
        mbx_rcv_rd_write(1);
    }

#else
    printf("msg len = %d\n", mbx_rcv_len_read());

    while(mbx_rcv_len_read())
    {
        /*
         * First FIFO entry is always ready for reading,
         * so read fifo.dout before assert fifo.re for the next entries
         * otherwise 1st FIFO entry will be missed
         */
        printf("%c", mbx_rcv_din_read());
        mbx_rcv_rd_write(1); /* Move to next FIFO entry */
    }
#endif
}
#endif