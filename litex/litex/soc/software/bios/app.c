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

/* This variable contains sample set */
volatile uint16_t g_sample[3];
volatile bool g_sendflag;

#ifdef PLATFORM_ACCEL_SIM
volatile uint16_t g_sample_num = 510;
uint16_t sample_buff[512];
#endif

#ifdef CSR_MBX_SND_BASE

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
#endif

#ifdef CSR_ACCEL_BASE

void soc_ready(void)
{
    accel_soc2ip_st_write(1);
}

void soc_not_ready(void)
{
    accel_soc2ip_st_write(0);
}

void csr_write_samples(uint16_t x, uint16_t y, uint16_t z)
{
    accel_soc2ip_dx_write(x);
    accel_soc2ip_dy_write(y);
    accel_soc2ip_dz_write(z);

    /* Wait for free FIFO */
    while(accel_soc2ip_full_read());
    accel_soc2ip_we_write(0);
    accel_soc2ip_we_write(1);
    accel_soc2ip_we_write(0);

    /* Wait until done */
    while(!accel_soc2ip_done_read());
}

#else

void soc_ready(void){}
void soc_not_ready(void){}
void csr_write_samples(uint16_t x, uint16_t y, uint16_t z){}

#endif

void convert_data(int16_t sample, uint16_t *val, uint8_t axis)
{
    if(sample < 0)
    {
        /* asb(sample) + 2^12 */
        *val = sample + 4096;
    }
    else
    {
        *val = sample;
    }

    /* signed extension bits */
    if(*val & 0x0800)
    {
        *val |= 0x3000;
    }
    else
    {
        *val &= 0xCFFF;
    }

    /* mask axis type bits */
    /* also for x axis */
    *val &= 0x3FFF;

    if(axis == 1) /* for y axis */
    {
        *val |= 0x4000;
    }
    else if (axis == 2) /* for z axis */
    {
        *val |= 0x8000;
    }
}

void reset_sample(void)
{
    g_sample[0] = 0;
    g_sample[1] = 0x4000;
    g_sample[2] = 0x8000;
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

bool convert_to_sample_set(char *str)
{
    char *p = str;
    int i = 0;
    int16_t sample[3];

    /* Find x, y, z number from string line */
    while (*p)
    {
        if (isdigit(*p) || ((*p=='-'||*p=='+') && isdigit(*(p+1))))
        {
            sample[i] = strtol(p, &p, 10);
            //printf("%d ", sample[i]);
            convert_data(sample[i], (uint16_t *)&g_sample[i], i);
            //printf("%x ", g_sample[i]);

            if(i < 3)
            {
                i++;
            }
            else
            {
                break;
            }
        }
        else
        {
            p++;
        }
    }

    /* Check there are enough samples */
    if(i != 3)
    {
        reset_sample();
        return false;
    }

    return true;
}

void accel_data_read(void)
{
    FATFS fs;
    FIL fil;
    char buffer[100];
    char *substr;
    bool soc_ready_flag = false;

#ifdef PLATFORM_ACCEL_SIM
    uint16_t i = 0;
    uint32_t crc_32;
#endif

    /* Mount SD Card */
    if(f_mount(&fs, "", 0) != FR_OK)
    {
        printf("SD Card mount failed\n");
        return;
    }

    /* Open file to read */
    if(f_open(&fil, "ACCEL.csv", FA_READ) != FR_OK)
    {
        printf("Open file error\n");
        return;
    }
    else
    {
        printf("Open file succesfully\n");
    }

    printf("Data sending...\n");

    reset_sample();

#ifdef CSR_GPIO_LED_BASE
    gpio_led_led_write(0);
#endif

    while(f_gets(buffer, sizeof(buffer), &fil))
    {
        substr = (char *)strchr(buffer, ',');
        convert_to_sample_set(substr);

        #ifdef PLATFORM_ACCEL_SIM
        if(g_sample_num != 0)
        {
            sample_buff[i++] = g_sample[0];
            sample_buff[i++] = g_sample[1];
            sample_buff[i++] = g_sample[2];

            /* Send CRC to tester */
            if(i == g_sample_num)
            {
                crc_32 = crc32((const unsigned char *)sample_buff, g_sample_num*2);
                mbx_snd_dout_write((uint8_t)crc_32);
                mbx_snd_dout_write((uint8_t)(crc_32 >> 8));
                mbx_snd_dout_write((uint8_t)(crc_32 >> 16));
                mbx_snd_dout_write((uint8_t)(crc_32 >> 24));
                i = 0;
            }
        }
        #endif

        /* Data dump debugging */
        dump_sample(g_sample[0]);
        dump_sample(g_sample[1]);
        dump_sample(g_sample[2]);
        printf("\n");

        /* Notify that data ready for transfering */
        if(!soc_ready_flag)
        {
            soc_ready_flag = true;
            soc_ready();
        }

        /* Just wating for interrupt complete */
        g_sendflag = true;
        while(g_sendflag);
    }

    //soc_not_ready();

    /* Close file */
    if(f_close(&fil) != FR_OK)
    {
        printf("Close file error\n");
    }

    /* Unmount SDCARD */
    if(f_mount(NULL, "", 1) != FR_OK)
    {
        printf("SD Card unmount error\n");
    }

    printf("Done\n");

#ifdef CSR_GPIO_LED_BASE
    gpio_led_led_write(1);
#endif

}

void main_app (void)
{
    /* Init Fatfs */
    MX_FATFS_Init();

    printf("Start ADXL362 accelerometer simulator\n");

    /* Read data and send */
    accel_data_read();

    /* Reboot the SoC */
    //ctrl_reset_write(1);
    while(1);
}

#ifdef ACCEL_INTERRUPT
void accel_irq (void)
{
    csr_write_samples(g_sample[0], g_sample[1], g_sample[2]);
    //printf("%4X %4X %4X\n", g_sample[0], g_sample[1], g_sample[2]);
    g_sendflag = false;
}
#endif

#ifdef MBX_RCV_INTERRUPT
void mbx_rcv_irq (void)
{

#ifdef PLATFORM_ACCEL_SIM
    uint8_t hi, low;

    printf("receive mailbox len = %d\n", mbx_rcv_len_read());

    /* Accel test soc send 2 bytes checking */
    if(mbx_rcv_len_read() == 2)
    {
        low = mbx_rcv_din_read();
        mbx_rcv_rd_write(1);
        hi = mbx_rcv_din_read();
        mbx_rcv_rd_write(1);
        g_sample_num = (hi << 8) | low;
        printf("Setting sample number %d\n", g_sample_num);

        /* Send the response */
        mbx_send_msg((uint8_t *)"O", 1);
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

    /* Send the echo message */
    mbx_send_msg((uint8_t *)"hello accel test program\n", strlen("hello accel test program\n"));
#endif

}
#endif