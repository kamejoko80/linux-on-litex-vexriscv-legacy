#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <ctype.h>
#include <uart.h>
#include <console.h>
#include <generated/csr.h>
#include <generated/mem.h>
#include "fatfs.h"
#include "boot.h"

/* This variable contains sample set */
uint16_t g_sample[3];
volatile bool g_sendflag;

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
        /* asb(sample) + 2^11 */
        *val = abs(sample) + 2048;
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
            convert_data(sample[i], &g_sample[i], i);
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
  
    soc_ready();
  
	while(f_gets(buffer, sizeof(buffer), &fil))
	{
        substr = (char *)strchr(buffer, ',');
        convert_to_sample_set(substr);
        printf("%4X %4X %4X\n", g_sample[0], g_sample[1], g_sample[2]);
      
        /* Just wating for interrupt complete */    
        g_sendflag = true;
        while(g_sendflag);
	}
    
    soc_not_ready();
    
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
    
}

void main_app (void)
{
    /* Init Fatfs */
    MX_FATFS_Init();
    
    printf("Start ADXL362 accelerometer simulator\n");

    /* Repeat sending data infinity */
    while(1)
    {
        accel_data_read();
    }
    
    /* Reboot the SoC */
    //ctrl_reset_write(1);
    while(1);
}

#ifdef CSR_ACCEL_BASE
void accel_irq (void)
{ 
    csr_write_samples(g_sample[0], g_sample[1], g_sample[2]);
    //printf("%4X %4X %4X\n", g_sample[0], g_sample[1], g_sample[2]);
    g_sendflag = false;
}
#endif