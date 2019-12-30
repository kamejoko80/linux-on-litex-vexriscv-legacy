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

#include "can.h"

#ifdef CAN_CTRL_BASE

void sja1000_init(void)
{
    // Enter reset mode
    MOD = (1 << RM) | ( 1<< AFM);
    while((MOD & (1 << RM)) == 0);
    
    // Choose PeliCAN-Mode
	CDR = (1 << CANMODE) | 0x07;
    
	// Select the bitrate configuration
    BTR0 = 0x09;
    BTR1 = 0x2F;
    
	// Filter are not practical useable, so we disable them
    AMR0 = 0xFF;
    AMR1 = 0xFF;
    AMR2 = 0xFF;
    AMR3 = 0xFF;
    
	// Enable receive interrupt
	IER = (1 << RIE);
	
	// Leave reset-mode
	MOD = (1 << AFM);
    while((MOD & (1 << RM)) != 0);
}

void sja1000_set_mode(can_mode_t mode)
{
	uint8_t reg = 0;
	
	// Enter reset mode, enable self test mode
	MOD = (1 << AFM) | ( 1 << RM) | ( 1 << STM);
	
	if (mode == LISTEN_ONLY_MODE) 
    {
		reg = (1 << LOM);
	}
	else if (mode == LOOPBACK_MODE) 
    {
		reg = (1 << STM);
	}
	
	// Set new mode
	MOD = ( 1 << AFM ) | ( 1 << RM) | reg;
	
	// Leave reset mode
	MOD = (1<<AFM) | reg;
}

_Bool sja1000_check_message(void)
{
	// If RBS bit is set, there is a received message	
	return ((SR & (1 << RBS)) ? true : false);
}

_Bool sja1000_check_free_buffer(void)
{
	// If the TBS bit is set the CPU may write a message into the transmit buffer
	return ((SR & (1 << TBS)) ? true : false);
}

can_error_register_t sja1000_read_error_register(void)
{
	can_error_register_t error;
	
	error.tx = TXERR;
	error.rx = RXERR;
	
	return error;
}

_Bool sja1000_check_bus_off(void)
{
	return (SR & (1 << BS));
}

void sja1000_reset_bus_off(void)
{
	uint8_t status = SR & ( 1 << BS);
	SR = status & ~(1 << BS);
}

_Bool sja1000_get_message(can_t *msg)
{
	uint8_t frame_info;
	uint8_t address;
	
	// check if there is actually a message in the buffers
	if (!sja1000_check_message())
		return false;
	
	frame_info = RX_INFO;
	msg->length = frame_info & 0x0f;
	
	if (frame_info & (1<<FF))
	{
		// read extended identifier
		msg->flags.extended = 1;
		
		uint32_t tmp;
		uint8_t *ptr = (uint8_t *) &tmp;
		
		*ptr       = RX_DATA1;
		*(ptr + 1) = RX_DATA0;
		*(ptr + 2) = RX_ID0;
		*(ptr + 3) = RX_ID1;
        
		msg->id = tmp >> 3;
		
		/* equivalent to:
		msg->id	 = sja1000_read(20) >> 3;
		msg->id |= (uint16_t) sja1000_read(19) << 5;
		msg->id |= (uint32_t) sja1000_read(18) << 13;
		msg->id |= (uint32_t) sja1000_read(17) << 21;*/
		
		address = 21;
	}
	else
	{
		// read standard identifier
		msg->flags.extended = 0;
		
		uint32_t *ptr32 = &msg->id;		// used to supress a compiler warning
		uint16_t *ptr = (uint16_t *) ptr32;
		
		*(ptr + 1) = 0;
		
		*ptr  = RX_ID0 >> 5;
		*ptr |= RX_ID1 << 3;
		
		address = 19;
	}
	
	
	if (frame_info & (1<<RTR)) {
		msg->flags.rtr = 1;
	}
	else {
		msg->flags.rtr = 0;
        
		uint32_t *ptr = (uint32_t *) CAN_CTRL_BASE;
		
        // read data
		for (uint8_t i = 0; i < msg->length; i++) {
			msg->data[i] = (uint8_t)ptr[4*(address + i)]; 
		}
	}
	
	// release buffer
	CMR |= 1 << RRB;
	
	// CAN_INDICATE_RX_TRAFFIC_FUNCTION;
	
	return true;
}

static void delay(uint32_t n)
{
    uint32_t i, j;
    
    for(i=1; i<=n;i++)
    {
        for(j=0;j<100000;j++);
    }
}

_Bool sja1000_send_message(const can_t *msg)
{
	uint8_t frame_info;
	uint8_t address;
	
	if (!sja1000_check_free_buffer() || (msg->length > 8))
		return false;
	
	frame_info = msg->length | ((msg->flags.rtr) ? (1<<RTR) : 0);
	
	if (msg->flags.extended)
	{
		// write frame info
		TX_INFO = frame_info | (1 << FF);
		
		// write extended identifier
		TX_DATA1 = msg->id << 3;
		TX_DATA0 = msg->id >> 5;
		TX_ID0   = msg->id >> 13;
		TX_ID1   = msg->id >> 21;
        
		address = 21;
	}
	else
	{
		// write frame info
		TX_INFO = frame_info;
		
		const uint32_t *ptr32 = &msg->id;		// used to supress a compiler warning
		uint16_t *ptr = (uint16_t *) ptr32;
		
		// write standard identifier
		TX_ID0 = *ptr << 5;
		TX_ID1 = *ptr >> 3;
		
		address = 19;
	}
	
	if (!msg->flags.rtr)
	{
        uint32_t *ptr = (uint32_t *) CAN_CTRL_BASE;

		for (uint8_t i = 0;i < msg->length; i++) {
            ptr[4*(address + i)] = (uint32_t)msg->data[i];
		}
	}
	
	// send buffer
	CMR = 1 << TR;
	
    // Waitfor trasmit complete
    while((SR & (1<<TCS))==0)
    {
        printf("SR=%X\r\n");
        delay(1000);
    }
    
	//CAN_INDICATE_TX_TRAFFIC_FUNCTION;
	
	return true;
}

#endif /* CAN_CTRL_BASE */
