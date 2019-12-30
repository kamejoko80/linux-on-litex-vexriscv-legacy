#include <stdio.h>
#include <stdlib.h>
#include <console.h>
#include <string.h>
#include <uart.h>
#include <system.h>
#include <id.h>
#include <irq.h>
#include <crc.h>

#include <generated/csr.h>
#include <generated/mem.h>

#ifdef CSR_ETHMAC_BASE
#include <net/microudp.h>
#endif

#include "sdram.h"
#include "boot.h"
#include "can.h"
#include "spi.h"

extern void main_app(void);
extern void accel_test(void);
extern void mbx_message(void);

#ifdef FLASH_BOOT_ADDRESS
extern void flash_boot_raw(void);
#endif

/* General address space functions */
#ifdef ACCEL_INTERRUPT
void accel_isr_init(void);
void accel_isr_init(void)
{
  accel_ev_pending_write(accel_ev_pending_read());
  accel_ev_enable_write(1);
  irq_setmask(irq_getmask() | (1 << ACCEL_INTERRUPT));
}
#endif

#ifdef MBX_RCV_INTERRUPT
void mbx_rcv_isr_init(void);
void mbx_rcv_isr_init(void)
{
  mbx_rcv_ev_pending_write(mbx_rcv_ev_pending_read());
  mbx_rcv_ev_enable_write(1);
  irq_setmask(irq_getmask() | (1 << MBX_RCV_INTERRUPT));
}
#endif

#ifdef SPI_MASTER_INTERRUPT
void spi_master_isr_init(void);
void spi_master_isr_init(void)
{
  spi_master_ev_pending_write(spi_master_ev_pending_read());
  spi_master_ev_enable_write(1);
  irq_setmask(irq_getmask() | (1 << SPI_MASTER_INTERRUPT));
}
#endif

#ifdef GPIO_ISR_INTERRUPT
void gpio_isr_init(void);
void gpio_isr_init(void)
{
  gpio_isr_ev_pending_write(gpio_isr_ev_pending_read());
  gpio_isr_ev_enable_write(1);
  irq_setmask(irq_getmask() | (1 << GPIO_ISR_INTERRUPT));
}
#endif

#ifdef SPI_MASTER_BASE
static void adc_read(char *chanel)
{
    char *c;
    uint16_t adc;
    unsigned char chan;

    if(*chanel == 0) {
        printf("adc <chanel>\n");
        return;
    }

    chan = strtoul(chanel, &c, 0);
    if(*c != 0) {
        printf("incorrect chanel\n");
        return;
    }

    /* Workaround adc sample delay problem */
    adc = spi_adc_read(chan);
    adc = spi_adc_read(chan);
    printf("adc = %X\r\n", adc);
}
#endif

#ifdef CAN_CTRL_INTERRUPT
void can_ctrl_isr_init(void);
void can_ctrl_isr_init(void)
{
  can_ctrl_ev_pending_write(can_ctrl_ev_pending_read());
  can_ctrl_ev_enable_write(1);
  irq_setmask(irq_getmask() | (1 << CAN_CTRL_INTERRUPT));
}
#endif

#ifdef CSR_ADDER8_BASE
static void adder8(char *op1, char *op2)
{
	char *c;
	unsigned char opr1;
	unsigned char opr2;

	if((*op1 == 0) || (*op1 == 0)) {
		printf("adder8 <op1> <op2>\n");
		return;
	}

	opr1 = strtoul(op1, &c, 0);
	if(*c != 0) {
		printf("incorrect op1 value\n");
		return;
	}

	opr2 = strtoul(op2, &c, 0);
	if(*c != 0) {
		printf("incorrect op2 value\n");
		return;
	}

    adder8_op1_write(opr1);
    adder8_op2_write(opr2);
    adder8_ena_write(1);
    printf("sum = %d\n", adder8_sum_read());
    adder8_ena_write(0);
}
#endif

#ifdef CSR_MY_UART_BASE
static void my_uart_put_char(char c)
{
    while(my_uart_tx_bsy_read());
    my_uart_tx_dat_write((unsigned char)c);
    my_uart_tx_ena_write(1);
    my_uart_tx_ena_write(0);
}

static void my_uart_print(char *message)
{
    int i;

    for(i=0; i<strlen(message); i++)
    {
        my_uart_put_char(message[i]);
    }
}
#endif

#ifdef CAN_CTRL_BASE
static void canregs(void)
{
    int i;
    unsigned int *reg = (unsigned int*)CAN_CTRL_BASE;
    for(i = 0; i < 32; i++)
    {
        printf("Reg %u : %X h\n", i, *reg);
        reg++;
    }
}

static void delay(uint32_t n)
{
    uint32_t i, j;

    for(i=1; i<=n;i++)
    {
        for(j=0;j<100000;j++);
    }
}

static void basic_can_self_test_init(void)
{
    // Enter basic can mode
    CDR = 0x03;

    // Enter reset mode, enable transmit interrupt
    CR = (1 << CR_RR) | ( 1<< CR_TIE);

    // Can baudrate setting
    BTR0 = 0x09;
    BTR1 = 0x2F;

    // Clear TX, RX error counter
    EWL   = 0xFF;
    RXERR = 0x00;
    TXERR = 0x00;

    // Set Acceptance Code and Acceptance Mask registers
    ACR = 0xFF; // acceptance code
    AMR = 0xFF; // acceptance mask

    // Switch-off reset mode
    CR &= ~(1 << CR_RR);  // reset_off, all irqs enabled.
}

static void basic_can_clr_tx_rx_error(void)
{
    // Enter reset mode
    CR |= (1 << CR_RR);

    // Clear TX, RX error counter
    EWL = 0xFF;
    RXERR = 0x00;
    TXERR = 0x00;

    // Switch-off reset mode
    CR &= ~(1 << CR_RR);
}

static void basic_can_self_test(void)
{
    static uint32_t timeout;

    basic_can_self_test_init();

    /* Wait for transmit buffer ready */
    while((SR & (1 << TBS))==0);

    TX_DATA_0 = 0x55; // Writing ID[10:3] = 0x55
    TX_DATA_1 = 0x07; // Writing ID[2:0] = 0x3, rtr = 0, length = 7
    TX_DATA_2 = 0xAA; // data byte 1
    TX_DATA_3 = 0xAA; // data byte 2
    TX_DATA_4 = 0xAA; // data byte 3
    TX_DATA_5 = 0xAA; // data byte 4
    TX_DATA_6 = 0xAA; // data byte 5
    TX_DATA_7 = 0xAA; // data byte 6
    TX_DATA_8 = 0xAA; // data byte 7
    TX_DATA_9 = 0x00; // data byte 8

    // Transmit, receive request
	CMR |= (1 << TR);

    // Wait for trasmition complete
    timeout = 0;
    while((SR & (1<<TCS))==0)
    {
        printf("SR=%X\r\n", SR);
        delay(10000);
        timeout++;
        if(timeout >= 50)
        {
            printf("Error trasmit timeout\r\n");
            printf("EWL   = %X\r\n", EWL);
            printf("RXERR = %X\r\n", RXERR);
            printf("TXERR = %X\r\n", TXERR);
            basic_can_clr_tx_rx_error();
            return;
        }
    }
}

static void can_transmit_demo(void)
{
   basic_can_self_test();
   printf("Can sent\r\n");
}
#endif

#define NUMBER_OF_BYTES_ON_A_LINE 16
static void dump_bytes(unsigned int *ptr, int count, unsigned long addr)
{
	char *data = (char *)ptr;
	int line_bytes = 0, i = 0;

	putsnonl("Memory dump:");
	while(count > 0){
		line_bytes =
			(count > NUMBER_OF_BYTES_ON_A_LINE)?
				NUMBER_OF_BYTES_ON_A_LINE : count;

		printf("\n0x%08x  ", addr);
		for(i=0;i<line_bytes;i++)
			printf("%02x ", *(unsigned char *)(data+i));

		for(;i<NUMBER_OF_BYTES_ON_A_LINE;i++)
			printf("   ");

		printf(" ");

		for(i=0;i<line_bytes;i++) {
			if((*(data+i) < 0x20) || (*(data+i) > 0x7e))
				printf(".");
			else
				printf("%c", *(data+i));
		}

		for(;i<NUMBER_OF_BYTES_ON_A_LINE;i++)
			printf(" ");

		data += (char)line_bytes;
		count -= line_bytes;
		addr += line_bytes;
	}
	printf("\n");
}

static void mr(char *startaddr, char *len)
{
	char *c;
	unsigned int *addr;
	unsigned int length;

	if(*startaddr == 0) {
		printf("mr <address> [length]\n");
		return;
	}
	addr = (unsigned *)strtoul(startaddr, &c, 0);
	if(*c != 0) {
		printf("incorrect address\n");
		return;
	}
	if(*len == 0) {
		length = 4;
	} else {
		length = strtoul(len, &c, 0);
		if(*c != 0) {
			printf("incorrect length\n");
			return;
		}
	}

	dump_bytes(addr, length, (unsigned long)addr);
}

static void mrb(char *startaddr, char *len)
{
	char *c;
	unsigned int *addr;
	unsigned int length;

	if(*startaddr == 0) {
		printf("mr <address> [length]\n");
		return;
	}
	addr = (unsigned *)strtoul(startaddr, &c, 0);
	if(*c != 0) {
		printf("incorrect address\n");
		return;
	}
	if(*len == 0) {
		length = 4;
	} else {
		length = strtoul(len, &c, 0);
		if(*c != 0) {
			printf("incorrect length\n");
			return;
		}
	}

	dump_bytes(addr, length, (unsigned long)addr);
}

static void mw(char *addr, char *value, char *count)
{
	char *c;
	unsigned int *addr2;
	unsigned int value2;
	unsigned int count2;
	unsigned int i;

	if((*addr == 0) || (*value == 0)) {
		printf("mw <address> <value> [count]\n");
		return;
	}
	addr2 = (unsigned int *)strtoul(addr, &c, 0);
	if(*c != 0) {
		printf("incorrect address\n");
		return;
	}
	value2 = strtoul(value, &c, 0);
	if(*c != 0) {
		printf("incorrect value\n");
		return;
	}
	if(*count == 0) {
		count2 = 1;
	} else {
		count2 = strtoul(count, &c, 0);
		if(*c != 0) {
			printf("incorrect count\n");
			return;
		}
	}
	for (i=0;i<count2;i++) *addr2++ = value2;
}

static void mwb(char *addr, char *value, char *count)
{
	char *c;
	unsigned char *addr2;
	unsigned int value2;
	unsigned int count2;
	unsigned int i;

	if((*addr == 0) || (*value == 0)) {
		printf("mw <address> <value> [count]\n");
		return;
	}
	addr2 = (unsigned char *)strtoul(addr, &c, 0);
	if(*c != 0) {
		printf("incorrect address\n");
		return;
	}
	value2 = strtoul(value, &c, 0);
	if(*c != 0) {
		printf("incorrect value\n");
		return;
	}
	if(*count == 0) {
		count2 = 1;
	} else {
		count2 = strtoul(count, &c, 0);
		if(*c != 0) {
			printf("incorrect count\n");
			return;
		}
	}
	for (i=0;i<count2;i++) *addr2++ = (char)value2;
}

static void mc(char *dstaddr, char *srcaddr, char *count)
{
	char *c;
	unsigned int *dstaddr2;
	unsigned int *srcaddr2;
	unsigned int count2;
	unsigned int i;

	if((*dstaddr == 0) || (*srcaddr == 0)) {
		printf("mc <dst> <src> [count]\n");
		return;
	}
	dstaddr2 = (unsigned int *)strtoul(dstaddr, &c, 0);
	if(*c != 0) {
		printf("incorrect destination address\n");
		return;
	}
	srcaddr2 = (unsigned int *)strtoul(srcaddr, &c, 0);
	if(*c != 0) {
		printf("incorrect source address\n");
		return;
	}
	if(*count == 0) {
		count2 = 1;
	} else {
		count2 = strtoul(count, &c, 0);
		if(*c != 0) {
			printf("incorrect count\n");
			return;
		}
	}
	for (i=0;i<count2;i++) *dstaddr2++ = *srcaddr2++;
}

static void crc(char *startaddr, char *len)
{
	char *c;
	char *addr;
	unsigned int length;

	if((*startaddr == 0)||(*len == 0)) {
		printf("crc <address> <length>\n");
		return;
	}
	addr = (char *)strtoul(startaddr, &c, 0);
	if(*c != 0) {
		printf("incorrect address\n");
		return;
	}
	length = strtoul(len, &c, 0);
	if(*c != 0) {
		printf("incorrect length\n");
		return;
	}

	printf("CRC32: %08x\n", crc32((unsigned char *)addr, length));
}

static void ident(void)
{
	char buffer[IDENT_SIZE];

	get_ident(buffer);
	printf("Ident: %s\n", buffer);
}

/* Init + command line */

static void help(void)
{
	puts("LiteX BIOS, available commands:");
	puts("mr         - read address space");
    puts("mrb        - readb address space");
	puts("mw         - write address space");
	puts("mwb        - writeb address space");
	puts("mc         - copy address space");
	puts("");
	puts("crc        - compute CRC32 of a part of the address space");
	puts("ident      - display identifier");
	puts("");
#ifdef PLATFORM_AE4GX
    puts("accel      - start accel simulator");
#endif
#ifdef CAN_CTRL_BASE
    puts("canregs    - dump can controller registers");
    puts("candemo    - run sja1000 can demo");
#endif
#ifdef CSR_ADDER8_BASE
    puts("adder8     - Adder 8bit demo");
#endif
#ifdef SPI_MASTER_BASE
    puts("adc        - ADC read");
#endif
#ifdef PLATFORM_ACCEL_TEST
#ifdef CSR_MBX_SND_BASE
    puts("mbx_send   - Mailbox message send demo");
#endif
#endif
#ifdef CSR_CTRL_BASE
	puts("reboot     - reset processor");
#endif
#ifdef CSR_ETHMAC_BASE
	puts("netboot    - boot via TFTP");
#endif
	puts("serialboot - boot via SFL");
#ifdef FLASH_BOOT_ADDRESS
	puts("flashboot  - boot from flash");
    puts("fbraw      - boot from fash without integrity checking");
#endif
#ifdef ROM_BOOT_ADDRESS
	puts("romboot    - boot from embedded rom");
#endif
	puts("");
#ifdef CSR_SDRAM_BASE
	puts("memtest    - run a memory test");
#endif
}

static char *get_token(char **str)
{
	char *c, *d;

	c = (char *)strchr(*str, ' ');
	if(c == NULL) {
		d = *str;
		*str = *str+strlen(*str);
		return d;
	}
	*c = 0;
	d = *str;
	*str = c+1;
	return d;
}

#ifdef CSR_CTRL_BASE
static void reboot(void)
{
	ctrl_reset_write(1);
}
#endif

static void do_command(char *c)
{
	char *token;

	token = get_token(&c);

	if(strcmp(token, "mr") == 0) mr(get_token(&c), get_token(&c));
	else if(strcmp(token, "mrb") == 0) mrb(get_token(&c), get_token(&c));
	else if(strcmp(token, "mw") == 0) mw(get_token(&c), get_token(&c), get_token(&c));
	else if(strcmp(token, "mwb") == 0) mwb(get_token(&c), get_token(&c), get_token(&c));
	else if(strcmp(token, "mc") == 0) mc(get_token(&c), get_token(&c), get_token(&c));
	else if(strcmp(token, "crc") == 0) crc(get_token(&c), get_token(&c));
	else if(strcmp(token, "ident") == 0) ident();
#ifdef PLATFORM_AE4GX
	else if(strcmp(token, "accel") == 0) main_app();
#endif
#ifdef CAN_CTRL_BASE
	else if(strcmp(token, "canregs") == 0) canregs();
	else if(strcmp(token, "candemo") == 0) can_transmit_demo();
#endif
#ifdef CSR_ADDER8_BASE
	else if(strcmp(token, "adder8") == 0) adder8(get_token(&c), get_token(&c));
#endif
#ifdef SPI_MASTER_BASE
	else if(strcmp(token, "adc") == 0) adc_read(get_token(&c));
#endif
#ifdef PLATFORM_ACCEL_TEST
#ifdef CSR_MBX_SND_BASE
	else if(strcmp(token, "mbx_send") == 0) mbx_message();
#endif
#endif
#ifdef L2_SIZE
	else if(strcmp(token, "flushl2") == 0) flush_l2_cache();
#endif
#ifdef CSR_CTRL_BASE
	else if(strcmp(token, "reboot") == 0) reboot();
#endif
#ifdef FLASH_BOOT_ADDRESS
	else if(strcmp(token, "flashboot") == 0) flashboot();
    else if(strcmp(token, "fbraw") == 0) flash_boot_raw();
#endif
#ifdef ROM_BOOT_ADDRESS
	else if(strcmp(token, "romboot") == 0) romboot();
#endif
	else if(strcmp(token, "serialboot") == 0) serialboot();
#ifdef CSR_ETHMAC_BASE
	else if(strcmp(token, "netboot") == 0) netboot();
#endif

	else if(strcmp(token, "help") == 0) help();

#ifdef CSR_SDRAM_BASE
	else if(strcmp(token, "sdrrow") == 0) sdrrow(get_token(&c));
	else if(strcmp(token, "sdrsw") == 0) sdrsw();
	else if(strcmp(token, "sdrhw") == 0) sdrhw();
	else if(strcmp(token, "sdrrdbuf") == 0) sdrrdbuf(-1);
	else if(strcmp(token, "sdrrd") == 0) sdrrd(get_token(&c), get_token(&c));
	else if(strcmp(token, "sdrrderr") == 0) sdrrderr(get_token(&c));
	else if(strcmp(token, "sdrwr") == 0) sdrwr(get_token(&c));
#ifdef CSR_DDRPHY_BASE
	else if(strcmp(token, "sdrinit") == 0) sdrinit();
#ifdef CSR_DDRPHY_WLEVEL_EN_ADDR
	else if(strcmp(token, "sdrwlon") == 0) sdrwlon();
	else if(strcmp(token, "sdrwloff") == 0) sdrwloff();
#endif
	else if(strcmp(token, "sdrlevel") == 0) sdrlevel();
#endif
	else if(strcmp(token, "memtest") == 0) memtest();
#endif

	else if(strcmp(token, "") != 0)
		printf("Command not found\n");
}

extern unsigned int _ftext, _edata;

static void crcbios(void)
{
	unsigned long offset_bios;
	unsigned long length;
	unsigned int expected_crc;
	unsigned int actual_crc;

	/*
	 * _edata is located right after the end of the flat
	 * binary image. The CRC tool writes the 32-bit CRC here.
	 * We also use the address of _edata to know the length
	 * of our code.
	 */
	offset_bios = (unsigned long)&_ftext;
	expected_crc = _edata;
	length = (unsigned long)&_edata - offset_bios;
	actual_crc = crc32((unsigned char *)offset_bios, length);
	if(expected_crc == actual_crc)
		printf(" BIOS CRC passed (%08x)\n", actual_crc);
	else {
		printf(" BIOS CRC failed (expected %08x, got %08x)\n", expected_crc, actual_crc);
		printf(" The system will continue, but expect problems.\n");
	}
}

static void readstr(char *s, int size)
{
	static char skip = 0;
	char c[2];
	int ptr;

	c[1] = 0;
	ptr = 0;
	while(1) {
		c[0] = readchar();
		if (c[0] == skip)
			continue;
		skip = 0;
		switch(c[0]) {
			case 0x7f:
			case 0x08:
				if(ptr > 0) {
					ptr--;
					putsnonl("\x08 \x08");
				}
				break;
			case 0x07:
				break;
			case '\r':
				skip = '\n';
				s[ptr] = 0x00;
				putsnonl("\n");
				return;
			case '\n':
				skip = '\r';
				s[ptr] = 0x00;
				putsnonl("\n");
				return;
			default:
				putsnonl(c);
				s[ptr] = c[0];
				ptr++;
				break;
		}
	}
}

#if 0
static void boot_sequence(void)
{
	if(serialboot()) {
#ifdef FLASH_BOOT_ADDRESS
		flashboot();
#endif
#ifdef ROM_BOOT_ADDRESS
		romboot();
#endif
#ifdef CSR_ETHMAC_BASE
#ifdef CSR_ETHPHY_MODE_DETECTION_MODE_ADDR
		eth_mode();
#endif
		netboot();
#endif
		printf("No boot medium found\n");
	}
}
#endif

int main(int i, char **c)
{
	char buffer[64];
	int sdr_ok;

	irq_setmask(0);
	irq_setie(1);
	uart_init();

#ifdef SPI_MASTER_BASE
    spi_init();
#endif

#ifdef ACCEL_INTERRUPT
    accel_isr_init();
#endif

#ifdef MBX_RCV_INTERRUPT
    mbx_rcv_isr_init();
#endif

#ifdef SPI_MASTER_INTERRUPT
    spi_master_isr_init();
#endif

#ifdef CAN_CTRL_INTERRUPT
    can_ctrl_isr_init();
#endif

#ifdef GPIO_ISR_INTERRUPT
    gpio_isr_init();
#endif

	printf("\n");
	printf("\e[1m        __   _ __      _  __\e[0m\n");
	printf("\e[1m       / /  (_) /____ | |/_/\e[0m\n");
	printf("\e[1m      / /__/ / __/ -_)>  <\e[0m\n");
	printf("\e[1m     /____/_/\\__/\\__/_/|_|\e[0m\n");
	printf("\n");
	printf(" (c) Copyright 2012-2019 Enjoy-Digital\n");
	printf(" (c) Copyright 2007-2015 M-Labs Ltd\n");
	printf("\n");
	printf(" BIOS built on "__DATE__" "__TIME__"\n");
	crcbios();
	printf("\n");

	printf("--============ \e[1mSoC info\e[0m ================--\n");
	printf("\e[1mCPU\e[0m:       ");
#ifdef __lm32__
	printf("LM32");
#elif __or1k__
	printf("MOR1KX");
#elif __picorv32__
	printf("PicoRV32");
#elif __vexriscv__
	printf("VexRiscv");
#elif __minerva__
	printf("Minerva");
#elif __rocket__
	printf("RocketRV64[imac]");
#else
	printf("Unknown");
#endif
	printf(" @ %dMHz\n", SYSTEM_CLOCK_FREQUENCY/1000000);
	printf("\e[1mROM\e[0m:       %dKB\n", ROM_SIZE/1024);
	printf("\e[1mSRAM\e[0m:      %dKB\n", SRAM_SIZE/1024);
#ifdef L2_SIZE
	printf("\e[1mL2\e[0m:        %dKB\n", L2_SIZE/1024);
#endif
#ifdef MAIN_RAM_SIZE
	printf("\e[1mMAIN-RAM\e[0m:  %dKB\n", MAIN_RAM_SIZE/1024);
#endif
	printf("\n");

	printf("--========= \e[1mPeripherals init\e[0m ===========--\n");
#ifdef CSR_ETHPHY_CRG_RESET_ADDR
	eth_init();
#endif
#ifdef CSR_SDRAM_BASE
	sdr_ok = sdrinit();
#else
#ifdef MAIN_RAM_TEST
	sdr_ok = memtest();
#else
	sdr_ok = 1;
#endif
#endif
	if (sdr_ok !=1)
		printf("Memory initialization failed\n");
	printf("\n");

	if(sdr_ok) {
		printf("--========== \e[1mBoot sequence\e[0m =============--\n");
		// boot_sequence();
		printf("\n");
	}

	printf("--============= \e[1mConsole\e[0m ================--\n");

#if defined(PLATFORM_AE4GX) || \
    defined(PLATFORM_BASYS3) || \
    defined(PLATFORM_ACCEL_SIM) || \
    defined(PLATFORM_AT7CORE) || \
    defined(PLATFORM_ACCEL_SIM_RELEASE)
    main_app();
#endif

#if defined(PLATFORM_ACCEL_TEST)
    accel_test();
#endif

#ifdef PLATFORM_ICE40_HX8K_B_EVN
    flash_boot_raw();
#endif

#ifdef CSR_MY_UART_BASE
    my_uart_print("HELLO RISC_V\r\n");
#endif

	while(1) {
		putsnonl("\e[1mBIOS>\e[0m ");
		readstr(buffer, 64);
		do_command(buffer);
	}
	return 0;
}
