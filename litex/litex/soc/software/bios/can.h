#ifndef __CAN_H
#define __CAN_H

#include <generated/csr.h>

#define true  1
#define false 0

#define SUPPORT_EXTENDED_CANID 1

/**
 * \ingroup can_interface
 * \brief   Modus des CAN Interfaces
 */
typedef enum {
    LISTEN_ONLY_MODE,
    LOOPBACK_MODE,
    NORMAL_MODE,
    SLEEP_MODE
} can_mode_t;

/**
 * \ingroup can_interface
 * \brief   Inhalt der Fehler-Register
 */
typedef struct {
    uint8_t rx; //!< Empfangs-Register
    uint8_t tx; //!< Sende-Register
} can_error_register_t;

/**
 * \ingroup can_interface
 * \brief   Datenstruktur zum Aufnehmen von CAN Nachrichten
 */
typedef struct
{
    #if SUPPORT_EXTENDED_CANID
        uint32_t id;                //!< ID der Nachricht (11 oder 29 Bit)
        struct {
            int rtr : 1;            //!< Remote-Transmit-Request-Frame?
            int extended : 1;       //!< extended ID?
        } flags;
    #else
        uint16_t id;                //!< ID der Nachricht (11 Bit)
        struct {
            int rtr : 1;            //!< Remote-Transmit-Request-Frame?
        } flags;
    #endif

    uint8_t length;                 //!< Anzahl der Datenbytes
    uint8_t data[8];                //!< Die Daten der CAN Nachricht

    #if SUPPORT_TIMESTAMPS
        uint16_t timestamp;
    #endif
} can_t;

/* Function prototypes */
#ifdef CAN_CTRL_BASE
void sja1000_init(void);
void sja1000_set_mode(can_mode_t mode);
_Bool sja1000_check_message(void);
_Bool sja1000_check_free_buffer(void);
can_error_register_t sja1000_read_error_register(void);
_Bool sja1000_check_bus_off(void);
void sja1000_reset_bus_off(void);
_Bool sja1000_get_message(can_t *msg);
_Bool sja1000_send_message(const can_t *msg);
#endif

/* SJA1000 PeliCAN and basic mode register set */
#define CR          (*(volatile unsigned char *)(CAN_CTRL_BASE +  0*4)) // Control (basic mode)
#define MOD         (*(volatile unsigned char *)(CAN_CTRL_BASE +  0*4)) // Mode (peliCAN mode)
#define CMR         (*(volatile unsigned char *)(CAN_CTRL_BASE +  1*4)) // Command
#define SR          (*(volatile unsigned char *)(CAN_CTRL_BASE +  2*4)) // Status
#define IR          (*(volatile unsigned char *)(CAN_CTRL_BASE +  3*4)) // Interrupt
#define BTR0        (*(volatile unsigned char *)(CAN_CTRL_BASE +  6*4)) // Bus timing 0
#define BTR1        (*(volatile unsigned char *)(CAN_CTRL_BASE +  7*4)) // Bus timing 1
#define OCR         (*(volatile unsigned char *)(CAN_CTRL_BASE +  8*4)) // Output control (not suported)
#define CDR         (*(volatile unsigned char *)(CAN_CTRL_BASE + 31*4)) // Clock divider 

/* Acceptance code (Basic mode only) */
#define ACR         (*(volatile unsigned char *)(CAN_CTRL_BASE +  4*4)) // Acceptance code 
#define AMR         (*(volatile unsigned char *)(CAN_CTRL_BASE +  5*4)) // Acceptance mask 

/* Acceptance code (PeliCAN mode only) */
#define ACR0        (*(volatile unsigned char *)(CAN_CTRL_BASE + 16*4)) // acceptance code0
#define ACR1        (*(volatile unsigned char *)(CAN_CTRL_BASE + 17*4)) // acceptance code1
#define ACR2        (*(volatile unsigned char *)(CAN_CTRL_BASE + 18*4)) // acceptance code2
#define ACR3        (*(volatile unsigned char *)(CAN_CTRL_BASE + 19*4)) // acceptance code3
#define AMR0        (*(volatile unsigned char *)(CAN_CTRL_BASE + 20*4)) // acceptance mask0
#define AMR1        (*(volatile unsigned char *)(CAN_CTRL_BASE + 21*4)) // acceptance mask1
#define AMR2        (*(volatile unsigned char *)(CAN_CTRL_BASE + 22*4)) // acceptance mask2
#define AMR3        (*(volatile unsigned char *)(CAN_CTRL_BASE + 23*4)) // acceptance mask3

/* PeliCAN mode only */
#define IER         (*(volatile unsigned char *)(CAN_CTRL_BASE +  4*4)) // Interrupt enable 
#define ALC         (*(volatile unsigned char *)(CAN_CTRL_BASE + 11*4)) // Arbitration lost capture
#define ECC         (*(volatile unsigned char *)(CAN_CTRL_BASE + 12*4)) // Error code capture
#define EWL         (*(volatile unsigned char *)(CAN_CTRL_BASE + 13*4)) // Error warning limit
#define RXERR       (*(volatile unsigned char *)(CAN_CTRL_BASE + 14*4)) // RX error counter
#define TXERR       (*(volatile unsigned char *)(CAN_CTRL_BASE + 15*4)) // TX error counter
#define RMC         (*(volatile unsigned char *)(CAN_CTRL_BASE + 29*4)) // RX message counter
#define RBSA        (*(volatile unsigned char *)(CAN_CTRL_BASE + 30*4)) // RX_buffer_start_adress (not suported)

/* TX data buffer (basic mode only) */
#define TX_DATA_0   (*(volatile unsigned char *)(CAN_CTRL_BASE + 10*4)) // Identifier (10 to 3)
#define TX_DATA_1   (*(volatile unsigned char *)(CAN_CTRL_BASE + 11*4)) // Identifier (2 to 0), RTR and DLC
#define TX_DATA_2   (*(volatile unsigned char *)(CAN_CTRL_BASE + 12*4)) // Transmit data byte 2
#define TX_DATA_3   (*(volatile unsigned char *)(CAN_CTRL_BASE + 13*4)) // Transmit data byte 3
#define TX_DATA_4   (*(volatile unsigned char *)(CAN_CTRL_BASE + 14*4)) // Transmit data byte 4
#define TX_DATA_5   (*(volatile unsigned char *)(CAN_CTRL_BASE + 15*4)) // Transmit data byte 5
#define TX_DATA_6   (*(volatile unsigned char *)(CAN_CTRL_BASE + 16*4)) // Transmit data byte 6
#define TX_DATA_7   (*(volatile unsigned char *)(CAN_CTRL_BASE + 17*4)) // Transmit data byte 7
#define TX_DATA_8   (*(volatile unsigned char *)(CAN_CTRL_BASE + 18*4)) // Transmit data byte 8
#define TX_DATA_9   (*(volatile unsigned char *)(CAN_CTRL_BASE + 19*4)) // Transmit data byte 9

/* RX data buffer (basic mode only) */
#define RX_DATA_0   (*(volatile unsigned char *)(CAN_CTRL_BASE + 20*4)) // Identifier (10 to 3)
#define RX_DATA_1   (*(volatile unsigned char *)(CAN_CTRL_BASE + 21*4)) // Identifier (2 to 0), RTR and DLC
#define RX_DATA_2   (*(volatile unsigned char *)(CAN_CTRL_BASE + 22*4)) // Receive data byte 2
#define RX_DATA_3   (*(volatile unsigned char *)(CAN_CTRL_BASE + 23*4)) // Receive data byte 3
#define RX_DATA_4   (*(volatile unsigned char *)(CAN_CTRL_BASE + 24*4)) // Receive data byte 4
#define RX_DATA_5   (*(volatile unsigned char *)(CAN_CTRL_BASE + 25*4)) // Receive data byte 5
#define RX_DATA_6   (*(volatile unsigned char *)(CAN_CTRL_BASE + 26*4)) // Receive data byte 6
#define RX_DATA_7   (*(volatile unsigned char *)(CAN_CTRL_BASE + 27*4)) // Receive data byte 7
#define RX_DATA_8   (*(volatile unsigned char *)(CAN_CTRL_BASE + 28*4)) // Receive data byte 8
#define RX_DATA_9   (*(volatile unsigned char *)(CAN_CTRL_BASE + 29*4)) // Receive data byte 9

/* TX data buffer (PeliCAN mode only) */
#define TX_INFO     (*(volatile unsigned char *)(CAN_CTRL_BASE + 16*4))
#define TX_ID0      (*(volatile unsigned char *)(CAN_CTRL_BASE + 17*4))
#define TX_ID1      (*(volatile unsigned char *)(CAN_CTRL_BASE + 18*4))
#define TX_DATA0    (*(volatile unsigned char *)(CAN_CTRL_BASE + 19*4))
#define TX_DATA1    (*(volatile unsigned char *)(CAN_CTRL_BASE + 20*4))
#define TX_DATA2    (*(volatile unsigned char *)(CAN_CTRL_BASE + 21*4))
#define TX_DATA3    (*(volatile unsigned char *)(CAN_CTRL_BASE + 22*4))
#define TX_DATA4    (*(volatile unsigned char *)(CAN_CTRL_BASE + 23*4))
#define TX_DATA5    (*(volatile unsigned char *)(CAN_CTRL_BASE + 24*4))
#define TX_DATA6    (*(volatile unsigned char *)(CAN_CTRL_BASE + 25*4))
#define TX_DATA7    (*(volatile unsigned char *)(CAN_CTRL_BASE + 26*4))

/* RX data buffer (PeliCAN mode only) */
#define RX_INFO     (*(volatile unsigned char *)(CAN_CTRL_BASE + 16*4))
#define RX_ID1      (*(volatile unsigned char *)(CAN_CTRL_BASE + 17*4))
#define RX_ID0      (*(volatile unsigned char *)(CAN_CTRL_BASE + 18*4))
#define RX_DATA0    (*(volatile unsigned char *)(CAN_CTRL_BASE + 19*4))
#define RX_DATA1    (*(volatile unsigned char *)(CAN_CTRL_BASE + 20*4))
#define RX_DATA2    (*(volatile unsigned char *)(CAN_CTRL_BASE + 21*4))
#define RX_DATA3    (*(volatile unsigned char *)(CAN_CTRL_BASE + 22*4))
#define RX_DATA4    (*(volatile unsigned char *)(CAN_CTRL_BASE + 23*4))
#define RX_DATA5    (*(volatile unsigned char *)(CAN_CTRL_BASE + 24*4))
#define RX_DATA6    (*(volatile unsigned char *)(CAN_CTRL_BASE + 25*4))
#define RX_DATA7    (*(volatile unsigned char *)(CAN_CTRL_BASE + 26*4))

/**
 * \brief   Bitdefinition von CR
 */
#define CR_OIE       4
#define CR_EIE       3
#define CR_TIE       2
#define CR_RIE       1
#define CR_RR        0

/**
 * \brief   Bitdefinition von MOD
 */
#define SM          4
#define AFM         3
#define STM         2
#define LOM         1
#define RM          0

/**
 * \brief   Bitdefinition von CMR
 */
#define SRR         4
#define CDO         3
#define RRB         2
#define AT          1
#define TR          0

/**
 * \brief   Bitdefinition von SR
 */
#define BS          7
#define ES          6
#define TS          5
#define RS          4
#define TCS         3
#define TBS         2
#define DOS         1
#define RBS         0

/**
 * \brief   Bitdefinition von IR
 */
#define BEI         7
#define ALI         6
#define EPI         5
#define WUI         4
#define DOI         3
#define EI          2
#define TI          1
#define RI          0

/**
* \brief    Bitdefinition von IER / CR
 */
#define BEIE        7
#define ALIE        6
#define EPIE        5
#define WUIE        4
#define DOIE        3
#define EIE         2
#define TIE         1
#define RIE         0

/**
 * \brief   Bitdefinition von BTR0
 */
#define _SJW1       7
#define _SJW0       6
#define _BRP5       5
#define _BRP4       4
#define _BRP3       3
#define _BRP2       2
#define _BRP1       1
#define _BRP0       0

/**
 * \brief   Bitdefinition von BTR1
 */
#define SAM         7
#define TSEG22      6
#define TSEG21      5
#define TSEG20      4
#define TSEG13      3
#define TSEG12      2
#define TSEG11      1
#define TSEG10      0

/**
 * \brief   Bitdefinition von OCR
 */
#define OCTP1       7
#define OCTN1       6
#define OCPOL1      5
#define OCTP0       4
#define OCTN0       3
#define OCPOL0      2
#define OCMODE1     1
#define OCMODE0	    0

/**
 * \brief   Bitdefinition von ALC
 */
#define BITNO4      4
#define BITNO3      3
#define BITNO2      2
#define BITNO1      1
#define BITNO0      0

/**
 * \brief   Bitdefinition von ECC
 */
#define ERRC1       7
#define ERRC0       6
#define DIR         5
#define SEG4        4
#define SEG3        3
#define SEG2        2
#define SEG1        1
#define SEG0        0

/**
 * \brief   Bitdefinition von EWL
 */
#define ERRC1       7
#define ERRC0       6
#define DIR         5
#define SEG4        4
#define SEG3        3
#define SEG2        2
#define SEG1        1
#define SEG0        0

/**
 * \brief   Bitdefinition von CDR
 */
#define CANMODE     7
#define CBP         6
#define RXINTEN     5
#define CLKOFF      3
#define CD2         2
#define CD1         1
#define CD0         0

/**
 * \brief   Bitdefinition von RX_INFO und TX_INFO
 */
#define FF          7
#define RTR         6

#endif /* __SDRAM_H */