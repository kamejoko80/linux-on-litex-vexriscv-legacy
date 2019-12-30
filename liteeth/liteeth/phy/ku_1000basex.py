# This file is Copyright (c) 2019 M-Labs Ltd
# License: BSD

from migen import *
from migen.genlib.resetsync import AsyncResetSynchronizer
from migen.genlib.cdc import PulseSynchronizer

from liteeth.phy.pcs_1000basex import *


class Gearbox(Module):
    def __init__(self):
        self.tx_data = Signal(10)
        self.tx_data_half = Signal(20)
        self.rx_data_half = Signal(20)
        self.rx_data = Signal(10)

        # TX
        buf = Signal(20)
        self.sync.eth_tx += buf.eq(Cat(buf[10:], self.tx_data))
        self.sync.eth_tx_half += self.tx_data_half.eq(buf)

        # RX
        phase_half = Signal()
        phase_half_rereg = Signal()
        self.sync.eth_rx_half += phase_half_rereg.eq(phase_half)
        self.sync.eth_rx += [
            If(phase_half == phase_half_rereg,
                self.rx_data.eq(self.rx_data_half[10:])
            ).Else(
                self.rx_data.eq(self.rx_data_half[:10])
            ),
            phase_half.eq(~phase_half),
        ]


# Configured for 200MHz transceiver reference clock
class KU_1000BASEX(Module):
    dw = 8
    def __init__(self, refclk_or_clk_pads, data_pads, sys_clk_freq):
        pcs = PCS(lsb_first=True)
        self.submodules += pcs

        self.sink = pcs.sink
        self.source = pcs.source
        self.link_up = pcs.link_up

        self.clock_domains.cd_eth_tx = ClockDomain()
        self.clock_domains.cd_eth_rx = ClockDomain()
        self.clock_domains.cd_eth_tx_half = ClockDomain(reset_less=True)
        self.clock_domains.cd_eth_rx_half = ClockDomain(reset_less=True)

        # for specifying clock constraints. 125MHz clocks.
        self.txoutclk = Signal()
        self.rxoutclk = Signal()

        # # #

        if isinstance(refclk_or_clk_pads, Signal):
            refclk = refclk_or_clk_pads
        else:
            refclk = Signal()
            self.specials += Instance("IBUFDS_GTE3",
                i_I=refclk_or_clk_pads.p,i_IB=refclk_or_clk_pads.n,
                i_CEB=0, o_O=refclk)

        gtpowergood = Signal()
        pll_reset = Signal(reset=1)
        pll_locked = Signal()

        tx_reset = Signal()
        tx_data = Signal(20)
        tx_reset_done = Signal()

        rx_reset = Signal()
        rx_data = Signal(20)
        rx_reset_done = Signal()

        xilinx_mess = dict(
            p_ACJTAG_DEBUG_MODE=0b0,
            p_ACJTAG_MODE=0b0,
            p_ACJTAG_RESET=0b0,
            p_ADAPT_CFG0=0b1111100000000000,
            p_ADAPT_CFG1=0b0000000000000000,
            p_ALIGN_COMMA_DOUBLE="FALSE",
            p_ALIGN_COMMA_ENABLE=0b0000000000,
            p_ALIGN_COMMA_WORD=1,
            p_ALIGN_MCOMMA_DET="FALSE",
            p_ALIGN_MCOMMA_VALUE=0b1010000011,
            p_ALIGN_PCOMMA_DET="FALSE",
            p_ALIGN_PCOMMA_VALUE=0b0101111100,
            p_A_RXOSCALRESET=0b0,
            p_A_RXPROGDIVRESET=0b0,
            p_A_TXPROGDIVRESET=0b0,
            p_CBCC_DATA_SOURCE_SEL="ENCODED",
            p_CDR_SWAP_MODE_EN=0b0,
            p_CHAN_BOND_KEEP_ALIGN="FALSE",
            p_CHAN_BOND_MAX_SKEW=1,
            p_CHAN_BOND_SEQ_1_1=0b0000000000,
            p_CHAN_BOND_SEQ_1_2=0b0000000000,
            p_CHAN_BOND_SEQ_1_3=0b0000000000,
            p_CHAN_BOND_SEQ_1_4=0b0000000000,
            p_CHAN_BOND_SEQ_1_ENABLE=0b1111,
            p_CHAN_BOND_SEQ_2_1=0b0000000000,
            p_CHAN_BOND_SEQ_2_2=0b0000000000,
            p_CHAN_BOND_SEQ_2_3=0b0000000000,
            p_CHAN_BOND_SEQ_2_4=0b0000000000,
            p_CHAN_BOND_SEQ_2_ENABLE=0b1111,
            p_CHAN_BOND_SEQ_2_USE="FALSE",
            p_CHAN_BOND_SEQ_LEN=1,
            p_CLK_CORRECT_USE="FALSE",
            p_CLK_COR_KEEP_IDLE="FALSE",
            p_CLK_COR_MAX_LAT=6,
            p_CLK_COR_MIN_LAT=4,
            p_CLK_COR_PRECEDENCE="TRUE",
            p_CLK_COR_REPEAT_WAIT=0,
            p_CLK_COR_SEQ_1_1=0b0000000000,
            p_CLK_COR_SEQ_1_2=0b0000000000,
            p_CLK_COR_SEQ_1_3=0b0000000000,
            p_CLK_COR_SEQ_1_4=0b0000000000,
            p_CLK_COR_SEQ_1_ENABLE=0b1111,
            p_CLK_COR_SEQ_2_1=0b0000000000,
            p_CLK_COR_SEQ_2_2=0b0000000000,
            p_CLK_COR_SEQ_2_3=0b0000000000,
            p_CLK_COR_SEQ_2_4=0b0000000000,
            p_CLK_COR_SEQ_2_ENABLE=0b1111,
            p_CLK_COR_SEQ_2_USE="FALSE",
            p_CLK_COR_SEQ_LEN=1,
            p_CPLL_CFG0=0b0110011111111000,
            p_CPLL_CFG1=0b1010010010101100,
            p_CPLL_CFG2=0b0000000000000111,
            p_CPLL_CFG3=0b000000,
            p_CPLL_FBDIV=5,
            p_CPLL_FBDIV_45=5,
            p_CPLL_INIT_CFG0=0b0000001010110010,
            p_CPLL_INIT_CFG1=0b00000000,
            p_CPLL_LOCK_CFG=0b0000000111101000,
            p_CPLL_REFCLK_DIV=2,
            p_DDI_CTRL=0b00,
            p_DDI_REALIGN_WAIT=15,
            p_DEC_MCOMMA_DETECT="FALSE",
            p_DEC_PCOMMA_DETECT="FALSE",
            p_DEC_VALID_COMMA_ONLY="FALSE",
            p_DFE_D_X_REL_POS=0b0,
            p_DFE_VCM_COMP_EN=0b0,
            p_DMONITOR_CFG0=0b0000000000,
            p_DMONITOR_CFG1=0b00000000,
            p_ES_CLK_PHASE_SEL=0b0,
            p_ES_CONTROL=0b000000,
            p_ES_ERRDET_EN="FALSE",
            p_ES_EYE_SCAN_EN="FALSE",
            p_ES_HORZ_OFFSET=0b000000000000,
            p_ES_PMA_CFG=0b0000000000,
            p_ES_PRESCALE=0b00000,
            p_ES_QUALIFIER0=0b0000000000000000,
            p_ES_QUALIFIER1=0b0000000000000000,
            p_ES_QUALIFIER2=0b0000000000000000,
            p_ES_QUALIFIER3=0b0000000000000000,
            p_ES_QUALIFIER4=0b0000000000000000,
            p_ES_QUAL_MASK0=0b0000000000000000,
            p_ES_QUAL_MASK1=0b0000000000000000,
            p_ES_QUAL_MASK2=0b0000000000000000,
            p_ES_QUAL_MASK3=0b0000000000000000,
            p_ES_QUAL_MASK4=0b0000000000000000,
            p_ES_SDATA_MASK0=0b0000000000000000,
            p_ES_SDATA_MASK1=0b0000000000000000,
            p_ES_SDATA_MASK2=0b0000000000000000,
            p_ES_SDATA_MASK3=0b0000000000000000,
            p_ES_SDATA_MASK4=0b0000000000000000,
            p_EVODD_PHI_CFG=0b00000000000,
            p_EYE_SCAN_SWAP_EN=0b0,
            p_FTS_DESKEW_SEQ_ENABLE=0b1111,
            p_FTS_LANE_DESKEW_CFG=0b1111,
            p_FTS_LANE_DESKEW_EN="FALSE",
            p_GEARBOX_MODE=0b00000,
            p_GM_BIAS_SELECT=0b0,
            p_LOCAL_MASTER=0b1,
            p_OOBDIVCTL=0b00,
            p_OOB_PWRUP=0b0,
            p_PCI3_AUTO_REALIGN="OVR_1K_BLK",
            p_PCI3_PIPE_RX_ELECIDLE=0b0,
            p_PCI3_RX_ASYNC_EBUF_BYPASS=0b00,
            p_PCI3_RX_ELECIDLE_EI2_ENABLE=0b0,
            p_PCI3_RX_ELECIDLE_H2L_COUNT=0b000000,
            p_PCI3_RX_ELECIDLE_H2L_DISABLE=0b000,
            p_PCI3_RX_ELECIDLE_HI_COUNT=0b000000,
            p_PCI3_RX_ELECIDLE_LP4_DISABLE=0b0,
            p_PCI3_RX_FIFO_DISABLE=0b0,
            p_PCIE_BUFG_DIV_CTRL=0b0001000000000000,
            p_PCIE_RXPCS_CFG_GEN3=0b0000001010100100,
            p_PCIE_RXPMA_CFG=0b0000000000001010,
            p_PCIE_TXPCS_CFG_GEN3=0b0010110010100100,
            p_PCIE_TXPMA_CFG=0b0000000000001010,
            p_PCS_PCIE_EN="FALSE",
            p_PCS_RSVD0=0b0000000000000000,
            p_PCS_RSVD1=0b000,
            p_PD_TRANS_TIME_FROM_P2=0b000000111100,
            p_PD_TRANS_TIME_NONE_P2=0b00011001,
            p_PD_TRANS_TIME_TO_P2=0b01100100,
            p_PLL_SEL_MODE_GEN12=0b00,
            p_PLL_SEL_MODE_GEN3=0b11,
            p_PMA_RSV1=0b1111000000000000,
            p_PROCESS_PAR=0b010,
            p_RATE_SW_USE_DRP=0b1,
            p_RESET_POWERSAVE_DISABLE=0b0,
        )
        xilinx_mess.update(
            p_RXBUFRESET_TIME=0b00011,
            p_RXBUF_ADDR_MODE="FAST",
            p_RXBUF_EIDLE_HI_CNT=0b1000,
            p_RXBUF_EIDLE_LO_CNT=0b0000,
            p_RXBUF_EN="TRUE",
            p_RXBUF_RESET_ON_CB_CHANGE="TRUE",
            p_RXBUF_RESET_ON_COMMAALIGN="FALSE",
            p_RXBUF_RESET_ON_EIDLE="FALSE",
            p_RXBUF_RESET_ON_RATE_CHANGE="TRUE",
            p_RXBUF_THRESH_OVFLW=61,
            p_RXBUF_THRESH_OVRD="TRUE",
            p_RXBUF_THRESH_UNDFLW=1,
            p_RXCDRFREQRESET_TIME=0b00001,
            p_RXCDRPHRESET_TIME=0b00001,
            p_RXCDR_CFG0=0b0000000000000000,
            p_RXCDR_CFG0_GEN3=0b0000000000000000,
            p_RXCDR_CFG1=0b0000000000000000,
            p_RXCDR_CFG1_GEN3=0b0000000000000000,
            p_RXCDR_CFG2=0b0000011111000110,
            p_RXCDR_CFG2_GEN3=0b0000011111100110,
            p_RXCDR_CFG3=0b0000000000000000,
            p_RXCDR_CFG3_GEN3=0b0000000000000000,
            p_RXCDR_CFG4=0b0000000000000000,
            p_RXCDR_CFG4_GEN3=0b0000000000000000,
            p_RXCDR_CFG5=0b0000000000000000,
            p_RXCDR_CFG5_GEN3=0b0000000000000000,
            p_RXCDR_FR_RESET_ON_EIDLE=0b0,
            p_RXCDR_HOLD_DURING_EIDLE=0b0,
            p_RXCDR_LOCK_CFG0=0b0100010010000000,
            p_RXCDR_LOCK_CFG1=0b0101111111111111,
            p_RXCDR_LOCK_CFG2=0b0111011111000011,
            p_RXCDR_PH_RESET_ON_EIDLE=0b0,
            p_RXCFOK_CFG0=0b0100000000000000,
            p_RXCFOK_CFG1=0b0000000001100101,
            p_RXCFOK_CFG2=0b0000000000101110,
            p_RXDFELPMRESET_TIME=0b0001111,
            p_RXDFELPM_KL_CFG0=0b0000000000000000,
            p_RXDFELPM_KL_CFG1=0b0000000000110010,
            p_RXDFELPM_KL_CFG2=0b0000000000000000,
            p_RXDFE_CFG0=0b0000101000000000,
            p_RXDFE_CFG1=0b0000000000000000,
            p_RXDFE_GC_CFG0=0b0000000000000000,
            p_RXDFE_GC_CFG1=0b0111100001110000,
            p_RXDFE_GC_CFG2=0b0000000000000000,
            p_RXDFE_H2_CFG0=0b0000000000000000,
            p_RXDFE_H2_CFG1=0b0000000000000000,
            p_RXDFE_H3_CFG0=0b0100000000000000,
            p_RXDFE_H3_CFG1=0b0000000000000000,
            p_RXDFE_H4_CFG0=0b0010000000000000,
            p_RXDFE_H4_CFG1=0b0000000000000011,
            p_RXDFE_H5_CFG0=0b0010000000000000,
            p_RXDFE_H5_CFG1=0b0000000000000011,
            p_RXDFE_H6_CFG0=0b0010000000000000,
            p_RXDFE_H6_CFG1=0b0000000000000000,
            p_RXDFE_H7_CFG0=0b0010000000000000,
            p_RXDFE_H7_CFG1=0b0000000000000000,
            p_RXDFE_H8_CFG0=0b0010000000000000,
            p_RXDFE_H8_CFG1=0b0000000000000000,
            p_RXDFE_H9_CFG0=0b0010000000000000,
            p_RXDFE_H9_CFG1=0b0000000000000000,
            p_RXDFE_HA_CFG0=0b0010000000000000,
            p_RXDFE_HA_CFG1=0b0000000000000000,
            p_RXDFE_HB_CFG0=0b0010000000000000,
            p_RXDFE_HB_CFG1=0b0000000000000000,
            p_RXDFE_HC_CFG0=0b0000000000000000,
            p_RXDFE_HC_CFG1=0b0000000000000000,
            p_RXDFE_HD_CFG0=0b0000000000000000,
            p_RXDFE_HD_CFG1=0b0000000000000000,
            p_RXDFE_HE_CFG0=0b0000000000000000,
            p_RXDFE_HE_CFG1=0b0000000000000000,
            p_RXDFE_HF_CFG0=0b0000000000000000,
            p_RXDFE_HF_CFG1=0b0000000000000000,
            p_RXDFE_OS_CFG0=0b1000000000000000,
            p_RXDFE_OS_CFG1=0b0000000000000000,
            p_RXDFE_UT_CFG0=0b1000000000000000,
            p_RXDFE_UT_CFG1=0b0000000000000011,
            p_RXDFE_VP_CFG0=0b1010101000000000,
            p_RXDFE_VP_CFG1=0b0000000000110011,
            p_RXDLY_CFG=0b0000000000011111,
            p_RXDLY_LCFG=0b0000000000110000,
            p_RXELECIDLE_CFG="Sigcfg_4",
            p_RXGBOX_FIFO_INIT_RD_ADDR=4,
            p_RXGEARBOX_EN="FALSE",
            p_RXISCANRESET_TIME=0b00001,
            p_RXLPM_CFG=0b0000000000000000,
            p_RXLPM_GC_CFG=0b0001000000000000,
            p_RXLPM_KH_CFG0=0b0000000000000000,
            p_RXLPM_KH_CFG1=0b0000000000000010,
            p_RXLPM_OS_CFG0=0b1000000000000000,
            p_RXLPM_OS_CFG1=0b0000000000000010,
            p_RXOOB_CFG=0b000000110,
            p_RXOOB_CLK_CFG="PMA",
            p_RXOSCALRESET_TIME=0b00011,
            p_RXOUT_DIV=4,
            p_RXPCSRESET_TIME=0b00011,
            p_RXPHBEACON_CFG=0b0000000000000000,
            p_RXPHDLY_CFG=0b0010000000100000,
            p_RXPHSAMP_CFG=0b0010000100000000,
            p_RXPHSLIP_CFG=0b0110011000100010,
            p_RXPH_MONITOR_SEL=0b00000,
            p_RXPI_CFG0=0b01,
            p_RXPI_CFG1=0b01,
            p_RXPI_CFG2=0b01,
            p_RXPI_CFG3=0b01,
            p_RXPI_CFG4=0b1,
            p_RXPI_CFG5=0b1,
            p_RXPI_CFG6=0b011,
            p_RXPI_LPM=0b0,
            p_RXPI_VREFSEL=0b0,
            p_RXPMACLK_SEL="DATA",
            p_RXPMARESET_TIME=0b00011,
            p_RXPRBS_ERR_LOOPBACK=0b0,
            p_RXPRBS_LINKACQ_CNT=15,
            p_RXSLIDE_AUTO_WAIT=7,
            p_RXSLIDE_MODE="OFF",
            p_RXSYNC_MULTILANE=0b0,
            p_RXSYNC_OVRD=0b0,
            p_RXSYNC_SKIP_DA=0b0,
            p_RX_AFE_CM_EN=0b0,
            p_RX_BIAS_CFG0=0b0000101010110100,
            p_RX_BUFFER_CFG=0b000000,
            p_RX_CAPFF_SARC_ENB=0b0,
            p_RX_CLK25_DIV=8,
            p_RX_CLKMUX_EN=0b1,
            p_RX_CLK_SLIP_OVRD=0b00000,
            p_RX_CM_BUF_CFG=0b1010,
            p_RX_CM_BUF_PD=0b0,
            p_RX_CM_SEL=0b11,
            p_RX_CM_TRIM=0b1010,
            p_RX_CTLE3_LPF=0b00000001,
            p_RX_DATA_WIDTH=20,
            p_RX_DDI_SEL=0b000000,
            p_RX_DEFER_RESET_BUF_EN="TRUE",
            p_RX_DFELPM_CFG0=0b0110,
            p_RX_DFELPM_CFG1=0b1,
            p_RX_DFELPM_KLKH_AGC_STUP_EN=0b1,
            p_RX_DFE_AGC_CFG0=0b10,
            p_RX_DFE_AGC_CFG1=0b000,
            p_RX_DFE_KL_LPM_KH_CFG0=0b01,
            p_RX_DFE_KL_LPM_KH_CFG1=0b000,
            p_RX_DFE_KL_LPM_KL_CFG0=0b01,
            p_RX_DFE_KL_LPM_KL_CFG1=0b000,
            p_RX_DFE_LPM_HOLD_DURING_EIDLE=0b0,
            p_RX_DISPERR_SEQ_MATCH="TRUE",
            p_RX_DIVRESET_TIME=0b00001,
            p_RX_EN_HI_LR=0b0,
            p_RX_EYESCAN_VS_CODE=0b0000000,
            p_RX_EYESCAN_VS_NEG_DIR=0b0,
            p_RX_EYESCAN_VS_RANGE=0b00,
            p_RX_EYESCAN_VS_UT_SIGN=0b0,
            p_RX_FABINT_USRCLK_FLOP=0b0,
            p_RX_INT_DATAWIDTH=0,
            p_RX_PMA_POWER_SAVE=0b0,
            p_RX_PROGDIV_CFG=20.0,
            p_RX_SAMPLE_PERIOD=0b111,
            p_RX_SIG_VALID_DLY=11,
            p_RX_SUM_DFETAPREP_EN=0b0,
            p_RX_SUM_IREF_TUNE=0b1100,
            p_RX_SUM_RES_CTRL=0b11,
            p_RX_SUM_VCMTUNE=0b0000,
            p_RX_SUM_VCM_OVWR=0b0,
            p_RX_SUM_VREF_TUNE=0b000,
            p_RX_TUNE_AFE_OS=0b10,
            p_RX_WIDEMODE_CDR=0b0,
            p_RX_XCLK_SEL="RXDES",
        )
        xilinx_mess.update(
            p_SAS_MAX_COM=64,
            p_SAS_MIN_COM=36,
            p_SATA_BURST_SEQ_LEN=0b1110,
            p_SATA_CPLL_CFG="VCO_3000MHZ",
            p_SATA_MAX_BURST=8,
            p_SATA_MAX_INIT=21,
            p_SATA_MAX_WAKE=7,
            p_SATA_MIN_BURST=4,
            p_SATA_MIN_INIT=12,
            p_SATA_MIN_WAKE=4,
            p_SHOW_REALIGN_COMMA="FALSE",
            p_SIM_RECEIVER_DETECT_PASS="TRUE",
            p_SIM_RESET_SPEEDUP="TRUE",
            p_SIM_TX_EIDLE_DRIVE_LEVEL=0b0,
            p_SIM_VERSION=2,
            p_TAPDLY_SET_TX=0b00,
            p_TEMPERATUR_PAR=0b0010,
            p_TERM_RCAL_CFG=0b100001000010000,
            p_TERM_RCAL_OVRD=0b000,
            p_TRANS_TIME_RATE=0b00001110,
            p_TST_RSV0=0b00000000,
            p_TST_RSV1=0b00000000,
            p_TXBUF_EN="TRUE",
            p_TXBUF_RESET_ON_RATE_CHANGE="TRUE",
            p_TXDLY_CFG=0b0000000000001001,
            p_TXDLY_LCFG=0b0000000001010000,
            p_TXDRVBIAS_N=0b1010,
            p_TXDRVBIAS_P=0b1010,
            p_TXFIFO_ADDR_CFG="LOW",
            p_TXGBOX_FIFO_INIT_RD_ADDR=4,
            p_TXGEARBOX_EN="FALSE",
            p_TXOUT_DIV=4,
            p_TXPCSRESET_TIME=0b00011,
            p_TXPHDLY_CFG0=0b0010000000100000,
            p_TXPHDLY_CFG1=0b0000000001110101,
            p_TXPH_CFG=0b0000100110000000,
            p_TXPH_MONITOR_SEL=0b00000,
            p_TXPI_CFG0=0b01,
            p_TXPI_CFG1=0b01,
            p_TXPI_CFG2=0b01,
            p_TXPI_CFG3=0b1,
            p_TXPI_CFG4=0b1,
            p_TXPI_CFG5=0b011,
            p_TXPI_GRAY_SEL=0b0,
            p_TXPI_INVSTROBE_SEL=0b0,
            p_TXPI_LPM=0b0,
            p_TXPI_PPMCLK_SEL="TXUSRCLK2",
            p_TXPI_PPM_CFG=0b00000000,
            p_TXPI_SYNFREQ_PPM=0b001,
            p_TXPI_VREFSEL=0b0,
            p_TXPMARESET_TIME=0b00011,
            p_TXSYNC_MULTILANE=0b0,
            p_TXSYNC_OVRD=0b0,
            p_TXSYNC_SKIP_DA=0b0,
            p_TX_CLK25_DIV=8,
            p_TX_CLKMUX_EN=0b1,
            p_TX_DATA_WIDTH=20,
            p_TX_DCD_CFG=0b000010,
            p_TX_DCD_EN=0b0,
            p_TX_DEEMPH0=0b000000,
            p_TX_DEEMPH1=0b000000,
            p_TX_DIVRESET_TIME=0b00001,
            p_TX_DRIVE_MODE="DIRECT",
            p_TX_EIDLE_ASSERT_DELAY=0b100,
            p_TX_EIDLE_DEASSERT_DELAY=0b011,
            p_TX_EML_PHI_TUNE=0b0,
            p_TX_FABINT_USRCLK_FLOP=0b0,
            p_TX_IDLE_DATA_ZERO=0b0,
            p_TX_INT_DATAWIDTH=0,
            p_TX_LOOPBACK_DRIVE_HIZ="FALSE",
            p_TX_MAINCURSOR_SEL=0b0,
            p_TX_MARGIN_FULL_0=0b1001111,
            p_TX_MARGIN_FULL_1=0b1001110,
            p_TX_MARGIN_FULL_2=0b1001100,
            p_TX_MARGIN_FULL_3=0b1001010,
            p_TX_MARGIN_FULL_4=0b1001000,
            p_TX_MARGIN_LOW_0=0b1000110,
            p_TX_MARGIN_LOW_1=0b1000101,
            p_TX_MARGIN_LOW_2=0b1000011,
            p_TX_MARGIN_LOW_3=0b1000010,
            p_TX_MARGIN_LOW_4=0b1000000,
            p_TX_MODE_SEL=0b000,
            p_TX_PMADATA_OPT=0b0,
            p_TX_PMA_POWER_SAVE=0b0,
            p_TX_PROGCLK_SEL="CPLL",
            p_TX_PROGDIV_CFG=20.0,
            p_TX_QPI_STATUS_EN=0b0,
            p_TX_RXDETECT_CFG=0b00000000110010,
            p_TX_RXDETECT_REF=0b100,
            p_TX_SAMPLE_PERIOD=0b111,
            p_TX_SARC_LPBK_ENB=0b0,
            p_TX_XCLK_SEL="TXOUT",
            p_USE_PCS_CLK_PHASE_SEL=0b0,
            p_WB_MODE=0b00,
        )
        xilinx_mess.update(
            i_CFGRESET=0b0,
            i_CLKRSVD0=0b0,
            i_CLKRSVD1=0b0,
            i_CPLLLOCKDETCLK=0b0,
            i_CPLLLOCKEN=0b1,
            i_CPLLPD=pll_reset,
            i_CPLLREFCLKSEL=0b001,
            i_CPLLRESET=0b0,
            i_DMONFIFORESET=0b0,
            i_DMONITORCLK=0b0,
            i_DRPADDR=0b000000000,
            i_DRPCLK=0b0,
            i_DRPDI=0b0000000000000000,
            i_DRPEN=0b0,
            i_DRPWE=0b0,
            i_EVODDPHICALDONE=0b0,
            i_EVODDPHICALSTART=0b0,
            i_EVODDPHIDRDEN=0b0,
            i_EVODDPHIDWREN=0b0,
            i_EVODDPHIXRDEN=0b0,
            i_EVODDPHIXWREN=0b0,
            i_EYESCANMODE=0b0,
            i_EYESCANRESET=0b0,
            i_EYESCANTRIGGER=0b0,
            i_GTGREFCLK=0b0,
            i_GTHRXN=data_pads.rxn,
            i_GTHRXP=data_pads.rxp,
            i_GTNORTHREFCLK0=0b0,
            i_GTNORTHREFCLK1=0b0,
            i_GTREFCLK0=refclk,
            i_GTREFCLK1=0b0,
            i_GTRESETSEL=0b0,
            i_GTRSVD=0b0000000000000000,
            i_GTRXRESET=rx_reset,
            i_GTSOUTHREFCLK0=0b0,
            i_GTSOUTHREFCLK1=0b0,
            i_GTTXRESET=tx_reset,
            i_LOOPBACK=0b000,
            i_LPBKRXTXSEREN=0b0,
            i_LPBKTXRXSEREN=0b0,
            i_PCIEEQRXEQADAPTDONE=0b0,
            i_PCIERSTIDLE=0b0,
            i_PCIERSTTXSYNCSTART=0b0,
            i_PCIEUSERRATEDONE=0b0,
            i_PCSRSVDIN2=0b00000,
            i_PCSRSVDIN=0b0000000000000000,
            i_PMARSVDIN=0b00000,
            i_QPLL0CLK=0b0,
            i_QPLL0REFCLK=0b0,
            i_QPLL1CLK=0b0,
            i_QPLL1REFCLK=0b0,
            i_RESETOVRD=0b0,
            i_RSTCLKENTX=0b0,
            i_RX8B10BEN=0b0,
            i_RXBUFRESET=0b0,
            i_RXCDRFREQRESET=0b0,
            i_RXCDRHOLD=0b0,
            i_RXCDROVRDEN=0b0,
            i_RXCDRRESETRSV=0b0,
            i_RXCDRRESET=0b0,
            i_RXCHBONDEN=0b0,
            i_RXCHBONDI=0b00000,
            i_RXCHBONDLEVEL=0b000,
            i_RXCHBONDMASTER=0b0,
            i_RXCHBONDSLAVE=0b0,
            i_RXCOMMADETEN=0b0,
            i_RXDFEAGCCTRL=0b01,
            i_RXDFEAGCHOLD=0b0,
            i_RXDFEAGCOVRDEN=0b0,
            i_RXDFELFHOLD=0b0,
            i_RXDFELFOVRDEN=0b0,
            i_RXDFELPMRESET=0b0,
            i_RXDFETAP10HOLD=0b0,
            i_RXDFETAP10OVRDEN=0b0,
            i_RXDFETAP11HOLD=0b0,
            i_RXDFETAP11OVRDEN=0b0,
            i_RXDFETAP12HOLD=0b0,
            i_RXDFETAP12OVRDEN=0b0,
            i_RXDFETAP13HOLD=0b0,
            i_RXDFETAP13OVRDEN=0b0,
            i_RXDFETAP14HOLD=0b0,
            i_RXDFETAP14OVRDEN=0b0,
            i_RXDFETAP15HOLD=0b0,
            i_RXDFETAP15OVRDEN=0b0,
            i_RXDFETAP2HOLD=0b0,
            i_RXDFETAP2OVRDEN=0b0,
            i_RXDFETAP3HOLD=0b0,
            i_RXDFETAP3OVRDEN=0b0,
            i_RXDFETAP4HOLD=0b0,
            i_RXDFETAP4OVRDEN=0b0,
            i_RXDFETAP5HOLD=0b0,
            i_RXDFETAP5OVRDEN=0b0,
            i_RXDFETAP6HOLD=0b0,
            i_RXDFETAP6OVRDEN=0b0,
            i_RXDFETAP7HOLD=0b0,
            i_RXDFETAP7OVRDEN=0b0,
            i_RXDFETAP8HOLD=0b0,
            i_RXDFETAP8OVRDEN=0b0,
            i_RXDFETAP9HOLD=0b0,
            i_RXDFETAP9OVRDEN=0b0,
            i_RXDFEUTHOLD=0b0,
            i_RXDFEUTOVRDEN=0b0,
            i_RXDFEVPHOLD=0b0,
            i_RXDFEVPOVRDEN=0b0,
            i_RXDFEVSEN=0b0,
            i_RXDFEXYDEN=0b1,
            i_RXDLYBYPASS=0b1,
            i_RXDLYEN=0b0,
            i_RXDLYOVRDEN=0b0,
            i_RXDLYSRESET=0b0,
            i_RXELECIDLEMODE=0b11,
            i_RXGEARBOXSLIP=0b0,
            i_RXLATCLK=0b0,
            i_RXLPMEN=0b1,
            i_RXLPMGCHOLD=0b0,
            i_RXLPMGCOVRDEN=0b0,
            i_RXLPMHFHOLD=0b0,
            i_RXLPMHFOVRDEN=0b0,
            i_RXLPMLFHOLD=0b0,
            i_RXLPMLFKLOVRDEN=0b0,
            i_RXLPMOSHOLD=0b0,
            i_RXLPMOSOVRDEN=0b0,
            i_RXMCOMMAALIGNEN=0b0,
            i_RXMONITORSEL=0b00,
            i_RXOOBRESET=0b0,
            i_RXOSCALRESET=0b0,
            i_RXOSHOLD=0b0,
            i_RXOSINTCFG=0b1101,
            i_RXOSINTEN=0b1,
            i_RXOSINTHOLD=0b0,
            i_RXOSINTOVRDEN=0b0,
            i_RXOSINTSTROBE=0b0,
            i_RXOSINTTESTOVRDEN=0b0,
            i_RXOSOVRDEN=0b0,
            i_RXOUTCLKSEL=0b101,
            i_RXPCOMMAALIGNEN=0b0,
            i_RXPCSRESET=0b0,
            i_RXPD=0b00,
            i_RXPHALIGNEN=0b0,
            i_RXPHALIGN=0b0,
            i_RXPHDLYPD=0b1,
            i_RXPHDLYRESET=0b0,
            i_RXPHOVRDEN=0b0,
            i_RXPLLCLKSEL=0b00,
            i_RXPMARESET=0b0,
            i_RXPOLARITY=0b0,
            i_RXPRBSCNTRESET=0b0,
            i_RXPRBSSEL=0b0000,
            i_RXPROGDIVRESET=0b0,
            i_RXQPIEN=0b0,
            i_RXRATEMODE=0b0,
            i_RXRATE=0b000,
            i_RXSLIDE=0b0,
            i_RXSLIPOUTCLK=0b0,
            i_RXSLIPPMA=0b0,
            i_RXSYNCALLIN=0b0,
            i_RXSYNCIN=0b0,
            i_RXSYNCMODE=0b0,
            i_RXSYSCLKSEL=0b00,
            i_RXUSERRDY=0b1,
            i_RXUSRCLK2=ClockSignal("eth_rx_half"),
            i_RXUSRCLK=ClockSignal("eth_rx_half"),
            #i_SATA_BURST=0b100,
            #i_SATA_EIDLE=0b100,
            i_SIGVALIDCLK=0b0,
            i_TSTIN=0b00000000000000000000,
            i_TX8B10BBYPASS=0b00000000,
            i_TX8B10BEN=0b0,
            i_TXBUFDIFFCTRL=0b000,
            i_TXCOMINIT=0b0,
            i_TXCOMSAS=0b0,
            i_TXCOMWAKE=0b0,
            i_TXCTRL0=Cat(*[tx_data[10*i+8] for i in range(2)]),
            i_TXCTRL1=Cat(*[tx_data[10*i+9] for i in range(2)]),
            i_TXCTRL2=0b00000000,
            i_TXDATAEXTENDRSVD=0b00000000,
            i_TXDATA=Cat(*[tx_data[10*i:10*i+8] for i in range(2)]),
            i_TXDEEMPH=0b0,
            i_TXDETECTRX=0b0,
            i_TXDIFFCTRL=0b1100,
            i_TXDIFFPD=0b0,
            i_TXDLYBYPASS=0b1,
            i_TXDLYEN=0b0,
            i_TXDLYHOLD=0b0,
            i_TXDLYOVRDEN=0b0,
            i_TXDLYSRESET=0b0,
            i_TXDLYUPDOWN=0b0,
            i_TXELECIDLE=0b0,
            i_TXHEADER=0b000000,
            i_TXINHIBIT=0b0,
            i_TXLATCLK=0b0,
            i_TXMAINCURSOR=0b1000000,
            i_TXMARGIN=0b000,
            i_TXOUTCLKSEL=0b101,
            i_TXPCSRESET=0b0,
            i_TXPDELECIDLEMODE=0b0,
            i_TXPD=0b00,
            i_TXPHALIGNEN=0b0,
            i_TXPHALIGN=0b0,
            i_TXPHDLYPD=0b1,
            i_TXPHDLYRESET=0b0,
            i_TXPHDLYTSTCLK=0b0,
            i_TXPHINIT=0b0,
            i_TXPHOVRDEN=0b0,
            i_TXPIPPMEN=0b0,
            i_TXPIPPMOVRDEN=0b0,
            i_TXPIPPMPD=0b0,
            i_TXPIPPMSEL=0b0,
            i_TXPIPPMSTEPSIZE=0b00000,
            i_TXPISOPD=0b0,
            i_TXPLLCLKSEL=0b00,
            i_TXPMARESET=0b0,
            i_TXPOLARITY=0b0,
            i_TXPOSTCURSORINV=0b0,
            i_TXPOSTCURSOR=0b00000,
            i_TXPRBSFORCEERR=0b0,
            i_TXPRBSSEL=0b0000,
            i_TXPRECURSORINV=0b0,
            i_TXPRECURSOR=0b00000,
            i_TXPROGDIVRESET=0b0,
            i_TXQPIBIASEN=0b0,
            i_TXQPISTRONGPDOWN=0b0,
            i_TXQPIWEAKPUP=0b0,
            i_TXRATEMODE=0b0,
            i_TXRATE=0b000,
            i_TXSEQUENCE=0b0000000,
            i_TXSWING=0b0,
            i_TXSYNCALLIN=0b0,
            i_TXSYNCIN=0b0,
            i_TXSYNCMODE=0b0,
            i_TXSYSCLKSEL=0b00,
            i_TXUSERRDY=0b1,
            i_TXUSRCLK2=ClockSignal("eth_tx_half"),
            i_TXUSRCLK=ClockSignal("eth_tx_half"),
        )
        xilinx_mess.update(
            # o_BUFGTCE=,
            # o_BUFGTCEMASK=,
            # o_BUFGTDIV=,
            # o_BUFGTRESET=,
            # o_BUFGTRSTMASK=,
            # o_CPLLFBCLKLOST=,
            o_CPLLLOCK=pll_locked,
            # o_CPLLREFCLKLOST=,
            # o_DMONITOROUT=,
            # o_DRPDO=,
            # o_DRPRDY=,
            # o_EYESCANDATAERROR=,
            o_GTHTXN=data_pads.txn,
            o_GTHTXP=data_pads.txp,
            o_GTPOWERGOOD=gtpowergood,
            # o_GTREFCLKMONITOR=,
            # o_PCIERATEGEN3=,
            # o_PCIERATEIDLE=,
            # o_PCIERATEQPLLPD=,
            # o_PCIERATEQPLLRESET=,
            # o_PCIESYNCTXSYNCDONE=,
            # o_PCIEUSERGEN3RDY=,
            # o_PCIEUSERPHYSTATUSRST=,
            # o_PCIEUSERRATESTART=,
            # o_PCSRSVDOUT=,
            # o_PHYSTATUS=,
            # o_PINRSRVDAS=,
            # o_RESETEXCEPTION=,
            # o_RXBUFSTATUS=,
            # o_RXBYTEISALIGNED=,
            # o_RXBYTEREALIGN=,
            # o_RXCDRLOCK=,
            # o_RXCDRPHDONE=,
            # o_RXCHANBONDSEQ=,
            # o_RXCHANISALIGNED=,
            # o_RXCHANREALIGN=,
            # o_RXCHBONDO=,
            # o_RXCLKCORCNT=,
            # o_RXCOMINITDET=,
            # o_RXCOMMADET=,
            # o_RXCOMSASDET=,
            # o_RXCOMWAKEDET=,
            o_RXCTRL0=Cat(*[rx_data[10*i+8] for i in range(2)]),
            o_RXCTRL1=Cat(*[rx_data[10*i+9] for i in range(2)]),
            # o_RXCTRL2=,
            # o_RXCTRL3=,
            o_RXDATA=Cat(*[rx_data[10*i:10*i+8] for i in range(2)]),
            # o_RXDATAEXTENDRSVD=,
            # o_RXDATAVALID=,
            # o_RXDLYSRESETDONE=,
            # o_RXELECIDLE=,
            # o_RXHEADER=,
            # o_RXHEADERVALID=,
            # o_RXMONITOROUT=,
            # o_RXOSINTDONE=,
            # o_RXOSINTSTARTED=,
            # o_RXOSINTSTROBEDONE=,
            # o_RXOSINTSTROBESTARTED=,
            o_RXOUTCLK=self.rxoutclk,
            # o_RXOUTCLKFABRIC=,
            # o_RXOUTCLKPCS=,
            # o_RXPHALIGNDONE=,
            # o_RXPHALIGNERR=,
            # o_RXPMARESETDONE=,
            # o_RXPRBSERR=,
            # o_RXPRBSLOCKED=,
            # o_RXPRGDIVRESETDONE=,
            # o_RXQPISENN=,
            # o_RXQPISENP=,
            # o_RXRATEDONE=,
            # o_RXRECCLKOUT=,
            o_RXRESETDONE=rx_reset_done,
            # o_RXSLIDERDY=,
            # o_RXSLIPDONE=,
            # o_RXSLIPOUTCLKRDY=,
            # o_RXSLIPPMARDY=,
            # o_RXSTARTOFSEQ=,
            # o_RXSTATUS=,
            # o_RXSYNCDONE=,
            # o_RXSYNCOUT=,
            # o_RXVALID=,
            # o_TXBUFSTATUS=,
            # o_TXCOMFINISH=,
            # o_TXDLYSRESETDONE=,
            o_TXOUTCLK=self.txoutclk,
            # o_TXOUTCLKFABRIC=,
            # o_TXOUTCLKPCS=,
            # o_TXPHALIGNDONE=,
            # o_TXPHINITDONE=,
            # o_TXPMARESETDONE=,
            # o_TXPRGDIVRESETDONE=,
            # o_TXQPISENN=,
            # o_TXQPISENP=,
            # o_TXRATEDONE=,
            o_TXRESETDONE=tx_reset_done,
            # o_TXSYNCDONE=,
            # o_TXSYNCOUT=,
        )
        tx_bufg_gt_ce = Signal()
        tx_bufg_gt_clr = Signal()
        rx_bufg_gt_ce = Signal()
        rx_bufg_gt_clr = Signal()
        self.specials += [
            Instance("GTHE3_CHANNEL", **xilinx_mess),
            Instance("BUFG_GT", i_I=self.txoutclk, o_O=self.cd_eth_tx.clk, i_DIV=0),
            Instance("BUFG_GT", i_I=self.txoutclk, o_O=self.cd_eth_tx_half.clk, i_DIV=1),
            Instance("BUFG_GT", i_I=self.rxoutclk, o_O=self.cd_eth_rx.clk, i_DIV=0),
            Instance("BUFG_GT", i_I=self.rxoutclk, o_O=self.cd_eth_rx_half.clk, i_DIV=1),
            AsyncResetSynchronizer(self.cd_eth_tx, ~tx_reset_done),
            AsyncResetSynchronizer(self.cd_eth_rx, ~rx_reset_done),
        ]

        # Transceiver reset
        pll_reset_cycles = round(300000*sys_clk_freq//1000000000)
        reset_counter = Signal(max=pll_reset_cycles+1)
        self.sync += [
            If(~gtpowergood,
                pll_reset.eq(1),
                reset_counter.eq(0)
            ).Else(
                If(reset_counter == pll_reset_cycles,
                    pll_reset.eq(0)
                ).Else(
                    reset_counter.eq(reset_counter + 1)
                )
            )
        ]
        self.comb += [
            tx_reset.eq(pll_reset | ~pll_locked),
            rx_reset.eq(pll_reset | ~pll_locked | pcs.restart)
        ]

        # Gearbox and PCS connection
        gearbox = Gearbox()
        self.submodules += gearbox

        self.comb += [
            tx_data.eq(gearbox.tx_data_half),
            gearbox.rx_data_half.eq(rx_data),

            gearbox.tx_data.eq(pcs.tbi_tx),
            pcs.tbi_rx.eq(gearbox.rx_data)
        ]
