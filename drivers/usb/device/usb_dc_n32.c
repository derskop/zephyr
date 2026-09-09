/*
 * Copyright (c) 2026 Nations Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief USB device controller shim driver for N32G45x devices
 *
 * The N32G45x embeds a full-speed USB FS device controller with a dedicated
 * packet memory (PMA).  This driver implements the Zephyr USB device
 * controller API with a self-built register layer plus a command-level (LL)
 * layer, all contained in this file.  The ISR only posts events to a
 * lower-half thread, so no PMA access or upper stack calls run in interrupt
 * context.  N32 specifics: the 48 MHz USB clock is derived from the PLL
 * output through the RCC_CFG.USBPRES prescaler, and the internal D+ pull-up
 * is a bit in a GPIO-system register (no disconnect GPIO).
 */

#include <stdbool.h>
#include <string.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/irq.h>

#define DT_DRV_COMPAT nsing_n32_usb

#define LOG_LEVEL CONFIG_USB_DRIVER_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usb_dc_n32);

#ifdef CONFIG_SOC_SERIES_N32G45X

#include <n32g45x.h>

/*
 * ------------------------------------------------------------------------- *
 * Register layer                                                             *
 * ------------------------------------------------------------------------- *
 * Register map and PMA of the N32G45x USB FS follow the bit definitions of
 * Nations' usb_regs.h, adapted to the N32 "USB_Module" struct (32-bit
 * register fields at 0x40005c00) plus the Packet Memory Area at 0x40006000.
 *
 * Buffer addresses are written into the buffer table ADDR<15:1> fields in
 * "PMA address" units in which one unit is one 16-bit halfword (physical byte
 * offset = value * USB_DC_N32_PMA_ACCESS).  This matches Nations'
 * usb_regs.h/_SetEPxxxAddr macros (wAddr >> 1 << 1).
 */
/* USB IP register base */
#define USB_DC_N32_REGBASE 0x40005C00UL

/* Packet Memory Area base */
#define USB_DC_N32_PMA_BASE 0x40006000UL

/* Each PMA "address" unit is one halfword; byte granularity of the DMA. */
#define USB_DC_N32_PMA_ACCESS 2U

/* Buffer table base; packet buffers are allocated right after it */
#define USB_DC_N32_BTABLE_ADDRESS 0U

/* Internal DP pull-up control register (bit 28) */
#define USB_DC_N32_DP_CTRL     0x40001820UL
#define USB_DC_N32_DP_CTRL_PUP (0x1UL << 28)

/* ------------------------------------------------------------------ */
/* ISTR (STS) interrupt events (clear by write)                        */
/* ------------------------------------------------------------------ */
#define USB_DC_N32_STS_CTRS  0x8000U /* Correct TRansfer (clear-only bit) */
#define USB_DC_N32_STS_DOVR  0x4000U /* DMA OVeR/underrun (clear-only bit) */
#define USB_DC_N32_STS_ERROR 0x2000U /* ERRor (clear-only bit) */
#define USB_DC_N32_STS_WKUP  0x1000U /* WaKe UP (clear-only bit) */
#define USB_DC_N32_STS_SUSPD 0x0800U /* SUSPend (clear-only bit) */
#define USB_DC_N32_STS_RST   0x0400U /* RESET (clear-only bit) */
#define USB_DC_N32_STS_SOF   0x0200U /* Start Of Frame (clear-only bit) */
#define USB_DC_N32_STS_ESOF  0x0100U /* Expected Start Of Frame (clear-only bit) */
#define USB_DC_N32_STS_DIR   0x0010U /* DIRection of transaction (RO) */
#define USB_DC_N32_STS_EP_ID 0x000FU /* EndPoint IDentifier (RO) */

/* ------------------------------------------------------------------ */
/* CNTR (CTRL) control/mask bits                                       */
/* ------------------------------------------------------------------ */
#define USB_DC_N32_CTRL_CTRSM   0x8000U /* Correct TRansfer Mask */
#define USB_DC_N32_CTRL_DOVRM   0x4000U /* DMA OVeR/underrun Mask */
#define USB_DC_N32_CTRL_ERRORM  0x2000U /* ERRor Mask */
#define USB_DC_N32_CTRL_WKUPM   0x1000U /* WaKe UP Mask */
#define USB_DC_N32_CTRL_SUSPDM  0x0800U /* SUSPend Mask */
#define USB_DC_N32_CTRL_RSTM    0x0400U /* RESET Mask */
#define USB_DC_N32_CTRL_SOFM    0x0200U /* Start Of Frame Mask */
#define USB_DC_N32_CTRL_ESOFM   0x0100U /* Expected Start Of Frame Mask */
#define USB_DC_N32_CTRL_RESUM   0x0010U /* RESUME request */
#define USB_DC_N32_CTRL_FSUSPD  0x0008U /* Force SUSPend */
#define USB_DC_N32_CTRL_LP_MODE 0x0004U /* Low-power MODE */
#define USB_DC_N32_CTRL_PD      0x0002U /* Power DoWN */
#define USB_DC_N32_CTRL_FRST    0x0001U /* Force USB RESet */

/* ------------------------------------------------------------------ */
/* DADDR (ADDR)                                                        */
/* ------------------------------------------------------------------ */
#define USB_DC_N32_ADDR_EFUC 0x80U /* Enable Function */
#define USB_DC_N32_ADDR_ADDR 0x7FU /* Device Address */

/* ------------------------------------------------------------------ */
/* Endpoint register (EPnR) bits                                       */
/* ------------------------------------------------------------------ */
#define USB_DC_N32_EP_CTRS_RX   0x8000U /* EndPoint Correct TRansfer RX */
#define USB_DC_N32_EP_DATTOG_RX 0x4000U /* EndPoint Data TOGGLE RX */
#define USB_DC_N32_EPRX_STS     0x3000U /* EndPoint RX STATus bit field */
#define USB_DC_N32_EP_SETUP     0x0800U /* EndPoint SETUP */
#define USB_DC_N32_EP_T_FIELD   0x0600U /* EndPoint TYPE */
#define USB_DC_N32_EP_KIND      0x0100U /* EndPoint KIND */
#define USB_DC_N32_EP_CTRS_TX   0x0080U /* EndPoint Correct TRansfer TX */
#define USB_DC_N32_EP_DATTOG_TX 0x0040U /* EndPoint Data TOGGLE TX */
#define USB_DC_N32_EPTX_STS     0x0030U /* EndPoint TX STATus bit field */
#define USB_DC_N32_EPADDR_FIELD 0x000FU /* EndPoint ADDRess FIELD */

/* EPnR fields kept by a RMW (no toggle semantics) */
#define USB_DC_N32_EPREG_MASK                                                                      \
	(USB_DC_N32_EP_CTRS_RX | USB_DC_N32_EP_SETUP | USB_DC_N32_EP_T_FIELD |                     \
	 USB_DC_N32_EP_KIND | USB_DC_N32_EP_CTRS_TX | USB_DC_N32_EPADDR_FIELD)

/* EP_TYPE[1:0] */
#define USB_DC_N32_EP_TYPE_MASK   0x0600U
#define USB_DC_N32_EP_CONTROL     0x0200U /* CONTROL */
#define USB_DC_N32_EP_SYNC_NONE   0x0000U /* bulk */
#define USB_DC_N32_EP_BULK        0x0000U /* BULK */
#define USB_DC_N32_EP_ISOCHRONOUS 0x0400U /* ISOCHRONOUS */
#define USB_DC_N32_EP_INTERRUPT   0x0600U /* INTERRUPT */
#define USB_DC_N32_EP_T_MASK      (~USB_DC_N32_EP_T_FIELD & USB_DC_N32_EPREG_MASK)
#define USB_DC_N32_EPKIND_MASK    (~USB_DC_N32_EP_KIND & USB_DC_N32_EPREG_MASK)

/* STAT_TX[1:0] */
#define USB_DC_N32_EP_TX_DIS       0x0000U
#define USB_DC_N32_EP_TX_STALL     0x0010U
#define USB_DC_N32_EP_TX_NAK       0x0020U
#define USB_DC_N32_EP_TX_VALID     0x0030U
#define USB_DC_N32_EPTX_DATTOG1    0x0010U
#define USB_DC_N32_EPTX_DATTOG2    0x0020U
#define USB_DC_N32_EPTX_DATTOGMASK (USB_DC_N32_EPTX_STS | USB_DC_N32_EPREG_MASK)

/* STAT_RX[1:0] */
#define USB_DC_N32_EP_RX_DIS       0x0000U
#define USB_DC_N32_EP_RX_STALL     0x1000U
#define USB_DC_N32_EP_RX_NAK       0x2000U
#define USB_DC_N32_EP_RX_VALID     0x3000U
#define USB_DC_N32_EPRX_DATTOG1    0x1000U
#define USB_DC_N32_EPRX_DATTOG2    0x2000U
#define USB_DC_N32_EPRX_DATTOGMASK (USB_DC_N32_EPRX_STS | USB_DC_N32_EPREG_MASK)

/* ------------------------------------------------------------------ */
/* Register accessors (16-bit semantics in 32-bit USB_Module fields)   */
/* ------------------------------------------------------------------ */

static inline uint16_t usb_dc_n32_get_ctrl(USB_Module *usb)
{
	return (uint16_t)usb->CTRL;
}

static inline void usb_dc_n32_set_ctrl(USB_Module *usb, uint16_t val)
{
	usb->CTRL = val;
}

static inline uint16_t usb_dc_n32_get_istr(USB_Module *usb)
{
	return (uint16_t)usb->STS;
}

static inline void usb_dc_n32_set_istr(USB_Module *usb, uint16_t val)
{
	/* ISTR status flags are "clear-only": a flag is cleared by writing 0
	 * at its bit position, i.e. the write value is the complement (~STS_x),
	 * matching the Nations _SetISTR(CLR_x) convention.  Writing the flag
	 * itself leaves it set (and would re-trigger the interrupt forever).
	 */
	usb->STS = val;
}

static inline uint16_t usb_dc_n32_get_daddr(USB_Module *usb)
{
	return (uint16_t)usb->ADDR;
}

static inline void usb_dc_n32_set_daddr(USB_Module *usb, uint16_t val)
{
	usb->ADDR = val;
}

static inline uint16_t usb_dc_n32_get_btable(USB_Module *usb)
{
	return (uint16_t)usb->BUFTAB;
}

static inline void usb_dc_n32_set_btable(USB_Module *usb, uint16_t val)
{
	usb->BUFTAB = (uint16_t)(val & 0xFFF8U);
}

static inline uint16_t usb_dc_n32_get_ep(USB_Module *usb, uint8_t epnum)
{
	return (uint16_t)((volatile uint32_t *)(&usb->EP0))[epnum];
}

static inline void usb_dc_n32_set_ep(USB_Module *usb, uint8_t epnum, uint16_t val)
{
	((volatile uint32_t *)(&usb->EP0))[epnum] = val;
}

/* SET / status-toggle helpers (mirror Nations _SetEPTxStatus/_SetEPRxStatus) */
static inline void usb_dc_n32_set_ep_type(USB_Module *usb, uint8_t epnum, uint16_t type)
{
	usb_dc_n32_set_ep(
		usb, epnum,
		(uint16_t)((usb_dc_n32_get_ep(usb, epnum) & USB_DC_N32_EP_T_MASK) | type));
}

static inline void usb_dc_n32_set_ep_address(USB_Module *usb, uint8_t epnum, uint8_t addr)
{
	usb_dc_n32_set_ep(usb, epnum,
			  (uint16_t)(USB_DC_N32_EP_CTRS_RX | USB_DC_N32_EP_CTRS_TX |
				     (usb_dc_n32_get_ep(usb, epnum) & USB_DC_N32_EPREG_MASK) |
				     addr));
}

static inline uint8_t usb_dc_n32_get_ep_address(USB_Module *usb, uint8_t epnum)
{
	return (uint8_t)(usb_dc_n32_get_ep(usb, epnum) & USB_DC_N32_EPADDR_FIELD);
}

static inline void usb_dc_n32_set_ep_tx_status(USB_Module *usb, uint8_t epnum, uint16_t state)
{
	uint16_t wreg = usb_dc_n32_get_ep(usb, epnum) & USB_DC_N32_EPTX_DATTOGMASK;

	if ((USB_DC_N32_EPTX_DATTOG1 & state) != 0U) {
		wreg ^= USB_DC_N32_EPTX_DATTOG1;
	}
	if ((USB_DC_N32_EPTX_DATTOG2 & state) != 0U) {
		wreg ^= USB_DC_N32_EPTX_DATTOG2;
	}

	usb_dc_n32_set_ep(usb, epnum,
			  (uint16_t)(wreg | USB_DC_N32_EP_CTRS_RX | USB_DC_N32_EP_CTRS_TX));
}

static inline void usb_dc_n32_set_ep_rx_status(USB_Module *usb, uint8_t epnum, uint16_t state)
{
	uint16_t wreg = usb_dc_n32_get_ep(usb, epnum) & USB_DC_N32_EPRX_DATTOGMASK;

	if ((USB_DC_N32_EPRX_DATTOG1 & state) != 0U) {
		wreg ^= USB_DC_N32_EPRX_DATTOG1;
	}
	if ((USB_DC_N32_EPRX_DATTOG2 & state) != 0U) {
		wreg ^= USB_DC_N32_EPRX_DATTOG2;
	}

	usb_dc_n32_set_ep(usb, epnum,
			  (uint16_t)(wreg | USB_DC_N32_EP_CTRS_RX | USB_DC_N32_EP_CTRS_TX));
}

static inline void usb_dc_n32_set_ep_rxtx_status(USB_Module *usb, uint8_t epnum, uint16_t state_rx,
						 uint16_t state_tx)
{
	uint16_t wreg =
		usb_dc_n32_get_ep(usb, epnum) & (USB_DC_N32_EPRX_DATTOGMASK | USB_DC_N32_EPTX_STS);

	if ((USB_DC_N32_EPRX_DATTOG1 & state_rx) != 0U) {
		wreg ^= USB_DC_N32_EPRX_DATTOG1;
	}
	if ((USB_DC_N32_EPRX_DATTOG2 & state_rx) != 0U) {
		wreg ^= USB_DC_N32_EPRX_DATTOG2;
	}
	if ((USB_DC_N32_EPTX_DATTOG1 & state_tx) != 0U) {
		wreg ^= USB_DC_N32_EPTX_DATTOG1;
	}
	if ((USB_DC_N32_EPTX_DATTOG2 & state_tx) != 0U) {
		wreg ^= USB_DC_N32_EPTX_DATTOG2;
	}

	usb_dc_n32_set_ep(usb, epnum,
			  (uint16_t)(wreg | USB_DC_N32_EP_CTRS_RX | USB_DC_N32_EP_CTRS_TX));
}

static inline uint16_t usb_dc_n32_get_ep_tx_status(USB_Module *usb, uint8_t epnum)
{
	return (uint16_t)(usb_dc_n32_get_ep(usb, epnum) & USB_DC_N32_EPTX_STS);
}

static inline uint16_t usb_dc_n32_get_ep_rx_status(USB_Module *usb, uint8_t epnum)
{
	return (uint16_t)(usb_dc_n32_get_ep(usb, epnum) & USB_DC_N32_EPRX_STS);
}

static inline void usb_dc_n32_clear_ep_ctr_rx(USB_Module *usb, uint8_t epnum)
{
	usb_dc_n32_set_ep(
		usb, epnum,
		(uint16_t)(usb_dc_n32_get_ep(usb, epnum) & 0x7FFFU & USB_DC_N32_EPREG_MASK));
}

static inline void usb_dc_n32_clear_ep_ctr_tx(USB_Module *usb, uint8_t epnum)
{
	usb_dc_n32_set_ep(
		usb, epnum,
		(uint16_t)(usb_dc_n32_get_ep(usb, epnum) & 0xFF7FU & USB_DC_N32_EPREG_MASK));
}

static inline void usb_dc_n32_toggle_dtog_rx(USB_Module *usb, uint8_t epnum)
{
	usb_dc_n32_set_ep(usb, epnum,
			  (uint16_t)(USB_DC_N32_EP_CTRS_RX | USB_DC_N32_EP_CTRS_TX |
				     USB_DC_N32_EP_DATTOG_RX |
				     (usb_dc_n32_get_ep(usb, epnum) & USB_DC_N32_EPREG_MASK)));
}

static inline void usb_dc_n32_toggle_dtog_tx(USB_Module *usb, uint8_t epnum)
{
	usb_dc_n32_set_ep(usb, epnum,
			  (uint16_t)(USB_DC_N32_EP_CTRS_RX | USB_DC_N32_EP_CTRS_TX |
				     USB_DC_N32_EP_DATTOG_TX |
				     (usb_dc_n32_get_ep(usb, epnum) & USB_DC_N32_EPREG_MASK)));
}

static inline void usb_dc_n32_clear_dtog_rx(USB_Module *usb, uint8_t epnum)
{
	if ((usb_dc_n32_get_ep(usb, epnum) & USB_DC_N32_EP_DATTOG_RX) != 0U) {
		usb_dc_n32_toggle_dtog_rx(usb, epnum);
	}
}

static inline void usb_dc_n32_clear_dtog_tx(USB_Module *usb, uint8_t epnum)
{
	if ((usb_dc_n32_get_ep(usb, epnum) & USB_DC_N32_EP_DATTOG_TX) != 0U) {
		usb_dc_n32_toggle_dtog_tx(usb, epnum);
	}
}

static inline void usb_dc_n32_set_ep_kind(USB_Module *usb, uint8_t epnum)
{
	usb_dc_n32_set_ep(usb, epnum,
			  (uint16_t)(USB_DC_N32_EP_CTRS_RX | USB_DC_N32_EP_CTRS_TX |
				     ((usb_dc_n32_get_ep(usb, epnum) | USB_DC_N32_EP_KIND) &
				      USB_DC_N32_EPREG_MASK)));
}

/* ------------------------------------------------------------------ */
/* Buffer table (PMA) cell access                                      */
/* ------------------------------------------------------------------ */

/* one cell = 16-bit word; cell index -> byte address: val * 2 */
static inline volatile uint16_t *usb_dc_n32_pma_cell(USB_Module *usb, uint8_t epnum, uint8_t cell)
{
	return (volatile uint16_t *)(USB_DC_N32_PMA_BASE +
				     ((uint32_t)(usb_dc_n32_get_btable(usb) + epnum * 8 + cell) *
				      USB_DC_N32_PMA_ACCESS));
}

#define USB_DC_N32_EP_TX_ADDR_CELL 0U
#define USB_DC_N32_EP_TX_CNT_CELL  2U
#define USB_DC_N32_EP_RX_ADDR_CELL 4U
#define USB_DC_N32_EP_RX_CNT_CELL  6U

static inline void usb_dc_n32_set_ep_tx_addr(USB_Module *usb, uint8_t epnum, uint16_t addr)
{
	*usb_dc_n32_pma_cell(usb, epnum, USB_DC_N32_EP_TX_ADDR_CELL) = (uint16_t)((addr >> 1) << 1);
}

static inline void usb_dc_n32_set_ep_rx_addr(USB_Module *usb, uint8_t epnum, uint16_t addr)
{
	*usb_dc_n32_pma_cell(usb, epnum, USB_DC_N32_EP_RX_ADDR_CELL) = (uint16_t)((addr >> 1) << 1);
}

static inline uint16_t usb_dc_n32_get_ep_tx_addr(USB_Module *usb, uint8_t epnum)
{
	return *usb_dc_n32_pma_cell(usb, epnum, USB_DC_N32_EP_TX_ADDR_CELL);
}

static inline void usb_dc_n32_set_ep_tx_cnt(USB_Module *usb, uint8_t epnum, uint16_t count)
{
	*usb_dc_n32_pma_cell(usb, epnum, USB_DC_N32_EP_TX_CNT_CELL) = count;
}

/* RX count in number of 2-byte (or 32-byte) blocks, mirror of
 * _SetEPCountRxReg (block-size flag at bit 15, block count at 14:10).
 */
static inline void usb_dc_n32_set_ep_rx_cnt(USB_Module *usb, uint8_t epnum, uint16_t count)
{
	volatile uint16_t *cell = usb_dc_n32_pma_cell(usb, epnum, USB_DC_N32_EP_RX_CNT_CELL);
	uint16_t nblocks;

	if (count > 62U) {
		nblocks = (uint16_t)(count >> 5);
		if ((count & 0x1FU) == 0U) {
			nblocks--;
		}
		*cell = (uint16_t)((nblocks << 10) | 0x8000U);
	} else {
		nblocks = (uint16_t)(count >> 1);
		if ((count & 0x1U) != 0U) {
			nblocks++;
		}
		*cell = (uint16_t)(nblocks << 10);
	}
}

static inline uint16_t usb_dc_n32_get_ep_tx_cnt(USB_Module *usb, uint8_t epnum)
{
	return (uint16_t)(*usb_dc_n32_pma_cell(usb, epnum, USB_DC_N32_EP_TX_CNT_CELL) & 0x3FFU);
}

/* Hardware fills COUNT_RX<9:0> with the received byte count. */
static inline uint16_t usb_dc_n32_get_ep_rx_cnt(USB_Module *usb, uint8_t epnum)
{
	return (uint16_t)(*usb_dc_n32_pma_cell(usb, epnum, USB_DC_N32_EP_RX_CNT_CELL) & 0x3FFU);
}

/*
 * ------------------------------------------------------------------------- *
 * Command-level (LL) layer                                                   *
 * ------------------------------------------------------------------------- *
 * Command-style endpoint helpers and ISR dispatch, rebuilt on the register
 * accessors above.  Single-buffer endpoints only: the N32 USB FS macrocell
 * also supports double-buffered endpoints, but double buffering mainly
 * benefits isochronous traffic and is left out of this first implementation,
 * which targets the single-buffer environment validated on hardware.
 */
/* Maximum number of bi-directional endpoints of the N32 USB FS IP */
#define USB_DC_N32_NUM_BIDIR_ENDPOINTS 8U

/* Byte address of a PMA data buffer given its address in halfword units
 * (mirror of BaseAddr + 0x400 + wPMABufAddr * PMA_ACCESS).
 */
static inline uint32_t usb_dc_n32_ll_pma_data(USB_Module *usb, uint16_t pma_address)
{
	(void)usb;

	return (uint32_t)(USB_DC_N32_PMA_BASE + (uint32_t)pma_address * USB_DC_N32_PMA_ACCESS);
}

/* Endpoint types (values match the EPnR EPTYPE field encoding) */
#define USB_DC_N32_EP_TYPE_CTRL 0U
#define USB_DC_N32_EP_TYPE_ISOC 1U
#define USB_DC_N32_EP_TYPE_BULK 2U
#define USB_DC_N32_EP_TYPE_INTR 3U

/* Endpoint state kept by the lower layer for ongoing transfers. */
struct usb_dc_n32_ll_ep {
	uint8_t num;         /* EP number */
	uint8_t is_in;       /* 1: IN endpoint, 0: OUT endpoint */
	uint8_t type;        /* USB_DC_N32_EP_TYPE_* */
	uint16_t maxpacket;  /* endpoint max packet size in bytes */
	uint16_t pmaadress;  /* PMA buffer address in halfword units */
	uint8_t *xfer_buff;  /* user data buffer */
	uint16_t xfer_len;   /* remaining bytes to transfer */
	uint16_t xfer_count; /* bytes transferred so far */
};

/* Device handle: per-instance state used by the ISR dispatch. */
struct usb_dc_n32_ll_pcd {
	USB_Module *instance;

	struct usb_dc_n32_ll_ep in_ep[USB_DC_N32_NUM_BIDIR_ENDPOINTS];
	struct usb_dc_n32_ll_ep out_ep[USB_DC_N32_NUM_BIDIR_ENDPOINTS];

	uint16_t usb_address; /* deferred device address (F1 semantics) */
	uint8_t setup[8];     /* last SETUP packet received */

	void (*reset_cb)(struct usb_dc_n32_ll_pcd *hpcd);
	void (*suspend_cb)(struct usb_dc_n32_ll_pcd *hpcd);
	void (*resume_cb)(struct usb_dc_n32_ll_pcd *hpcd);
	void (*setup_stage_cb)(struct usb_dc_n32_ll_pcd *hpcd);
	void (*data_out_stage_cb)(struct usb_dc_n32_ll_pcd *hpcd, uint8_t epnum);
	void (*data_in_stage_cb)(struct usb_dc_n32_ll_pcd *hpcd, uint8_t epnum);
	void (*sof_cb)(struct usb_dc_n32_ll_pcd *hpcd);
};

static void usb_dc_n32_ll_dev_init(struct usb_dc_n32_ll_pcd *hpcd);
static void usb_dc_n32_ll_disable_global_int(struct usb_dc_n32_ll_pcd *hpcd);
static void usb_dc_n32_ll_enable_global_int(struct usb_dc_n32_ll_pcd *hpcd);
static void usb_dc_n32_ll_stop_device(struct usb_dc_n32_ll_pcd *hpcd);
static void usb_dc_n32_ll_set_dev_address(struct usb_dc_n32_ll_pcd *hpcd, uint8_t address);
static void usb_dc_n32_ll_activate_ep(struct usb_dc_n32_ll_pcd *hpcd, struct usb_dc_n32_ll_ep *ep);
static void usb_dc_n32_ll_deactivate_ep(struct usb_dc_n32_ll_pcd *hpcd,
					struct usb_dc_n32_ll_ep *ep);
static int usb_dc_n32_ll_ep_start_xfer(struct usb_dc_n32_ll_pcd *hpcd, struct usb_dc_n32_ll_ep *ep);
static void usb_dc_n32_ll_ep_set_stall(struct usb_dc_n32_ll_pcd *hpcd, struct usb_dc_n32_ll_ep *ep);
static void usb_dc_n32_ll_ep_clear_stall(struct usb_dc_n32_ll_pcd *hpcd,
					 struct usb_dc_n32_ll_ep *ep);
static void usb_dc_n32_ll_activate_remote_wakeup(struct usb_dc_n32_ll_pcd *hpcd);
static void usb_dc_n32_ll_deactivate_remote_wakeup(struct usb_dc_n32_ll_pcd *hpcd);
static void usb_dc_n32_ll_write_pma(struct usb_dc_n32_ll_pcd *hpcd, const uint8_t *buf,
				    uint16_t pma_address, uint16_t nbytes);
static void usb_dc_n32_ll_read_pma(struct usb_dc_n32_ll_pcd *hpcd, uint8_t *buf,
				   uint16_t pma_address, uint16_t nbytes);
static void usb_dc_n32_ll_irq_handler(struct usb_dc_n32_ll_pcd *hpcd);

static void usb_dc_n32_ll_dev_init(struct usb_dc_n32_ll_pcd *hpcd)
{
	/* USB_DevInit */
	USB_Module *usb = hpcd->instance;

	usb_dc_n32_set_ctrl(usb, USB_DC_N32_CTRL_FRST); /* CNTR_FRES = 1 */
	usb_dc_n32_set_ctrl(usb, 0U);                   /* CNTR_FRES = 0 */
	usb_dc_n32_set_istr(usb, 0U);                   /* clear pending ints */
	usb_dc_n32_set_btable(usb, USB_DC_N32_BTABLE_ADDRESS);
}

static void usb_dc_n32_ll_disable_global_int(struct usb_dc_n32_ll_pcd *hpcd)
{
	/* USB_DisableGlobalInt */
	USB_Module *usb = hpcd->instance;
	uint16_t mask = USB_DC_N32_CTRL_CTRSM | USB_DC_N32_CTRL_WKUPM | USB_DC_N32_CTRL_SUSPDM |
			USB_DC_N32_CTRL_ERRORM | USB_DC_N32_CTRL_SOFM | USB_DC_N32_CTRL_ESOFM |
			USB_DC_N32_CTRL_RSTM;

	usb_dc_n32_set_ctrl(usb, (uint16_t)(usb_dc_n32_get_ctrl(usb) & ~mask));
}

static void usb_dc_n32_ll_enable_global_int(struct usb_dc_n32_ll_pcd *hpcd)
{
	/* USB_EnableGlobalInt */
	USB_Module *usb = hpcd->instance;
	uint16_t mask = USB_DC_N32_CTRL_CTRSM | USB_DC_N32_CTRL_WKUPM | USB_DC_N32_CTRL_SUSPDM |
			USB_DC_N32_CTRL_ERRORM | USB_DC_N32_CTRL_SOFM | USB_DC_N32_CTRL_ESOFM |
			USB_DC_N32_CTRL_RSTM;

	usb_dc_n32_set_istr(usb, 0U); /* clear pending interrupts */
	usb_dc_n32_set_ctrl(usb, mask);
}

static void usb_dc_n32_ll_stop_device(struct usb_dc_n32_ll_pcd *hpcd)
{
	/* USB_StopDevice */
	USB_Module *usb = hpcd->instance;

	usb_dc_n32_set_ctrl(usb, USB_DC_N32_CTRL_FRST);
	usb_dc_n32_set_istr(usb, 0U);
	usb_dc_n32_set_ctrl(usb, (uint16_t)(USB_DC_N32_CTRL_FRST | USB_DC_N32_CTRL_PD));
}

static void usb_dc_n32_ll_set_dev_address(struct usb_dc_n32_ll_pcd *hpcd, uint8_t address)
{
	/* HAL_PCD_SetAddress / USB_SetDevAddress (F1 USB_SetDevAddress only
	 * handles address 0; the non-zero address is applied by the ISR once
	 * the EP0 IN status stage has completed).
	 */
	USB_Module *usb = hpcd->instance;

	hpcd->usb_address = address;

	if (address == 0U) {
		usb_dc_n32_set_daddr(usb, USB_DC_N32_ADDR_EFUC); /* DADDR_EF */
	}
}

static void usb_dc_n32_ll_activate_ep(struct usb_dc_n32_ll_pcd *hpcd, struct usb_dc_n32_ll_ep *ep)
{
	/* USB_ActivateEndpoint (single buffer only) */
	USB_Module *usb = hpcd->instance;
	uint16_t wreg = usb_dc_n32_get_ep(usb, ep->num) & USB_DC_N32_EP_T_MASK;
	uint16_t type;

	switch (ep->type) {
	case USB_DC_N32_EP_TYPE_CTRL:
		type = USB_DC_N32_EP_CONTROL;
		break;
	case USB_DC_N32_EP_TYPE_ISOC:
		type = USB_DC_N32_EP_ISOCHRONOUS;
		break;
	case USB_DC_N32_EP_TYPE_BULK:
		type = USB_DC_N32_EP_BULK;
		break;
	case USB_DC_N32_EP_TYPE_INTR:
		type = USB_DC_N32_EP_INTERRUPT;
		break;
	default:
		return;
	}

	usb_dc_n32_set_ep(usb, ep->num,
			  (uint16_t)(wreg | type | USB_DC_N32_EP_CTRS_RX | USB_DC_N32_EP_CTRS_TX));
	usb_dc_n32_set_ep_address(usb, ep->num, ep->num);

	if (ep->is_in != 0U) {
		usb_dc_n32_set_ep_tx_addr(usb, ep->num, ep->pmaadress);
		usb_dc_n32_clear_dtog_tx(usb, ep->num);

		if (ep->type != USB_DC_N32_EP_TYPE_ISOC) {
			usb_dc_n32_set_ep_tx_status(usb, ep->num, USB_DC_N32_EP_TX_NAK);
		} else {
			usb_dc_n32_set_ep_tx_status(usb, ep->num, USB_DC_N32_EP_TX_DIS);
		}
	} else {
		usb_dc_n32_set_ep_rx_addr(usb, ep->num, ep->pmaadress);
		usb_dc_n32_set_ep_rx_cnt(usb, ep->num, ep->maxpacket);
		usb_dc_n32_clear_dtog_rx(usb, ep->num);

		if (ep->num == 0U) {
			usb_dc_n32_set_ep_rx_status(usb, ep->num, USB_DC_N32_EP_RX_VALID);
		} else {
			usb_dc_n32_set_ep_rx_status(usb, ep->num, USB_DC_N32_EP_RX_NAK);
		}
	}
}

static void usb_dc_n32_ll_deactivate_ep(struct usb_dc_n32_ll_pcd *hpcd, struct usb_dc_n32_ll_ep *ep)
{
	/* USB_DeactivateEndpoint (single buffer only) */
	USB_Module *usb = hpcd->instance;

	if (ep->is_in != 0U) {
		usb_dc_n32_clear_dtog_tx(usb, ep->num);
		usb_dc_n32_set_ep_tx_status(usb, ep->num, USB_DC_N32_EP_TX_DIS);
	} else {
		usb_dc_n32_clear_dtog_rx(usb, ep->num);
		usb_dc_n32_set_ep_rx_status(usb, ep->num, USB_DC_N32_EP_RX_DIS);
	}
}

static int usb_dc_n32_ll_ep_start_xfer(struct usb_dc_n32_ll_pcd *hpcd, struct usb_dc_n32_ll_ep *ep)
{
	/* USB_EPStartXfer (single buffer only) */
	USB_Module *usb = hpcd->instance;
	uint16_t len;

	if (ep->is_in != 0U) {
		/* IN endpoint */
		len = MIN(ep->xfer_len, ep->maxpacket);

		usb_dc_n32_ll_write_pma(hpcd, ep->xfer_buff, ep->pmaadress, len);
		usb_dc_n32_set_ep_tx_cnt(usb, ep->num, len);
		usb_dc_n32_set_ep_tx_status(usb, ep->num, USB_DC_N32_EP_TX_VALID);
	} else {
		/* OUT endpoint */
		if (ep->xfer_len > ep->maxpacket) {
			len = ep->maxpacket;
			ep->xfer_len = (uint16_t)(ep->xfer_len - len);
		} else {
			len = ep->xfer_len;
			ep->xfer_len = 0U;
		}

		usb_dc_n32_set_ep_rx_cnt(usb, ep->num, len);
		usb_dc_n32_set_ep_rx_status(usb, ep->num, USB_DC_N32_EP_RX_VALID);
	}

	return 0;
}

static void usb_dc_n32_ll_ep_set_stall(struct usb_dc_n32_ll_pcd *hpcd, struct usb_dc_n32_ll_ep *ep)
{
	/* USB_EPSetStall */
	USB_Module *usb = hpcd->instance;

	if (ep->is_in != 0U) {
		usb_dc_n32_set_ep_tx_status(usb, ep->num, USB_DC_N32_EP_TX_STALL);
	} else {
		usb_dc_n32_set_ep_rx_status(usb, ep->num, USB_DC_N32_EP_RX_STALL);
	}
}

static void usb_dc_n32_ll_ep_clear_stall(struct usb_dc_n32_ll_pcd *hpcd,
					 struct usb_dc_n32_ll_ep *ep)
{
	/* USB_EPClearStall (single buffer only) */
	USB_Module *usb = hpcd->instance;

	if (ep->is_in != 0U) {
		usb_dc_n32_clear_dtog_tx(usb, ep->num);

		if (ep->type != USB_DC_N32_EP_TYPE_ISOC) {
			usb_dc_n32_set_ep_tx_status(usb, ep->num, USB_DC_N32_EP_TX_NAK);
		}
	} else {
		usb_dc_n32_clear_dtog_rx(usb, ep->num);
		usb_dc_n32_set_ep_rx_status(usb, ep->num, USB_DC_N32_EP_RX_VALID);
	}
}

static void usb_dc_n32_ll_activate_remote_wakeup(struct usb_dc_n32_ll_pcd *hpcd)
{
	/* USB_ActivateRemoteWakeup */
	USB_Module *usb = hpcd->instance;

	usb_dc_n32_set_ctrl(usb, (uint16_t)(usb_dc_n32_get_ctrl(usb) | USB_DC_N32_CTRL_RESUM));
}

static void usb_dc_n32_ll_deactivate_remote_wakeup(struct usb_dc_n32_ll_pcd *hpcd)
{
	/* USB_DeActivateRemoteWakeup */
	USB_Module *usb = hpcd->instance;

	usb_dc_n32_set_ctrl(usb, (uint16_t)(usb_dc_n32_get_ctrl(usb) & ~USB_DC_N32_CTRL_RESUM));
}

static void usb_dc_n32_ll_write_pma(struct usb_dc_n32_ll_pcd *hpcd, const uint8_t *buf,
				    uint16_t pma_address, uint16_t nbytes)
{
	/* USB_WritePMA: byte offset into the PMA = pma_address * PMA_ACCESS */
	USB_Module *usb = hpcd->instance;
	volatile uint16_t *pma = (volatile uint16_t *)(usb_dc_n32_ll_pma_data(usb, pma_address));
	uint32_t n = ((uint32_t)nbytes + 1U) >> 1;

	while (n-- != 0U) {
		uint16_t wval;

		wval = (uint16_t)(buf[0] | ((uint16_t)buf[1] << 8));
		*pma = wval;
		pma += USB_DC_N32_PMA_ACCESS;
		buf += 2;
	}
}

static void usb_dc_n32_ll_read_pma(struct usb_dc_n32_ll_pcd *hpcd, uint8_t *buf,
				   uint16_t pma_address, uint16_t nbytes)
{
	/* USB_ReadPMA */
	USB_Module *usb = hpcd->instance;
	volatile uint16_t *pma = (volatile uint16_t *)(usb_dc_n32_ll_pma_data(usb, pma_address));
	uint32_t n = (uint32_t)nbytes >> 1;

	while (n-- != 0U) {
		uint16_t rval = *pma;

		*buf++ = (uint8_t)(rval & 0xFFU);
		*buf++ = (uint8_t)((rval >> 8) & 0xFFU);
		pma += USB_DC_N32_PMA_ACCESS;
	}

	if ((nbytes & 1U) != 0U) {
		*buf = (uint8_t)(*pma & 0xFFU);
	}
}

/* ------------------------------------------------------------------ */
/* ISR dispatch                                                        */
/* ------------------------------------------------------------------ */

/* Endpoint correct-transfer dispatch: mirrors PCD_EP_ISR_Handler
 * (single-buffer only).
 */
static void usb_dc_n32_ll_ep_isr(struct usb_dc_n32_ll_pcd *hpcd)
{
	USB_Module *usb = hpcd->instance;
	struct usb_dc_n32_ll_ep *ep;
	uint16_t wistr;
	uint16_t wepval;
	uint16_t count;
	uint16_t txsize;
	uint8_t epindex;

	/* stay in loop while pending interrupts */
	while ((usb_dc_n32_get_istr(usb) & USB_DC_N32_STS_CTRS) != 0U) {
		wistr = usb_dc_n32_get_istr(usb);

		/* extract highest priority endpoint number */
		epindex = (uint8_t)(wistr & USB_DC_N32_STS_EP_ID);

		if (epindex == 0U) {
			/* Decode and service control endpoint interrupt */

			/* DIR bit = origin of the interrupt */
			if ((wistr & USB_DC_N32_STS_DIR) == 0U) {
				/* DIR = 0 => IN interrupt (EP_CTR_TX) */
				usb_dc_n32_clear_ep_ctr_tx(usb, 0U);

				ep = &hpcd->in_ep[0];
				ep->xfer_count = usb_dc_n32_get_ep_tx_cnt(usb, ep->num);
				ep->xfer_buff += ep->xfer_count;

				/* TX COMPLETE */
				hpcd->data_in_stage_cb(hpcd, 0U);

				if ((hpcd->usb_address > 0U) && (ep->xfer_len == 0U)) {
					usb_dc_n32_set_daddr(usb, (uint16_t)(hpcd->usb_address |
									     USB_DC_N32_ADDR_EFUC));
					hpcd->usb_address = 0U;
				}
			} else {
				/* DIR = 1 => SETUP or OUT interrupt */
				ep = &hpcd->out_ep[0];
				wepval = usb_dc_n32_get_ep(usb, 0U);

				if ((wepval & USB_DC_N32_EP_SETUP) != 0U) {
					/* Get SETUP packet */
					ep->xfer_count = usb_dc_n32_get_ep_rx_cnt(usb, ep->num);
					usb_dc_n32_ll_read_pma(hpcd, hpcd->setup, ep->pmaadress,
							       ep->xfer_count);

					/* SETUP bit is kept frozen while
					 * CTR_RX = 1
					 */
					usb_dc_n32_clear_ep_ctr_rx(usb, 0U);

					hpcd->setup_stage_cb(hpcd);
				} else if ((wepval & USB_DC_N32_EP_CTRS_RX) != 0U) {
					usb_dc_n32_clear_ep_ctr_rx(usb, 0U);

					/* Get control OUT data packet */
					ep->xfer_count = usb_dc_n32_get_ep_rx_cnt(usb, ep->num);

					if ((ep->xfer_count != 0U) && (ep->xfer_buff != 0U)) {
						usb_dc_n32_ll_read_pma(hpcd, ep->xfer_buff,
								       ep->pmaadress,
								       ep->xfer_count);
						ep->xfer_buff += ep->xfer_count;

						hpcd->data_out_stage_cb(hpcd, 0U);
					}

					wepval = usb_dc_n32_get_ep(usb, 0U);

					if (((wepval & USB_DC_N32_EP_SETUP) == 0U) &&
					    ((wepval & USB_DC_N32_EPRX_STS) !=
					     USB_DC_N32_EP_RX_VALID)) {
						usb_dc_n32_set_ep_rx_cnt(usb, 0U, ep->maxpacket);
						usb_dc_n32_set_ep_rx_status(usb, 0U,
									    USB_DC_N32_EP_RX_VALID);
					}
				}
			}
		} else {
			/* Decode and service non-control endpoints */
			wepval = usb_dc_n32_get_ep(usb, epindex);

			if ((wepval & USB_DC_N32_EP_CTRS_RX) != 0U) {
				usb_dc_n32_clear_ep_ctr_rx(usb, epindex);
				ep = &hpcd->out_ep[epindex];

				count = usb_dc_n32_get_ep_rx_cnt(usb, ep->num);

				if (count != 0U) {
					usb_dc_n32_ll_read_pma(hpcd, ep->xfer_buff, ep->pmaadress,
							       count);
				}

				/* multi-packet on non-control OUT EP */
				ep->xfer_count = (uint16_t)(ep->xfer_count + count);
				ep->xfer_buff += count;

				if ((ep->xfer_len == 0U) || (count < ep->maxpacket)) {
					/* RX COMPLETE */
					hpcd->data_out_stage_cb(hpcd, ep->num);
				} else {
					usb_dc_n32_ll_ep_start_xfer(hpcd, ep);
				}
			}

			if ((wepval & USB_DC_N32_EP_CTRS_TX) != 0U) {
				usb_dc_n32_clear_ep_ctr_tx(usb, epindex);
				ep = &hpcd->in_ep[epindex];

				if (ep->type == USB_DC_N32_EP_TYPE_ISOC) {
					ep->xfer_len = 0U;

					/* TX COMPLETE */
					hpcd->data_in_stage_cb(hpcd, ep->num);
				} else {
					/* multi-packet on non-control IN EP */
					txsize = usb_dc_n32_get_ep_tx_cnt(usb, ep->num);

					if (ep->xfer_len > txsize) {
						ep->xfer_len = (uint16_t)(ep->xfer_len - txsize);
					} else {
						ep->xfer_len = 0U;
					}

					/* Zero Length Packet ? */
					if (ep->xfer_len == 0U) {
						/* TX COMPLETE */
						hpcd->data_in_stage_cb(hpcd, ep->num);
					} else {
						/* transfer not yet done */
						ep->xfer_buff += txsize;
						ep->xfer_count =
							(uint16_t)(ep->xfer_count + txsize);
						usb_dc_n32_ll_ep_start_xfer(hpcd, ep);
					}
				}
			}
		}
	}
}

static void usb_dc_n32_ll_irq_handler(struct usb_dc_n32_ll_pcd *hpcd)
{
	/* HAL_PCD_IRQHandler (F1 branch) */
	USB_Module *usb = hpcd->instance;
	uint32_t wistr = usb_dc_n32_get_istr(usb);
	uint16_t store_ep[USB_DC_N32_NUM_BIDIR_ENDPOINTS];
	uint8_t i;

	if ((wistr & USB_DC_N32_STS_CTRS) != 0U) {
		usb_dc_n32_ll_ep_isr(hpcd);
		return;
	}

	if ((wistr & USB_DC_N32_STS_RST) != 0U) {
		usb_dc_n32_set_istr(usb, (uint16_t)~USB_DC_N32_STS_RST);

		hpcd->reset_cb(hpcd);
		usb_dc_n32_ll_set_dev_address(hpcd, 0U);

		return;
	}

	if ((wistr & USB_DC_N32_STS_DOVR) != 0U) {
		usb_dc_n32_set_istr(usb, (uint16_t)~USB_DC_N32_STS_DOVR);
		return;
	}

	if ((wistr & USB_DC_N32_STS_ERROR) != 0U) {
		usb_dc_n32_set_istr(usb, (uint16_t)~USB_DC_N32_STS_ERROR);
		return;
	}

	if ((wistr & USB_DC_N32_STS_WKUP) != 0U) {
		usb_dc_n32_set_ctrl(
			usb, (uint16_t)(usb_dc_n32_get_ctrl(usb) & ~USB_DC_N32_CTRL_LP_MODE));
		usb_dc_n32_set_ctrl(usb,
				    (uint16_t)(usb_dc_n32_get_ctrl(usb) & ~USB_DC_N32_CTRL_FSUSPD));

		hpcd->resume_cb(hpcd);

		usb_dc_n32_set_istr(usb, (uint16_t)~USB_DC_N32_STS_WKUP);

		return;
	}

	if ((wistr & USB_DC_N32_STS_SUSPD) != 0U) {
		/* WA: clear wakeup flag if raised with suspend signal. */

		/* store endpoint registers */
		for (i = 0U; i < USB_DC_N32_NUM_BIDIR_ENDPOINTS; i++) {
			store_ep[i] = usb_dc_n32_get_ep(usb, i);
		}

		/* force reset */
		usb_dc_n32_set_ctrl(usb,
				    (uint16_t)(usb_dc_n32_get_ctrl(usb) | USB_DC_N32_CTRL_FRST));
		usb_dc_n32_set_ctrl(usb,
				    (uint16_t)(usb_dc_n32_get_ctrl(usb) & ~USB_DC_N32_CTRL_FRST));

		/* wait for reset flag in ISTR */
		while ((usb_dc_n32_get_istr(usb) & USB_DC_N32_STS_RST) == 0U) {
		}

		/* clear reset flag */
		usb_dc_n32_set_istr(usb, (uint16_t)~USB_DC_N32_STS_RST);

		/* restore registers */
		for (i = 0U; i < USB_DC_N32_NUM_BIDIR_ENDPOINTS; i++) {
			usb_dc_n32_set_ep(usb, i, store_ep[i]);
		}

		/* force low power mode in the macrocell */
		usb_dc_n32_set_ctrl(usb,
				    (uint16_t)(usb_dc_n32_get_ctrl(usb) | USB_DC_N32_CTRL_FSUSPD));

		/* clear of the ISTR bit must be done after setting FSUSPD */
		usb_dc_n32_set_istr(usb, (uint16_t)~USB_DC_N32_STS_SUSPD);

		usb_dc_n32_set_ctrl(usb,
				    (uint16_t)(usb_dc_n32_get_ctrl(usb) | USB_DC_N32_CTRL_LP_MODE));

		hpcd->suspend_cb(hpcd);

		return;
	}

	if ((wistr & USB_DC_N32_STS_SOF) != 0U) {
		usb_dc_n32_set_istr(usb, (uint16_t)~USB_DC_N32_STS_SOF);

		hpcd->sof_cb(hpcd);

		return;
	}

	if ((wistr & USB_DC_N32_STS_ESOF) != 0U) {
		usb_dc_n32_set_istr(usb, (uint16_t)~USB_DC_N32_STS_ESOF);
		return;
	}
}
#define EP0_MPS 64U
#define EP_MPS  64U

/* Size of the EPnR buffer table stored at the beginning of the PMA:
 * 4 bytes per endpoint.
 */
#define USB_BTABLE_SIZE (8 * USB_NUM_BIDIR_ENDPOINTS)

/* Priority of ISR lower half thread */
#define USB_ISR_LOWER_HALF_THREAD_PRIO K_PRIO_COOP(2)

/* Size of ISR lower half thread stack */
#define USB_ISR_LOWER_HALF_THREAD_STACK_SIZE 1024

/* Number of messages in ISR -> lower half thread msgq */
#define USB_ISR_MSGQ_SIZE 16

/* Special "target" indicating message targets USB stack */
#define USB_MSG_TARGET_STACK 0xFFu

/* Size of a USB SETUP packet */
#define SETUP_SIZE 8

/* Helper macros to make it easier to work with endpoint numbers */
#define EP0_IDX 0
#define EP0_IN  (EP0_IDX | USB_EP_DIR_IN)
#define EP0_OUT (EP0_IDX | USB_EP_DIR_OUT)

#define USB_IRQ                 DT_INST_IRQN(0)
#define USB_IRQ_PRI             DT_INST_IRQ(0, priority)
#define USB_BASE_ADDRESS        DT_INST_REG_ADDR(0)
#define USB_NUM_BIDIR_ENDPOINTS DT_INST_PROP(0, num_bidir_endpoints)
#define USB_RAM_SIZE            DT_INST_PROP(0, ram_size)

/* Endpoint state */
struct usb_dc_n32_ep_state {
	uint16_t ep_mps;         /**< Endpoint max packet size */
	uint16_t ep_pma_buf_len; /**< Previously allocated buffer size */
	uint8_t ep_type;         /**< Endpoint type (LL enum) */
	uint8_t ep_stalled;      /**< Endpoint stall flag */
	usb_dc_ep_callback cb;   /**< Endpoint callback function */
	uint32_t read_count;     /**< Number of bytes in read buffer  */
	uint32_t read_offset;    /**< Current offset in read buffer */
	struct k_sem write_sem;  /**< Write boolean semaphore */
};

/* ISR -> lower half message */
struct usb_dc_n32_msg {
	union {
		enum usb_dc_status_code stack_cb_status;
		enum usb_dc_ep_cb_status_code ep_cb_status;
	};
	/* endpoint address or USB_MSG_TARGET_STACK */
	uint8_t target;
};

/* Driver state */
struct usb_dc_n32_state {
	struct usb_dc_n32_ll_pcd pcd;     /* LL/PCD handle */
	usb_dc_status_callback status_cb; /* Status callback */
	struct usb_dc_n32_ep_state out_ep_state[USB_NUM_BIDIR_ENDPOINTS];
	struct usb_dc_n32_ep_state in_ep_state[USB_NUM_BIDIR_ENDPOINTS];
	struct k_thread isr_lower_half_thread;
	struct k_msgq isr_msgq;
	uint8_t ep_buf[USB_NUM_BIDIR_ENDPOINTS][EP_MPS];
	char msgq_buf[USB_ISR_MSGQ_SIZE * sizeof(struct usb_dc_n32_msg)];
	uint32_t pma_offset;
};

K_THREAD_STACK_DEFINE(usb_dc_n32_thr_stk, USB_ISR_LOWER_HALF_THREAD_STACK_SIZE);

static struct usb_dc_n32_state usb_dc_n32_state;

/* Internal functions */

/* these helpers are defined as macros such that LOG_ERR() call
 * is performed in caller's scope and has proper function name
 */
#define SEND_MSG_TO_LOWERHALF(_status_field, _status, _target)                                     \
	do {                                                                                       \
		struct usb_dc_n32_msg msg = {                                                      \
			._status_field = _status,                                                  \
			.target = _target,                                                         \
		};                                                                                 \
                                                                                                   \
		int _errcode = k_msgq_put(&usb_dc_n32_state.isr_msgq, &msg, K_NO_WAIT);            \
		if (_errcode != 0) {                                                               \
			LOG_ERR("k_msgq_put() failed: %d", _errcode);                              \
			__ASSERT_NO_MSG(0);                                                        \
		}                                                                                  \
	} while (0)

#define usb_dc_n32_send_stack_msg(_status)                                                         \
	SEND_MSG_TO_LOWERHALF(stack_cb_status, _status, USB_MSG_TARGET_STACK)

#define usb_dc_n32_send_ep_msg(_ep, _status) SEND_MSG_TO_LOWERHALF(ep_cb_status, _status, _ep)

int usb_dc_ep_start_read(uint8_t ep, uint8_t *data, uint32_t max_data_len);

static void usb_dc_n32_isr_lower_half(void *a1, void *a2, void *a3)
{
	ARG_UNUSED(a1);
	ARG_UNUSED(a2);
	ARG_UNUSED(a3);

	struct usb_dc_n32_state *state = &usb_dc_n32_state;
	struct usb_dc_n32_msg msg;
	int res;

	while (true) {
		res = k_msgq_get(&state->isr_msgq, &msg, K_FOREVER);
		__ASSERT_NO_MSG(res == 0);

		if (msg.target == USB_MSG_TARGET_STACK) {
			if (state->status_cb != NULL) {
				state->status_cb(msg.stack_cb_status, NULL);
			}

			continue;
		}

		/* Endpoint target */
		struct usb_dc_n32_ep_state *ep_state;
		const uint8_t ep_idx = USB_EP_GET_IDX(msg.target);

		if (USB_EP_DIR_IS_IN(msg.target)) {
			ep_state = &usb_dc_n32_state.in_ep_state[ep_idx];
		} else { /* target is OUT ep */
			ep_state = &usb_dc_n32_state.out_ep_state[ep_idx];
		}

		if (ep_state->cb == NULL) {
			continue;
		}

		ep_state->cb(msg.target, msg.ep_cb_status);

		/* Special handling for SETUP event */
		if (msg.ep_cb_status == USB_DC_EP_SETUP) {
			struct usb_setup_packet *setup = (void *)usb_dc_n32_state.pcd.setup;

			if (!(setup->wLength == 0U) && usb_reqtype_is_to_device(setup)) {
				usb_dc_ep_start_read(EP0_OUT, usb_dc_n32_state.ep_buf[EP0_IDX],
						     setup->wLength);
			}
		}
	}
}

static void usb_dc_n32_phystate_pullup(bool on)
{
	volatile uint32_t *dp_ctrl = (volatile uint32_t *)USB_DC_N32_DP_CTRL;

	if (on) {
		*dp_ctrl |= USB_DC_N32_DP_CTRL_PUP;
	} else {
		*dp_ctrl &= ~USB_DC_N32_DP_CTRL_PUP;
	}
}

static int usb_dc_n32_clock_enable(void)
{
	const struct device *const clk = DEVICE_DT_GET(DT_NODELABEL(rcc));
	static const uint32_t clk_cell = DT_INST_CLOCKS_CELL(0, bits);
	uint32_t usbpres;

	if (!device_is_ready(clk)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* The USB FS macrocell must run from a 48 MHz clock obtained by
	 * dividing the PLL output through RCC_CFG.USBPRES.  Derive the
	 * prescaler from the current system clock.
	 */
	switch (SystemCoreClock) {
	case MHZ(48):
		usbpres = RCC_CFG_USBPRES_PLLDIV1;
		break;
	case MHZ(72):
		usbpres = RCC_CFG_USBPRES_PLLDIV1_5;
		break;
	case MHZ(96):
		usbpres = RCC_CFG_USBPRES_PLLDIV2;
		break;
	case MHZ(144):
		usbpres = RCC_CFG_USBPRES_PLLDIV3;
		break;
	default:
		LOG_ERR("Unsupported SYSCLK %u Hz for USB 48 MHz clock", SystemCoreClock);
		return -EINVAL;
	}

	RCC->CFG = (RCC->CFG & ~RCC_CFG_USBPRES) | usbpres;

	if (clock_control_on(clk, (clock_control_subsys_t)&clk_cell) != 0) {
		LOG_ERR("Unable to enable USB clock");
		return -EIO;
	}

	return 0;
}

static int usb_dc_n32_clock_disable(void)
{
	const struct device *const clk = DEVICE_DT_GET(DT_NODELABEL(rcc));
	static const uint32_t clk_cell = DT_INST_CLOCKS_CELL(0, bits);

	if (clock_control_off(clk, (clock_control_subsys_t)&clk_cell) != 0) {
		LOG_ERR("Unable to disable USB clock");
		return -EIO;
	}

	return 0;
}

static struct usb_dc_n32_ep_state *usb_dc_n32_get_ep_state(uint8_t ep)
{
	struct usb_dc_n32_ep_state *ep_state_base;

	if (USB_EP_GET_IDX(ep) >= USB_NUM_BIDIR_ENDPOINTS) {
		return NULL;
	}

	if (USB_EP_DIR_IS_OUT(ep)) {
		ep_state_base = usb_dc_n32_state.out_ep_state;
	} else {
		ep_state_base = usb_dc_n32_state.in_ep_state;
	}

	return ep_state_base + USB_EP_GET_IDX(ep);
}

static void usb_dc_n32_isr(const void *arg)
{
	usb_dc_n32_ll_irq_handler(&usb_dc_n32_state.pcd);
}

static void usb_dc_n32_ll_ep_open(uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type)
{
	/* HAL_PCD_EP_Open (F1 branch), single-buffer only */
	uint8_t ep_idx = USB_EP_GET_IDX(ep_addr);
	bool is_in = USB_EP_DIR_IS_IN(ep_addr);
	struct usb_dc_n32_ll_ep *ep =
		is_in ? &usb_dc_n32_state.pcd.in_ep[ep_idx] : &usb_dc_n32_state.pcd.out_ep[ep_idx];

	ep->num = ep_idx;
	ep->is_in = is_in;
	ep->maxpacket = ep_mps;
	ep->type = ep_type;
	ep->xfer_buff = NULL;
	ep->xfer_len = 0U;
	ep->xfer_count = 0U;

	usb_dc_n32_ll_activate_ep(&usb_dc_n32_state.pcd, ep);
}

static void usb_dc_n32_pma_config(uint8_t ep_addr, uint16_t pmaadress)
{
	/* HAL_PCDEx_PMAConfig (single-buffer only) */
	uint8_t ep_idx = USB_EP_GET_IDX(ep_addr);

	if (USB_EP_DIR_IS_IN(ep_addr)) {
		usb_dc_n32_state.pcd.in_ep[ep_idx].pmaadress = pmaadress;
	} else {
		usb_dc_n32_state.pcd.out_ep[ep_idx].pmaadress = pmaadress;
	}
}

static void usb_dc_n32_reset_cb(struct usb_dc_n32_ll_pcd *hpcd);
static void usb_dc_n32_suspend_cb(struct usb_dc_n32_ll_pcd *hpcd);
static void usb_dc_n32_resume_cb(struct usb_dc_n32_ll_pcd *hpcd);
static void usb_dc_n32_sof_cb(struct usb_dc_n32_ll_pcd *hpcd);
static void usb_dc_n32_setup_stage_cb(struct usb_dc_n32_ll_pcd *hpcd);
static void usb_dc_n32_data_out_stage_cb(struct usb_dc_n32_ll_pcd *hpcd, uint8_t epnum);
static void usb_dc_n32_data_in_stage_cb(struct usb_dc_n32_ll_pcd *hpcd, uint8_t epnum);

static int usb_dc_n32_init(void)
{
	unsigned int i;

	usb_dc_n32_state.pcd.instance = (USB_Module *)USB_BASE_ADDRESS;

	/* PCD callbacks */
	usb_dc_n32_state.pcd.reset_cb = usb_dc_n32_reset_cb;
	usb_dc_n32_state.pcd.suspend_cb = usb_dc_n32_suspend_cb;
	usb_dc_n32_state.pcd.resume_cb = usb_dc_n32_resume_cb;
	usb_dc_n32_state.pcd.setup_stage_cb = usb_dc_n32_setup_stage_cb;
	usb_dc_n32_state.pcd.data_out_stage_cb = usb_dc_n32_data_out_stage_cb;
	usb_dc_n32_state.pcd.data_in_stage_cb = usb_dc_n32_data_in_stage_cb;
	usb_dc_n32_state.pcd.sof_cb = usb_dc_n32_sof_cb;

	usb_dc_n32_ll_disable_global_int(&usb_dc_n32_state.pcd);

	LOG_DBG("USB_DevInit");
	usb_dc_n32_ll_dev_init(&usb_dc_n32_state.pcd);

	/* Initialize IN/OUT endpoint structures (HAL_PCD_Init) */
	for (i = 0U; i < USB_NUM_BIDIR_ENDPOINTS; i++) {
		usb_dc_n32_state.pcd.in_ep[i].is_in = 1U;
		usb_dc_n32_state.pcd.in_ep[i].num = i;
		/* Control until ep is activated */
		usb_dc_n32_state.pcd.in_ep[i].type = USB_DC_N32_EP_TYPE_CTRL;
		usb_dc_n32_state.pcd.in_ep[i].maxpacket = 0U;
		usb_dc_n32_state.pcd.in_ep[i].xfer_buff = NULL;
		usb_dc_n32_state.pcd.in_ep[i].xfer_len = 0U;
		usb_dc_n32_state.pcd.in_ep[i].xfer_count = 0U;

		usb_dc_n32_state.pcd.out_ep[i].is_in = 0U;
		usb_dc_n32_state.pcd.out_ep[i].num = i;
		/* Control until ep is activated */
		usb_dc_n32_state.pcd.out_ep[i].type = USB_DC_N32_EP_TYPE_CTRL;
		usb_dc_n32_state.pcd.out_ep[i].maxpacket = 0U;
		usb_dc_n32_state.pcd.out_ep[i].xfer_buff = NULL;
		usb_dc_n32_state.pcd.out_ep[i].xfer_len = 0U;
		usb_dc_n32_state.pcd.out_ep[i].xfer_count = 0U;
	}

	/* On a soft reset force USB to reset first and switch it off
	 * so the USB connection can get re-initialized.
	 */
	LOG_DBG("USB_StopDevice");
	usb_dc_n32_ll_stop_device(&usb_dc_n32_state.pcd);

	usb_dc_n32_state.out_ep_state[EP0_IDX].ep_mps = EP0_MPS;
	usb_dc_n32_state.out_ep_state[EP0_IDX].ep_type = USB_DC_N32_EP_TYPE_CTRL;
	usb_dc_n32_state.in_ep_state[EP0_IDX].ep_mps = EP0_MPS;
	usb_dc_n32_state.in_ep_state[EP0_IDX].ep_type = USB_DC_N32_EP_TYPE_CTRL;

	/* Buffer table is stored at the start of the PMA, data buffers
	 * start right after it.
	 */
	usb_dc_n32_state.pma_offset = USB_BTABLE_SIZE;

	for (i = 0U; i < USB_NUM_BIDIR_ENDPOINTS; i++) {
		k_sem_init(&usb_dc_n32_state.in_ep_state[i].write_sem, 1, 1);
	}

	k_msgq_init(&usb_dc_n32_state.isr_msgq, usb_dc_n32_state.msgq_buf,
		    sizeof(struct usb_dc_n32_msg), USB_ISR_MSGQ_SIZE);

	k_thread_create(&usb_dc_n32_state.isr_lower_half_thread, usb_dc_n32_thr_stk,
			K_THREAD_STACK_SIZEOF(usb_dc_n32_thr_stk), usb_dc_n32_isr_lower_half, NULL,
			NULL, NULL, USB_ISR_LOWER_HALF_THREAD_PRIO, K_ESSENTIAL, K_NO_WAIT);

	/* Pull up D+ before the interrupts are enabled.  On an idle bus
	 * (host not yet able to see the device) the suspend handling forces
	 * a self-reset, which would otherwise keep the ISR busy re-entering
	 * forever and never let the pull-up below be executed.  Raising D+
	 * first lets the host drive a proper bus reset once it sees the
	 * device, so the first RST interrupt is serviced normally.
	 */
	usb_dc_n32_phystate_pullup(true);

	IRQ_CONNECT(USB_IRQ, USB_IRQ_PRI, usb_dc_n32_isr, 0, 0);
	irq_enable(USB_IRQ);

	/* HAL_PCD_Start() */
	usb_dc_n32_ll_enable_global_int(&usb_dc_n32_state.pcd);

	return 0;
}

/* Zephyr USB device controller API implementation */

int usb_dc_attach(void)
{
	int ret;

	LOG_DBG("");

	ret = usb_dc_n32_clock_enable();
	if (ret) {
		return ret;
	}

	ret = usb_dc_n32_init();
	if (ret) {
		return ret;
	}

	return 0;
}

int usb_dc_ep_set_callback(const uint8_t ep, const usb_dc_ep_callback cb)
{
	struct usb_dc_n32_ep_state *ep_state = usb_dc_n32_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	ep_state->cb = cb;

	return 0;
}

void usb_dc_set_status_callback(const usb_dc_status_callback cb)
{
	LOG_DBG("");

	usb_dc_n32_state.status_cb = cb;
}

int usb_dc_set_address(const uint8_t addr)
{
	LOG_DBG("addr %u (0x%02x)", addr, addr);

	/* Non-zero addresses are stored and applied by the ISR once EP0
	 * completes the status stage (see usb_dc_n32_ll_set_dev_address).
	 */
	usb_dc_n32_ll_set_dev_address(&usb_dc_n32_state.pcd, addr);

	return 0;
}

int usb_dc_ep_start_read(uint8_t ep, uint8_t *data, uint32_t max_data_len)
{
	/* we flush EP0_IN by doing a 0 length receive on it */
	if (!USB_EP_DIR_IS_OUT(ep) && (ep != EP0_IN || max_data_len)) {
		LOG_ERR("invalid ep 0x%02x", ep);
		return -EINVAL;
	}

	if (max_data_len > EP_MPS) {
		max_data_len = EP_MPS;
	}

	/* HAL_PCD_EP_Receive (F1 branch) */
	struct usb_dc_n32_ll_ep *ep_ptr = &usb_dc_n32_state.pcd.out_ep[USB_EP_GET_IDX(ep)];

	ep_ptr->xfer_buff = data;
	ep_ptr->xfer_len = max_data_len;
	ep_ptr->xfer_count = 0U;
	ep_ptr->is_in = 0U;
	ep_ptr->num = USB_EP_GET_IDX(ep);

	usb_dc_n32_ll_ep_start_xfer(&usb_dc_n32_state.pcd, ep_ptr);

	return 0;
}

int usb_dc_ep_get_read_count(uint8_t ep, uint32_t *read_bytes)
{
	if (!USB_EP_DIR_IS_OUT(ep) || !read_bytes) {
		LOG_ERR("invalid ep 0x%02x", ep);
		return -EINVAL;
	}

	*read_bytes = usb_dc_n32_state.pcd.out_ep[USB_EP_GET_IDX(ep)].xfer_count;

	return 0;
}

int usb_dc_ep_check_cap(const struct usb_dc_ep_cfg_data *const cfg)
{
	uint8_t ep_idx = USB_EP_GET_IDX(cfg->ep_addr);

	LOG_DBG("ep %x, mps %d, type %d", cfg->ep_addr, cfg->ep_mps, cfg->ep_type);

	if ((cfg->ep_type == USB_DC_EP_CONTROL) && ep_idx) {
		LOG_ERR("invalid endpoint configuration");
		return -EINVAL;
	}

	if (ep_idx > (USB_NUM_BIDIR_ENDPOINTS - 1)) {
		LOG_ERR("endpoint index/address out of range");
		return -EINVAL;
	}

	if (cfg->ep_type == USB_DC_EP_ISOCHRONOUS) {
		/* Double-buffered ISO endpoints are not implemented (see
		 * the LL layer section above).
		 */
		LOG_ERR("isochronous endpoints not supported");
		return -ENOTSUP;
	}

	return 0;
}

int usb_dc_ep_configure(const struct usb_dc_ep_cfg_data *const ep_cfg)
{
	uint8_t ep = ep_cfg->ep_addr;
	struct usb_dc_n32_ep_state *ep_state = usb_dc_n32_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_DBG("ep 0x%02x, previous ep_mps %u, ep_mps %u, ep_type %u", ep_cfg->ep_addr,
		ep_state->ep_mps, ep_cfg->ep_mps, ep_cfg->ep_type);

	/* Allocate a PMA buffer for the endpoint, growing it if needed,
	 * with a bounds check against the total PMA size.
	 */
	if (ep_cfg->ep_mps > ep_state->ep_pma_buf_len) {
		if (USB_RAM_SIZE <= (usb_dc_n32_state.pma_offset + ep_cfg->ep_mps)) {
			LOG_ERR("PMA size overflow for ep 0x%02x", ep);
			return -EINVAL;
		}

		usb_dc_n32_pma_config(ep, (uint16_t)usb_dc_n32_state.pma_offset);
		ep_state->ep_pma_buf_len = ep_cfg->ep_mps;
		usb_dc_n32_state.pma_offset += ep_cfg->ep_mps;
	}
	ep_state->ep_mps = ep_cfg->ep_mps;

	switch (ep_cfg->ep_type) {
	case USB_DC_EP_CONTROL:
		ep_state->ep_type = USB_DC_N32_EP_TYPE_CTRL;
		break;
	case USB_DC_EP_BULK:
		ep_state->ep_type = USB_DC_N32_EP_TYPE_BULK;
		break;
	case USB_DC_EP_INTERRUPT:
		ep_state->ep_type = USB_DC_N32_EP_TYPE_INTR;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int usb_dc_ep_set_stall(const uint8_t ep)
{
	struct usb_dc_n32_ep_state *ep_state = usb_dc_n32_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	if (USB_EP_GET_IDX(ep) == 0U) {
		/* Stalling EP0 stalls both directions (HAL_PCD_EP_SetStall) */
		usb_dc_n32_ll_ep_set_stall(&usb_dc_n32_state.pcd, &usb_dc_n32_state.pcd.in_ep[0]);
		usb_dc_n32_ll_ep_set_stall(&usb_dc_n32_state.pcd, &usb_dc_n32_state.pcd.out_ep[0]);
	} else {
		struct usb_dc_n32_ll_ep *ep_ptr =
			USB_EP_DIR_IS_IN(ep) ? &usb_dc_n32_state.pcd.in_ep[USB_EP_GET_IDX(ep)]
					     : &usb_dc_n32_state.pcd.out_ep[USB_EP_GET_IDX(ep)];

		usb_dc_n32_ll_ep_set_stall(&usb_dc_n32_state.pcd, ep_ptr);
	}

	ep_state->ep_stalled = 1U;

	return 0;
}

int usb_dc_ep_clear_stall(const uint8_t ep)
{
	struct usb_dc_n32_ep_state *ep_state = usb_dc_n32_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	if (!ep_state->ep_stalled) {
		return 0;
	}

	struct usb_dc_n32_ll_ep *ep_ptr =
		USB_EP_DIR_IS_IN(ep) ? &usb_dc_n32_state.pcd.in_ep[USB_EP_GET_IDX(ep)]
				     : &usb_dc_n32_state.pcd.out_ep[USB_EP_GET_IDX(ep)];

	usb_dc_n32_ll_ep_clear_stall(&usb_dc_n32_state.pcd, ep_ptr);

	ep_state->ep_stalled = 0U;
	ep_state->read_count = 0U;

	return 0;
}

int usb_dc_ep_is_stalled(const uint8_t ep, uint8_t *const stalled)
{
	struct usb_dc_n32_ep_state *ep_state = usb_dc_n32_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state || !stalled) {
		return -EINVAL;
	}

	*stalled = ep_state->ep_stalled;

	return 0;
}

int usb_dc_ep_enable(const uint8_t ep)
{
	struct usb_dc_n32_ep_state *ep_state = usb_dc_n32_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_DBG("EP_Open(0x%02x, %u, %u)", ep, ep_state->ep_mps, ep_state->ep_type);

	usb_dc_n32_ll_ep_open(ep, ep_state->ep_mps, ep_state->ep_type);

	if (USB_EP_DIR_IS_OUT(ep) && ep != EP0_OUT) {
		return usb_dc_ep_start_read(ep, usb_dc_n32_state.ep_buf[USB_EP_GET_IDX(ep)],
					    ep_state->ep_mps);
	}

	return 0;
}

int usb_dc_ep_disable(const uint8_t ep)
{
	struct usb_dc_n32_ep_state *ep_state = usb_dc_n32_get_ep_state(ep);

	LOG_DBG("ep 0x%02x", ep);

	if (!ep_state) {
		return -EINVAL;
	}

	/* HAL_PCD_EP_Close (single-buffer only) */
	struct usb_dc_n32_ll_ep *ep_ptr =
		USB_EP_DIR_IS_IN(ep) ? &usb_dc_n32_state.pcd.in_ep[USB_EP_GET_IDX(ep)]
				     : &usb_dc_n32_state.pcd.out_ep[USB_EP_GET_IDX(ep)];

	usb_dc_n32_ll_deactivate_ep(&usb_dc_n32_state.pcd, ep_ptr);

	return 0;
}

int usb_dc_ep_write(const uint8_t ep, const uint8_t *const data, const uint32_t data_len,
		    uint32_t *const ret_bytes)
{
	struct usb_dc_n32_ep_state *ep_state = usb_dc_n32_get_ep_state(ep);
	uint32_t len = data_len;
	int ret = 0;

	LOG_DBG("ep 0x%02x, len %u", ep, data_len);

	if (!ep_state || !USB_EP_DIR_IS_IN(ep)) {
		LOG_ERR("invalid ep 0x%02x", ep);
		return -EINVAL;
	}

	ret = k_sem_take(&ep_state->write_sem, K_NO_WAIT);
	if (ret) {
		LOG_ERR("Unable to get write lock (%d)", ret);
		return -EAGAIN;
	}

	if (!k_is_in_isr()) {
		irq_disable(USB_IRQ);
	}

	if (ep == EP0_IN && len > USB_MAX_CTRL_MPS) {
		len = USB_MAX_CTRL_MPS;
	}

	/* HAL_PCD_EP_Transmit (F1 branch) */
	struct usb_dc_n32_ll_ep *ep_ptr = &usb_dc_n32_state.pcd.in_ep[USB_EP_GET_IDX(ep)];

	ep_ptr->xfer_buff = (uint8_t *)data;
	ep_ptr->xfer_len = len;
	ep_ptr->xfer_count = 0U;
	ep_ptr->is_in = 1U;
	ep_ptr->num = USB_EP_GET_IDX(ep);

	usb_dc_n32_ll_ep_start_xfer(&usb_dc_n32_state.pcd, ep_ptr);

	if (!ret && ep == EP0_IN && len > 0) {
		/* Wait for an empty package as from the host.
		 * This also flushes the TX FIFO to the host.
		 */
		usb_dc_ep_start_read(ep, NULL, 0);
	}

	if (!k_is_in_isr()) {
		irq_enable(USB_IRQ);
	}

	if (!ret && ret_bytes) {
		*ret_bytes = len;
	}

	return ret;
}

int usb_dc_ep_read_wait(uint8_t ep, uint8_t *data, uint32_t max_data_len, uint32_t *read_bytes)
{
	struct usb_dc_n32_ep_state *ep_state = usb_dc_n32_get_ep_state(ep);
	uint32_t read_count;

	if (!ep_state) {
		LOG_ERR("Invalid Endpoint %x", ep);
		return -EINVAL;
	}

	read_count = ep_state->read_count;

	LOG_DBG("ep 0x%02x, %u bytes, %u+%u, %p", ep, max_data_len, ep_state->read_offset,
		read_count, (void *)data);

	if (!USB_EP_DIR_IS_OUT(ep)) { /* check if OUT ep */
		LOG_ERR("Wrong endpoint direction: 0x%02x", ep);
		return -EINVAL;
	}

	/* When both buffer and max data to read are zero, just ignore reading
	 * and return available data in buffer. Otherwise, return data
	 * previously stored in the buffer.
	 */
	if (data) {
		read_count = MIN(read_count, max_data_len);
		memcpy(data, usb_dc_n32_state.ep_buf[USB_EP_GET_IDX(ep)] + ep_state->read_offset,
		       read_count);
		ep_state->read_count -= read_count;
		ep_state->read_offset += read_count;
	} else if (max_data_len) {
		LOG_ERR("Wrong arguments");
	}

	if (read_bytes) {
		*read_bytes = read_count;
	}

	return 0;
}

int usb_dc_ep_read_continue(uint8_t ep)
{
	struct usb_dc_n32_ep_state *ep_state = usb_dc_n32_get_ep_state(ep);

	if (!ep_state || !USB_EP_DIR_IS_OUT(ep)) { /* Check if OUT ep */
		LOG_ERR("Not valid endpoint: %02x", ep);
		return -EINVAL;
	}

	/* If no more data in the buffer, start a new read transaction.
	 * DataOutStageCallback will called on transaction complete.
	 */
	if (!ep_state->read_count) {
		usb_dc_ep_start_read(ep, usb_dc_n32_state.ep_buf[USB_EP_GET_IDX(ep)],
				     ep_state->ep_mps);
	}

	return 0;
}

int usb_dc_ep_read(const uint8_t ep, uint8_t *const data, const uint32_t max_data_len,
		   uint32_t *const read_bytes)
{
	if (usb_dc_ep_read_wait(ep, data, max_data_len, read_bytes) != 0) {
		return -EINVAL;
	}

	if (usb_dc_ep_read_continue(ep) != 0) {
		return -EINVAL;
	}

	return 0;
}

int usb_dc_ep_halt(const uint8_t ep)
{
	return usb_dc_ep_set_stall(ep);
}

int usb_dc_ep_flush(const uint8_t ep)
{
	struct usb_dc_n32_ep_state *ep_state = usb_dc_n32_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	LOG_ERR("Not implemented");

	return 0;
}

int usb_dc_ep_mps(const uint8_t ep)
{
	struct usb_dc_n32_ep_state *ep_state = usb_dc_n32_get_ep_state(ep);

	if (!ep_state) {
		return -EINVAL;
	}

	return ep_state->ep_mps;
}

int usb_dc_wakeup_request(void)
{
	/* Activate remote wakeup signalling, keep it active from 1ms to 15ms
	 * as per the reference manual.
	 */
	usb_dc_n32_ll_activate_remote_wakeup(&usb_dc_n32_state.pcd);
	k_sleep(K_MSEC(2));
	usb_dc_n32_ll_deactivate_remote_wakeup(&usb_dc_n32_state.pcd);

	return 0;
}

int usb_dc_detach(void)
{
	int err;

	LOG_DBG("");

	/* Power down the macrocell (USB_StopDevice in HAL_PCD_DeInit) */
	usb_dc_n32_ll_stop_device(&usb_dc_n32_state.pcd);

	/* Release the D+ pull-up so the host sees a disconnect */
	usb_dc_n32_phystate_pullup(false);

	err = usb_dc_n32_clock_disable();
	if (err) {
		return err;
	}

	if (irq_is_enabled(USB_IRQ)) {
		irq_disable(USB_IRQ);
	}

	return 0;
}

int usb_dc_reset(void)
{
	LOG_ERR("Not implemented");

	return 0;
}

/* Callbacks from the low layer */

static void usb_dc_n32_reset_cb(struct usb_dc_n32_ll_pcd *hpcd)
{
	int i;

	LOG_DBG("");

	/*
	 * When an USB RESET occurs, the USB core goes to unconfigured state:
	 * clear all endpoint and open the control endpoint (EP0).
	 */
	usb_dc_n32_ll_ep_open(EP0_IN, EP0_MPS, USB_DC_N32_EP_TYPE_CTRL);
	usb_dc_n32_ll_ep_open(EP0_OUT, EP0_MPS, USB_DC_N32_EP_TYPE_CTRL);

	/* The DataInCallback will never be called at this point for any pending
	 * transactions. Reset the IN semaphores to prevent perpetual locked state.
	 */
	for (i = 0; i < USB_NUM_BIDIR_ENDPOINTS; i++) {
		k_sem_give(&usb_dc_n32_state.in_ep_state[i].write_sem);
	}

	usb_dc_n32_send_stack_msg(USB_DC_RESET);
}

static void usb_dc_n32_suspend_cb(struct usb_dc_n32_ll_pcd *hpcd)
{
	LOG_DBG("");

	usb_dc_n32_send_stack_msg(USB_DC_SUSPEND);
}

static void usb_dc_n32_resume_cb(struct usb_dc_n32_ll_pcd *hpcd)
{
	LOG_DBG("");

	usb_dc_n32_send_stack_msg(USB_DC_RESUME);
}

static void usb_dc_n32_sof_cb(struct usb_dc_n32_ll_pcd *hpcd)
{
	LOG_DBG("");

	if (IS_ENABLED(CONFIG_USB_DEVICE_SOF)) {
		usb_dc_n32_send_stack_msg(USB_DC_SOF);
	}
}

static void usb_dc_n32_setup_stage_cb(struct usb_dc_n32_ll_pcd *hpcd)
{
	struct usb_dc_n32_ep_state *ep_state;

	LOG_DBG("");

	ep_state = usb_dc_n32_get_ep_state(EP0_OUT); /* can't fail for ep0 */
	__ASSERT(ep_state, "No corresponding ep_state for EP0");

	ep_state->read_count = SETUP_SIZE;
	ep_state->read_offset = 0U;
	memcpy(&usb_dc_n32_state.ep_buf[EP0_IDX], usb_dc_n32_state.pcd.setup, ep_state->read_count);

	usb_dc_n32_send_ep_msg(EP0_OUT, USB_DC_EP_SETUP);
}

static void usb_dc_n32_data_out_stage_cb(struct usb_dc_n32_ll_pcd *hpcd, uint8_t epnum)
{
	uint8_t ep_idx = USB_EP_GET_IDX(epnum);
	uint8_t ep = ep_idx | USB_EP_DIR_OUT;
	struct usb_dc_n32_ep_state *ep_state = usb_dc_n32_get_ep_state(ep);

	LOG_DBG("epnum 0x%02x", epnum);

	/* Transaction complete, data is now stored in the buffer and ready
	 * for the upper stack (usb_dc_ep_read to retrieve).
	 */
	ep_state->read_count = usb_dc_n32_state.pcd.out_ep[ep_idx].xfer_count;
	ep_state->read_offset = 0U;

	usb_dc_n32_send_ep_msg(ep, USB_DC_EP_DATA_OUT);
}

static void usb_dc_n32_data_in_stage_cb(struct usb_dc_n32_ll_pcd *hpcd, uint8_t epnum)
{
	uint8_t ep_idx = USB_EP_GET_IDX(epnum);
	uint8_t ep = ep_idx | USB_EP_DIR_IN;
	struct usb_dc_n32_ep_state *ep_state = usb_dc_n32_get_ep_state(ep);

	LOG_DBG("epnum 0x%02x", epnum);

	__ASSERT(ep_state, "No corresponding ep_state for ep");

	k_sem_give(&ep_state->write_sem);

	usb_dc_n32_send_ep_msg(ep, USB_DC_EP_DATA_IN);
}

#endif /* CONFIG_SOC_SERIES_N32G45X */
