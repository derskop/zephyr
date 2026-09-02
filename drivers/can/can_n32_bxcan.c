/*
 * Copyright (c) 2018 Alexander Wachter
 * Copyright (c) 2022 Martin Jäger <martin@libre.solar>
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr CAN driver for Nations Technologies N32G45x bxCAN controllers.
 *
 * The N32G45x CAN peripheral is a register-compatible clone of the classic
 * STM32 bxCAN (CAN1/CAN2 each feature their own independent block of 14
 * filter banks, unlike the shared filter block on STM32F1).
 */

/* Include soc.h prior to Zephyr CAN headers to pull in HAL */
#include <soc.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/can/transceiver.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/n32_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(can_n32, CONFIG_CAN_LOG_LEVEL);

#define CAN_INIT_TIMEOUT (10 * (sys_clock_hw_cycles_per_sec() / MSEC_PER_SEC))

#define DT_DRV_COMPAT nsing_n32_bxcan

#define CAN_N32_NUM_FILTER_BANKS (14)
#define CAN_N32_MAX_FILTER_ID                                                                      \
	(CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS + CONFIG_CAN_N32_BXCAN_MAX_STD_ID_FILTERS)

/* CAN frame identifier register (RMI) bit fields */
#define CAN_RMI_STD_ID_POS (21U)
#define CAN_RMI_STD_ID_Msk (0x7FFUL << CAN_RMI_STD_ID_POS)
#define CAN_RMI_EXID_Pos   (3U)
#define CAN_RMI_EXID_Msk   (0x1FFFFUL << CAN_RMI_EXID_Pos)
#define CAN_RMI_RTR        (0x1UL << 1U)
#define CAN_RMI_IDE        (0x1UL << 2U)

/* 16-bit filter scale encoding (standard identifier filters) */
#define CAN_FILTER_STD_ID_POS  5U
#define CAN_FILTER_STD_RTR_POS 4U
#define CAN_FILTER_STD_IDE_POS 3U

/* CAN frame data / status register (RMDT) bit fields */
#define CAN_RMDT_DLC_Pos  (0U)
#define CAN_RMDT_DLC_Msk  (0xFUL << CAN_RMDT_DLC_Pos)
#define CAN_RMDT_FMI_Pos  (8U)
#define CAN_RMDT_FMI_Msk  (0xFFUL << CAN_RMDT_FMI_Pos)
#define CAN_RMDT_TIME_Pos (16U)
#define CAN_RMDT_TIME_Msk (0xFFFFUL << CAN_RMDT_TIME_Pos)

/*
 * Register bit fields of the N32 bxCAN IP. The N32G45x CAN peripheral is a
 * clone of the classic STM32 bxCAN, so bit positions match ST's definitions
 * (the N32 HAL header does not expose these as macros, hence the local
 * definitions).
 */

/* Control register (MCTRL) bit fields */
#define CAN_MCTRL_INRQ  (0x1UL << 0U)
#define CAN_MCTRL_SLEEP (0x1UL << 1U)
/* CAN_MCTRL_TXFP/RFLM/NART/ABOM/TTCM are provided by the N32 HAL */
#define CAN_MCTRL_AWUM  (0x1UL << 5U)

/* Status register (MSTS) bit fields */
#define CAN_MSTS_INAK (0x1UL << 0U)
#define CAN_MSTS_SLAK (0x1UL << 1U)
#define CAN_MSTS_ERRI (0x1UL << 2U)

/* Transmit status register (TSTS) bit fields */
#define CAN_TSTS_RQCP0 (0x1UL << 0U)
#define CAN_TSTS_TXOK0 (0x1UL << 1U)
#define CAN_TSTS_ALST0 (0x1UL << 2U)
#define CAN_TSTS_TERR0 (0x1UL << 3U)
#define CAN_TSTS_ABRQ0 (0x1UL << 7U)
#define CAN_TSTS_RQCP1 (0x1UL << 8U)
#define CAN_TSTS_TXOK1 (0x1UL << 9U)
#define CAN_TSTS_ALST1 (0x1UL << 10U)
#define CAN_TSTS_TERR1 (0x1UL << 11U)
#define CAN_TSTS_ABRQ1 (0x1UL << 15U)
#define CAN_TSTS_RQCP2 (0x1UL << 16U)
#define CAN_TSTS_TXOK2 (0x1UL << 17U)
#define CAN_TSTS_ALST2 (0x1UL << 18U)
#define CAN_TSTS_TERR2 (0x1UL << 19U)
#define CAN_TSTS_ABRQ2 (0x1UL << 23U)
#define CAN_TSTS_TME   (0x7UL << 26U)
#define CAN_TSTS_TME0  (0x1UL << 26U)
#define CAN_TSTS_TME1  (0x1UL << 27U)
#define CAN_TSTS_TME2  (0x1UL << 28U)

/* Receive FIFO 0 register (RFF0) bit fields */
#define CAN_RFF0_FMP0  (0x3UL << 0U)
#define CAN_RFF0_FOVR0 (0x1UL << 4U)
#define CAN_RFF0_RFOM0 (0x1UL << 5U)

/* Interrupt enable register (INTE) bit fields */
#define CAN_INTE_TMEIE  (0x1UL << 0U)
#define CAN_INTE_FMPIE0 (0x1UL << 1U)
#define CAN_INTE_FMPIE1 (0x1UL << 4U)
#define CAN_INTE_EWGIE  (0x1UL << 8U)
#define CAN_INTE_EPVIE  (0x1UL << 9U)
#define CAN_INTE_BOFIE  (0x1UL << 10U)
#define CAN_INTE_LECIE  (0x1UL << 11U)
#define CAN_INTE_ERRIE  (0x1UL << 15U)

/* Error status register (ESTS) bit fields */
#define CAN_ESTS_EWGF    (0x1UL << 0U)
#define CAN_ESTS_EPVF    (0x1UL << 1U)
#define CAN_ESTS_BOFF    (0x1UL << 2U)
#define CAN_ESTS_LEC_Pos (4U)
#define CAN_ESTS_LEC_Msk (0x7UL << CAN_ESTS_LEC_Pos)
/* CAN_ESTS_LEC_0/1/2 are provided by the N32 HAL */
#define CAN_ESTS_TEC_Pos (16U)
#define CAN_ESTS_TEC_Msk (0xFFUL << CAN_ESTS_TEC_Pos)
#define CAN_ESTS_REC_Pos (24U)
#define CAN_ESTS_REC_Msk (0xFFUL << CAN_ESTS_REC_Pos)

/* Bit timing register (BTIM) bit fields */
#define CAN_BTIM_BRP_Pos (0U)
#define CAN_BTIM_BRP_Msk (0x3FFUL << CAN_BTIM_BRP_Pos)
#define CAN_BTIM_TS1_Pos (16U)
#define CAN_BTIM_TS1_Msk (0xFUL << CAN_BTIM_TS1_Pos)
#define CAN_BTIM_TS2_Pos (20U)
#define CAN_BTIM_TS2_Msk (0x7UL << CAN_BTIM_TS2_Pos)
#define CAN_BTIM_SJW_Pos (24U)
#define CAN_BTIM_SJW_Msk (0x3UL << CAN_BTIM_SJW_Pos)
#define CAN_BTIM_LBKM    (0x1UL << 30U)
#define CAN_BTIM_SILM    (0x1UL << 31U)

/* TX mailbox identifier register (TMI) bit fields */
#define CAN_TMI_TXRQ     (0x1UL << 0U)
#define CAN_TMI_RTR      (0x1UL << 1U)
#define CAN_TMI_IDE      (0x1UL << 2U)
#define CAN_TMI_EXID_Pos (3U)
#define CAN_TMI_STID_POS (21U)

/* TX mailbox data/status register (TMDT) bit fields */
#define CAN_TMDT_DLC_Pos (0U)

/* Filter mode control register (FMC) bit fields */
#define CAN_FMC_FINIT (0x1UL << 0U)

#if (CONFIG_CAN_N32_BXCAN_MAX_STD_ID_FILTERS + CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS * 2) >      \
	(CAN_N32_NUM_FILTER_BANKS * 2)
#error Number of configured filters exceeds available filter bank slots.
#endif

struct can_n32_mailbox {
	can_tx_callback_t tx_callback;
	void *callback_arg;
};

struct can_n32_data {
	struct can_driver_data common;
	struct k_mutex inst_mutex;
	struct k_sem tx_int_sem;
	struct can_n32_mailbox mb0;
	struct can_n32_mailbox mb1;
	struct can_n32_mailbox mb2;
	can_rx_callback_t rx_cb_std[CONFIG_CAN_N32_BXCAN_MAX_STD_ID_FILTERS];
	can_rx_callback_t rx_cb_ext[CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS];
	void *cb_arg_std[CONFIG_CAN_N32_BXCAN_MAX_STD_ID_FILTERS];
	void *cb_arg_ext[CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS];
	enum can_state state;
};

struct can_n32_config {
	const struct can_driver_config common;
	CAN_Module *can;  /*!< CAN Registers */
	uint32_t clk_cfg; /*!< RCC clock cell for this CAN instance */
	void (*config_irq)(CAN_Module *can);
	const struct pinctrl_dev_config *pcfg;
};

static void can_n32_signal_tx_complete(const struct device *dev, struct can_n32_mailbox *mb,
				       int status)
{
	can_tx_callback_t callback = mb->tx_callback;

	if (callback != NULL) {
		callback(dev, status, mb->callback_arg);
		mb->tx_callback = NULL;
	}
}

static void can_n32_rx_fifo_pop(CAN_FIFOMailBox_Param *mbox, struct can_frame *frame)
{
	memset(frame, 0, sizeof(*frame));

	if (mbox->RMI & CAN_RMI_IDE) {
		frame->id = mbox->RMI >> CAN_RMI_EXID_Pos;
		frame->flags |= CAN_FRAME_IDE;
	} else {
		frame->id = mbox->RMI >> CAN_RMI_STD_ID_POS;
	}

	if ((mbox->RMI & CAN_RMI_RTR) != 0) {
		frame->flags |= CAN_FRAME_RTR;
	} else {
		frame->data_32[0] = mbox->RMDL;
		frame->data_32[1] = mbox->RMDH;
	}

	frame->dlc = mbox->RMDT & CAN_RMDT_DLC_Msk;
#ifdef CONFIG_CAN_RX_TIMESTAMP
	frame->timestamp = ((mbox->RMDT & CAN_RMDT_TIME_Msk) >> CAN_RMDT_TIME_Pos);
#endif
}

static inline void can_n32_rx_isr_handler(const struct device *dev)
{
	struct can_n32_data *data = dev->data;
	const struct can_n32_config *cfg = dev->config;
	CAN_Module *can = cfg->can;
	CAN_FIFOMailBox_Param *mbox;
	int filter_id, index;
	struct can_frame frame;
	can_rx_callback_t callback = NULL;
	void *cb_arg;

	while (can->RFF0 & CAN_RFF0_FMP0) {
		mbox = &can->sFIFOMailBox[0];
		filter_id = ((mbox->RMDT & CAN_RMDT_FMI_Msk) >> CAN_RMDT_FMI_Pos);

		LOG_DBG("Message on filter_id %d", filter_id);

		can_n32_rx_fifo_pop(mbox, &frame);

		if (filter_id < CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS) {
			callback = data->rx_cb_ext[filter_id];
			cb_arg = data->cb_arg_ext[filter_id];
		} else if (filter_id < CAN_N32_MAX_FILTER_ID) {
			index = filter_id - CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS;
			callback = data->rx_cb_std[index];
			cb_arg = data->cb_arg_std[index];
		}

		if (callback) {
			callback(dev, &frame, cb_arg);
		}

		/* Release message */
		can->RFF0 |= CAN_RFF0_RFOM0;
	}

	if (can->RFF0 & CAN_RFF0_FOVR0) {
		LOG_ERR("RX FIFO Overflow");
		CAN_STATS_RX_OVERRUN_INC(dev);
	}
}

static int can_n32_get_state(const struct device *dev, enum can_state *state,
			     struct can_bus_err_cnt *err_cnt)
{
	const struct can_n32_config *cfg = dev->config;
	struct can_n32_data *data = dev->data;
	CAN_Module *can = cfg->can;

	if (state != NULL) {
		if (!data->common.started) {
			*state = CAN_STATE_STOPPED;
		} else if (can->ESTS & CAN_ESTS_BOFF) {
			*state = CAN_STATE_BUS_OFF;
		} else if (can->ESTS & CAN_ESTS_EPVF) {
			*state = CAN_STATE_ERROR_PASSIVE;
		} else if (can->ESTS & CAN_ESTS_EWGF) {
			*state = CAN_STATE_ERROR_WARNING;
		} else {
			*state = CAN_STATE_ERROR_ACTIVE;
		}
	}

	if (err_cnt != NULL) {
		err_cnt->tx_err_cnt = ((can->ESTS & CAN_ESTS_TEC_Msk) >> CAN_ESTS_TEC_Pos);
		err_cnt->rx_err_cnt = ((can->ESTS & CAN_ESTS_REC_Msk) >> CAN_ESTS_REC_Pos);
	}

	return 0;
}

static inline void can_n32_bus_state_change_isr(const struct device *dev)
{
	struct can_n32_data *data = dev->data;
	struct can_bus_err_cnt err_cnt;
	enum can_state state;
	const can_state_change_callback_t cb = data->common.state_change_cb;
	void *state_change_cb_data = data->common.state_change_cb_user_data;

#ifdef CONFIG_CAN_STATS
	const struct can_n32_config *cfg = dev->config;
	CAN_Module *can = cfg->can;

	switch (can->ESTS & CAN_ESTS_LEC_Msk) {
	case (CAN_ESTS_LEC_0):
		CAN_STATS_STUFF_ERROR_INC(dev);
		break;
	case (CAN_ESTS_LEC_1):
		CAN_STATS_FORM_ERROR_INC(dev);
		break;
	case (CAN_ESTS_LEC_1 | CAN_ESTS_LEC_0):
		CAN_STATS_ACK_ERROR_INC(dev);
		break;
	case (CAN_ESTS_LEC_2):
		CAN_STATS_BIT1_ERROR_INC(dev);
		break;
	case (CAN_ESTS_LEC_2 | CAN_ESTS_LEC_0):
		CAN_STATS_BIT0_ERROR_INC(dev);
		break;
	case (CAN_ESTS_LEC_2 | CAN_ESTS_LEC_1):
		CAN_STATS_CRC_ERROR_INC(dev);
		break;
	default:
		break;
	}

	/* Clear last error code flag */
	can->ESTS |= CAN_ESTS_LEC_Msk;
#endif /* CONFIG_CAN_STATS */

	(void)can_n32_get_state(dev, &state, &err_cnt);

	if (state != data->state) {
		data->state = state;

		if (cb != NULL) {
			cb(dev, state, err_cnt, state_change_cb_data);
		}
	}
}

static inline void can_n32_tx_isr_handler(const struct device *dev)
{
	struct can_n32_data *data = dev->data;
	const struct can_n32_config *cfg = dev->config;
	CAN_Module *can = cfg->can;
	uint32_t bus_off;
	int status;

	bus_off = can->ESTS & CAN_ESTS_BOFF;

	if ((can->TSTS & CAN_TSTS_RQCP0) | bus_off) {
		status = can->TSTS & CAN_TSTS_TXOK0   ? 0
			 : can->TSTS & CAN_TSTS_TERR0 ? -EIO
			 : can->TSTS & CAN_TSTS_ALST0 ? -EBUSY
			 : bus_off                    ? -ENETUNREACH
						      : -EIO;
		/* clear the request. */
		can->TSTS |= CAN_TSTS_RQCP0;
		can_n32_signal_tx_complete(dev, &data->mb0, status);
	}

	if ((can->TSTS & CAN_TSTS_RQCP1) | bus_off) {
		status = can->TSTS & CAN_TSTS_TXOK1   ? 0
			 : can->TSTS & CAN_TSTS_TERR1 ? -EIO
			 : can->TSTS & CAN_TSTS_ALST1 ? -EBUSY
			 : bus_off                    ? -ENETUNREACH
						      : -EIO;
		/* clear the request. */
		can->TSTS |= CAN_TSTS_RQCP1;
		can_n32_signal_tx_complete(dev, &data->mb1, status);
	}

	if ((can->TSTS & CAN_TSTS_RQCP2) | bus_off) {
		status = can->TSTS & CAN_TSTS_TXOK2   ? 0
			 : can->TSTS & CAN_TSTS_TERR2 ? -EIO
			 : can->TSTS & CAN_TSTS_ALST2 ? -EBUSY
			 : bus_off                    ? -ENETUNREACH
						      : -EIO;
		/* clear the request. */
		can->TSTS |= CAN_TSTS_RQCP2;
		can_n32_signal_tx_complete(dev, &data->mb2, status);
	}

	if (can->TSTS & CAN_TSTS_TME) {
		k_sem_give(&data->tx_int_sem);
	}
}

static void can_n32_rx_isr(const struct device *dev)
{
	can_n32_rx_isr_handler(dev);
}

static void can_n32_tx_isr(const struct device *dev)
{
	can_n32_tx_isr_handler(dev);
}

static void can_n32_state_change_isr(const struct device *dev)
{
	const struct can_n32_config *cfg = dev->config;
	CAN_Module *can = cfg->can;

	/* Signal bus-off to waiting tx */
	if (can->MSTS & CAN_MSTS_ERRI) {
		can_n32_tx_isr_handler(dev);
		can_n32_bus_state_change_isr(dev);
		can->MSTS |= CAN_MSTS_ERRI;
	}
}

static int can_n32_enter_init_mode(CAN_Module *can)
{
	uint32_t start_time;

	can->MCTRL |= CAN_MCTRL_INRQ;
	start_time = k_cycle_get_32();

	while ((can->MSTS & CAN_MSTS_INAK) == 0U) {
		if (k_cycle_get_32() - start_time > CAN_INIT_TIMEOUT) {
			can->MCTRL &= ~CAN_MCTRL_INRQ;
			return -EAGAIN;
		}
	}

	return 0;
}

static int can_n32_leave_init_mode(CAN_Module *can)
{
	uint32_t start_time;

	/*
	 * The N32 reference HAL enters normal mode by clearing both the
	 * initialization and sleep requests.  Clearing only INRQ can leave the
	 * controller in sleep mode, in which case INAK does not reliably clear.
	 */
	can->MCTRL &= ~(CAN_MCTRL_SLEEP | CAN_MCTRL_INRQ);
	start_time = k_cycle_get_32();

	while ((can->MSTS & (CAN_MSTS_SLAK | CAN_MSTS_INAK)) != 0U) {
		if (k_cycle_get_32() - start_time > CAN_INIT_TIMEOUT) {
			LOG_ERR("normal mode timeout: MCTRL=0x%08x MSTS=0x%08x", can->MCTRL,
				can->MSTS);
			return -EAGAIN;
		}
	}

	return 0;
}

static int can_n32_leave_sleep_mode(CAN_Module *can)
{
	uint32_t start_time;

	can->MCTRL &= ~CAN_MCTRL_SLEEP;
	start_time = k_cycle_get_32();

	while ((can->MSTS & CAN_MSTS_SLAK) != 0) {
		if (k_cycle_get_32() - start_time > CAN_INIT_TIMEOUT) {
			return -EAGAIN;
		}
	}

	return 0;
}

static int can_n32_get_capabilities(const struct device *dev, can_mode_t *cap)
{
	ARG_UNUSED(dev);

	*cap = CAN_MODE_NORMAL | CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_ONE_SHOT;

	if (IS_ENABLED(CONFIG_CAN_MANUAL_RECOVERY_MODE)) {
		*cap |= CAN_MODE_MANUAL_RECOVERY;
	}

	return 0;
}

static int can_n32_start(const struct device *dev)
{
	const struct can_n32_config *cfg = dev->config;
	struct can_n32_data *data = dev->data;
	CAN_Module *can = cfg->can;
	int ret = 0;

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	if (data->common.started) {
		ret = -EALREADY;
		goto unlock;
	}

	if (cfg->common.phy != NULL) {
		ret = can_transceiver_enable(cfg->common.phy, data->common.mode);
		if (ret != 0) {
			LOG_ERR("failed to enable CAN transceiver (err %d)", ret);
			goto unlock;
		}
	}

	CAN_STATS_RESET(dev);

	ret = can_n32_leave_init_mode(can);
	if (ret < 0) {
		LOG_ERR("Failed to leave init mode");

		if (cfg->common.phy != NULL) {
			/* Attempt to disable the CAN transceiver in case of error */
			(void)can_transceiver_disable(cfg->common.phy);
		}

		ret = -EIO;
		goto unlock;
	}

	data->common.started = true;

unlock:
	k_mutex_unlock(&data->inst_mutex);

	return ret;
}

static int can_n32_stop(const struct device *dev)
{
	const struct can_n32_config *cfg = dev->config;
	struct can_n32_data *data = dev->data;
	CAN_Module *can = cfg->can;
	int ret = 0;

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	if (!data->common.started) {
		ret = -EALREADY;
		goto unlock;
	}

	ret = can_n32_enter_init_mode(can);
	if (ret < 0) {
		LOG_ERR("Failed to enter init mode");
		ret = -EIO;
		goto unlock;
	}

	/* Abort any pending transmissions */
	can_n32_signal_tx_complete(dev, &data->mb0, -ENETDOWN);
	can_n32_signal_tx_complete(dev, &data->mb1, -ENETDOWN);
	can_n32_signal_tx_complete(dev, &data->mb2, -ENETDOWN);
	can->TSTS |= CAN_TSTS_ABRQ2 | CAN_TSTS_ABRQ1 | CAN_TSTS_ABRQ0;

	if (cfg->common.phy != NULL) {
		ret = can_transceiver_disable(cfg->common.phy);
		if (ret != 0) {
			LOG_ERR("failed to disable CAN transceiver (err %d)", ret);
			goto unlock;
		}
	}

	data->common.started = false;

unlock:
	k_mutex_unlock(&data->inst_mutex);

	return ret;
}

static int can_n32_set_mode(const struct device *dev, can_mode_t mode)
{
	can_mode_t supported = CAN_MODE_LOOPBACK | CAN_MODE_LISTENONLY | CAN_MODE_ONE_SHOT;
	const struct can_n32_config *cfg = dev->config;
	CAN_Module *can = cfg->can;
	struct can_n32_data *data = dev->data;

	LOG_DBG("Set mode %d", mode);

	if (IS_ENABLED(CONFIG_CAN_MANUAL_RECOVERY_MODE)) {
		supported |= CAN_MODE_MANUAL_RECOVERY;
	}

	if ((mode & ~(supported)) != 0) {
		LOG_ERR("unsupported mode: 0x%08x", mode);
		return -ENOTSUP;
	}

	if (data->common.started) {
		return -EBUSY;
	}

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	if ((mode & CAN_MODE_LOOPBACK) != 0) {
		/* Loopback mode */
		can->BTIM |= CAN_BTIM_LBKM;
	} else {
		can->BTIM &= ~CAN_BTIM_LBKM;
	}

	if ((mode & CAN_MODE_LISTENONLY) != 0) {
		/* Silent mode */
		can->BTIM |= CAN_BTIM_SILM;
	} else {
		can->BTIM &= ~CAN_BTIM_SILM;
	}

	if ((mode & CAN_MODE_ONE_SHOT) != 0) {
		/* No automatic retransmission */
		can->MCTRL |= CAN_MCTRL_NART;
	} else {
		can->MCTRL &= ~CAN_MCTRL_NART;
	}

	if (IS_ENABLED(CONFIG_CAN_MANUAL_RECOVERY_MODE)) {
		if ((mode & CAN_MODE_MANUAL_RECOVERY) != 0) {
			/* No automatic recovery from bus-off */
			can->MCTRL &= ~CAN_MCTRL_ABOM;
		} else {
			can->MCTRL |= CAN_MCTRL_ABOM;
		}
	}

	data->common.mode = mode;

	k_mutex_unlock(&data->inst_mutex);

	return 0;
}

static int can_n32_set_timing(const struct device *dev, const struct can_timing *timing)
{
	const struct can_n32_config *cfg = dev->config;
	CAN_Module *can = cfg->can;
	struct can_n32_data *data = dev->data;

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	if (data->common.started) {
		k_mutex_unlock(&data->inst_mutex);
		return -EBUSY;
	}

	can->BTIM = (can->BTIM &
		     ~(CAN_BTIM_SJW_Msk | CAN_BTIM_BRP_Msk | CAN_BTIM_TS1_Msk | CAN_BTIM_TS2_Msk)) |
		    (((timing->sjw - 1) << CAN_BTIM_SJW_Pos) & CAN_BTIM_SJW_Msk) |
		    (((timing->phase_seg1 - 1) << CAN_BTIM_TS1_Pos) & CAN_BTIM_TS1_Msk) |
		    (((timing->phase_seg2 - 1) << CAN_BTIM_TS2_Pos) & CAN_BTIM_TS2_Msk) |
		    (((timing->prescaler - 1) << CAN_BTIM_BRP_Pos) & CAN_BTIM_BRP_Msk);

	k_mutex_unlock(&data->inst_mutex);

	return 0;
}

static int can_n32_get_core_clock(const struct device *dev, uint32_t *rate)
{
	const struct can_n32_config *cfg = dev->config;
	const struct device *clk;
	int ret;

	clk = DEVICE_DT_GET(DT_NODELABEL(rcc));

	ret = clock_control_get_rate(clk, (clock_control_subsys_t)&cfg->clk_cfg, rate);
	if (ret != 0) {
		LOG_ERR("Failed call clock_control_get_rate: return [%d]", ret);
		return -EIO;
	}

	return 0;
}

static int can_n32_get_max_filters(const struct device *dev, bool ide)
{
	ARG_UNUSED(dev);

	if (ide) {
		return CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS;
	} else {
		return CONFIG_CAN_N32_BXCAN_MAX_STD_ID_FILTERS;
	}
}

static int can_n32_init(const struct device *dev)
{
	const struct can_n32_config *cfg = dev->config;
	struct can_n32_data *data = dev->data;
	CAN_Module *can = cfg->can;
	struct can_timing timing = {0};
	const struct device *clk = DEVICE_DT_GET(DT_NODELABEL(rcc));
	int ret;

	k_mutex_init(&data->inst_mutex);
	k_sem_init(&data->tx_int_sem, 0, 1);

	if (cfg->common.phy != NULL) {
		if (!device_is_ready(cfg->common.phy)) {
			LOG_ERR_DEVICE_NOT_READY(cfg->common.phy);
			return -ENODEV;
		}
	}

	ret = clock_control_on(clk, (clock_control_subsys_t)&cfg->clk_cfg);
	if (ret != 0) {
		LOG_ERR("clock control on failed: %d", ret);
		return -EIO;
	}

	if (cfg->pcfg != NULL) {
		/* Configure dt provided device signals when available */
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("CAN pinctrl setup failed (%d)", ret);
			return ret;
		}
	}

	ret = can_n32_enter_init_mode(can);
	if (ret) {
		LOG_ERR("Failed to enter init mode");
		return ret;
	}

	ret = can_n32_leave_sleep_mode(can);
	if (ret) {
		LOG_ERR("Failed to exit sleep mode");
		return ret;
	}

	/*
	 * Configure the filter banks 0 .. CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS-1
	 * as 32-bit (ext ID) filters; the remaining banks are used as 16-bit
	 * (std ID) filters. N32 CAN1/CAN2 each have an independent set of
	 * 14 filter banks, so no cross-instance sharing is required.
	 */
	can->FMC |= CAN_FMC_FINIT;
	can->FS1 |= ((1U << CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS) - 1);
	can->FMC &= ~CAN_FMC_FINIT;

	can->MCTRL &= ~CAN_MCTRL_TTCM & ~CAN_MCTRL_ABOM & ~CAN_MCTRL_AWUM & ~CAN_MCTRL_NART &
		      ~CAN_MCTRL_RFLM & ~CAN_MCTRL_TXFP;
#ifdef CONFIG_CAN_RX_TIMESTAMP
	can->MCTRL |= CAN_MCTRL_TTCM;
#endif

	/* Enable automatic bus-off recovery */
	can->MCTRL |= CAN_MCTRL_ABOM;

	ret = can_calc_timing(dev, &timing, cfg->common.bitrate, cfg->common.sample_point);
	if (ret == -EINVAL) {
		LOG_ERR("Can't find timing for given param");
		return -EIO;
	}
	LOG_DBG("Presc: %d, TS1: %d, TS2: %d", timing.prescaler, timing.phase_seg1,
		timing.phase_seg2);
	LOG_DBG("Sample-point err : %d", ret);

	ret = can_set_timing(dev, &timing);
	if (ret) {
		return ret;
	}

	ret = can_n32_set_mode(dev, CAN_MODE_NORMAL);
	if (ret) {
		return ret;
	}

	(void)can_n32_get_state(dev, &data->state, NULL);

	cfg->config_irq(can);
	can->INTE |= CAN_INTE_TMEIE;

	return 0;
}

static void can_n32_set_state_change_callback(const struct device *dev,
					      can_state_change_callback_t cb, void *user_data)
{
	struct can_n32_data *data = dev->data;
	const struct can_n32_config *cfg = dev->config;
	CAN_Module *can = cfg->can;

	data->common.state_change_cb = cb;
	data->common.state_change_cb_user_data = user_data;

	if (cb == NULL) {
		can->INTE &= ~(CAN_INTE_BOFIE | CAN_INTE_EPVIE | CAN_INTE_EWGIE);
	} else {
		can->INTE |= CAN_INTE_BOFIE | CAN_INTE_EPVIE | CAN_INTE_EWGIE;
	}
}

#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
static int can_n32_recover(const struct device *dev, k_timeout_t timeout)
{
	const struct can_n32_config *cfg = dev->config;
	struct can_n32_data *data = dev->data;
	CAN_Module *can = cfg->can;
	int ret = -EAGAIN;
	int64_t start_time;

	if (!data->common.started) {
		return -ENETDOWN;
	}

	if ((data->common.mode & CAN_MODE_MANUAL_RECOVERY) == 0U) {
		return -ENOTSUP;
	}

	if (!(can->ESTS & CAN_ESTS_BOFF)) {
		return 0;
	}

	if (k_mutex_lock(&data->inst_mutex, K_FOREVER)) {
		return -EAGAIN;
	}

	ret = can_n32_enter_init_mode(can);
	if (ret) {
		goto done;
	}

	can_n32_leave_init_mode(can);

	start_time = k_uptime_ticks();

	while (can->ESTS & CAN_ESTS_BOFF) {
		if (!K_TIMEOUT_EQ(timeout, K_FOREVER) &&
		    k_uptime_ticks() - start_time >= timeout.ticks) {
			goto done;
		}
	}

	ret = 0;

done:
	k_mutex_unlock(&data->inst_mutex);
	return ret;
}
#endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */

static int can_n32_send(const struct device *dev, const struct can_frame *frame,
			k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
	const struct can_n32_config *cfg = dev->config;
	struct can_n32_data *data = dev->data;
	CAN_Module *can = cfg->can;
	uint32_t transmit_status_register = 0;
	CAN_TxMailBox_Param *mailbox = NULL;
	struct can_n32_mailbox *mb = NULL;

	LOG_DBG("Sending %d bytes on %s. "
		"Id: 0x%x, "
		"ID type: %s, "
		"Remote Frame: %s",
		frame->dlc, dev->name, frame->id,
		(frame->flags & CAN_FRAME_IDE) != 0 ? "extended" : "standard",
		(frame->flags & CAN_FRAME_RTR) != 0 ? "yes" : "no");

	if (frame->dlc > CAN_MAX_DLC) {
		LOG_ERR("DLC of %d exceeds maximum (%d)", frame->dlc, CAN_MAX_DLC);
		return -EINVAL;
	}

	if ((frame->flags & ~(CAN_FRAME_IDE | CAN_FRAME_RTR)) != 0) {
		LOG_ERR("unsupported CAN frame flags 0x%02x", frame->flags);
		return -ENOTSUP;
	}

	if (!data->common.started) {
		return -ENETDOWN;
	}

	if (can->ESTS & CAN_ESTS_BOFF) {
		return -ENETUNREACH;
	}

	k_mutex_lock(&data->inst_mutex, K_FOREVER);
	transmit_status_register = can->TSTS;
	while (!(transmit_status_register & CAN_TSTS_TME)) {
		k_mutex_unlock(&data->inst_mutex);
		LOG_DBG("Transmit buffer full");
		if (k_sem_take(&data->tx_int_sem, timeout)) {
			return -EAGAIN;
		}

		k_mutex_lock(&data->inst_mutex, K_FOREVER);
		transmit_status_register = can->TSTS;
	}

	if (transmit_status_register & CAN_TSTS_TME0) {
		LOG_DBG("Using TX mailbox 0");
		mailbox = &can->sTxMailBox[0];
		mb = &(data->mb0);
	} else if (transmit_status_register & CAN_TSTS_TME1) {
		LOG_DBG("Using TX mailbox 1");
		mailbox = &can->sTxMailBox[1];
		mb = &data->mb1;
	} else if (transmit_status_register & CAN_TSTS_TME2) {
		LOG_DBG("Using TX mailbox 2");
		mailbox = &can->sTxMailBox[2];
		mb = &data->mb2;
	} else {
		CODE_UNREACHABLE;
		/* We should never end up here */
		k_mutex_unlock(&data->inst_mutex);
		return -EIO;
	}

	mb->tx_callback = callback;
	mb->callback_arg = user_data;

	/* mailbox identifier register setup */
	mailbox->TMI &= CAN_TMI_TXRQ;

	if ((frame->flags & CAN_FRAME_IDE) != 0) {
		mailbox->TMI |= (frame->id << CAN_TMI_EXID_Pos) | CAN_TMI_IDE;
	} else {
		mailbox->TMI |= (frame->id << CAN_TMI_STID_POS);
	}

	if ((frame->flags & CAN_FRAME_RTR) != 0) {
		mailbox->TMI |= CAN_TMI_RTR;
	} else {
		mailbox->TMDL = frame->data_32[0];
		mailbox->TMDH = frame->data_32[1];
	}

	mailbox->TMDT = ((frame->dlc & 0xF) << CAN_TMDT_DLC_Pos);

	mailbox->TMI |= CAN_TMI_TXRQ;
	k_mutex_unlock(&data->inst_mutex);

	return 0;
}

static void can_n32_set_filter_bank(int filter_id, CAN_FilterRegister_Param *filter_reg, bool ide,
				    uint32_t id, uint32_t mask)
{
	if (ide) {
		filter_reg->FR1 = id;
		filter_reg->FR2 = mask;
	} else {
		if ((filter_id - CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS) % 2 == 0) {
			/* even std filter id: first 1/2 bank */
			filter_reg->FR1 = id | (mask << 16);
		} else {
			/* uneven std filter id: second 1/2 bank */
			filter_reg->FR2 = id | (mask << 16);
		}
	}
}

static inline uint32_t can_n32_filter_to_std_mask(const struct can_filter *filter)
{
	uint32_t rtr_mask = !IS_ENABLED(CONFIG_CAN_ACCEPT_RTR);

	return (filter->mask << CAN_FILTER_STD_ID_POS) | (rtr_mask << CAN_FILTER_STD_RTR_POS) |
	       (1U << CAN_FILTER_STD_IDE_POS);
}

static inline uint32_t can_n32_filter_to_ext_mask(const struct can_filter *filter)
{
	uint32_t rtr_mask = !IS_ENABLED(CONFIG_CAN_ACCEPT_RTR);

	return (filter->mask << CAN_RMI_EXID_Pos) | (rtr_mask << 1U) | (1U << 2U);
}

static inline uint32_t can_n32_filter_to_std_id(const struct can_filter *filter)
{
	return (filter->id << CAN_FILTER_STD_ID_POS);
}

static inline uint32_t can_n32_filter_to_ext_id(const struct can_filter *filter)
{
	return (filter->id << CAN_RMI_EXID_Pos) | (1U << 2U);
}

static inline int can_n32_set_filter(const struct device *dev, const struct can_filter *filter)
{
	const struct can_n32_config *cfg = dev->config;
	struct can_n32_data *data = dev->data;
	CAN_Module *can = cfg->can;
	uint32_t mask = 0U;
	uint32_t id = 0U;
	int filter_id = -ENOSPC;
	int bank_num;

	if ((filter->flags & CAN_FILTER_IDE) != 0) {
		for (int i = 0; i < CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS; i++) {
			if (data->rx_cb_ext[i] == NULL) {
				id = can_n32_filter_to_ext_id(filter);
				mask = can_n32_filter_to_ext_mask(filter);
				filter_id = i;
				bank_num = i;
				break;
			}
		}
	} else {
		for (int i = 0; i < CONFIG_CAN_N32_BXCAN_MAX_STD_ID_FILTERS; i++) {
			if (data->rx_cb_std[i] == NULL) {
				id = can_n32_filter_to_std_id(filter);
				mask = can_n32_filter_to_std_mask(filter);
				filter_id = CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS + i;
				bank_num = CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS + i / 2;
				break;
			}
		}
	}

	if (filter_id != -ENOSPC) {
		LOG_DBG("Adding filter_id %d, CAN ID: 0x%x, mask: 0x%x", filter_id, filter->id,
			filter->mask);

		/* set the filter init mode */
		can->FMC |= CAN_FMC_FINIT;

		can_n32_set_filter_bank(filter_id, &can->sFilterRegister[bank_num],
					(filter->flags & CAN_FILTER_IDE) != 0, id, mask);

		can->FA1 |= 1U << bank_num;
		can->FMC &= ~(CAN_FMC_FINIT);
	} else {
		LOG_WRN("No free filter left");
	}

	return filter_id;
}

/*
 * This driver uses masked mode for all filters (CAN_FM1 left at reset value
 * 0x00) in order to simplify mapping between filter match index from the FIFOs
 * and array index for the callbacks. All ext ID filters are stored in the
 * banks below CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS, followed by the std ID
 * filters, which consume only 1/2 bank per filter.
 *
 * The more complicated list mode must be implemented if someone requires more
 * than 28 std ID or 14 ext ID filters per instance.
 *
 * Currently, all filter banks are assigned to FIFO 0 and FIFO 1 is not used.
 */
static int can_n32_add_rx_filter(const struct device *dev, can_rx_callback_t cb, void *cb_arg,
				 const struct can_filter *filter)
{
	struct can_n32_data *data = dev->data;
	int filter_id;

	if ((filter->flags & ~(CAN_FILTER_IDE)) != 0) {
		LOG_ERR("unsupported CAN filter flags 0x%02x", filter->flags);
		return -ENOTSUP;
	}

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	filter_id = can_n32_set_filter(dev, filter);
	if (filter_id >= 0) {
		if ((filter->flags & CAN_FILTER_IDE) != 0) {
			data->rx_cb_ext[filter_id] = cb;
			data->cb_arg_ext[filter_id] = cb_arg;
		} else {
			data->rx_cb_std[filter_id - CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS] = cb;
			data->cb_arg_std[filter_id - CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS] =
				cb_arg;
		}
	}

	k_mutex_unlock(&data->inst_mutex);

	return filter_id;
}

static void can_n32_remove_rx_filter(const struct device *dev, int filter_id)
{
	const struct can_n32_config *cfg = dev->config;
	struct can_n32_data *data = dev->data;
	CAN_Module *can = cfg->can;
	bool ide;
	int bank_num;
	bool bank_unused;

	if (filter_id < 0 || filter_id >= CAN_N32_MAX_FILTER_ID) {
		LOG_ERR("filter ID %d out of bounds", filter_id);
		return;
	}

	k_mutex_lock(&data->inst_mutex, K_FOREVER);

	if (filter_id < CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS) {
		ide = true;
		bank_num = filter_id;

		data->rx_cb_ext[filter_id] = NULL;
		data->cb_arg_ext[filter_id] = NULL;

		bank_unused = true;
	} else {
		int filter_index = filter_id - CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS;

		ide = false;
		bank_num = CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS +
			   (filter_id - CONFIG_CAN_N32_BXCAN_MAX_EXT_ID_FILTERS) / 2;

		data->rx_cb_std[filter_index] = NULL;
		data->cb_arg_std[filter_index] = NULL;

		if (filter_index % 2 == 1) {
			bank_unused = data->rx_cb_std[filter_index - 1] == NULL;
		} else if (filter_index + 1 < CONFIG_CAN_N32_BXCAN_MAX_STD_ID_FILTERS) {
			bank_unused = data->rx_cb_std[filter_index + 1] == NULL;
		} else {
			bank_unused = true;
		}
	}

	LOG_DBG("Removing filter_id %d, ide %d", filter_id, ide);

	can->FMC |= CAN_FMC_FINIT;

	can_n32_set_filter_bank(filter_id, &can->sFilterRegister[bank_num], ide, 0, 0xFFFFFFFF);

	if (bank_unused) {
		can->FA1 &= ~(1U << bank_num);
		LOG_DBG("Filter bank %d is unused -> deactivate", bank_num);
	}

	can->FMC &= ~(CAN_FMC_FINIT);

	k_mutex_unlock(&data->inst_mutex);
}

static DEVICE_API(can,
		  can_api_funcs) = {.get_capabilities = can_n32_get_capabilities,
				    .start = can_n32_start,
				    .stop = can_n32_stop,
				    .set_mode = can_n32_set_mode,
				    .set_timing = can_n32_set_timing,
				    .send = can_n32_send,
				    .add_rx_filter = can_n32_add_rx_filter,
				    .remove_rx_filter = can_n32_remove_rx_filter,
				    .get_state = can_n32_get_state,
#ifdef CONFIG_CAN_MANUAL_RECOVERY_MODE
				    .recover = can_n32_recover,
#endif /* CONFIG_CAN_MANUAL_RECOVERY_MODE */
				    .set_state_change_callback = can_n32_set_state_change_callback,
				    .get_core_clock = can_n32_get_core_clock,
				    .get_max_filters = can_n32_get_max_filters,
				    .timing_min = {.sjw = 0x1,
						   .prop_seg = 0x00,
						   .phase_seg1 = 0x01,
						   .phase_seg2 = 0x01,
						   .prescaler = 0x01},
				    .timing_max = {.sjw = 0x04,
						   .prop_seg = 0x00,
						   .phase_seg1 = 0x10,
						   .phase_seg2 = 0x08,
						   .prescaler = 0x400}};

#define CAN_N32_IRQ_INST(inst)                                                                     \
	static void config_can_##inst##_irq(CAN_Module *can)                                       \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, rx, irq),                                    \
			    DT_INST_IRQ_BY_NAME(inst, rx, priority), can_n32_rx_isr,               \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, rx, irq));                                    \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, tx, irq),                                    \
			    DT_INST_IRQ_BY_NAME(inst, tx, priority), can_n32_tx_isr,               \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, tx, irq));                                    \
		IRQ_CONNECT(DT_INST_IRQ_BY_NAME(inst, sce, irq),                                   \
			    DT_INST_IRQ_BY_NAME(inst, sce, priority), can_n32_state_change_isr,    \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQ_BY_NAME(inst, sce, irq));                                   \
		can->INTE |= CAN_INTE_TMEIE | CAN_INTE_ERRIE | CAN_INTE_FMPIE0 | CAN_INTE_FMPIE1 | \
			     CAN_INTE_BOFIE;                                                       \
		if (IS_ENABLED(CONFIG_CAN_STATS)) {                                                \
			can->INTE |= CAN_INTE_LECIE;                                               \
		}                                                                                  \
	}

#define CAN_N32_CONFIG_INST(inst)                                                                  \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, pinctrl_0),				\
		    (PINCTRL_DT_INST_DEFINE(inst);), ())                                   \
	static const struct can_n32_config can_n32_cfg_##inst = {                                  \
		.common = CAN_DT_DRIVER_CONFIG_INST_GET(inst, 0, 1000000),                         \
		.can = (CAN_Module *)DT_INST_REG_ADDR(inst),                                       \
		.clk_cfg = DT_INST_CLOCKS_CELL(inst, bits),                                        \
		.config_irq = config_can_##inst##_irq,                                             \
		.pcfg = COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, pinctrl_0),		\
				    (PINCTRL_DT_INST_DEV_CONFIG_GET(inst)), (NULL)),                                \
		};

#define CAN_N32_DATA_INST(inst) static struct can_n32_data can_n32_dev_data_##inst;

#define CAN_N32_DEFINE_INST(inst)                                                                  \
	CAN_DEVICE_DT_INST_DEFINE(inst, can_n32_init, NULL, &can_n32_dev_data_##inst,              \
				  &can_n32_cfg_##inst, POST_KERNEL, CONFIG_CAN_INIT_PRIORITY,      \
				  &can_api_funcs);

#define CAN_N32_INST(inst)                                                                         \
	CAN_N32_IRQ_INST(inst)                                                                     \
	CAN_N32_CONFIG_INST(inst)                                                                  \
	CAN_N32_DATA_INST(inst)                                                                    \
	CAN_N32_DEFINE_INST(inst)

DT_INST_FOREACH_STATUS_OKAY(CAN_N32_INST)
