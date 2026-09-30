/*
 * Copyright (c) 2026 Nations Technologies
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/phy.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>
#include <ethernet/eth_stats.h>

#include <n32g457_eth.h>

#include "eth.h"

#define DT_DRV_COMPAT nsing_n32_eth

LOG_MODULE_REGISTER(eth_n32, CONFIG_ETHERNET_LOG_LEVEL);

#define ETH_N32_MTU NET_ETH_MTU

/* Descriptor and buffer counts, buffer size, from the HAL */
#define ETH_N32_RX_BUF_COUNT ETH_RXBUFNB
#define ETH_N32_TX_BUF_COUNT ETH_TXBUFNB
#define ETH_N32_RX_BUF_SIZE  ETH_RX_BUF_SIZE
#define ETH_N32_TX_BUF_SIZE  ETH_TX_BUF_SIZE

/* The RX frame length includes the 4 byte FCS */
#define ETH_N32_FCS_LEN 4

/* Address filtering bits of MACFFLT: PRM/HUC/HMC/PAM/HPF */
#define ETH_N32_MACFFLT_MASK                                                                       \
	(ETH_MACFFLT_PRM | ETH_MACFFLT_HUC | ETH_MACFFLT_HMC | ETH_MACFFLT_PAM | ETH_MACFFLT_HPF)

BUILD_ASSERT(ETH_N32_RX_BUF_SIZE % 4 == 0, "Rx buffer size must be a multiple of 4");

#define ETH_N32_DMA_BUF(dev)  (((const struct eth_n32_dev_cfg *)(dev)->config)->dma_buf)
#define ETH_N32_DMA_DESC(dev) (((const struct eth_n32_dev_cfg *)(dev)->config)->dma_desc)

struct eth_n32_dma_buf {
	uint8_t rx_buf[ETH_N32_RX_BUF_COUNT][ETH_N32_RX_BUF_SIZE];
	uint8_t tx_buf[ETH_N32_TX_BUF_COUNT][ETH_N32_TX_BUF_SIZE];
};

struct eth_n32_dma_desc {
	ETH_DMADescType rx_desc[ETH_N32_RX_BUF_COUNT];
	ETH_DMADescType tx_desc[ETH_N32_TX_BUF_COUNT];
};

struct eth_n32_dev_cfg {
	const struct pinctrl_dev_config *pcfg;
	void (*config_func)(void);
	const struct net_eth_mac_config mac_cfg;
	const struct device *phy_dev;
	const struct device *clock_dev;
	uint32_t clock_bits;
	struct eth_n32_dma_buf *dma_buf;
	struct eth_n32_dma_desc *dma_desc;
};

struct eth_n32_dev_data {
	struct net_if *iface;
	uint8_t mac_addr[NET_ETH_ADDR_LEN];
	/* MAC configuration, re-applied with the current speed and duplex */
	ETH_InitType hal_init;
	/* Descriptor indexes, kept in step with the HAL chain pointers */
	uint32_t tx_desc_idx;
	uint32_t rx_desc_idx;
	struct k_sem rx_int_sem;

	K_KERNEL_STACK_MEMBER(rx_thread_stack, CONFIG_ETH_N32_RX_THREAD_STACK_SIZE);
	struct k_thread rx_thread;
#if defined(CONFIG_ETH_N32_MULTICAST_FILTER)
	uint8_t hash_index_cnt[64];
#endif
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	struct net_stats_eth stats;
#endif
};

static struct net_pkt *eth_n32_rx(const struct device *dev);

/* PHY configuration callback of ETH_Init(); the PHY is driven by the PHY API */
static uint32_t eth_n32_phy_config(ETH_InitType *init)
{
	ARG_UNUSED(init);

	return ETH_SUCCESS;
}

/* Write the address filtering of hal_init to MACFFLT */
static void eth_n32_macfflt_update(const struct device *dev)
{
	struct eth_n32_dev_data *data = dev->data;
	uint32_t tmp;

	tmp = ETH->MACFFLT & ~ETH_N32_MACFFLT_MASK;
	tmp |= data->hal_init.PromiscuousMode | data->hal_init.MulticastFramesFilter |
	       data->hal_init.UnicastFramesFilter;

	ETH->MACFFLT = tmp;

	/* The write takes effect after four TX_CLK/RX_CLK cycles */
	k_sleep(K_MSEC(1));
	ETH->MACFFLT = tmp;
}

static void eth_n32_start(const struct device *dev)
{
	struct eth_n32_dev_data *data = dev->data;
	const struct eth_n32_dev_cfg *cfg = dev->config;

	/* Clear the OWN bits left over from a previous run */
	for (uint32_t i = 0; i < ETH_N32_TX_BUF_COUNT; i++) {
		cfg->dma_desc->tx_desc[i].Status = 0;
	}

	ETH_DMATxDescChainInit(&cfg->dma_desc->tx_desc[0], &cfg->dma_buf->tx_buf[0][0],
			       ETH_N32_TX_BUF_COUNT);
	ETH_DMARxDescChainInit(&cfg->dma_desc->rx_desc[0], &cfg->dma_buf->rx_buf[0][0],
			       ETH_N32_RX_BUF_COUNT);

	data->tx_desc_idx = 0;
	data->rx_desc_idx = 0;

	/* NIS/AIS gate the interrupt line for their group of status bits. */
	ETH_DMAITConfig(ETH_DMA_INT_NIS | ETH_DMA_INT_RX | ETH_DMA_INT_RX_BUF_UA, ENABLE);
	ETH_DMAITConfig(ETH_DMA_INT_AIS | ETH_DMA_INT_RX_OVERFLOW, ENABLE);
	ETH_Start();
}

static void eth_n32_stop(const struct device *dev)
{
	ARG_UNUSED(dev);

	ETH_MACTransmissionCmd(DISABLE);
	ETH_MACReceptionCmd(DISABLE);
	ETH_DMATransmissionCmd(DISABLE);
	ETH_DMAReceptionCmd(DISABLE);
}

static void eth_n32_set_mac_config(const struct device *dev, struct phy_link_state *state)
{
	struct eth_n32_dev_data *data = dev->data;

	data->hal_init.SpeedMode =
		PHY_LINK_IS_SPEED_100M(state->speed) ? ETH_SPEED_MODE_100M : ETH_SPEED_MODE_10M;
	data->hal_init.DuplexMode =
		PHY_LINK_IS_FULL_DUPLEX(state->speed) ? ETH_DUPLEX_MODE_FULL : ETH_DUPLEX_MODE_HALF;

	if (ETH_Init(&data->hal_init, eth_n32_phy_config) != ETH_SUCCESS) {
		LOG_ERR("Failed to apply MAC configuration");
	}

	eth_n32_macfflt_update(dev);
}

static int eth_n32_mac_init(const struct device *dev)
{
	struct eth_n32_dev_data *data = dev->data;

	ETH_InitStruct(&data->hal_init);

	data->hal_init.AutoNegotiation = ETH_AUTONEG_ENABLE;
	data->hal_init.SpeedMode = ETH_SPEED_MODE_100M;
	data->hal_init.DuplexMode = ETH_DUPLEX_MODE_FULL;
	data->hal_init.ChecksumOffload = IS_ENABLED(CONFIG_ETH_N32_HW_CHECKSUM)
						 ? ETH_CHECKSUM_OFFLOAD_ENABLE
						 : ETH_CHECKSUM_OFFLOAD_DISABLE;
	data->hal_init.BroadcastFramesReception = ETH_BROADCAST_FRAMES_RECEPTION_ENABLE;
	data->hal_init.MulticastFramesFilter = IS_ENABLED(CONFIG_ETH_N32_MULTICAST_FILTER)
						       ? ETH_MULTICAST_FRAMES_FILTER_HASH_TABLE
						       : ETH_MULTICAST_FRAMES_FILTER_NONE;
	data->hal_init.UnicastFramesFilter = ETH_UNICAST_FRAMES_FILTER_PERFECT;

	if (ETH_Init(&data->hal_init, eth_n32_phy_config) != ETH_SUCCESS) {
		LOG_ERR("ETH_Init failed");
		return -EIO;
	}

	ETH_MACAddressConfig(ETH_MAC_ADDR0, data->mac_addr);
	eth_n32_macfflt_update(dev);

	return 0;
}

static void eth_n32_rx_thread(void *p1, void *p2, void *p3)
{
	const struct device *dev = p1;
	struct eth_n32_dev_data *data = dev->data;
	struct net_pkt *pkt;

	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_sem_take(&data->rx_int_sem, K_FOREVER);

		while ((pkt = eth_n32_rx(dev)) != NULL) {
			if (net_recv_data(data->iface, pkt) < 0) {
				eth_stats_update_errors_rx(data->iface);
				net_pkt_unref(pkt);
			}
		}

		/* Wake the RX engine, which suspended when it ran out of
		 * descriptors and can only be restarted after they are freed.
		 */
		ETH_ResumeDMAReception();
	}
}

static void eth_n32_isr(const struct device *dev)
{
	struct eth_n32_dev_data *data = dev->data;

	if (ETH_GetDMAITStatus(ETH_DMA_INT_RX) == SET) {
		ETH_DMAClearITPendingBit(ETH_DMA_INT_RX);
		k_sem_give(&data->rx_int_sem);
	}

	/* Polling the RX engine here would re-assert RU before the RX thread
	 * has freed any descriptor, keeping the interrupt line asserted.
	 */
	if (ETH_GetDMAITStatus(ETH_DMA_INT_RX_BUF_UA) == SET) {
		ETH_DMAClearITPendingBit(ETH_DMA_INT_RX_BUF_UA);
		k_sem_give(&data->rx_int_sem);
	}

	if (ETH_GetDMAITStatus(ETH_DMA_INT_RX_OVERFLOW) == SET) {
		ETH_DMAClearITPendingBit(ETH_DMA_INT_RX_OVERFLOW);
		eth_stats_update_errors_rx(data->iface);
	}

	/* NIS/AIS latch their sources and hold the interrupt line asserted
	 * until written back.
	 */
	ETH_DMAClearITPendingBit(ETH_DMA_INT_NIS | ETH_DMA_INT_AIS);
}

/* Read one complete frame, or return NULL when no frame is available */
static struct net_pkt *eth_n32_rx(const struct device *dev)
{
	struct eth_n32_dev_data *data = dev->data;
	ETH_DMADescType *desc = &ETH_N32_DMA_DESC(dev)->rx_desc[data->rx_desc_idx];
	uint32_t status = desc->Status;
	uint32_t idx = data->rx_desc_idx;
	size_t len;
	struct net_pkt *pkt = NULL;

	if ((status & ETH_DMA_RX_DESC_OWN) != 0U) {
		return NULL;
	}

	data->rx_desc_idx = (idx + 1U) % ETH_N32_RX_BUF_COUNT;

	/* Drop fragmented and errored frames */
	if ((status & ETH_DMA_RX_DESC_FS) == 0U || (status & ETH_DMA_RX_DESC_LS) == 0U ||
	    (status & ETH_DMA_RX_DESC_ES) != 0U) {
		eth_stats_update_errors_rx(data->iface);
		goto release_desc;
	}

	len = ETH_GetDMARxDescFrameLength(desc) - ETH_N32_FCS_LEN;

	pkt = net_pkt_rx_alloc_with_buffer(data->iface, len, NET_AF_UNSPEC, 0, K_MSEC(100));
	if (pkt == NULL) {
		LOG_ERR("Failed to allocate RX packet");
		eth_stats_update_errors_rx(data->iface);
		goto release_desc;
	}

	if (net_pkt_write(pkt, ETH_N32_DMA_BUF(dev)->rx_buf[idx], len) != 0) {
		LOG_ERR("Failed to write RX packet");
		eth_stats_update_errors_rx(data->iface);
		net_pkt_unref(pkt);
		pkt = NULL;
		goto release_desc;
	}

	eth_stats_update_pkts_rx(data->iface);
	eth_stats_update_bytes_rx(data->iface, len);

release_desc:
	/* Hand the buffer back to the DMA once it is copied out */
	ETH_SetDMARxDescOwnBit(desc);

	return pkt;
}

static int eth_n32_send(const struct device *dev, struct net_pkt *pkt)
{
	struct eth_n32_dev_data *data = dev->data;
	ETH_DMADescType *desc = &ETH_N32_DMA_DESC(dev)->tx_desc[data->tx_desc_idx];
	uint32_t idx = data->tx_desc_idx;
	size_t len = net_pkt_get_len(pkt);

	if (len > ETH_N32_TX_BUF_SIZE) {
		LOG_ERR("Frame too large: %zu", len);
		eth_stats_update_errors_tx(data->iface);
		return -EMSGSIZE;
	}

	/* Wait for the DMA to release the descriptor */
	while ((desc->Status & ETH_DMA_TX_DESC_OWN) != 0U) {
		k_yield();
	}

	if (net_pkt_read(pkt, ETH_N32_DMA_BUF(dev)->tx_buf[idx], len) != 0) {
		eth_stats_update_errors_tx(data->iface);
		return -ENOBUFS;
	}

	if (IS_ENABLED(CONFIG_ETH_N32_HW_CHECKSUM)) {
		ETH_DMATxDescChecksumInsertionConfig(desc, ETH_DMA_TX_DESC_CIC_TCPUDPICMP_FULL);
	}

	if (ETH_PrepareTransmitDescriptors(len) != ETH_SUCCESS) {
		LOG_ERR("Failed to prepare TX descriptor");
		eth_stats_update_errors_tx(data->iface);
		return -EIO;
	}

	data->tx_desc_idx = (idx + 1U) % ETH_N32_TX_BUF_COUNT;
	eth_stats_update_pkts_tx(data->iface);
	eth_stats_update_bytes_tx(data->iface, len);

	return 0;
}

#if defined(CONFIG_ETH_N32_MULTICAST_FILTER)
static void eth_n32_mcast_filter(const struct device *dev, const struct ethernet_filter *filter)
{
	struct eth_n32_dev_data *data = dev->data;
	uint32_t hash_table[2];
	uint8_t hash_index;

	hash_index =
		(__RBIT(crc32_ieee(filter->mac_address.addr, sizeof(struct net_eth_addr))) >> 26) &
		0x3fU;

	hash_table[0] = ETH->MACHASHLO;
	hash_table[1] = ETH->MACHASHHI;

	if (filter->set) {
		data->hash_index_cnt[hash_index]++;
		hash_table[hash_index / 32U] |= (1U << (hash_index % 32U));
	} else {
		__ASSERT_NO_MSG(data->hash_index_cnt[hash_index] > 0U);

		data->hash_index_cnt[hash_index]--;
		if (data->hash_index_cnt[hash_index] == 0U) {
			hash_table[hash_index / 32U] &= ~(1U << (hash_index % 32U));
		}
	}

	ETH->MACHASHLO = hash_table[0];
	ETH->MACHASHHI = hash_table[1];

	/* ETH_Init() rewrites the hash table, keep the structure in sync */
	data->hal_init.HashTableLow = hash_table[0];
	data->hal_init.HashTableHigh = hash_table[1];
}
#endif /* CONFIG_ETH_N32_MULTICAST_FILTER */

static void eth_n32_phy_link_state_changed(const struct device *phy_dev,
					   struct phy_link_state *state, void *user_data)
{
	const struct device *dev = user_data;
	struct eth_n32_dev_data *data = dev->data;

	ARG_UNUSED(phy_dev);

	/* Stop the MAC before reconfiguring it for the new speed and duplex */
	if (state->is_up) {
		eth_n32_stop(dev);
		eth_n32_set_mac_config(dev, state);
		eth_n32_start(dev);
	}

	net_eth_carrier_set(data->iface, state->is_up);
}

static void eth_n32_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct eth_n32_dev_data *data = dev->data;
	const struct eth_n32_dev_cfg *cfg = dev->config;

	data->iface = iface;

	net_if_set_link_addr(iface, data->mac_addr, sizeof(data->mac_addr), NET_LINK_ETHERNET);

	ethernet_init(iface);

	net_if_carrier_off(iface);

	if (device_is_ready(cfg->phy_dev)) {
		phy_link_callback_set(cfg->phy_dev, eth_n32_phy_link_state_changed, (void *)dev);
	} else {
		LOG_ERR("PHY device not ready");
	}

	cfg->config_func();

	k_thread_create(&data->rx_thread, data->rx_thread_stack,
			K_KERNEL_STACK_SIZEOF(data->rx_thread_stack), eth_n32_rx_thread,
			(void *)dev, NULL, NULL, K_PRIO_COOP(CONFIG_ETH_N32_RX_THREAD_PRIO), 0,
			K_NO_WAIT);
	k_thread_name_set(&data->rx_thread, "n32_eth");
}

static int eth_n32_initialize(const struct device *dev)
{
	struct eth_n32_dev_data *data = dev->data;
	const struct eth_n32_dev_cfg *cfg = dev->config;
	int ret;

	ret = clock_control_on(cfg->clock_dev, (clock_control_subsys_t)&cfg->clock_bits);
	if (ret < 0) {
		LOG_ERR("Failed to enable ETH clock: %d", ret);
		return ret;
	}

	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("Failed to apply ETH pinctrl state: %d", ret);
		return ret;
	}

	ret = net_eth_mac_load(&cfg->mac_cfg, data->mac_addr);
	if (ret < 0) {
		LOG_ERR("Failed to load MAC address: %d", ret);
		return ret;
	}

	ret = eth_n32_mac_init(dev);
	if (ret < 0) {
		return ret;
	}

	k_sem_init(&data->rx_int_sem, 0, K_SEM_MAX_LIMIT);

	LOG_DBG("MAC %02x:%02x:%02x:%02x:%02x:%02x", data->mac_addr[0], data->mac_addr[1],
		data->mac_addr[2], data->mac_addr[3], data->mac_addr[4], data->mac_addr[5]);

	return 0;
}

static enum ethernet_hw_caps eth_n32_get_capabilities(const struct device *dev,
						      struct net_if *iface)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(iface);

	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE
#if defined(CONFIG_NET_PROMISCUOUS_MODE)
	       | ETHERNET_PROMISC_MODE
#endif
#if defined(CONFIG_ETH_N32_HW_CHECKSUM)
	       | ETHERNET_HW_TX_CHKSUM_OFFLOAD
#endif
#if defined(CONFIG_ETH_N32_MULTICAST_FILTER)
	       | ETHERNET_HW_FILTERING
#endif
		;
}

static const struct device *eth_n32_get_phy(const struct device *dev, struct net_if *iface)
{
	const struct eth_n32_dev_cfg *cfg = dev->config;

	ARG_UNUSED(iface);

	return cfg->phy_dev;
}

static int eth_n32_set_config(const struct device *dev, struct net_if *iface,
			      enum ethernet_config_type type, const struct ethernet_config *config)
{
	struct eth_n32_dev_data *data = dev->data;

	ARG_UNUSED(iface);

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(data->mac_addr, config->mac_address.addr, sizeof(data->mac_addr));
		ETH_MACAddressConfig(ETH_MAC_ADDR0, data->mac_addr);
		return 0;
#if defined(CONFIG_NET_PROMISCUOUS_MODE)
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:
		data->hal_init.PromiscuousMode = config->promisc_mode
							 ? ETH_PROMISCUOUS_MODE_ENABLE
							 : ETH_PROMISCUOUS_MODE_DISABLE;
		eth_n32_macfflt_update(dev);
		return 0;
#endif /* CONFIG_NET_PROMISCUOUS_MODE */
#if defined(CONFIG_ETH_N32_MULTICAST_FILTER)
	case ETHERNET_CONFIG_TYPE_FILTER:
		eth_n32_mcast_filter(dev, &config->filter);
		return 0;
#endif /* CONFIG_ETH_N32_MULTICAST_FILTER */
	default:
		break;
	}

	return -ENOTSUP;
}

static int eth_n32_get_config(const struct device *dev, struct net_if *iface,
			      enum ethernet_config_type type, struct ethernet_config *config)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(iface);

	switch (type) {
#if defined(CONFIG_ETH_N32_HW_CHECKSUM)
	case ETHERNET_CONFIG_TYPE_TX_CHECKSUM_SUPPORT:
		/* The MAC inserts the IPv4 header checksum and the TCP, UDP and
		 * ICMPv4 payload checksums. IPv6 is left to the network stack.
		 */
		config->chksum_support = ETHERNET_CHECKSUM_SUPPORT_IPV4_HEADER |
					 ETHERNET_CHECKSUM_SUPPORT_IPV4_ICMP |
					 ETHERNET_CHECKSUM_SUPPORT_TCP |
					 ETHERNET_CHECKSUM_SUPPORT_UDP;
		return 0;
#endif /* CONFIG_ETH_N32_HW_CHECKSUM */
	default:
		return -ENOTSUP;
	}
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *eth_n32_get_stats(const struct device *dev, struct net_if *iface)
{
	struct eth_n32_dev_data *data = dev->data;

	ARG_UNUSED(iface);

	return &data->stats;
}
#endif /* CONFIG_NET_STATISTICS_ETHERNET */

static const struct ethernet_api eth_n32_api = {
	.iface_api.init = eth_n32_iface_init,
	.get_capabilities = eth_n32_get_capabilities,
	.set_config = eth_n32_set_config,
	.get_config = eth_n32_get_config,
	.get_phy = eth_n32_get_phy,
	.send = eth_n32_send,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = eth_n32_get_stats,
#endif /* CONFIG_NET_STATISTICS_ETHERNET */
};

#define ETH_N32_DMA_BUF_DEFN(n) static struct eth_n32_dma_buf eth##n##_dma_buf __aligned(4)

#define ETH_N32_DMA_DESC_DEFN(n) static struct eth_n32_dma_desc eth##n##_dma_desc __aligned(4)

#define ETH_N32_IRQ_CONFIG_DEFN(n)                                                                 \
	static void eth##n##_irq_config(void)                                                      \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), eth_n32_isr,                \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

#define ETH_N32_PINCTRL_DEFN(n) PINCTRL_DT_INST_DEFINE(n)

#define ETH_N32_DATA_DEFN(n) static struct eth_n32_dev_data eth##n##_data

#define ETH_N32_CFG_DEFN(n)                                                                        \
	static const struct eth_n32_dev_cfg eth##n##_config = {                                    \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                         \
		.config_func = eth##n##_irq_config,                                                \
		.mac_cfg = NET_ETH_MAC_DT_INST_CONFIG_INIT(n),                                     \
		.phy_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, phy_handle)),                          \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_bits = DT_INST_CLOCKS_CELL_BY_IDX(n, 0, bits),                              \
		.dma_buf = &eth##n##_dma_buf,                                                      \
		.dma_desc = &eth##n##_dma_desc,                                                    \
	}

#define ETH_N32_DEVICE(n)                                                                          \
	ETH_N32_DMA_BUF_DEFN(n);                                                                   \
	ETH_N32_DMA_DESC_DEFN(n);                                                                  \
	ETH_N32_IRQ_CONFIG_DEFN(n);                                                                \
	ETH_N32_PINCTRL_DEFN(n);                                                                   \
	ETH_N32_DATA_DEFN(n);                                                                      \
	ETH_N32_CFG_DEFN(n);                                                                       \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, eth_n32_initialize, NULL, &eth##n##_data,                 \
				      &eth##n##_config, CONFIG_ETH_INIT_PRIORITY, &eth_n32_api,    \
				      ETH_N32_MTU);

DT_INST_FOREACH_STATUS_OKAY(ETH_N32_DEVICE)
