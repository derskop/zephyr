/*
 * Copyright (c) 2024 Nations Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_DMA_N32_DMA_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_DMA_N32_DMA_H_

/*
 * Peripheral request codes.
 *
 * These are the values written to a channel's CHSEL register, taken verbatim
 * from the DMA1_REMAP_* / DMA2_REMAP_* table in the vendor's n32g45x_dma.h.
 *
 * Note that DMA1_REQ_ADC1 and DMA2_REQ_TIM5_CH4 are 0x00, which is also the
 * value the driver treats as "leave this channel on its hardwired default
 * mapping". A channel cannot be remapped onto those two requests.
 */

/* DMA1 request codes */
#define N32_DMA1_REQ_ADC1           0x00000000
#define N32_DMA1_REQ_UART5_TX       0x00000001
#define N32_DMA1_REQ_I2C3_TX        0x00000002
#define N32_DMA1_REQ_TIM2_CH3       0x00000003
#define N32_DMA1_REQ_TIM4_CH1       0x00000004
#define N32_DMA1_REQ_USART3_TX      0x00000005
#define N32_DMA1_REQ_I2C3_RX        0x00000006
#define N32_DMA1_REQ_TIM1_CH1       0x00000007
#define N32_DMA1_REQ_TIM2_UP        0x00000008
#define N32_DMA1_REQ_TIM3_CH3       0x00000009
#define N32_DMA1_REQ_SPI1_RX        0x0000000A
#define N32_DMA1_REQ_USART3_RX      0x0000000B
#define N32_DMA1_REQ_TIM1_CH2       0x0000000C
#define N32_DMA1_REQ_TIM3_CH4       0x0000000D
#define N32_DMA1_REQ_TIM3_UP        0x0000000E
#define N32_DMA1_REQ_SPI1_TX        0x0000000F
#define N32_DMA1_REQ_USART1_TX      0x00000010
#define N32_DMA1_REQ_TIM1_CH4       0x00000011
#define N32_DMA1_REQ_TIM1_TRIG      0x00000012
#define N32_DMA1_REQ_TIM1_COM       0x00000013
#define N32_DMA1_REQ_TIM4_CH2       0x00000014
#define N32_DMA1_REQ_SPI_I2S2_RX    0x00000015
#define N32_DMA1_REQ_I2C2_TX        0x00000016
#define N32_DMA1_REQ_USART1_RX      0x00000017
#define N32_DMA1_REQ_TIM1_UP        0x00000018
#define N32_DMA1_REQ_SPI_I2S2_TX    0x00000019
#define N32_DMA1_REQ_TIM2_CH1       0x0000001A
#define N32_DMA1_REQ_TIM4_CH3       0x0000001B
#define N32_DMA1_REQ_I2C2_RX        0x0000001C
#define N32_DMA1_REQ_USART2_RX      0x0000001D
#define N32_DMA1_REQ_TIM1_CH3       0x0000001E
#define N32_DMA1_REQ_TIM3_CH1       0x0000001F
#define N32_DMA1_REQ_TIM3_TRIG      0x00000020
#define N32_DMA1_REQ_I2C1_TX        0x00000021
#define N32_DMA1_REQ_USART2_TX      0x00000022
#define N32_DMA1_REQ_TIM2_CH2       0x00000023
#define N32_DMA1_REQ_TIM2_CH4       0x00000024
#define N32_DMA1_REQ_TIM4_UP        0x00000025
#define N32_DMA1_REQ_I2C1_RX        0x00000026
#define N32_DMA1_REQ_ADC2           0x00000027
#define N32_DMA1_REQ_UART5_RX       0x00000028

/* DMA2 request codes */
#define N32_DMA2_REQ_TIM5_CH4       0x00000000
#define N32_DMA2_REQ_TIM5_TRIG      0x00000001
#define N32_DMA2_REQ_TIM8_CH3       0x00000002
#define N32_DMA2_REQ_TIM8_UP        0x00000003
#define N32_DMA2_REQ_SPI_I2S3_RX    0x00000004
#define N32_DMA2_REQ_UART6_RX       0x00000005
#define N32_DMA2_REQ_TIM8_CH4       0x00000006
#define N32_DMA2_REQ_TIM8_TRIG      0x00000007
#define N32_DMA2_REQ_TIM8_COM       0x00000008
#define N32_DMA2_REQ_TIM5_CH3       0x00000009
#define N32_DMA2_REQ_TIM5_UP        0x0000000A
#define N32_DMA2_REQ_SPI_I2S3_TX    0x0000000B
#define N32_DMA2_REQ_UART6_TX       0x0000000C
#define N32_DMA2_REQ_TIM8_CH1       0x0000000D
#define N32_DMA2_REQ_UART4_RX       0x0000000E
#define N32_DMA2_REQ_TIM6_UP        0x0000000F
#define N32_DMA2_REQ_DAC1           0x00000010
#define N32_DMA2_REQ_TIM5_CH2       0x00000011
#define N32_DMA2_REQ_SDIO           0x00000012
#define N32_DMA2_REQ_TIM7_UP        0x00000013
#define N32_DMA2_REQ_DAC2           0x00000014
#define N32_DMA2_REQ_ADC3           0x00000015
#define N32_DMA2_REQ_TIM8_CH2       0x00000016
#define N32_DMA2_REQ_TIM5_CH1       0x00000017
#define N32_DMA2_REQ_UART4_TX       0x00000018
#define N32_DMA2_REQ_QSPI_RX        0x00000019
#define N32_DMA2_REQ_I2C4_TX        0x0000001A
#define N32_DMA2_REQ_UART7_RX       0x0000001B
#define N32_DMA2_REQ_QSPI_TX        0x0000001C
#define N32_DMA2_REQ_I2C4_RX        0x0000001D
#define N32_DMA2_REQ_UART7_TX       0x0000001E
#define N32_DMA2_REQ_ADC4           0x0000001F
#define N32_DMA2_REQ_DVP            0x00000020

/*
 * Channel configuration flags.
 *
 * These line up with the CHCFG register bits defined in n32g45x.h, so the
 * driver can OR them straight into CHCFG. Only the fields that a consumer
 * cannot express through struct dma_config are useful here: direction, data
 * widths and address incrementation are all driven by the runtime fields of
 * dma_config and must not be set from devicetree.
 */

/* Channel priority, CHCFG bits 12-13 */
#define N32_DMA_CH_CFG_PRIORITY(val)   (((val) & 0x3) << 12)
#define N32_DMA_CH_CFG_PRIORITY_GET(cfg) (((cfg) >> 12) & 0x3)

#define N32_DMA_PRIORITY_LOW           0x0
#define N32_DMA_PRIORITY_MEDIUM        0x1
#define N32_DMA_PRIORITY_HIGH          0x2
#define N32_DMA_PRIORITY_VERY_HIGH     0x3

/* Circular mode, CHCFG bit 5 */
#define N32_DMA_CH_CFG_CIRCULAR        ((uint32_t)0x00000020)

/* Memory to memory mode, CHCFG bit 14 */
#define N32_DMA_CH_CFG_MEM2MEM         ((uint32_t)0x00004000)

/* Bits a devicetree config cell is allowed to carry */
#define N32_DMA_CH_CFG_VALID_BITS \
	(N32_DMA_CH_CFG_PRIORITY(0x3) | N32_DMA_CH_CFG_CIRCULAR | N32_DMA_CH_CFG_MEM2MEM)

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_DMA_N32_DMA_H_ */
