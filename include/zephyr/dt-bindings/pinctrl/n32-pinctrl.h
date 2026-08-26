/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_N32_PINCTRL_COMMON_H_
#define ZEPHYR_N32_PINCTRL_COMMON_H_

#include <zephyr/sys/util_macro.h>
#include <zephyr/sys/util_listify.h>
#include <zephyr/devicetree/pinctrl.h>


/*
	Pinmux: 
	  - bit[15:0] PIN
	  - bit[19:16] PORT
	  - bit[23:20] ALT FUNC
 */
#define N32_PORT_A       0x0
#define N32_PORT_B       0x1
#define N32_PORT_C       0x2
#define N32_PORT_D       0x3
#define N32_PORT_E       0x4
#define N32_PORT_F       0x5
#define N32_PORT_G       0x6


/** @addtogroup GPIO_Remap_define
 * @{
 */
 
#define REM_CFG_REG               (0U)    //Used to identify whether the register currently to be configured is AFIO_RMP_CFG
#define REM_CFG3_REG              (1U)    //Used to identify whether the register currently to be configured is AFIO_RMP_CFG3
#define REM_CFG4_REG              (2U)    //Used to identify whether the register currently to be configured is AFIO_RMP_CFG4
#define REM_CFG5_REG              (4U)    //Used to identify whether the register currently to be configured is AFIO_RMP_CFG5

#define REM_NO_RMP                (0U)
/* AFIO_RMP_CFG */
#define N32_RMP_SPI1              (0x00000001) /*!< SPI1 Alternate Function mapping */
#define N32_RMP_I2C1              (0x00000002) /*!< I2C1 Alternate Function mapping */
#define N32_RMP_USART1            (0x00000004) /*!< USART1 Alternate Function mapping */
#define N32_RMP_USART2            (0x00000008) /*!< USART2 Alternate Function mapping */
#define N32_PART_RMP_USART3       (0x00140010) /*!< USART3 Partial Alternate Function mapping */
#define N32_ALL_RMP_USART3        (0x00140030) /*!< USART3 Full Alternate Function mapping */
#define N32_PART1_RMP_TIM1        (0x00160040) /*!< TIM1 Partial Alternate Function mapping */
#define N32_PART2_RMP_TIM1        (0x00160080) /*!< TIM1 Partial Alternate Function mapping */
#define N32_ALL_RMP_TIM1          (0x001600C0) /*!< TIM1 Full Alternate Function mapping */
#define N32_PartialRemap1_TIM2    (0x00180100) /*!< TIM2 Partial1 Alternate Function mapping */
#define N32_PART2_RMP_TIM2        (0x00180200) /*!< TIM2 Partial2 Alternate Function mapping */
#define N32_ALL_RMP_TIM2          (0x00180300) /*!< TIM2 Full Alternate Function mapping */
#define N32_PART1_RMP_TIM3        (0x001A0800) /*!< TIM3 Partial Alternate Function mapping */
#define N32_ALL_RMP_TIM3          (0x001A0C00) /*!< TIM3 Full Alternate Function mapping */
#define N32_RMP_TIM4              (0x00001000) /*!< TIM4 Alternate Function mapping */
#define N32_RMP1_CAN1             (0x001D2000) /*!< CAN1 Alternate Function mapping */
#define N32_RMP2_CAN1             (0x001D4000) /*!< CAN1 Alternate Function mapping */
#define N32_RMP3_CAN1             (0x001D6000) /*!< CAN1 Alternate Function mapping */
#define N32_RMP_PD01              (0x00008000) /*!< PD01 Alternate Function mapping */
#define N32_RMP_TIM5CH4           (0x00200001) /*!< LSI connected to TIM5 Channel4 input capture for calibration */
#define N32_RMP_ADC1_ETRI         (0x00200002) /*!< ADC1 External Trigger Injected Conversion remapping */
#define N32_RMP_ADC1_ETRR         (0x00200004) /*!< ADC1 External Trigger Regular Conversion remapping */
#define N32_RMP_ADC2_ETRI         (0x00200008) /*!< ADC2 External Trigger Injected Conversion remapping */
#define N32_RMP_ADC2_ETRR         (0x00200010) /*!< ADC2 External Trigger Regular Conversion remapping */
#define N32_RMP_MII_RMII_SEL      (0x00200080) /*!< MII_RMII_SEL remapping */
#define N32_RMP_SW_JTAG_NO_NJTRST (0x00300100) /*!< Full SWJ Enabled (JTAG-DP + SW-DP) but without JTRST */
#define N32_RMP_SW_JTAG_SW_ENABLE (0x00300200) /*!< JTAG-DP Disabled and SW-DP Enabled */
#define N32_RMP_SW_JTAG_DISABLE   (0x00300400) /*!< Full SWJ Disabled (JTAG-DP + SW-DP) */

/* AFIO_RMP_CFG3 */
#define N32_RMP_SDIO              (0x40000001) /*!< SDIO Alternate Function mapping */
#define N32_RMP1_CAN2             (0x40110002) /*!< CAN2 Alternate Function mapping */
#define N32_RMP3_CAN2             (0x40110006) /*!< CAN2 Alternate Function mapping */
#define N32_RMP1_QSPI             (0x40140020) /*!< QSPI Alternate Function mapping */
#define N32_RMP3_QSPI             (0x40140030) /*!< QSPI Alternate Function mapping */
#define N32_RMP1_I2C2             (0x40160040) /*!< I2C2 Alternate Function mapping */
#define N32_RMP3_I2C2             (0x401600C0) /*!< I2C2 Alternate Function mapping */
#define N32_RMP2_I2C3             (0x40180200) /*!< I2C3 Alternate Function mapping */
#define N32_RMP3_I2C3             (0x40180300) /*!< I2C3 Alternate Function mapping */
#define N32_RMP1_I2C4             (0x401A0400) /*!< I2C4 Alternate Function mapping */
#define N32_RMP3_I2C4             (0x401A0C00) /*!< I2C4 Alternate Function mapping */
#define N32_RMP1_SPI2             (0x401C1000) /*!< SPI2 Alternate Function mapping */
#define N32_RMP2_SPI2             (0x401C3000) /*!< SPI2 Alternate Function mapping */
#define N32_RMP1_SPI3             (0x401E4000) /*!< SPI3 Alternate Function mapping */
#define N32_RMP2_SPI3             (0x401E8000) /*!< SPI3 Alternate Function mapping */
#define N32_RMP3_SPI3             (0x401EC000) /*!< SPI3 Alternate Function mapping */
#define N32_RMP1_ETH              (0x40300001) /*!< ETH Alternate Function mapping */
#define N32_RMP2_ETH              (0x40300002) /*!< ETH Alternate Function mapping */
#define N32_RMP3_ETH              (0x40300003) /*!< ETH Alternate Function mapping */
#define N32_RMP1_SPI1             (0x41200000) /*!< SPI1 Alternate Function mapping */
#define N32_RMP2_SPI1             (0x41200004) /*!< SPI1 Alternate Function mapping */
#define N32_RMP3_SPI1             (0x43200004) /*!< SPI1 Alternate Function mapping */
#define N32_RMP1_USART2           (0x44200000) /*!< USART2 Alternate Function mapping */
#define N32_RMP2_USART2           (0x44200008) /*!< USART2 Alternate Function mapping */
#define N32_RMP3_USART2           (0x46200008) /*!< USART2 Alternate Function mapping */
#define N32_RMP1_UART4            (0x40340010) /*!< UART4 Alternate Function mapping */
#define N32_RMP2_UART4            (0x40340020) /*!< UART4 Alternate Function mapping */
#define N32_RMP3_UART4            (0x40340030) /*!< UART4 Alternate Function mapping */
#define N32_RMP1_UART5            (0x40360040) /*!< UART5 Alternate Function mapping */
#define N32_RMP2_UART5            (0x40360080) /*!< UART5 Alternate Function mapping */
#define N32_RMP3_UART5            (0x403600C0) /*!< UART5 Alternate Function mapping */
#define N32_RMP2_UART6            (0x40380200) /*!< UART6 Alternate Function mapping */
#define N32_RMP3_UART6            (0x40380300) /*!< UART6 Alternate Function mapping */
#define N32_RMP1_UART7            (0x403A0400) /*!< UART7 Alternate Function mapping */
#define N32_RMP3_UART7            (0x403A0C00) /*!< UART7 Alternate Function mapping */
#define N32_RMP1_TIM8             (0x403E4000) /*!< TIM8 Alternate Function mapping */
#define N32_RMP3_TIM8             (0x403EC000) /*!< TIM8 Alternate Function mapping */

/* AFIO_RMP_CFG4 */
#define N32_RMP1_COMP1            (0x20100001) /*!< COMP1 Alternate Function mapping */
#define N32_RMP2_COMP1            (0x20100002) /*!< COMP1 Alternate Function mapping */
#define N32_RMP3_COMP1            (0x20100003) /*!< COMP1 Alternate Function mapping */
#define N32_RMP1_COMP2            (0x20120004) /*!< COMP2 Alternate Function mapping */
#define N32_RMP2_COMP2            (0x20120008) /*!< COMP2 Alternate Function mapping */
#define N32_RMP3_COMP2            (0x2012000C) /*!< COMP2 Alternate Function mapping */
#define N32_RMP1_COMP3            (0x20140010) /*!< COMP3 Alternate Function mapping */
#define N32_RMP3_COMP3            (0x20140030) /*!< COMP3 Alternate Function mapping */
#define N32_RMP1_COMP4            (0x20160040) /*!< COMP4 Alternate Function mapping */
#define N32_RMP3_COMP4            (0x201600C0) /*!< COMP4 Alternate Function mapping */
#define N32_RMP1_COMP5            (0x20180100) /*!< COMP5 Alternate Function mapping */
#define N32_RMP2_COMP5            (0x20180200) /*!< COMP5 Alternate Function mapping */
#define N32_RMP3_COMP5            (0x20180300) /*!< COMP5 Alternate Function mapping */
#define N32_RMP1_COMP6            (0x201A0400) /*!< COMP6 Alternate Function mapping */
#define N32_RMP3_COMP6            (0x201A0C00) /*!< COMP6 Alternate Function mapping */
#define N32_RMP_COMP7             (0x20001000) /*!< COMP7 Alternate Function mapping */
#define N32_RMP_ADC3_ETRI         (0x20004000) /*!< ADC3_ETRGINJ Alternate Function mapping */
#define N32_RMP_ADC3_ETRR         (0x20008000) /*!< ADC3_ETRGREG Alternate Function mapping */
#define N32_RMP_ADC4_ETRI         (0x20200001) /*!< ADC4_ETRGINJ Alternate Function mapping */
#define N32_RMP_ADC4_ETRR         (0x20200002) /*!< ADC4_ETRGREG Alternate Function mapping */
#define N32_RMP_TSC_OUT_CTRL      (0x20200004) /*!< TSC_OUT_CTRL Alternate Function mapping */
#define N32_RMP_QSPI_XIP_EN       (0x20200008) /*!< QSPI_XIP_EN Alternate Function mapping */
#define N32_RMP1_DVP              (0x20340010) /*!< DVP Alternate Function mapping */
#define N32_RMP3_DVP              (0x20340030) /*!< DVP Alternate Function mapping */
#define N32_Remap_SPI1_NSS        (0x20200040) /*!< SPI1 NSS Alternate Function mapping */
#define N32_Remap_SPI2_NSS        (0x20200080) /*!< SPI2 NSS Alternate Function mapping */
#define N32_Remap_SPI3_NSS        (0x20200100) /*!< SPI3 NSS Alternate Function mapping */
#define N32_Remap_QSPI_MISO       (0x20200200) /*!< QSPI MISO Alternate Function mapping */

/* AFIO_RMP_CFG5 */
#define N32_Remap_DET_EN_EGB4    (0x10200080) /*!< EGB4 Detect Alternate Function mapping*/
#define N32_Remap_DET_EN_EGB3    (0x10200040) /*!< EGB4 Detect Alternate Function mapping*/
#define N32_Remap_DET_EN_EGB2    (0x10200020) /*!< EGB4 Detect Alternate Function mapping*/
#define N32_Remap_DET_EN_EGB1    (0x10200010) /*!< EGB4 Detect Alternate Function mapping*/
#define N32_Remap_DET_EN_EGBN4   (0x10200008) /*!< EGBN4 Detect Alternate Function mapping*/
#define N32_Remap_DET_EN_EGBN3   (0x10200004) /*!< EGBN3 Detect Alternate Function mapping*/
#define N32_Remap_DET_EN_EGBN2   (0x10200002) /*!< EGBN2 Detect Alternate Function mapping*/
#define N32_Remap_DET_EN_EGBN1   (0x10200001) /*!< EGBN1 Detect Alternate Function mapping*/
#define N32_Remap_DET_EN_ECLAMP4 (0x10008000) /*!< ECLAMP4 Detect Alternate Function mapping*/
#define N32_Remap_DET_EN_ECLAMP3 (0x10004000) /*!< ECLAMP3 Detect Alternate Function mapping*/
#define N32_Remap_DET_EN_ECLAMP2 (0x10002000) /*!< ECLAMP2 Detect Alternate Function mapping*/
#define N32_Remap_DET_EN_ECLAMP1 (0x10001000) /*!< ECLAMP1 Detect Alternate Function mapping*/
#define N32_Remap_RST_EN_EGB4    (0x10000800) /*!< EGB4 Reset Alternate Function mapping*/
#define N32_Remap_RST_EN_EGB3    (0x10000400) /*!< EGB3 Reset Alternate Function mapping*/
#define N32_Remap_RST_EN_EGB2    (0x10000200) /*!< EGB2 Reset Alternate Function mapping*/
#define N32_Remap_RST_EN_EGB1    (0x10000100) /*!< EGB1 Reset Alternate Function mapping*/
#define N32_Remap_RST_EN_EGBN4   (0x10000080) /*!< EGBN4 Reset Alternate Function mapping*/
#define N32_Remap_RST_EN_EGBN3   (0x10000040) /*!< EGBN3 Reset Alternate Function mapping*/
#define N32_Remap_RST_EN_EGBN2   (0x10000020) /*!< EGBN2 Reset Alternate Function mapping*/
#define N32_Remap_RST_EN_EGBN1   (0x10000010) /*!< EGBN1 Reset Alternate Function mapping*/
#define N32_Remap_RST_EN_ECLAMP4 (0x10000008) /*!< ECLAMP4 Reset Alternate Function mapping*/
#define N32_Remap_RST_EN_ECLAMP3 (0x10000004) /*!< ECLAMP3 Reset Alternate Function mapping*/
#define N32_Remap_RST_EN_ECLAMP2 (0x10000002) /*!< ECLAMP2 Reset Alternate Function mapping*/
#define N32_Remap_RST_EN_ECLAMP1 (0x10000001) /*!< ECLAMP1 Reset Alternate Function mapping*/


//GPIO driver Configuration Mode
#define AIN                  0x0     //Analog input mode
#define IN_FLOATING          0x1     //floating input mode
#define IPD                  0x2     //pull down pull 
#define IPU                  0x3     //pull-up input
#define Out_OD               0x4     //open-drain output
#define Out_PP               0x5     //push-pull output
#define AF_OD                0x6     //alternate function open-drain
#define AF_PP                0x7     //alternate function push-pull  
         
                           
//Output Maximum frequency selection
#define INPUT                0        //GPIO_INPUT      
#define SLEW_RATE_2MHZ       1        //GPIO_Speed_2MHz
#define SLEW_RATE_10MHZ      2        //GPIO_Speed_10MHz,
#define SLEW_RATE_50MHZ      3        //GPIO_Speed_50MHz






#define PIN_NUM_OFFSET       (0U)   //pin 0-15 
#define PORT_NUM_OFFSET      (4U)   //portA - portG (gpioa - gpiog)
#define PORT_CONFIG_OFFSET   (8U)   //port config,
#define SLEW_RATE_OFFSET     (11U)  //port mode
#define ALTERNATE_OFFSET     (13U)  //port alternate

#define PIN_NUM_MASK         (0x0F) //pin number 4bit
#define PORT_NUM_MASK        (0x0F) //port number 4bit
#define PORT_CONFIG_MASK     (0x07) //port config 3bit
#define SLEW_RATE_MASK       (0x03) //port slew rate 2bit
#define ALTERNATE_MASK       (0xEFFFF) //port mode 19bit



#define N32_PINMUX(port, pin, port_cfg, slew_rate)    ( ((port) << PORT_NUM_OFFSET)| \
                                                                    ((pin)  << PIN_NUM_OFFSET) | \
                                                                    ((port_cfg)  << PORT_CONFIG_OFFSET) | \
                                                                    ((slew_rate) << SLEW_RATE_OFFSET))
                                                                  
                                              



#define N32_PINMUX_GET_FUNC(pinmux)       ((pinmux >> ALTERNATE_NUM_OFFSET) & 0xF)
#define N32_PINMUX_GET_PORT_PIN(pinmux)   ((pinmux >> PIN_NUM_OFFSET) & 0xFFFFF)







#define N32_MODE_SHIFT  0U
#define N32_MODE_MASK   0x3U
#define N32_LINE_SHIFT  2U
#define N32_LINE_MASK   0xFU
#define N32_PORT_SHIFT  6U
#define N32_PORT_MASK   0xFU
#define N32_REMAP_SHIFT 10U
#define N32_REMAP_MASK  0x3FFU

#define GPIO_PIN_MASK_POS   8U

#define ALTERNATE	0x0  /* Alternate function output */
#define GPIO_IN		0x1  /* Input */
#define ANALOG		0x2  /* Analog */
#define GPIO_OUT	0x3  /* Output */


#define N32F1_PINMUX(port, line, mode, remap)				       \
		(((((port) - 'A') & N32_PORT_MASK) << N32_PORT_SHIFT) |    \
		(((line) & N32_LINE_MASK) << N32_LINE_SHIFT) |	       \
		(((mode) & N32_MODE_MASK) << N32_MODE_SHIFT) |	       \
		(((remap) & N32_REMAP_MASK) << N32_REMAP_SHIFT))










/* Port Mode */
#define N32_MODE_INOUT_SHIFT	0
#define N32_MODE_INPUT	        (0x0 << N32_MODE_INOUT_SHIFT)
#define N32_MODE_OUTPUT		    (0x1 << N32_MODE_INOUT_SHIFT)
#define N32_MODE_INOUT_MASK		0x1




/* Input Port configuration */
#define N32_CNF_IN_SHIFT		1
#define N32_CNF_IN_ANALOG		(0x0 << N32_CNF_IN_SHIFT)
#define N32_CNF_IN_FLOAT		(0x1 << N32_CNF_IN_SHIFT)
#define N32_CNF_IN_PUPD		    (0x2 << N32_CNF_IN_SHIFT)
#define N32_CNF_IN_MASK		    0x3


/* Output Port configuration */
#define N32_MODE_OSPEED_MASK	0x3
#define N32_MODE_OSPEED_SHIFT	3
#define N32_MODE_OUTPUT_MAX_10	(0x0 << N32_MODE_OSPEED_SHIFT)
#define N32_MODE_OUTPUT_MAX_2	(0x1 << N32_MODE_OSPEED_SHIFT)
#define N32_MODE_OUTPUT_MAX_50	(0x2 << N32_MODE_OSPEED_SHIFT)


#define N32_CNF_OUT_0_MASK		0x1
#define N32_CNF_OUT_0_SHIFT		5
#define N32_CNF_PUSH_PULL		    (0x0 << N32_CNF_OUT_0_SHIFT)
#define N32_CNF_OPEN_DRAIN		(0x1 << N32_CNF_OUT_0_SHIFT)


#define N32_CNF_OUT_1_MASK		0x1
#define N32_CNF_OUT_1_SHIFT		6
#define N32_CNF_GP_OUTPUT	    (0x0 << N32_CNF_OUT_1_SHIFT)
#define N32_CNF_ALT_FUNC	    (0x1 << N32_CNF_OUT_1_SHIFT)



/* GPIO High impedance/Pull-up/Pull-down */
#define N32_PUPD_MASK		  0x3
#define N32_PUPD_SHIFT		  7
#define N32_PUPD_NO_PULL	  (0x0 << N32_PUPD_SHIFT)
#define N32_PUPD_PULL_UP	  (0x1 << N32_PUPD_SHIFT)
#define N32_PUPD_PULL_DOWN	  (0x2 << N32_PUPD_SHIFT)


/* GPIO plain output value */
#define N32_ODR_MASK	      0x1
#define N32_ODR_SHIFT		  9
#define N32_ODR_0			  (0x0 << N32_ODR_SHIFT)
#define N32_ODR_1			  (0x1 << N32_ODR_SHIFT)




/**
 * @brief helper macro to encode an IO port pin in a numerical format
 */
#define N32PIN(_port, _pin) (_port << 4 | _pin)




/** Helper to extract IO pin number from N32PIN() encoded value */
#define N32_PIN(__pin)	((__pin) & 0xf)

/** Helper to extract IO port number from N32_PINMUX() encoded value */
#define N32_DT_PINMUX_PORT(__pin)	(((__pin) >> N32_PORT_SHIFT) & N32_PORT_MASK)

/** Helper to extract IO pin number from N32_PINMUX() encoded value */
#define N32_DT_PINMUX_LINE(__pin) (((__pin) >> N32_LINE_SHIFT) & N32_LINE_MASK)

/** Helper to extract IO pin func from N32_PINMUX() encoded value */
#define N32_DT_PINMUX_FUNC(__pin) (((__pin) >> N32_MODE_SHIFT) & N32_MODE_MASK)


/** Helper to extract IO pin remap from N32_PINMUX() encoded value */
#define N32_DT_PINMUX_REMAP(__pin) (((__pin) >> N32_REMAP_SHIFT) & N32_REMAP_MASK)





















#endif







