/*
 * Copyright (c) 2024, Nationstech
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nsing_n32g45_pinctrl




#include <zephyr/drivers/clock_control/n32_clock_control.h>
#include <zephyr/dt-bindings/pinctrl/n32-pinctrl.h>
#include <../../soc/nsing/nsing_n32/common/pinctrl_soc.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>


#include <n32g45x_gpio.h>



/**
 * @brief Array containing pointers to each GPIO port.
 *
 * Entries will be NULL if the GPIO port is not enabled.
 */
static const struct device *const gpio_ports[] = {

    DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioa)),
    DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiob)),
    DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioc)),
    DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiod)),
    DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpioe)),
    DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiof)),
    DEVICE_DT_GET_OR_NULL(DT_NODELABEL(gpiog)),
};


/**
 * @brief Configure the hardware.
   gpio_n32_configure_raw(const struct device *dev, int pin, int conf, int func)
 */
#if 1  
int n32_pin_configure(const struct device *dev, int pin, int conf, int slew_rate)
{

    struct n32_gpio_config {
        uint32_t reg_base_addr; 
        uint32_t rcc_reg_offset;
        uint32_t rcc_reg_mask;
        /* gpio_driver_config needs to be first */
        struct gpio_driver_config common;
    };

    struct n32_clock_control_subsys {
        uint32_t rcc_reg_offset; //rcc 
        uint32_t mask;   
    } subsys;
    
    const struct n32_gpio_config *cfg = dev->config;

    subsys.rcc_reg_offset = cfg->rcc_reg_offset;
    subsys.mask           = cfg->rcc_reg_mask;
    
    GPIO_Module *gpio = (GPIO_Module *)(cfg->reg_base_addr);

    clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)),  (clock_control_subsys_t)&subsys); 
    GPIO_InitType GPIO_InitStructure;
    

    switch (conf)
    {
        case AIN: /* HSI used as system clock */
            GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN; 
            break;
        case IN_FLOATING:
             GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;  
            break;
        case IPD: 
             GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;  
            break;
        case IPU:
            GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;  
            break;
        case Out_OD:
             GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;  
            break;
        case Out_PP: 
             GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;  
            break;    
        case AF_OD:
             GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
            break;
        case AF_PP: 
             GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP; 
            break; 

        default:
            break;
    }
   
    switch (slew_rate)
    {
        case SLEW_RATE_2MHZ: /* HSI used as system clock */
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
            break;
        case SLEW_RATE_10MHZ:
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
            break;
        case SLEW_RATE_50MHZ: 
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
            break;

        default:
            break;
    }

    GPIO_InitStructure.Pin   = (1 << pin);
    GPIO_InitPeripheral(gpio, &GPIO_InitStructure);

    return 0;
}
#endif





static int pinctrl_configure_pin(const pinctrl_soc_pin_t *pin_ctrl)
{
    const struct device *port_device;
    int ret = 0;
    // 比如 PA9 = USART1_TX，它的 mux = 0x1F09。
    uint32_t mux = pin_ctrl->pinmux;

    /* 重映射：remap 存的是完整 GPIO_RMP_xxx 值，0 表示不重映射 */
    if (pin_ctrl->remap != 0) {
        GPIO_ConfigPinRemap(pin_ctrl->remap, ENABLE);
    }

    uint32_t pin = (mux >> PIN_NUM_OFFSET) & PIN_NUM_MASK;

    uint32_t gpio_port = (mux >> PORT_NUM_OFFSET) & PORT_NUM_MASK;

    port_device  = gpio_ports[gpio_port];

    uint32_t pin_cgf   = (mux >> PORT_CONFIG_OFFSET) & PORT_CONFIG_MASK;
    uint32_t slew_rate = (mux >> SLEW_RATE_OFFSET) & SLEW_RATE_MASK;

    ret = n32_pin_configure(port_device, pin, pin_cgf, slew_rate);
    
    if (ret < 0) {
        return ret;
    }           

    return 0;
}


/**
 * @brief Configure a set of pins.
 *
 * This function will configure the necessary hardware blocks to make the
 * configuration immediately effective.
 *
 * @warning This function must never be used to configure pins used by an
 * instantiated device driver.
 *
 * @param pins List of pins to be configured.
 * @param pin_cnt Number of pins.
 * @param reg Device register (optional, use #PINCTRL_REG_NONE if not used).
 *
 * @retval 0 If succeeded
 * @retval -errno Negative errno for other failures.
 */
int pinctrl_configure_pins( const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
    ARG_UNUSED(reg);

    for (uint8_t i = 0U; i < pin_cnt; i++) 
    {
        int ret = pinctrl_configure_pin(&pins[i]);
        if (ret < 0) 
        {
            return ret;
        }
    }

    return 0;
}











