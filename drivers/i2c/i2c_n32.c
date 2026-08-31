/*
 * Copyright (c) 2024
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>

#include <zephyr/drivers/clock_control/n32_clock_control.h>

#include <n32g45x_i2c.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/printk.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

// #include <soc.h>



#include <zephyr/sys/sys_io.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/init.h>
#include <zephyr/drivers/clock_control.h>

#include <zephyr/linker/sections.h>
#include <zephyr/logging/log.h>

#define DT_DRV_COMPAT nsing_n32_i2c


LOG_MODULE_REGISTER(i2c_n32, CONFIG_I2C_LOG_LEVEL);

// #include "i2c-priv.h"

struct i2c_n32_config
{
    uint32_t reg_base;
    uint32_t apb_clk_en;
    uint32_t i2c_bitrate;
    //uint32_t cctl_offset;
    //uint32_t cctl_mask;
    const struct pinctrl_dev_config *pinctrl_config;
    void (*irq_config_func)(void);
};

struct i2c_n32_data
{
    struct k_sem bus_mutex;
    struct k_sem sync_sem;
    uint32_t dev_config;
    struct i2c_msg *msgs;
    uint32_t msg_num;
    uint32_t msg_idx;
    uint32_t buf_idx;
    
    uint16_t slave_addr;
    
};



#define I2CT_FLAG_TIMEOUT ((uint32_t)0x1000)
#define I2CT_LONG_TIMEOUT ((uint32_t)(20 * I2CT_FLAG_TIMEOUT)) 



static int i2c_n32_configure(const struct device *dev, uint32_t bitrate)
{
    const struct i2c_n32_config *cfg = dev->config;
    struct i2c_n32_data *data = dev->data;
    
    k_sem_take(&data->bus_mutex, K_FOREVER);
    
    I2C_DeInit((I2C_Module *)cfg->reg_base);
    
    I2C_InitType i2c_init = {
        .BusMode     = I2C_BUSMODE_I2C,
        .FmDutyCycle = I2C_FMDUTYCYCLE_2,
        .AckEnable   = I2C_ACKEN,
        .AddrMode    = I2C_ADDR_MODE_7BIT        
    };

    /* clock-frequency from devicetree is already the bitrate in Hz */
    i2c_init.ClkSpeed = bitrate;
    printk("i2c_configure: bitrate=%u Hz -> ClkSpeed=%u\n", bitrate, i2c_init.ClkSpeed);
    

    /* init i2c */
    I2C_Init((I2C_Module *)cfg->reg_base, &i2c_init);
    I2C_Enable((I2C_Module *)cfg->reg_base, ENABLE);
    
       /* release the mutex */
    k_sem_give(&data->bus_mutex);

    return 0;
}

static void n32_i2c_event_isr(const struct device *dev)
{
	struct i2c_n32_data *data = dev->data;
	const struct i2c_n32_config *config = dev->config;
	I2C_Module *i2c_port = (I2C_Module*)config->reg_base;

    unsigned int last_event = I2C_GetLastEvent(i2c_port);
    printk("I2C EV: 0x%08x\n", last_event);

    /* Spurious interrupt with no transfer in progress - nothing to serve. */
    if (data->msgs == NULL) {
        return;
    }

    if ((last_event & I2C_ROLE_MASTER) == I2C_ROLE_MASTER) // master mode
    {
        switch (last_event)
        {
            case I2C_EVT_MASTER_MODE_FLAG: // 0x00030001.EV5 start
                printk("EV5: flags=0x%02x %s len=%u slave=0x%02x\n",
                       data->msgs->flags,
                       (data->msgs->flags & I2C_MSG_READ) ? "READ" : "WRITE",
                       data->msgs->len, data->slave_addr);
                if (data->msgs->flags & I2C_MSG_READ) {    // read
                
                    //  uint8_t flags = data->msgs->flags ;
                    //  uint32_t len = data->msgs->len ;
                    
                    I2C_SendAddr7bit(i2c_port, data->slave_addr, I2C_DIRECTION_RECV);

                }
                else  { // write
                    I2C_SendAddr7bit(i2c_port, data->slave_addr, I2C_DIRECTION_SEND);
                  
                }
                break;
            case I2C_EVT_MASTER_TXMODE_FLAG: // 0x00070082.EV8 justafter EV6
                printk("EV6_TX: send byte 0x%02x (len=%u)\n", *data->msgs->buf, data->msgs->len);

                I2C_SendData(i2c_port,  *data->msgs->buf);
                data->msgs->buf++;
                data->msgs->len--;
                break;
            case I2C_EVT_MASTER_DATA_SENDING: // 0x00070080. transmitting data
                printk("EV8: len=%u\n", data->msgs->len);
  
                if (data->msgs->len > 0)
                {

                    I2C_SendData(i2c_port, *data->msgs->buf);
                    data->msgs->buf++;
                    data->msgs->len--;
                }
                break;
            case I2C_EVT_MASTER_DATA_SENDED: // 0x00070084.byte data send finish
                printk("EV8_2: len=%u\n", data->msgs->len);
                                           
                if (data->msgs->len == 0)  // data send finish
                {
                    if (data->msgs->flags & I2C_MSG_STOP) {
                        I2C_GenerateStop(i2c_port, ENABLE);
                    }
                    k_sem_give(&data->sync_sem);
                }
                break;
             
            case I2C_EVT_MASTER_RXMODE_FLAG: // 0x00030002.EV6
                printk("EV6_RX: len=%u\n", data->msgs->len);

                if (data->msgs->len == 1)
                {
                    I2C_ConfigAck(i2c_port, DISABLE);

                    I2C_GenerateStop(i2c_port, ENABLE);

                }
                else if (data->msgs->len  == 2)
                {
                    i2c_port->CTRL1 |= I2C_NACK_POS_NEXT; /// set ACKPOS
                    I2C_ConfigAck(i2c_port, DISABLE);
                }
                break;
            case I2C_EVT_MASTER_DATA_RECVD_FLAG: // one byte recved :EV7.//BUSY,MSL and RXDATNE flags.
            case I2C_EVT_MASTER_SFT_DATA_RECVD_FLAG: // BUSY, MSMODE(Master) and Data register not empty, BSF(Byte transfer finished)flags.
      
                *data->msgs->buf = I2C_RecvData(i2c_port);
                printk("EV7: recv 0x%02x (len=%u)\n", *data->msgs->buf, data->msgs->len);
                data->msgs->buf++;
                data->msgs->len--;
                
                if (data->msgs->len == 1)
                {
                    I2C_ConfigAck(i2c_port, DISABLE);   // Disable I2C acknowledgement.
                    {
                        I2C_GenerateStop(i2c_port, ENABLE); // Send I2C STOP Condition.
                        k_sem_give(&data->sync_sem);
                    }
                }
                else if (data->msgs->len == 0)
                {
                    k_sem_give(&data->sync_sem);
                }
                break;
            case 0x00030201: // Arbitration lost
            case 0x00030401: // Acknowledge failure
            case 0x00030501: // Acknowledge failure and Bus error
                printk("I2C ERROR event: 0x%08x\n", last_event);
                I2C_Enable(i2c_port, DISABLE);
                I2C_Enable(i2c_port, ENABLE);
                break;
            default:
                printk("I2C unhandled event: 0x%08x\n", last_event);
                break;
        }
    }
}


static void n32_i2c_error_isr(const struct device *dev)
{
	struct i2c_n32_data *data = dev->data;
	const struct i2c_n32_config *config = dev->config;
    I2C_Module *i2c_port = (I2C_Module*)config->reg_base;
    if(I2C_GetFlag(i2c_port, I2C_FLAG_ACKFAIL))
    {
        printk("I2C ERR: ACKFAIL (NACK)\n");
        I2C_ClrFlag(i2c_port, I2C_FLAG_ACKFAIL);
        I2C_GenerateStop(i2c_port, ENABLE); // Send I2C1 STOP Condition.
        k_sem_give(&data->sync_sem);
    }
    
}


static int i2c_n32_transfer_end(const struct device *dev)
{
	struct i2c_n32_data *data = dev->data;
	const struct i2c_n32_config *cfg = dev->config;
	I2C_Module *i2c_port = (I2C_Module *)cfg->reg_base;

	I2C_ConfigInt(i2c_port, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, DISABLE);

	/* Wait for stop condition only when this message ends with a STOP.
	 * For a write followed by a repeated START, the bus stays busy on purpose.
	 */
	if (data->msgs->flags & I2C_MSG_STOP) {
		while (I2C_GetFlag(i2c_port, I2C_FLAG_BUSY)) {
			/* NOP */
		}
	}

	return 0;
}


static int i2c_n32_msg_read(const struct device *dev, struct i2c_msg *msg,
			                uint16_t slave_addr)
{
	struct i2c_n32_data *data = dev->data;
	const struct i2c_n32_config *cfg = dev->config;
	I2C_Module *i2c_port = (I2C_Module *)cfg->reg_base;

	k_sem_reset(&data->sync_sem);

	I2C_ConfigInt(i2c_port, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, ENABLE);

	// I2C_ConfigPecLocation(i2c_port, I2C_PEC_POS_CURRENT);

	if (msg->flags & I2C_MSG_RESTART) {
		I2C_GenerateStart(i2c_port, ENABLE);
	}

	k_sem_take(&data->sync_sem, K_FOREVER);

	return i2c_n32_transfer_end(dev);
}

static int i2c_n32_msg_write(const struct device *dev, struct i2c_msg *msg,
			                 uint16_t saddr)
{
	struct i2c_n32_data *data = dev->data;
	const struct i2c_n32_config *config = dev->config;
	I2C_Module *i2c_port = (I2C_Module *)config->reg_base;

	k_sem_reset(&data->sync_sem);

	I2C_ConfigInt(i2c_port, I2C_INT_EVENT | I2C_INT_BUF | I2C_INT_ERR, ENABLE);

	// I2C_ConfigPecLocation(i2c_port, I2C_PEC_POS_CURRENT);

	if (msg->flags & I2C_MSG_RESTART) {
		I2C_GenerateStart(i2c_port, ENABLE);
	}

	k_sem_take(&data->sync_sem, K_FOREVER);

	return i2c_n32_transfer_end(dev);
}

static int i2c_n32_transfer(const struct device *dev, struct i2c_msg *msgs,
                            uint8_t num_msgs, uint16_t slave_addr)
{
    struct i2c_n32_data *data = dev->data;
    struct i2c_msg *current, *next;
    int ret = 0;

	current = msgs;

	/* First message flags set I2C_MSG_RESTART flag. */
	current->flags |= I2C_MSG_RESTART;

	for (uint8_t i = 1; i <= num_msgs; i++) {

		if (i < num_msgs) {
			next = current + 1;

			/*
			 * If there have a R/W transfer state change between messages,
			 * An explicit I2C_MSG_RESTART flag is needed for the second message.
			 */
			if ((current->flags & I2C_MSG_RW_MASK) !=
		    	(next->flags & I2C_MSG_RW_MASK)) {
				if ((next->flags & I2C_MSG_RESTART) == 0U) {
					return -EINVAL;
				}
			}

			/* Only the last message need I2C_MSG_STOP flag to free the Bus. */
			if (current->flags & I2C_MSG_STOP) {
				return -EINVAL;
			}
		} else {
			/* Last message flags contain I2C_MSG_STOP flag. */
			current->flags |= I2C_MSG_STOP;
		}

		if ((current->buf == NULL) ||
		    (current->len == 0U)) {
			return -EINVAL;
		}

		current++;
	}

    k_sem_take(&data->bus_mutex, K_FOREVER);

    current = msgs;

    for (int i = 0; i < num_msgs; i++)
    {
        data->msgs = &msgs[i];
        data->slave_addr = slave_addr;
        printk("transfer: msg[%d] addr=0x%02x len=%u flags=0x%02x\n",
               i, slave_addr, msgs[i].len, msgs[i].flags);
		//data->len = msgs[i].len;
 
        
        if ((current->flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE)
        {
            ret = i2c_n32_msg_write(dev, current, slave_addr);
        } 
        else {
            
            ret = i2c_n32_msg_read(dev, current, slave_addr);
        }
        
        
        if (ret < 0) {
            break;
        }
        current++;
    }

    /* No transfer in progress anymore. */
    data->msgs = NULL;

    /* release the mutex */
    k_sem_give(&data->bus_mutex);

    return 0;
}

/* API implementation: init */
static int i2c_n32_init(const struct device *dev)
{
    int status = 0;

    const struct i2c_n32_config *config = dev->config;
    struct i2c_n32_data *data = dev->data;
    //uint32_t dev_config = (I2C_MODE_MASTER); // | i2c_map_dt_bitrate(cfg->bitrate));
    
    /* Mutex semaphore to protect the i2c api in multi-thread env. */
    k_sem_init(&data->bus_mutex, 1, 1);
    
    /* Sync semaphore to sync i2c state between isr and transfer api. */
	k_sem_init(&data->sync_sem, 0, K_SEM_MAX_LIMIT);
    
    int ret = pinctrl_apply_state(config->pinctrl_config, PINCTRL_STATE_DEFAULT);
    if(ret != 0)
    {
        LOG_ERR("Failed to configure I2C pinctrl");
        return ret;
    }

    ret = clock_control_on(DEVICE_DT_GET(DT_NODELABEL(rcc)), (clock_control_subsys_t *)(&config->apb_clk_en));
    
    if (ret < 0) {
        return -EIO;
    }
    
    
    uint32_t bitrate_cfg = config->i2c_bitrate;;

    status = i2c_n32_configure(dev, bitrate_cfg);
    if (status != 0)
    {
        LOG_ERR("Failed to configure I2C on init");
        return status;
    }

    config->irq_config_func();
    
    return 0;
}

static const struct i2c_driver_api i2c_n32_api = {
    .configure = i2c_n32_configure,
    .transfer = i2c_n32_transfer,
};

 


#define N32_I2C_INIT(n)        \
                               \
    PINCTRL_DT_INST_DEFINE(n); \
    static void n32_i2c_irq_config_##n(void) \
    { \
        IRQ_CONNECT( \
            DT_INST_IRQ_BY_NAME(n, event, irq),    \
            DT_INST_IRQ_BY_NAME(n, event, priority),\
            n32_i2c_event_isr,     \
            DEVICE_DT_INST_GET(n), \
            0); \
        irq_enable(DT_INST_IRQ_BY_NAME(n, event, irq)); \
                                                    \
        IRQ_CONNECT(                                \
            DT_INST_IRQ_BY_NAME(n, error, irq),	    \
			DT_INST_IRQ_BY_NAME(n, error, priority),\
			n32_i2c_error_isr,			\
			DEVICE_DT_INST_GET(n), 0);		\
		irq_enable(DT_INST_IRQ_BY_NAME(n, error, irq));	\
    };  \
        \
    static struct i2c_n32_data i2c_n32_dev_data_##n;  \
    const static  struct i2c_n32_config i2c_n32_dev_cfg_##n = {  \
        .reg_base   = DT_INST_REG_ADDR(n), \
        .apb_clk_en = DT_INST_CLOCKS_CELL(n, bits),	\
        .i2c_bitrate = DT_INST_PROP(n, clock_frequency),	\
        .pinctrl_config  = PINCTRL_DT_INST_DEV_CONFIG_GET(n),  \
        .irq_config_func = n32_i2c_irq_config_##n \
    };  \
        \
    I2C_DEVICE_DT_INST_DEFINE(n, \
                              i2c_n32_init,              \
                              NULL,                      \
                              &i2c_n32_dev_data_##n,     \
                              &i2c_n32_dev_cfg_##n,      \
                              POST_KERNEL,               \
                              CONFIG_I2C_INIT_PRIORITY,  \
                              &i2c_n32_api);

DT_INST_FOREACH_STATUS_OKAY(N32_I2C_INIT);

