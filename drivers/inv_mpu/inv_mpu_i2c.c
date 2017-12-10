/*
* Copyright (C) 2012 Invensense, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/
#define pr_fmt(fmt) "inv_mpu: " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>

#include "inv_mpu_iio.h"

#include <linux/gpio.h>
#include <mach/eint.h>

#define CONFIG_DYNAMIC_DEBUG_I2C 0

#ifdef CONFIG_DTS_INV_MPU_IIO
#include "inv_mpu_dts.h"
#endif

static int sensor_read_data(struct i2c_client *client, u8 command, u8 *buf, int count)
{
#if 1
	int ret = 0;
	struct i2c_msg msgs[2] = {
			{
					.addr = INV_ADDR & I2C_MASK_FLAG,
					.flags = 0,
					.buf = buf,
					.len = 1,
					.timing = I2C_TRANS_TIMING,
					.ext_flag = 0x00,
			},
			{
					.addr = INV_ADDR & I2C_MASK_FLAG,
					.flags = I2C_M_RD,
					.buf = buf,
					.len = count,
					.timing = I2C_TRANS_TIMING,
					.ext_flag = 0x00,
			}
	};
	buf[0] = command;

	ret = i2c_transfer(client->adapter, msgs, 2);
	if( ret!=2 ) {
		printk("invn: %s read error, reg:0x%x,count:%d, ext_flag:0x%x\n",__func__,command,count,client->addr&0xff00);
	}
#endif
	return ret;
}

/**
 *  inv_i2c_read_base() - Read one or more bytes from the device registers.
 *  @st:	Device driver instance.
 *  @i2c_addr:  i2c address of device.
 *  @reg:	First device register to be read from.
 *  @length:	Number of bytes to read.
 *  @data:	Data read from device.
 *  NOTE:This is not re-implementation of i2c_smbus_read because i2c
 *       address could be specified in this case. We could have two different
 *       i2c address due to secondary i2c interface.
 */
int inv_i2c_read_base(struct inv_mpu_state *st, u16 i2c_addr,
	u8 reg, u16 size, u8 *data)
{

	int i,retval = 0;
	u8 addr = reg;
	u8 buf[1024]={0,};
	int times = (size>1024?1024:size)/I2C_WR_MAX;

	mutex_lock(&st->inv_rw_mutex);
	for (i = 0; i < times; i++) {
		if( reg!=REG_FIFO_R_W ) {
			retval |= sensor_read_data(st->client, addr + I2C_WR_MAX*i, buf + I2C_WR_MAX*i, I2C_WR_MAX);
		} else {
			retval |= sensor_read_data(st->client, addr, buf + I2C_WR_MAX*i, I2C_WR_MAX);
		}
	}

	if (size % I2C_WR_MAX) {
		if( reg!=REG_FIFO_R_W ) {
			retval |= sensor_read_data(st->client, addr + I2C_WR_MAX*i, buf + I2C_WR_MAX*i, size % I2C_WR_MAX);
		} else {
			retval |= sensor_read_data(st->client, addr, buf + I2C_WR_MAX*i, size % I2C_WR_MAX);
		}
	}

	memcpy(data, buf, size);

	mutex_unlock(&st->inv_rw_mutex);

	return (retval<0)?retval:0;
}

/**
 *  inv_i2c_single_write_base() - Write a byte to a device register.
 *  @st:	Device driver instance.
 *  @i2c_addr:  I2C address of the device.
 *  @reg:	Device register to be written to.
 *  @data:	Byte to write to device.
 *  NOTE:This is not re-implementation of i2c_smbus_write because i2c
 *       address could be specified in this case. We could have two different
 *       i2c address due to secondary i2c interface.
 */
int inv_i2c_single_write_base(struct inv_mpu_state *st,
	u16 i2c_addr, u8 reg, u8 data)
{

	int ret = 0;
	u8 buf[2];
	struct i2c_msg msgs[1] ={{0}};

	mutex_lock(&st->inv_rw_mutex);

#if 1

	buf[0] = reg;
	buf[1] = data;

	msgs[0].addr = i2c_addr & I2C_MASK_FLAG,
	msgs[0].flags = 0,
	msgs[0].buf = buf,
	msgs[0].len = sizeof(buf),
	msgs[0].timing = I2C_TRANS_TIMING,
	msgs[0].ext_flag = 0,

	ret = i2c_transfer(st->client->adapter, msgs, 1);
	if (ret!=1)
	{
		pr_err("invn: i2c_master_send data address 0x%x 0x%x error return %d\n",
				buf[0], buf[1], ret);
	}
#endif
	mutex_unlock(&st->inv_rw_mutex);

	return (ret < 0) ? ret : 0;
}

int inv_plat_single_write(struct inv_mpu_state *st, u8 reg, u8 data)
{
	return inv_i2c_single_write_base(st, st->i2c_addr, reg, data);
}

int inv_plat_read(struct inv_mpu_state *st, u8 reg, int len, u8 *data)
{
	return inv_i2c_read_base(st, st->i2c_addr, reg, len, data);
}



int mpu_memory_write(struct inv_mpu_state *st, u8 mpu_addr, u16 mem_addr,
		     u32 size, u8 const *data)
{

#if 1
    int length = size;
    int writebytes = 0;
    int ret = 0;
    u8 const *ptr = data;
    unsigned char buf[I2C_WR_MAX + 1];

    struct i2c_msg msgs[3]={{0}, };


    mutex_lock(&st->inv_rw_mutex);

    memset(buf, 0, sizeof(buf));

    while(length > 0)
    {
              if (length > I2C_WR_MAX)
                       writebytes = I2C_WR_MAX;
              else
                       writebytes = length;


				buf[0] = REG_MEM_BANK_SEL;
				buf[1] = mem_addr >> 8;
				msgs[0].addr = mpu_addr & I2C_MASK_FLAG;
				msgs[0].flags = 0;
				msgs[0].buf = buf;
				msgs[0].len = 2;
				msgs[0].timing = I2C_TRANS_TIMING;
				msgs[0].ext_flag = 0;

	          	ret = i2c_transfer(st->client->adapter, msgs, 1);
	              if (ret!=1)
	              {
	                       printk("invn: i2c transfer error ret:%d, write_bytes:%d, Line %d\n", ret, writebytes+1, __LINE__);
	                       goto write_error;
	              }

				buf[0] = REG_MEM_START_ADDR;
				buf[1] = mem_addr & 0xFF;
				msgs[0].addr = mpu_addr & I2C_MASK_FLAG;
				msgs[0].flags = 0;
				msgs[0].buf = buf;
				msgs[0].len = 2;
				msgs[0].timing = I2C_TRANS_TIMING;
				msgs[0].ext_flag = 0;

	          	ret = i2c_transfer(st->client->adapter, msgs, 1);
	              if (ret!=1)
	              {
	                       printk("invn: i2c transfer error ret:%d, write_bytes:%d, Line %d\n", ret, writebytes+1, __LINE__);
	                       goto write_error;
	              }

				memset(buf, 0, sizeof(buf));
				buf[0] = REG_MEM_R_W;
				memcpy(buf+1, ptr, writebytes);
				msgs[0].addr = mpu_addr & I2C_MASK_FLAG;
				msgs[0].flags = 0;
				msgs[0].buf = buf;
				msgs[0].len = writebytes+1;
				msgs[0].timing = I2C_TRANS_TIMING;
				msgs[0].ext_flag = 0;

          	ret = i2c_transfer(st->client->adapter, msgs, 1);
              if (ret!=1)
              {
                       printk("invn: i2c transfer error ret:%d, write_bytes:%d, Line %d\n", ret, writebytes+1, __LINE__);
                       goto write_error;
              }

              length -= writebytes;
              mem_addr += writebytes;
              ptr +=writebytes;
    }

    mutex_unlock(&st->inv_rw_mutex);
    return 0;
write_error:
	mutex_unlock(&st->inv_rw_mutex);
    return -1;

#endif
}

int mpu_memory_read(struct inv_mpu_state *st, u8 mpu_addr, u16 mem_addr,
		    u32 size, u8 *data)
{

    int length = size;
    int readbytes = 0;
    int ret = 0;
    unsigned char buf[I2C_WR_MAX];
    struct i2c_msg msgs[3] = {{0}, };

    mutex_lock(&st->inv_rw_mutex);

    while(length > 0)
    {
              if (length > I2C_WR_MAX)
                       readbytes = I2C_WR_MAX;
              else
                       readbytes = length;

              buf[0] = REG_MEM_BANK_SEL;
              buf[1] = mem_addr >> 8;
				msgs[0].addr = mpu_addr & I2C_MASK_FLAG;
				msgs[0].flags = 0;
				msgs[0].buf = buf;
				msgs[0].len = 2;
				msgs[0].timing = I2C_TRANS_TIMING;
				msgs[0].ext_flag = 0;

	          	ret = i2c_transfer(st->client->adapter, msgs, 1);
	              if (ret!=1)
	              {
	                       printk("invn: i2c transfer error ret:%d, Line %d\n", ret, __LINE__);
	                       goto read_error;
	              }

              buf[0] = REG_MEM_START_ADDR;
              buf[1] = mem_addr & 0xFF;
				msgs[0].addr = mpu_addr & I2C_MASK_FLAG;
				msgs[0].flags = 0;
				msgs[0].buf = buf;
				msgs[0].len = 2;
				msgs[0].timing = I2C_TRANS_TIMING;
				msgs[0].ext_flag = 0;

	          	ret = i2c_transfer(st->client->adapter, msgs, 1);
	              if (ret!=1)
	              {
	                       printk("invn: i2c transfer error ret:%d,  Line %d\n", ret,  __LINE__);
	                       goto read_error;
	              }


              buf[0] = REG_MEM_R_W;
				msgs[0].addr = mpu_addr & I2C_MASK_FLAG;
				msgs[0].flags = 0;
				msgs[0].buf = buf;
				msgs[0].len = 1;
				msgs[0].timing = I2C_TRANS_TIMING;
				msgs[0].ext_flag = 0;

	          	ret = i2c_transfer(st->client->adapter, msgs, 1);
	              if (ret!=1)
	              {
	                       printk("invn: i2c transfer error ret:%d, Line %d\n", ret, __LINE__);
	                       goto read_error;
	              }

				memset(buf, 0, sizeof(buf));
				msgs[0].addr = mpu_addr & I2C_MASK_FLAG;
				msgs[0].flags = I2C_M_RD;
				msgs[0].buf = buf;
				msgs[0].len = readbytes;
				msgs[0].timing = I2C_TRANS_TIMING;
				msgs[0].ext_flag = 0;

				ret = i2c_transfer(st->client->adapter, msgs, 1);
				  if (ret!=1)
				  {
						   printk("invn: i2c transfer error ret:%d, read_bytes:%d, Line %d\n", ret, readbytes, __LINE__);
						   goto read_error;
				  }

              length -= readbytes;
              mem_addr += readbytes;
              memcpy(data, buf, readbytes);
              data += readbytes;
    }

    mutex_unlock(&st->inv_rw_mutex);
    return 0;
read_error:
	mutex_unlock(&st->inv_rw_mutex);
	return -1;
}

#if 1//SUSPEND_DISABLE_IRQ
#if 1//def CONFIG_HAS_EARLYSUSPEND
static void sensor_early_suspend(struct early_suspend *h)
{
	struct inv_mpu_state *st = container_of(h, struct inv_mpu_state, sensor_early_suspend);
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	int irq_disable;
	int acc_wake;
	
	mutex_lock(&st->inv_suspend_mutex);
	irq_disable = get_irq_state();
	acc_wake = atomic_read(&st->acc_wake);
	INV_INFO("irq state:%s, acc_wake:%s\n",(st->irq_should_disable==DISABLE)?"disable":"enable", acc_wake==DISABLE?"disable":"enable");

	if( acc_wake==DISABLE ) {
		st->irq_should_disable = DISABLE;
		disable_irq_nosync(st->irq);
	} else {

	}
	mutex_unlock(&st->inv_suspend_mutex);
}

static void sensor_late_resume(struct early_suspend *h)
{
	struct inv_mpu_state *st = container_of(h, struct inv_mpu_state, sensor_early_suspend);
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	int acc_wake;

	mutex_lock(&st->inv_suspend_mutex);
	acc_wake = atomic_read(&st->acc_wake);

	INV_INFO("irq state:%s, acc_wake:%s\n",(st->irq_should_disable==DISABLE)?"disable":"enable", acc_wake==DISABLE?"disable":"enable");

	if( st->irq_should_disable==DISABLE ) {
		st->irq_should_disable = ENABLE;
		if( acc_wake==DISABLE )
			enable_irq(st->irq);
	} else {

	}

	mutex_unlock(&st->inv_suspend_mutex);
}
#endif
#endif
/*
 *  inv_mpu_probe() - probe function.
 */
static int inv_mpu_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	struct inv_mpu_state *st;
	struct iio_dev *indio_dev;
	int result, irq, retv;

#ifdef CONFIG_DTS_INV_MPU_IIO
	enable_irq_wake(client->irq);
#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		result = -ENOSYS;
		pr_err("I2c function error\n");
		goto out_no_free;
	}
#ifdef LINUX_KERNEL_3_10
	indio_dev = iio_device_alloc(sizeof(*st));
#else
	indio_dev = iio_allocate_device(sizeof(*st));
#endif
	if (indio_dev == NULL) {
		pr_err("memory allocation failed\n");
		result =  -ENOMEM;
		goto out_no_free;
	}
	st = iio_priv(indio_dev);
	st->client = client;
	st->sl_handle = client->adapter;
	st->i2c_addr = client->addr & 0xff;
#if SUSPEND_DISABLE_IRQ
	st->irq_should_disable = ENABLE;
#endif
	/*
	 * its safer.
	 */
	st->i2c_dis = 0;
	st->client->addr &= 0xff;
	st->client->timing = I2C_TRANS_TIMING;
	st->client->ext_flag = 0x00;
	atomic_set(&st->acc_wake, DISABLE);

	mutex_init(&st->inv_rw_mutex);
	mutex_init(&st->inv_suspend_mutex);

#ifdef CONFIG_DTS_INV_MPU_IIO
	result = invensense_mpu_parse_dt(&client->dev, &st->plat_data);
	if (result)
		goto out_free;

	/*Power on device.*/
	if (st->plat_data.power_on) {
		result = st->plat_data.power_on(&st->plat_data);
		if (result < 0) {
			dev_err(&client->dev,
					"power_on failed: %d\n", result);
			return result;
		}
	pr_info("%s: power on here.\n", __func__);
	}
	pr_info("%s: power on.\n", __func__);

	msleep(100);
#else
	st->plat_data =
		*(struct mpu_platform_data *)dev_get_platdata(&client->dev);
#endif
	/* power is turned on inside check chip type*/
	result = inv_check_chip_type(indio_dev, id->name);
	//result = inv_check_chip_type(indio_dev, "icm20645");
	//result = inv_check_chip_type(indio_dev, "icm10340");
	if (result)
		goto out_free;

	/* Make state variables available to all _show and _store functions. */
	i2c_set_clientdata(client, indio_dev);
	indio_dev->dev.parent = &client->dev;
	indio_dev->name = id->name;
	//indio_dev->name="icm20645";
	//indio_dev->name="icm10340";

	result = inv_mpu_configure_ring(indio_dev);
	if (result) {
		pr_err("configure ring buffer fail\n");
		goto out_free;
	}
	result = iio_buffer_register(indio_dev, indio_dev->channels,
					indio_dev->num_channels);
	if (result) {
		pr_err("ring buffer register fail\n");
		goto out_unreg_ring;
	}
	INV_I2C_SETIRQ(IRQ_MPU, client->irq);
	result = iio_device_register(indio_dev);
	if (result) {
		pr_err("IIO device register fail\n");
		goto out_remove_ring;
	}

	result = inv_create_dmp_sysfs(indio_dev);
	if (result) {
		pr_err("create dmp sysfs failed\n");
		goto out_unreg_iio;
	}
	sema_init(&st->suspend_resume_sema, 1);

#if SUSPEND_DISABLE_IRQ
#ifdef CONFIG_HAS_EARLYSUSPEND
	st->sensor_early_suspend.suspend = sensor_early_suspend;
	st->sensor_early_suspend.resume = sensor_late_resume;
	st->sensor_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    register_early_suspend(&st->sensor_early_suspend);
#endif
#endif

	INV_INFO("%s is ready to go!\n", indio_dev->name);

	return 0;
out_unreg_iio:
	iio_device_unregister(indio_dev);
out_remove_ring:
	iio_buffer_unregister(indio_dev);
out_unreg_ring:
	inv_mpu_unconfigure_ring(indio_dev);
out_free:
#ifdef LINUX_KERNEL_3_10
	iio_device_free(indio_dev);
#else
	iio_free_device(indio_dev);
#endif
out_no_free:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, result);

	return -EIO;
}

static void inv_mpu_shutdown(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct inv_mpu_state *st = iio_priv(indio_dev);
	int result;

	mutex_lock(&indio_dev->mlock);
	dev_dbg(&client->adapter->dev, "Shutting down %s...\n", st->hw->name);

	/* reset to make sure previous state are not there */
	result = inv_plat_single_write(st, REG_PWR_MGMT_1, BIT_H_RESET);
	if (result)
		dev_err(&client->adapter->dev, "Failed to reset %s\n",
			st->hw->name);
	msleep(POWER_UP_TIME);
	/* turn off power to ensure gyro engine is off */
	result = inv_set_power(st, false);
	if (result)
		dev_err(&client->adapter->dev, "Failed to turn off %s\n",
			st->hw->name);
	mutex_unlock(&indio_dev->mlock);
}

/*
 *  inv_mpu_remove() - remove function.
 */
static int inv_mpu_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	iio_device_unregister(indio_dev);
	iio_buffer_unregister(indio_dev);
	inv_mpu_unconfigure_ring(indio_dev);
#ifdef LINUX_KERNEL_3_10
	iio_device_free(indio_dev);
#else
	iio_free_device(indio_dev);
#endif

	dev_info(&client->adapter->dev, "inv-mpu-iio module removed.\n");

	return 0;
}


#ifdef CONFIG_PM
/*
 * inv_mpu_resume(): resume method for this driver.
 *    This method can be modified according to the request of different
 *    customers. It basically undo everything suspend_noirq is doing
 *    and recover the chip to what it was before suspend.
 */
static int inv_mpu_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct inv_mpu_state *st = iio_priv(indio_dev);
	u32 step=0;

	/* add code according to different request Start */
	pr_debug("%s inv_mpu_resume\n", st->hw->name);

	if (st->chip_config.dmp_on) {
		inv_switch_power_in_lp(st, true); //--yd
		if (st->batch.on) {
//--yd 			inv_switch_power_in_lp(st, true);
			write_be32_to_mem(st, st->batch.counter, BM_BATCH_THLD);
			inv_plat_single_write(st, REG_INT_ENABLE_2,
							BIT_FIFO_OVERFLOW_EN_0);
//--yd 			inv_switch_power_in_lp(st, false);
		}
		inv_write_cntl(st, GYRO_AUTO_OFF_EN, false, MOTION_EVENT_CTL);
		inv_switch_power_in_lp(st, false); //--yd
	} else {
		inv_switch_power_in_lp(st, true);
	}
	/* add code according to different request End */
//	up(&st->suspend_resume_sema);

	//add for meizu stepcounter
	if( st->step_counter_l_on||st->step_counter_wake_l_on ) {
	
		st->last_run_time = get_time_ns();
		inv_switch_power_in_lp(st, true);
		inv_get_pedometer_steps(st, &step);
		inv_switch_power_in_lp(st, false);
		inv_send_steps(st, step, st->last_run_time);
		
		INV_INFO("dpm  stepcounter prev_steps:%u, steps:%u, stamp:%llu\n",st->prev_steps, step, st->last_run_time);
		st->prev_steps = step;
	}

	return 0;
}

/*
 * inv_mpu_suspend(): suspend method for this driver.
 *    This method can be modified according to the request of different
 *    customers. If customer want some events, such as SMD to wake up the CPU,
 *    then data interrupt should be disabled in this interrupt to avoid
 *    unnecessary interrupts. If customer want pedometer running while CPU is
 *    asleep, then pedometer should be turned on while pedometer interrupt
 *    should be turned off.
 */
static int inv_mpu_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));
	struct inv_mpu_state *st = iio_priv(indio_dev);

	/* add code according to different request Start */
	INV_INFO("%s dmp_on:%u,wake_on%u,batch_on:%d inv_mpu_suspend\n",
			st->hw->name,st->chip_config.dmp_on,st->chip_config.wake_on,st->batch.on);

//	mutex_lock(&indio_dev->mlock);
	mutex_lock(&st->inv_suspend_mutex);
	if (st->chip_config.dmp_on) {
		inv_switch_power_in_lp(st, true); //--yd
		if (!st->chip_config.wake_on) {
			if (st->batch.on) {
//--yd 				inv_switch_power_in_lp(st, true);
				write_be32_to_mem(st, INT_MAX, BM_BATCH_THLD);
				inv_plat_single_write(st, REG_INT_ENABLE_2, 0);
//--yd 				inv_switch_power_in_lp(st, false);
			}
		}
		inv_write_cntl(st, GYRO_AUTO_OFF_EN, true, MOTION_EVENT_CTL);
		inv_switch_power_in_lp(st, false); //--yd
	} else {
		/* in non DMP case, just turn off the power */
		inv_set_power(st, false);
	}
//	mutex_unlock(&indio_dev->mlock);
	mutex_unlock(&st->inv_suspend_mutex);
	/* add code according to different request End */
//	down(&st->suspend_resume_sema);

	return 0;
}

static const struct dev_pm_ops inv_mpu_pmops = {
	.suspend       = inv_mpu_suspend,
	.resume        = inv_mpu_resume,
};
#define INV_MPU_PMOPS (&inv_mpu_pmops)
#else
#define INV_MPU_PMOPS NULL
#endif /* CONFIG_PM */

static const u16 normal_i2c[] = { I2C_CLIENT_END };
/* device id table is used to identify what device can be
 * supported by this driver
 */
static const struct i2c_device_id inv_mpu_id[] = {
#ifdef CONFIG_DTS_INV_MPU_IIO
	{"mpu6515", ICM20645},
#else
	{"mpu7400", ICM20645},
#endif
	{"icm20645", ICM20645},
	{"icm10340", ICM10340},
	{}
};

MODULE_DEVICE_TABLE(i2c, inv_mpu_id);

static struct i2c_driver inv_mpu_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		=	inv_mpu_probe,
	.remove		=	inv_mpu_remove,
	.shutdown	=	inv_mpu_shutdown,
	.id_table	=	inv_mpu_id,
	.driver = {
		.owner	=	THIS_MODULE,
		.name	=	"inv-mpu-iio",
		.pm     =       INV_MPU_PMOPS,
	},
	.address_list = normal_i2c,
};
#if 0 //def LINUX_KERNEL_3_10
module_i2c_driver(inv_mpu_driver);
#else

static struct mpu_platform_data gyro_platform_data = {
	.int_config  = 0x00,
	.level_shifter = 0,
	.orientation = {
	   0,  -1,  0,
		-1,  0,  0,
		0,  0,  -1 },
	.sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS,
	.sec_slave_id   = COMPASS_ID_AK09911,
	.secondary_i2c_addr = 0x0C,
	.secondary_orientation = {
	   -1,  0, 0,
	   0,  -1, 0,
	   0,  0, 1 },
};

static struct i2c_board_info __initdata single_chip_board_info[] = {
	{
		I2C_BOARD_INFO("icm20645", 0x68),
		.irq = 9,
		.platform_data = &gyro_platform_data,
	},
};

// postcore_initcall

static int __init inv_mpu_board_init(void) 
{
	int result;

	result = i2c_register_board_info(3, single_chip_board_info, 1);
	INV_INFO("board info result:%d\n",result);
	return 0;
}
static void __exit inv_mpu_board_exit(void)
{
}

static int __init inv_mpu_init(void)
{
	int result;

	result = i2c_add_driver(&inv_mpu_driver);
	if (result) {
		pr_err("failed\n");
		return result;
	}
	INV_INFO("driver info success\n");
	return 0;
}

static void __exit inv_mpu_exit(void)
{
	i2c_del_driver(&inv_mpu_driver);
}

postcore_initcall(inv_mpu_board_init);
module_exit(inv_mpu_board_exit);
module_init(inv_mpu_init);
module_exit(inv_mpu_exit);
#endif

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Invensense device driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("inv-mpu-iio");
