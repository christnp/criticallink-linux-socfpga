/*
 * rtc-ab18xx.c - RTC driver for some mostly-compatible I2C chips.
 *
 *  Copyright (C) 2013 Tim Iskander
 *  Based on rtc-ds1307.c
 *  Copyright (C) 2005 James Chapman (ds1337 core)
 *  Copyright (C) 2006 David Brownell
 *  Copyright (C) 2009 Matthias Fuchs (rx8025 support)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/string.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/of.h>


#define DRIVER_NAME "rtc-ab18xx"

/*
 * We can't determine type by probing, but if we expect pre-Linux code
 * to have set the chip up as a clock (turning on the oscillator and
 * setting the date and time), Linux can ignore the non-clock features.
 * That's a natural job for a factory or repair bench.
 */
enum ab_type {
	ab_1803,
	last_ab_type /* always last */
};


/* RTC registers don't differ much, except for the century flag */
#define AB18XX_REG_HDTHS	0x00	/* 00-99 */
#define AB18XX_REG_SECS		0x01	/* 00-59 */
#define AB18XX_REG_MIN		0x02	/* 00-59 */
#define AB18XX_REG_HOUR		0x03	/* 00-23, or 1-12{am,pm} */
#	define AB18XX_BIT_12HR		0x40	/* in REG_HOUR */
#	define AB18XX_BIT_PM		0x20	/* in REG_HOUR */
#	define AB18XX_BIT_CENTURY_EN	0x80	/* in REG_HOUR */
#	define AB18XX_BIT_CENTURY	0x40	/* in REG_HOUR */
#define AB18XX_REG_WDAY		0x07	/* 01-07 */
#define AB18XX_REG_MDAY		0x04	/* 01-31 */
#define AB18XX_REG_MONTH	0x05	/* 01-12 */
#define AB18XX_REG_YEAR		0x06	/* 00-99 */
#define AB18XX_REG_ALM_HDTHS	0x08	/* 00-99 */

/*
 * Other registers (control, status, alarms, trickle charge, NVRAM, etc)
 * start at 7, and they differ a LOT. Only control and status matter for
 * basic RTC date and time functionality; be careful using them.
 */
#define AB18XX_REG_STATUS		0x0F
#	define AB18XX_BIT_EX1		0x01
#	define AB18XX_BIT_EX2		0x02
#	define AB18XX_BIT_ALM		0X04
#	define AB18XX_BIT_TIM		0x08
#	define AB18XX_BIT_BL		0x10
#	define AB18XX_BIT_WDT		0x20
#	define AB18XX_BIT_BAT		0x40
#	define AB18XX_BIT_CB		0x80
#define AB18XX_REG_CTRL1		0x10
#	define AB18XX_BIT_WRTC		0x01
#	define AB18XX_BIT_PWR2		0x02
#	define AB18XX_BIT_ARST		0x04
#	define AB18XX_BIT_RSP		0x08
#	define AB18XX_BIT_OUT		0x10
#	define AB18XX_BIT_OUTB		0x20
#	define AB18XX_BIT_12_24		0x40
#	define AB18XX_BIT_STOP		0x80
#define AB18XX_REG_CTRL2		0x11
#	define AB18XX_BIT_OUT1S		0x03
#	define AB18XX_BIT_OUT2S		0x16
#	define AB18XX_BIT_RS1E		0x20
#	define AB18XX_BIT_OUTPP		0x80
#define AB18XX_REG_INTMASK		0x12
#	define AB18XX_BIT_EX1E		0x01
#	define AB18XX_BIT_EX2E		0x02
#	define AB18XX_BIT_AIE		0X04
#	define AB18XX_BIT_TIE		0x08
#	define AB18XX_BIT_BLIE		0x10
#	define AB18XX_BIT_IM		0x60
#	define AB18XX_BIT_CEB		0x80


#define NUM_REGS_MAX 256
#define NUM_DATE_TIME_REGS 8
struct ab18xx {
	u8			offset; /* register's offset */
	u8			regs[NUM_REGS_MAX];
	u16			nvram_offset;
	struct bin_attribute	*nvram;
	enum ab_type		type;
	unsigned long		flags;
#define HAS_NVRAM	0		/* bit 0 == sysfs file active */
#define HAS_ALARM	1		/* bit 1 == irq claimed */
	struct i2c_client	*client;
	struct rtc_device	*rtc;
	struct work_struct	work;
	s32 (*read_block_data)(const struct i2c_client *client, u8 command,
			       u8 length, u8 *values);
	s32 (*write_block_data)(const struct i2c_client *client, u8 command,
				u8 length, const u8 *values);
};

struct chip_desc {
	unsigned		alarm:1;
	u16			nvram_offset;
	u16			nvram_size;
	u16			trickle_charger_reg;
};

static const struct chip_desc chips[last_ab_type] = {
	[ab_1803] = {
		.alarm		= 1,
		.nvram_offset	= 40,
		.nvram_size	= 0xc0,
		.trickle_charger_reg = 0x20,
	},
};


/*----------------------------------------------------------------------*/

#define BLOCK_DATA_MAX_TRIES 10

static s32 ab18xx_read_block_data_once(const struct i2c_client *client,
				       u8 command, u8 length, u8 *values)
{
	s32 i, data;

	for (i = 0; i < length; i++) {
		data = i2c_smbus_read_byte_data(client, command + i);
		if (data < 0)
			return data;
		values[i] = data;
	}
	return i;
}

static s32 ab18xx_read_block_data(const struct i2c_client *client, u8 command,
				  u8 length, u8 *values)
{
	u8 oldvalues[I2C_SMBUS_BLOCK_MAX];
	s32 ret;
	int tries = 0;

	dev_dbg(&client->dev, "ab18xx_read_block_data (length=%d)\n", length);
	ret = ab18xx_read_block_data_once(client, command, length, values);
	if (ret < 0)
		return ret;
	do {
		if (++tries > BLOCK_DATA_MAX_TRIES) {
			dev_err(&client->dev,
				"ab18xx_read_block_data failed\n");
			return -EIO;
		}
		memcpy(oldvalues, values, length);
		ret = ab18xx_read_block_data_once(client, command, length,
						  values);
		if (ret < 0)
			return ret;
	} while (memcmp(oldvalues, values, length));
	return length;
}

static s32 ab18xx_write_block_data(const struct i2c_client *client, u8 command,
				   u8 length, const u8 *values)
{
	u8 currvalues[I2C_SMBUS_BLOCK_MAX];
	int tries = 0;

	dev_dbg(&client->dev, "ab18xx_write_block_data (length=%d)\n", length);
	do {
		s32 i, ret;

		if (++tries > BLOCK_DATA_MAX_TRIES) {
			dev_err(&client->dev,
				"ab18xx_write_block_data failed\n");
			return -EIO;
		}
		for (i = 0; i < length; i++) {
			ret = i2c_smbus_write_byte_data(client, command + i,
							values[i]);
			if (ret < 0)
				return ret;
		}
		ret = ab18xx_read_block_data_once(client, command, length,
						  currvalues);
		if (ret < 0)
			return ret;
	} while (memcmp(currvalues, values, length));
	return length;
}

/*----------------------------------------------------------------------*/

/*
 * The IRQ logic includes a "real" handler running in IRQ context just
 * long enough to schedule this workqueue entry.   We need a task context
 * to talk to the RTC, since I2C I/O calls require that; and disable the
 * IRQ until we clear its status on the chip, so that this handler can
 * work with any type of triggering (not just falling edge).
 *
 * The ds1337 and ds1339 both have two alarms, but we only use the first
 * one (with a "seconds" field).  For ds1337 we expect nINTA is our alarm
 * signal; ds1339 chips have only one alarm signal.
 */
static void ab18xx_work(struct work_struct *work)
{
	struct ab18xx		*ab18xx;
	struct i2c_client	*client;
	struct mutex		*lock;
	int			stat;

	ab18xx = container_of(work, struct ab18xx, work);
	client = ab18xx->client;
	lock = &ab18xx->rtc->ops_lock;

	mutex_lock(lock);
	stat = i2c_smbus_read_byte_data(client, AB18XX_REG_STATUS);
	if (stat < 0)
		goto out;

	if (stat & AB18XX_BIT_ALM) {
		stat &= ~AB18XX_BIT_ALM;
		i2c_smbus_write_byte_data(client, AB18XX_REG_STATUS, stat);

		rtc_update_irq(ab18xx->rtc, 1, RTC_AF | RTC_IRQF);
	}

out:
	if (test_bit(HAS_ALARM, &ab18xx->flags))
		enable_irq(client->irq);
	mutex_unlock(lock);
}

static irqreturn_t ab18xx_irq(int irq, void *dev_id)
{
	struct i2c_client	*client = dev_id;
	struct ab18xx		*ab18xx = i2c_get_clientdata(client);

	disable_irq_nosync(irq);
	schedule_work(&ab18xx->work);
	return IRQ_HANDLED;
}

/*----------------------------------------------------------------------*/

static int ab18xx_get_time(struct device *dev, struct rtc_time *t)
{
	struct ab18xx	*ab18xx = dev_get_drvdata(dev);
	int		tmp;

	/* read the RTC date and time registers all at once */
	tmp = ab18xx->read_block_data(ab18xx->client,
		ab18xx->offset, NUM_DATE_TIME_REGS, ab18xx->regs);
	if (tmp != NUM_DATE_TIME_REGS) {
		dev_err(dev, "%s error %d\n", "read", tmp);
		return -EIO;
	}

	dev_dbg(dev, "%s: %7ph\n", "read", ab18xx->regs);

	tmp = bcd2bin(ab18xx->regs[AB18XX_REG_HDTHS]);
	t->tm_sec = bcd2bin(ab18xx->regs[AB18XX_REG_SECS] & 0x7f);
	/* If more than 1/2 way through the second, add 1 */
	if(tmp > 49)
		t->tm_sec += 1;
	t->tm_min = bcd2bin(ab18xx->regs[AB18XX_REG_MIN] & 0x7f);
	tmp = ab18xx->regs[AB18XX_REG_HOUR] & 0x3f;
	t->tm_hour = bcd2bin(tmp);
	t->tm_wday = bcd2bin(ab18xx->regs[AB18XX_REG_WDAY] & 0x07);
	t->tm_mday = bcd2bin(ab18xx->regs[AB18XX_REG_MDAY] & 0x3f);
	tmp = ab18xx->regs[AB18XX_REG_MONTH] & 0x1f;
	t->tm_mon = bcd2bin(tmp);

	/* assume 20YY not 19YY */
	t->tm_year = bcd2bin(ab18xx->regs[AB18XX_REG_YEAR]) + 100;

	dev_dbg(dev, "%s secs=%d, mins=%d, "
		"hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
		"read", t->tm_sec, t->tm_min,
		t->tm_hour, t->tm_mday,
		t->tm_mon, t->tm_year, t->tm_wday);

	/* initial clock setting can be undefined */
	return rtc_valid_tm(t);
}

static int ab18xx_set_time(struct device *dev, struct rtc_time *t)
{
	struct ab18xx	*ab18xx = dev_get_drvdata(dev);
	int		result;
	int		tmp;
	u8		*buf = ab18xx->regs;

	dev_dbg(dev, "%s secs=%d, mins=%d, "
		"hours=%d, mday=%d, mon=%d, year=%d, wday=%d\n",
		"write", t->tm_sec, t->tm_min,
		t->tm_hour, t->tm_mday,
		t->tm_mon, t->tm_year, t->tm_wday);

	buf[AB18XX_REG_SECS] = bin2bcd(t->tm_sec);
	buf[AB18XX_REG_MIN] = bin2bcd(t->tm_min);
	buf[AB18XX_REG_HOUR] = bin2bcd(t->tm_hour);
	buf[AB18XX_REG_WDAY] = bin2bcd(t->tm_wday);
	buf[AB18XX_REG_MDAY] = bin2bcd(t->tm_mday);
	buf[AB18XX_REG_MONTH] = bin2bcd(t->tm_mon);

	/* assume 20YY not 19YY */
	tmp = t->tm_year - 100;
	buf[AB18XX_REG_YEAR] = bin2bcd(tmp);

	dev_dbg(dev, "%s: %7ph\n", "write", buf);

	result = ab18xx->write_block_data(ab18xx->client,
		ab18xx->offset, NUM_DATE_TIME_REGS, buf);
	if (result < 0) {
		dev_err(dev, "%s error %d\n", "write", result);
		return result;
	}
	return 0;
}

static int ds1337_read_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct i2c_client       *client = to_i2c_client(dev);
	struct ab18xx		*ab18xx = i2c_get_clientdata(client);
	u8			regs[16]; 
	int			ret;

	if (!test_bit(HAS_ALARM, &ab18xx->flags))
		return -EINVAL;

	/* read all ALARM, status and control, and status registers at once */
	ret = ab18xx->read_block_data(client,
			AB18XX_REG_ALM_HDTHS, 11, regs);
	if (ret != 11) {
		dev_err(dev, "%s error %d\n", "alarm read", ret);
		return -EIO;
	}

	dev_dbg(dev, "%s: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n",
			"alarm read",
			regs[0], regs[1],
			regs[2], regs[3],
			regs[4], regs[5],
			regs[6], regs[7],
			regs[8], regs[9],
			regs[10]);

	/*
	 * report alarm time (ALARM1); assume 24 hour and day-of-month modes,
	 * and that all four fields are checked matches
	 */
	t->time.tm_sec = bcd2bin(regs[1] & 0x7f);
	t->time.tm_min = bcd2bin(regs[2] & 0x7f);
	t->time.tm_hour = bcd2bin(regs[3] & 0x3f);
	t->time.tm_mday = bcd2bin(regs[4] & 0x3f);
	t->time.tm_mon = -1;
	t->time.tm_year = -1;
	t->time.tm_wday = -1;
	t->time.tm_yday = -1;
	t->time.tm_isdst = -1;

	/* ... and status */
	t->enabled = !!(regs[11] & AB18XX_BIT_AIE);
	t->pending = !!(regs[7] & AB18XX_BIT_ALM);

	dev_dbg(dev, "%s secs=%d, mins=%d, "
		"hours=%d, mday=%d, enabled=%d, pending=%d\n",
		"alarm read", t->time.tm_sec, t->time.tm_min,
		t->time.tm_hour, t->time.tm_mday,
		t->enabled, t->pending);

	return 0;
}

static int ds1337_set_alarm(struct device *dev, struct rtc_wkalrm *t)
{
	struct i2c_client	*client = to_i2c_client(dev);
	struct ab18xx		*ab18xx = i2c_get_clientdata(client);
	unsigned char		*buf = ab18xx->regs;
	u8			control, status;
	u8			regs[16]; 
	int			ret;

	if (!test_bit(HAS_ALARM, &ab18xx->flags))
		return -EINVAL;

	dev_dbg(dev, "%s secs=%d, mins=%d, "
		"hours=%d, mday=%d, enabled=%d, pending=%d\n",
		"alarm set", t->time.tm_sec, t->time.tm_min,
		t->time.tm_hour, t->time.tm_mday,
		t->enabled, t->pending);

	/* read current status of both alarms and the chip */
	ret = ab18xx->read_block_data(client,
			AB18XX_REG_ALM_HDTHS, 11, regs);
	if (ret != 11) {
		dev_err(dev, "%s error %d\n", "alarm write", ret);
		return -EIO;
	}
	control = ab18xx->regs[11];
	status = ab18xx->regs[7];

	dev_dbg(dev, "%s: %02x %02x %02x %02x, %02x %02x %02x, %02x %02x\n",
			"alarm set (old status)",
			regs[0], regs[1],
			regs[2], regs[3],
			regs[4], regs[5],
			regs[6], control, status);

	/* set ALARM, using 24 hour and day-of-month modes */
	buf[0] = 0;
	buf[1] = bin2bcd(t->time.tm_sec);
	buf[2] = bin2bcd(t->time.tm_min);
	buf[3] = bin2bcd(t->time.tm_hour);
	buf[4] = bin2bcd(t->time.tm_mday);
	buf[5] = bin2bcd(t->time.tm_mon);
	buf[6] = bin2bcd(t->time.tm_wday);

	/* optionally enable ALARM */
	buf[11] = control & ~(AB18XX_BIT_AIE);
	if (t->enabled) {
		dev_dbg(dev, "alarm IRQ armed\n");
		buf[11] |= AB18XX_BIT_AIE;
	}

	ret = ab18xx->write_block_data(client,
			AB18XX_REG_ALM_HDTHS, 7, regs);
	if (ret < 0) {
		dev_err(dev, "can't set alarm time\n");
		return ret;
	}
	ret = i2c_smbus_write_byte_data(client, AB18XX_REG_INTMASK, buf[11]);
	if (ret < 0) {
		dev_err(dev, "can't enable alarm\n");
		return ret;
	}

	return 0;
}

static int ab18xx_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct i2c_client	*client = to_i2c_client(dev);
	struct ab18xx		*ab18xx = i2c_get_clientdata(client);
	int			ret;

	if (!test_bit(HAS_ALARM, &ab18xx->flags))
		return -ENOTTY;

	ret = i2c_smbus_read_byte_data(client, AB18XX_REG_INTMASK);
	if (ret < 0)
		return ret;

	if (enabled)
		ret |= AB18XX_BIT_AIE;
	else
		ret &= ~AB18XX_BIT_AIE;

	ret = i2c_smbus_write_byte_data(client, AB18XX_REG_INTMASK, ret);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct rtc_class_ops ab18xx_rtc_ops = {
	.read_time	= ab18xx_get_time,
	.set_time	= ab18xx_set_time,
	.read_alarm	= ds1337_read_alarm,
	.set_alarm	= ds1337_set_alarm,
	.alarm_irq_enable = ab18xx_alarm_irq_enable,
};

/*----------------------------------------------------------------------*/

static ssize_t
ab18xx_nvram_read(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr,
		char *buf, loff_t off, size_t count)
{
	struct i2c_client	*client;
	struct ab18xx		*ab18xx;
	int			result;

	client = kobj_to_i2c_client(kobj);
	ab18xx = i2c_get_clientdata(client);

	if (unlikely(off >= ab18xx->nvram->size))
		return 0;
	if ((off + count) > ab18xx->nvram->size)
		count = ab18xx->nvram->size - off;
	if (unlikely(!count))
		return count;

	result = ab18xx->read_block_data(client, ab18xx->nvram_offset + off,
								count, buf);
	if (result < 0)
		dev_err(&client->dev, "%s error %d\n", "nvram read", result);
	return result;
}

static ssize_t
ab18xx_nvram_write(struct file *filp, struct kobject *kobj,
		struct bin_attribute *attr,
		char *buf, loff_t off, size_t count)
{
	struct i2c_client	*client;
	struct ab18xx		*ab18xx;
	int			result;

	client = kobj_to_i2c_client(kobj);
	ab18xx = i2c_get_clientdata(client);

	if (unlikely(off >= ab18xx->nvram->size))
		return -EFBIG;
	if ((off + count) > ab18xx->nvram->size)
		count = ab18xx->nvram->size - off;
	if (unlikely(!count))
		return count;

	result = ab18xx->write_block_data(client, ab18xx->nvram_offset + off,
								count, buf);
	if (result < 0) {
		dev_err(&client->dev, "%s error %d\n", "nvram write", result);
		return result;
	}
	return count;
}

/*----------------------------------------------------------------------*/

static int ab18xx_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ab18xx		*ab18xx;
	int			err = -ENODEV;
	int			tmp;
	const struct chip_desc	*chip = &chips[id->driver_data];
	struct i2c_adapter	*adapter = to_i2c_adapter(client->dev.parent);
	int			want_irq = false;
	unsigned char		*buf;
	/* struct ab18xx_platform_data *pdata = client->dev.platform_data; */

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA)
	    && !i2c_check_functionality(adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		return -EIO;

	ab18xx = kzalloc(sizeof(struct ab18xx), GFP_KERNEL);
	if (!ab18xx)
		return -ENOMEM;

	i2c_set_clientdata(client, ab18xx);

	ab18xx->client	= client;
	ab18xx->type	= id->driver_data;

	buf = ab18xx->regs;
	if (i2c_check_functionality(adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
		ab18xx->read_block_data = i2c_smbus_read_i2c_block_data;
		ab18xx->write_block_data = i2c_smbus_write_i2c_block_data;
	} else {
		ab18xx->read_block_data = ab18xx_read_block_data;
		ab18xx->write_block_data = ab18xx_write_block_data;
	}

	switch (ab18xx->type) {
	case ab_1803:
		/* get registers that the "rtc" read below won't read... */
		tmp = ab18xx->read_block_data(ab18xx->client,
				AB18XX_REG_CTRL1, 3, buf);
		if (tmp != 3) {
			pr_debug("read error %d\n", tmp);
			err = -EIO;
			goto exit_free;
		}

		/* oscillator off?  turn it on, so clock can tick. */
		if (ab18xx->regs[0] & AB18XX_BIT_STOP)
			ab18xx->regs[0] &= ~AB18XX_BIT_STOP;

		/*
		 * Using IRQ?  Disable the square wave and both alarms.
		 */
		if (ab18xx->client->irq > 0 && chip->alarm) {
			INIT_WORK(&ab18xx->work, ab18xx_work);

			ab18xx->regs[2] |= AB18XX_BIT_AIE;

			want_irq = true;
		}

		i2c_smbus_write_byte_data(client, AB18XX_REG_CTRL1,
							ab18xx->regs[0]);
		break;

	default:
		break;
	}

	/* read RTC registers */
	tmp = ab18xx->read_block_data(ab18xx->client, ab18xx->offset, NUM_DATE_TIME_REGS, buf);
	if (tmp != NUM_DATE_TIME_REGS) {
		pr_debug("read error %d\n", tmp);
		err = -EIO;
		goto exit_free;
	}
	ab18xx->rtc = rtc_device_register(client->name, &client->dev,
				&ab18xx_rtc_ops, THIS_MODULE);
	if (IS_ERR(ab18xx->rtc)) {
		err = PTR_ERR(ab18xx->rtc);
		dev_err(&client->dev,
			"unable to register the class device\n");
		goto exit_free;
	}

	if (chip->nvram_size) {
		ab18xx->nvram = kzalloc(sizeof(struct bin_attribute),
							GFP_KERNEL);

		if (!ab18xx->nvram) {
			err = -ENOMEM;
			goto exit_nvram;
		}
		ab18xx->nvram->attr.name = "nvram";
		ab18xx->nvram->attr.mode = S_IRUGO | S_IWUSR;
		sysfs_bin_attr_init(ab18xx->nvram);
		ab18xx->nvram->read = ab18xx_nvram_read,
		ab18xx->nvram->write = ab18xx_nvram_write,
		ab18xx->nvram->size = chip->nvram_size;
		ab18xx->nvram_offset = chip->nvram_offset;
		err = sysfs_create_bin_file(&client->dev.kobj, ab18xx->nvram);
		if (err) {
			kfree(ab18xx->nvram);
			goto exit_nvram;
		}
		set_bit(HAS_NVRAM, &ab18xx->flags);
		dev_info(&client->dev, "%zu bytes nvram\n", ab18xx->nvram->size);
	}

	return 0;

exit_nvram:
	rtc_device_unregister(ab18xx->rtc);
exit_free:
	kfree(ab18xx);
	return err;
}

static int ab18xx_remove(struct i2c_client *client)
{
	struct ab18xx *ab18xx = i2c_get_clientdata(client);

	if (test_and_clear_bit(HAS_ALARM, &ab18xx->flags)) {
		free_irq(client->irq, client);
		cancel_work_sync(&ab18xx->work);
	}

	if (test_and_clear_bit(HAS_NVRAM, &ab18xx->flags)) {
		sysfs_remove_bin_file(&client->dev.kobj, ab18xx->nvram);
		kfree(ab18xx->nvram);
	}

	rtc_device_unregister(ab18xx->rtc);
	kfree(ab18xx);
	return 0;
}

static const struct i2c_device_id ab18xx_id[] = {
	{ "ab1803", ab_1803 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ab18xx_id);

#ifdef CONFIG_OF
static const struct of_device_id ab18xx_of_match[] = {
	{ .compatible = "abracom,ab1803" },
	{ }
};
MODULE_DEVICE_TABLE(of, ab18xx_of_match);
#endif

static struct i2c_driver ab18xx_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ab18xx_of_match),
	},
	.probe = ab18xx_probe,
	.remove = ab18xx_remove,
	.id_table = ab18xx_id,
};
module_i2c_driver(ab18xx_driver);

MODULE_DESCRIPTION("RTC driver for AB18XX and similar chips");
MODULE_LICENSE("GPL");
