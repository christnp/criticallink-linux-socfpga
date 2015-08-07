/*
 * Copyright 2014 Critical Link LLC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#define DRV_NAME "i2c_cl"

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_data/i2c-cl.h>

#define I2C_CL_TIMEOUT	(1*HZ)

/* Register Offsets */
#define VER_OFFSET  0
#define IER_OFFSET  2
#define CSR_OFFSET  4
#define SAR_OFFSET  6
#define CNT_OFFSET  8
#define DIV_OFFSET  10
#define FDR_OFFSET  12

#define I2C_CL_FIFO_DEPTH 32

typedef union
{
    struct
    {
	unsigned int mbDoneIER        : 1;
	unsigned int mbTxHalfEmptyIER : 1;
	unsigned int mbRxHalfFullIER  : 1;
	unsigned int mnReserved       : 13;
    } msBits;
    unsigned short mnWord;
} tuI2cIER;

typedef union
{
    struct
    {
	unsigned int mbDone        : 1;
	unsigned int mbGo          : 1;
	unsigned int mbRnW         : 1;
	unsigned int mb10Bit       : 1;
	unsigned int mbStop        : 1;
	unsigned int mbSDA         : 1; /* for debug */
	unsigned int mbAckErr      : 1;
	unsigned int mbFifoReset   : 1;
		unsigned int mbTxHe        : 1;
		unsigned int mbRxHf        : 1;
		unsigned int mbFifoInEmpty : 1;
		unsigned int mbFifoOutEmpty: 1;
	unsigned int mnReserved    : 4;
    } msBits;
    unsigned short mnWord;
} tuI2cCSR;

typedef union
{
    struct
    {
        unsigned int mnSlaveAddr : 10;
        unsigned int mnReserved  : 6;
    } msBits;
    unsigned short mnWord;
} tuI2cSAR;

typedef union
{
    struct
    {
        unsigned int mnData      : 8;
        unsigned int mnReserved  : 6;
    } msBits;
    unsigned short mnWord;
} tuI2cFDR;

/**
 *  this is the device driver specific parameters tied to each i2c device in the system
 *  (the object data)
 */
struct i2c_cl {
	struct i2c_adapter adapter;
	struct device *dev;
	void __iomem *base;
	struct i2c_cl_platform_data pdata;

	struct completion		done;
	int				tx_remaining_bytes;
	int				rx_remaining_bytes;
	int				irq;
	const uint8_t			*tx_buf;
	uint8_t				*rx_buf;
};

/*
 * Low level master read/write transaction. This function is called
 * from i2c_xfer.
 *
 * \param[in] adap pointer to our adaptor device
 * \param[in] msg message to send
 * \param[in] stop true if this is the last message in a set
 *
 * \return number of bytes transmitted, or < 0 error condition
 */
static int
i2c_cl_xfer_msg(struct i2c_adapter *adap, struct i2c_msg *msg, int stop)
{
	int i, slp_cnt;
	tuI2cCSR CSR;
	tuI2cSAR SAR;
	unsigned short  *lpBaseReg;
	struct i2c_cl *dev;

	dev = i2c_get_adapdata(adap);
	lpBaseReg = (unsigned short*)dev->base;

	dev_dbg(adap->dev.parent, "Msg Transfer - R/W = %d, len = %d\n", msg->flags & I2C_M_RD ? 1 : 0, msg->len);

	/* Can't handle 10 bit addresses yet */
	if (msg->flags & I2C_M_TEN)
	{
		dev_err(adap->dev.parent, "Msg Transfer 10 Bit Address Requested\n");
		return -EIO;
	}

	if (msg->len > I2C_CL_FIFO_DEPTH)
	{
		dev_err(adap->dev.parent, "Msg Transfer Message Length too big, %d\n", msg->len);
		return -EIO;
	}

	SAR.mnWord = 0;
	SAR.msBits.mnSlaveAddr = msg->addr;

	lpBaseReg[SAR_OFFSET] = SAR.mnWord;

	lpBaseReg[CNT_OFFSET] = msg->len;

	slp_cnt = 0;

	/** TODO - if transfer length is > 32 we need to
	 *  loop in multiples of 32 (without setting stop)
	 *  until we're done...
	 */
	if (msg->flags & I2C_M_RD)
	{
		/* Handle Bus Read */
		CSR.mnWord = 0;
		CSR.msBits.mbFifoReset = 1;
		lpBaseReg[CSR_OFFSET] = CSR.mnWord;
		wmb();

		CSR.mnWord = 0;
		CSR.msBits.mb10Bit = 0;
		CSR.msBits.mbRnW = 1;
		CSR.msBits.mbStop = stop;
		CSR.msBits.mbDone = 1;
		CSR.msBits.mbGo = 1;
		lpBaseReg[CSR_OFFSET] = CSR.mnWord;
		wmb();
	}
	else
	{
		/* clear the buffers */
		dev_dbg(adap->dev.parent, "Loading buffer with %d bytes\n", msg->len);

		CSR.mnWord = 0;
		CSR.msBits.mbFifoReset = 1;
		lpBaseReg[CSR_OFFSET] = CSR.mnWord;
		wmb();

		CSR.msBits.mbFifoReset = 0;
		lpBaseReg[CSR_OFFSET] = CSR.mnWord;
		wmb();

		/* load the buffer */
		for (i = 0; i < msg->len; i++)
		{
			tuI2cFDR DataReg;
			DataReg.mnWord = 0;
			DataReg.msBits.mnData = msg->buf[i];
			lpBaseReg[FDR_OFFSET] = DataReg.mnWord;
			wmb();
		}

		/* Handle Bus Write */
		CSR.mnWord = 0;
		CSR.msBits.mb10Bit = 0;
		CSR.msBits.mbRnW = 0;
		CSR.msBits.mbStop = stop;
		CSR.msBits.mbDone = 1;
		CSR.msBits.mbGo = 1;
		lpBaseReg[CSR_OFFSET] = CSR.mnWord;
		wmb();
	}

	/** check status for ack errors, etc. */
	CSR.mnWord = lpBaseReg[CSR_OFFSET];
	if (CSR.msBits.mbAckErr) {
		dev_dbg(adap->dev.parent, "Ack Error\n");
		return -EIO;
	}

	dev_dbg(adap->dev.parent, "transfer returning %d\n", msg->len);
	return msg->len;
}

/*
 * Prepare controller for a transaction and call i2c_davinci_xfer_msg
 *
 * \param[in] adap pointer to our adaptor device
 * \param[in] msgs pointer to a list of messages to send
 * \param[in] num number of messages to transfer
 */
static int
i2c_cl_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	int i;
	int ret;
	struct i2c_cl *dev = i2c_get_adapdata(adap);

	dev_dbg(adap->dev.parent, "%s: msgs: %d\n", __func__, num);

	for (i = 0; i < num; i++) {
		init_completion(&dev->done);

		dev->rx_remaining_bytes = msgs[i].len;
		dev->rx_buf = msgs[i].buf;

		ret = i2c_cl_xfer_msg(adap, &msgs[i], (i == (num - 1)));

		ret |= wait_for_completion_timeout(&dev->done, HZ);			// Wait for done (1 second timeout)

		dev_dbg(adap->dev.parent, "%s [%d/%d] ret: %d\n", __func__, i + 1, num, ret);
		if (ret < 0)
		{
			dev_dbg(adap->dev.parent, "Failed to transfer message : %d\n", ret);
			return ret;
		}
	}

	return num;
}

static u32 i2c_cl_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm i2c_algo = {
	.master_xfer	= i2c_cl_xfer,
	.functionality	= i2c_cl_func,
};

static struct attribute *i2c_cl_attributes[] = {
		/* &dev_attr_pen_down.attr, */
		NULL,
};

static struct attribute_group i2c_cl_attr_group = {
		.attrs = i2c_cl_attributes,
};

/**
 * IRQ handler called when an ADS7843 core is asserting an interrupt
 * condition.  This method is called within the context of an ISR, and
 * must be atomic.
 *
 * \param[in] dev the device issuing the interrupt.
 * \return 0
 */
irqreturn_t i2c_cl_isr(int this_irq, void *dev_id)
{
	unsigned short  *lpBaseReg;
	struct i2c_cl *driver_data = (struct i2c_cl *)dev_id;
	tuI2cCSR CSR;
	int i = 0;

	lpBaseReg = (unsigned short*)driver_data->base;

	/* Check if this is our interrupt */
	CSR.mnWord = lpBaseReg[CSR_OFFSET];

	if (!CSR.msBits.mbDone) {
		/* bit not set means this isn't us */
		return IRQ_NONE;
	};

	/* clear the interrupt */
	CSR.msBits.mbDone = 1;
	lpBaseReg[CSR_OFFSET] = CSR.mnWord;

	/* go fetch the data if necessary */
	if (CSR.msBits.mbRnW) {
		for (i = 0; i < driver_data->rx_remaining_bytes; i++)
		{
			tuI2cFDR DataReg;
			DataReg.mnWord = lpBaseReg[FDR_OFFSET];
			driver_data->rx_buf[i] = DataReg.msBits.mnData;
			rmb();
		}
	}

	complete(&driver_data->done);

	return IRQ_HANDLED;
}

/**
 * This routine is called when a device is removed from the FPGA bus.
 *
 * \param[in] dev pointer to the device being removed.
 */
static int i2c_cl_remove(struct platform_device *pdev)
{
	int rv = 0;
	unsigned short  *lpBaseReg;
	struct i2c_cl *driver_data;
	tuI2cIER IER;

	driver_data = platform_get_drvdata(pdev);
	lpBaseReg = (unsigned short*)driver_data->base;

	dev_dbg(&pdev->dev, "i2c_remove entered\n");

	i2c_del_adapter(&driver_data->adapter);

	sysfs_remove_group(&pdev->dev.kobj, &i2c_cl_attr_group);

	/* disable interrupt bits in the IER */
	IER.mnWord = lpBaseReg[IER_OFFSET];
	IER.msBits.mbDoneIER = 0;
	IER.msBits.mbTxHalfEmptyIER = 0;
	IER.msBits.mbRxHalfFullIER = 0;
	lpBaseReg[IER_OFFSET] = IER.mnWord;

	kfree(driver_data);

	dev_dbg(&pdev->dev, "i2c_remove completed\n");
	return rv;
}

#ifdef CONFIG_OF
static int i2c_cl_of_get_pdata(struct platform_device *pdev,
				struct i2c_cl *pi2c_cl)
{
	struct device_node *np = pdev->dev.of_node;
	struct i2c_cl_platform_data *pdata = &pi2c_cl->pdata;
	unsigned int prop;

	if (of_property_read_u32(np, "clk-frequency", &prop)) {
		dev_err(&pdev->dev, "couldn't determine num-chipselect\n");
		return -ENXIO;
	}
	pdata->requested_speed_hz = prop;

	if (of_property_read_u32(np, "master-ref-clk", &prop)) {
		dev_err(&pdev->dev, "couldn't determine master-ref-clk\n");
		return -ENXIO;
	}
	pdata->master_ref_clk_hz = prop;

	return 0;
}

static const struct of_device_id i2c_cl_match[] = {
	{ .compatible = "cl,i2c-1.0", },
	{},
};
MODULE_DEVICE_TABLE(of, i2c_cl_match);
#else
static int i2c_cl_of_get_pdata(struct platform_device *pdev,
				struct i2c_cl *pi2c_cl) { return -ENXIO; }
#endif /* CONFIG_OF */

/**
 * The i2c_probe routine is called after the i2c driver is successfully
 * matched to an FPGA core with the same core ID.
 *
 * \param[in] dev device within an fpga_device structure.
 * return 0 on successful probe / initialization.
 */
static int i2c_cl_probe(struct platform_device *pdev)
{
	int rv = 0;
	struct i2c_adapter* adap;
	struct i2c_cl  *dev = NULL;
	unsigned short     *lpBaseReg = NULL;
	struct resource *mem;
	struct i2c_cl_platform_data *pdata;
	int divider;
	tuI2cIER IER;
	tuI2cCSR CSR;

	dev_dbg(&pdev->dev, "i2c_probe() entered\n");

	dev = kzalloc(sizeof(struct i2c_cl), GFP_KERNEL);
	if (!dev)
	{
		rv = -ENOMEM;
		goto probe_bail;
	}

	rv = sysfs_create_group(&pdev->dev.kobj, &i2c_cl_attr_group);
	if (rv)
	{
		dev_err(&pdev->dev, "i2c_probe() failed to add attributes group - %d\n", rv);
		goto probe_bail_free_driver;
	}

	platform_set_drvdata(pdev, dev);

	/* get our registers */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "platform_get_resource failed\n");
		rv = -ENXIO;
		goto probe_bail_free_driver;
	}

	if (!devm_request_mem_region(&pdev->dev, mem->start,
		resource_size(mem), pdev->name)) {
		dev_err(&pdev->dev, "request_mem_region failed\n");
		rv = -EBUSY;
		goto probe_bail_free_driver;
	}

	dev->base = devm_ioremap(&pdev->dev, mem->start, resource_size(mem));
	if (!dev->base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		rv = -ENOMEM;
		goto probe_bail_free_driver;
	}

	lpBaseReg = (unsigned short*)dev->base;

	if (IS_ERR(dev->base)) {
		rv = PTR_ERR(dev->base);
		goto probe_bail_free_driver;
	}

	/* Clear any lingering interrupts. */
	CSR.mnWord = lpBaseReg[CSR_OFFSET];
	CSR.msBits.mbDone = 1;
	lpBaseReg[CSR_OFFSET] = CSR.mnWord;

	/* IRQ */
	dev->irq = platform_get_irq(pdev, 0);
	if (dev->irq < 0) {
		dev_err(&pdev->dev, "No IRQ Assigned\n");
		rv = -EINVAL;
		goto probe_bail_free_driver;
	}

	rv = devm_request_irq(&pdev->dev, dev->irq, i2c_cl_isr, 0,
			       pdev->name, dev);
	if (rv) {
		dev_err(&pdev->dev, "Unable to allocated IRQ\n");
		goto probe_bail_free_driver;
	}

	/* Enable/disable interrupt enable bits in the IER */
	IER.mnWord = lpBaseReg[IER_OFFSET];
	IER.msBits.mbDoneIER = 1;
	IER.msBits.mbTxHalfEmptyIER = 0;
	IER.msBits.mbRxHalfFullIER = 0;
	lpBaseReg[IER_OFFSET] = IER.mnWord;

	/* setup the adapter */
	adap = &dev->adapter;
	i2c_set_adapdata(adap, dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	strlcpy(adap->name, "MitySOM FPGA I2C adapter", sizeof(adap->name));
	adap->algo = &i2c_algo;
	adap->dev.parent = &pdev->dev;
	adap->dev.of_node = pdev->dev.of_node;
	adap->timeout = I2C_CL_TIMEOUT;

	adap->nr = pdev->id;
	rv = i2c_add_numbered_adapter(adap);
	if (rv) {
		dev_err(&pdev->dev, "failure adding adapter\n");
		goto probe_bail_free_adapt;
	}

	if (dev_get_platdata(&pdev->dev)) {
		pdata = dev_get_platdata(&pdev->dev);
		dev->pdata = *pdata;
	} else {
		/* try device tree, bail if we don't get what's needed */
		rv = i2c_cl_of_get_pdata(pdev, dev);
		if (rv < 0)
			goto probe_bail_free_adapt;
	}

	divider = dev->pdata.master_ref_clk_hz / 2 / dev->pdata.requested_speed_hz;

	if (!divider) {
		//Error?
		divider = 1;
	}
	dev_dbg(&pdev->dev, "Setting i2c divider: 0x%X\n", divider);

	/* Set the frequency */
	lpBaseReg[DIV_OFFSET] = divider;
	wmb();

	return rv;

probe_bail_free_adapt:


probe_bail_free_driver:
	kfree(dev);

probe_bail:
	return rv;
}

static struct platform_driver i2c_cl_driver = {
	.probe = i2c_cl_probe,
	.remove = i2c_cl_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = NULL,
		.of_match_table = of_match_ptr(i2c_cl_match),
	},
};
module_platform_driver(i2c_cl_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Jeff Myers <jmyers@criticallink.com");
MODULE_DESCRIPTION("Driver for Critical Link SoC FPGA Based I2C Controller");
