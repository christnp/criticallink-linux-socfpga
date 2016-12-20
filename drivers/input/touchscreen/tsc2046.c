/*
 * Critical Link FPGA based TSC2046 Touchscreen driver
 *
 * Copyright (C) 2016 Daniel Vincelette <dvincelette@criticallink.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <asm/irq.h>

#define	MAX_12BIT      ((1<<12)-1) 
#define CSR_XY         ( 0x0  ) 
#define CSR_Z1Z2       ( 0x1  ) 
#define CSR_PENDOWN    ( 0x2  ) 
#define CSR_IPR        ( 0x3  ) 
#define CSR_TOUCH_MASK ( 1<<0 ) 
#define TS_XAXIS_OHMS  ( 400  ) 

struct tsc2046 {
	struct input_dev	*input;
	void				*baseaddr;
	int irq;
	bool swapxy;
	bool reversex;
	bool reversey;
};

/*--------------------------------------------------------------------------*/

static irqreturn_t tsc2046_irq(int irq, void *handle)
{
	struct tsc2046 *ts = (struct tsc2046 *)handle;
	unsigned int *BaseReg = (unsigned int *)ts->baseaddr;
	unsigned int x, y, z1, z2, pressure = 0, tmp;
	bool pendown;
	struct input_dev *input = ts->input;
	
	tmp = ioread32((uint32_t *) &BaseReg[CSR_XY]);
	x = (tmp >> 16) & MAX_12BIT;
	y = tmp & MAX_12BIT;

	if (ts->swapxy)
		swap(x, y);
	
	if (ts->reversex)
		x = MAX_12BIT - x;

	if (ts->reversey)
		y = MAX_12BIT - y;

	tmp = ioread32((uint32_t *) &BaseReg[CSR_Z1Z2]);
	z1 = (tmp >> 16) & MAX_12BIT;
	z2 = tmp &  MAX_12BIT;

	tmp = ioread32((uint32_t *) &BaseReg[CSR_PENDOWN]);
	tmp &= CSR_TOUCH_MASK;
	pendown = (tmp==1) ? true : false;

	if (pendown) {
		input_report_key(input, BTN_TOUCH, 1);
		input_report_abs(input, ABS_X, y);
		input_report_abs(input, ABS_Y, MAX_12BIT - x);

		pressure = TS_XAXIS_OHMS;
		pressure *= x / 4096;
		pressure *= (z2 / z1) - 1;

		input_report_abs(input, ABS_PRESSURE, pressure);
	}
	else {
		input_report_key(input, BTN_TOUCH, 0);
		input_report_abs(input, ABS_PRESSURE, 0);
	}
	input_sync(input);

	// Clear the touch interrupt
	iowrite32(CSR_TOUCH_MASK, &BaseReg[CSR_IPR]);

	return IRQ_HANDLED;
}


#ifdef CONFIG_OF
static const struct of_device_id tsc2046_dt_ids[] = {
	{ .compatible = "cl,tsc2046",},
	{ }
};
MODULE_DEVICE_TABLE(of, tsc2046_dt_ids);
#endif

static int tsc2046_get_conf(struct device *pdev, struct tsc2046 *pts)
{
#ifdef CONFIG_OF
	struct device_node *node = pdev->of_node;
	const struct of_device_id *match;

	if (!node) {
		dev_err(pdev, "Device does not have associated DT data\n");
		return -EINVAL;
	}

	match = of_match_device(tsc2046_dt_ids, pdev);
	if (!match) {
		dev_err(pdev, "Unknown device model\n");
		return -EINVAL;
	}

	pts->swapxy = of_property_read_bool(node, "cl,swap-xy");
	pts->reversex = of_property_read_bool(node, "cl,reverse-x");
	pts->reversey = of_property_read_bool(node, "cl,reverse-y");
#else
	pts->swapxy   = false;
	pts->reversex = false;
	pts->reversy  = false;
#endif

	return 0;
}

static int tsc2046_probe(struct platform_device *pdev)
{
	int rv = 0;
	struct tsc2046 *ts;
	struct resource *res;
	struct input_dev *input_dev;

	ts = kzalloc(sizeof(struct tsc2046), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts) {
		rv = -ENOMEM;
		goto probe_bail;
	}

	/* find and map our resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "platform_get_resource failed\n");
		rv = -ENXIO;
		goto probe_bail;
	}

	if (!devm_request_mem_region(&pdev->dev, res->start,
		resource_size(res), pdev->name)) {
		dev_err(&pdev->dev, "request_mem_region failed\n");
		rv = -EBUSY;
		goto probe_bail;
	}

	ts->baseaddr = devm_ioremap(&pdev->dev, res->start,
					resource_size(res));
	if (!ts->baseaddr) {
		dev_err(&pdev->dev, "ioremap failed\n");
		rv = -ENOMEM;
		goto probe_bail;
	}

	/* IRQ */
	ts->irq = platform_get_irq(pdev, 0);
	if (ts->irq < 0) {
		dev_err(&pdev->dev, "No IRQ Assigned\n");
		rv = -EINVAL;
		goto probe_bail;
	}

	rv = devm_request_irq(&pdev->dev, ts->irq, tsc2046_irq, 0, pdev->name, ts);
	if (rv) {
		dev_err(&pdev->dev, "Unable to allocated IRQ\n");
		goto probe_bail;
	}

	rv = tsc2046_get_conf(&pdev->dev, ts);
	if (rv) 
		goto probe_bail;

	ts->input = input_dev;
	input_dev->name = "TSC2046 Touchscreen";
	input_dev->dev.parent = &pdev->dev;
	input_dev->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	input_set_abs_params(input_dev, ABS_X, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MAX_12BIT, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, MAX_12BIT, 0, 0);
	rv = input_register_device(input_dev);
	if (rv)
		goto probe_bail;

	platform_set_drvdata(pdev, ts);

	return rv;

probe_bail:
	input_free_device(input_dev);
	kfree(ts);
	return rv;
}

static int tsc2046_remove(struct platform_device *pdev)
{
	struct tsc2046 *ts = platform_get_drvdata(pdev);

	input_unregister_device(ts->input);
	kfree(ts);

	dev_dbg(&pdev->dev, "unregistered touchscreen\n");

	return 0;
}

static struct platform_driver tsc2046_driver = {
	.driver = {
		.name	= "tsc2046",
		.owner	= THIS_MODULE,
		.pm	= NULL,
		.of_match_table = of_match_ptr(tsc2046_dt_ids),
	},
	.probe		= tsc2046_probe,
	.remove		= tsc2046_remove,
};

module_platform_driver(tsc2046_driver);

MODULE_DESCRIPTION("tsc2046 TouchScreen Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Vincelette <dvincelette@criticallink.com");
