/*
 * Copyright (C) 2014 Critical Link LLC
 * Based on gpio-camio.c
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/of_irq.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>

#define CAMIO_GPIO_PCR			0x00
#define CAMIO_GPIO_IER			0x04
#define CAMIO_GPIO_IPR			0x08
#define CAMIO_GPIO_EDGER		0x0C

struct camio_gpio_chip {
	struct of_mm_gpio_chip mmchip;
	struct irq_domain *irq;	/* GPIO controller IRQ number */
	spinlock_t gpio_lock;	/* Lock used for synchronization */
	int hwirq;
};

static void camio_gpio_irq_unmask(struct irq_data *d)
{
	struct camio_gpio_chip *camio_gc = irq_data_get_irq_chip_data(d);
	struct of_mm_gpio_chip *mm_gc = &camio_gc->mmchip;
	unsigned long flags;
	unsigned int intmask;

	spin_lock_irqsave(&camio_gc->gpio_lock, flags);
	intmask = readl(mm_gc->regs + CAMIO_GPIO_IER);
	/* Set CAMIO_GPIO_IRQ_MASK bit to unmask */
	intmask |= (1 << irqd_to_hwirq(d));
	writel(intmask, mm_gc->regs + CAMIO_GPIO_IER);
	spin_unlock_irqrestore(&camio_gc->gpio_lock, flags);
}

static void camio_gpio_irq_mask(struct irq_data *d)
{
	struct camio_gpio_chip *camio_gc = irq_data_get_irq_chip_data(d);
	struct of_mm_gpio_chip *mm_gc = &camio_gc->mmchip;
	unsigned long flags;
	unsigned int intmask;

	spin_lock_irqsave(&camio_gc->gpio_lock, flags);
	intmask = readl(mm_gc->regs + CAMIO_GPIO_IER);
	/* Clear CAMIO_GPIO_IRQ_MASK bit to mask */
	intmask &= ~(1 << irqd_to_hwirq(d));
	writel(intmask, mm_gc->regs + CAMIO_GPIO_IER);
	spin_unlock_irqrestore(&camio_gc->gpio_lock, flags);
}

static int camio_gpio_irq_set_type(struct irq_data *d,
				unsigned int type)
{
	struct camio_gpio_chip *camio_gc = irq_data_get_irq_chip_data(d);
	struct of_mm_gpio_chip *mm_gc = &camio_gc->mmchip;
	unsigned long flags;
	unsigned int edgemask;

	if (type == IRQ_TYPE_NONE)
		return 0;

	if (type == IRQ_TYPE_EDGE_RISING) {
		spin_lock_irqsave(&camio_gc->gpio_lock, flags);
		edgemask = readl(mm_gc->regs + CAMIO_GPIO_EDGER);
		edgemask |= (1 << irqd_to_hwirq(d));
		edgemask &= ~(1 << (irqd_to_hwirq(d) + 4));
		writel(edgemask, mm_gc->regs + CAMIO_GPIO_EDGER);
		spin_unlock_irqrestore(&camio_gc->gpio_lock, flags);
		return 0;
	} else if (type == IRQ_TYPE_EDGE_FALLING) {
		spin_lock_irqsave(&camio_gc->gpio_lock, flags);
		edgemask = readl(mm_gc->regs + CAMIO_GPIO_EDGER);
		edgemask &= ~(1 << irqd_to_hwirq(d));
		edgemask |= (1 << (irqd_to_hwirq(d) + 4));
		writel(edgemask, mm_gc->regs + CAMIO_GPIO_EDGER);
		spin_unlock_irqrestore(&camio_gc->gpio_lock, flags);
		return 0;
	} else if (type == IRQ_TYPE_EDGE_BOTH) {
		spin_lock_irqsave(&camio_gc->gpio_lock, flags);
		edgemask = readl(mm_gc->regs + CAMIO_GPIO_EDGER);
		edgemask |= (1 << irqd_to_hwirq(d));
		edgemask |= (1 << (irqd_to_hwirq(d) + 4));
		writel(edgemask, mm_gc->regs + CAMIO_GPIO_EDGER);
		spin_unlock_irqrestore(&camio_gc->gpio_lock, flags);
		return 0;
	}

	return -EINVAL;
}

static struct irq_chip camio_irq_chip = {
	.name		= "camio-gpio",
	.irq_mask	= camio_gpio_irq_mask,
	.irq_unmask	= camio_gpio_irq_unmask,
	.irq_set_type	= camio_gpio_irq_set_type,
};

static int camio_gpio_get(struct gpio_chip *gc, unsigned offset)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);

	return (readl(mm_gc->regs + CAMIO_GPIO_PCR) >> (offset * 8 + 2)) & 1;
}

static void camio_gpio_set(struct gpio_chip *gc, unsigned offset, int value)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);
	struct camio_gpio_chip *chip = container_of(mm_gc,
				struct camio_gpio_chip, mmchip);
	unsigned long flags;
	unsigned int data_reg;
	unsigned int off = offset * 8 + 3;

	spin_lock_irqsave(&chip->gpio_lock, flags);
	data_reg = readl(mm_gc->regs + CAMIO_GPIO_PCR);
	data_reg = (data_reg & ~(3 << off)) | (value << off);
	writel(data_reg, mm_gc->regs + CAMIO_GPIO_PCR);
	spin_unlock_irqrestore(&chip->gpio_lock, flags);
}

static int camio_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);
	struct camio_gpio_chip *chip = container_of(mm_gc,
				struct camio_gpio_chip, mmchip);
	unsigned long flags;
	unsigned int gpio_ddr;
	unsigned int off = offset * 8;

	spin_lock_irqsave(&chip->gpio_lock, flags);
	/* Set pin as input, assumes software controlled IP */
	gpio_ddr = readl(mm_gc->regs + CAMIO_GPIO_PCR);
	gpio_ddr &= ~(1 << off);
	writel(gpio_ddr, mm_gc->regs + CAMIO_GPIO_PCR);
	spin_unlock_irqrestore(&chip->gpio_lock, flags);

	return 0;
}

static int camio_gpio_direction_output(struct gpio_chip *gc,
		unsigned offset, int value)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);
	struct camio_gpio_chip *chip = container_of(mm_gc,
				struct camio_gpio_chip, mmchip);
	unsigned long flags;
	unsigned int data_reg;
	unsigned int off = offset * 8 + 3;

	spin_lock_irqsave(&chip->gpio_lock, flags);
	/* Sets the GPIO value */
	data_reg = readl(mm_gc->regs + CAMIO_GPIO_PCR);
	data_reg = (data_reg & ~(3 << off)) | (value << off);
	writel(data_reg, mm_gc->regs + CAMIO_GPIO_PCR);

	/* Set pin as output, assumes software controlled IP */
	off = offset * 8;
	data_reg |= (1 << off);
	writel(data_reg, mm_gc->regs + CAMIO_GPIO_PCR);
	spin_unlock_irqrestore(&chip->gpio_lock, flags);

	return 0;
}

static int camio_gpio_to_irq(struct gpio_chip *gc, unsigned offset)
{
	struct of_mm_gpio_chip *mm_gc = to_of_mm_gpio_chip(gc);
	struct camio_gpio_chip *camio_gc = container_of(mm_gc,
				struct camio_gpio_chip, mmchip);

	if (camio_gc->irq == 0)
		return -ENXIO;
	if ((camio_gc->irq && offset) < camio_gc->mmchip.gc.ngpio)
		return irq_create_mapping(camio_gc->irq, offset);
	else
		return -ENXIO;
}

static void camio_gpio_irq_handler(unsigned int irq, struct irq_desc *desc)
{
	struct camio_gpio_chip *camio_gc = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct of_mm_gpio_chip *mm_gc = &camio_gc->mmchip;
	unsigned long status;

	int base;
	chip->irq_mask(&desc->irq_data);

	status = readl(mm_gc->regs + CAMIO_GPIO_IPR);
	writel(status, mm_gc->regs + CAMIO_GPIO_IPR);

	status &= readl(mm_gc->regs + CAMIO_GPIO_IER);

	for (base = 0; base < mm_gc->gc.ngpio; base++) {
		if ((1 << base) & status) {
			generic_handle_irq(
				irq_linear_revmap(camio_gc->irq, base));
		}
	}
	chip->irq_eoi(irq_desc_get_irq_data(desc));
	chip->irq_unmask(&desc->irq_data);
}

static int camio_gpio_irq_map(struct irq_domain *h, unsigned int virq,
				irq_hw_number_t hw_irq_num)
{
	irq_set_chip_data(virq, h->host_data);
	irq_set_chip_and_handler(virq, &camio_irq_chip, handle_level_irq);
	irq_set_irq_type(virq, IRQ_TYPE_NONE);

	return 0;
}

static struct irq_domain_ops camio_gpio_irq_ops = {
	.map	= camio_gpio_irq_map,
	.xlate = irq_domain_xlate_onecell,
};

static ssize_t camio_altinput_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct camio_gpio_chip *camio_gc = platform_get_drvdata(pdev);
	unsigned int reg, val, i, offset;
	ssize_t	status;

	offset = 1;
	if (!strcmp(attr->attr.name, "input_polarity"))
		offset = 6;

	reg = readl(camio_gc->mmchip.regs + CAMIO_GPIO_PCR);
	val = 0;
	for (i = 0; i < 4; i++) {
		if (reg & (1 << (i * 8 + offset)))
			val |= (1 << i);
	}
	status = sprintf(buf, "0x%X\n", val);

	return status;
}

static ssize_t camio_altinput_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	/* struct gpio_desc	*desc = dev_get_drvdata(dev); */
	struct platform_device *pdev = to_platform_device(dev);
	struct camio_gpio_chip *camio_gc = platform_get_drvdata(pdev);
	unsigned int val, reg, i, offset;

	offset = 1;
	if (!strcmp(attr->attr.name, "input_polarity"))
		offset = 6;

	sscanf(buf, "%x", &val);

	/* Set pin as input, assumes software controlled IP */
	reg = readl(camio_gc->mmchip.regs + CAMIO_GPIO_PCR);
	for (i = 0; i < 4; i++) {
		reg &= ~(1 << (i * 8 + offset));
		if (val & (1 << i))
			reg |= (1 << (i * 8 + offset));
	}
	writel(reg, camio_gc->mmchip.regs + CAMIO_GPIO_PCR);

	return size;
}

static ssize_t camio_altoutput_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct camio_gpio_chip *camio_gc = platform_get_drvdata(pdev);
	unsigned int reg, i, offset;
	ssize_t	status;
	unsigned int val[4];

	offset = 3;
	reg = readl(camio_gc->mmchip.regs + CAMIO_GPIO_PCR);
	for (i = 0; i < 4; i++)
		val[i] = ((reg & (0x3 << (i * 8 + offset)))
				>> (i * 8 + offset));

	status = sprintf(buf, "0x%X 0x%X 0x%X 0x%X\n", val[0], val[1], val[2],
			 val[3]);

	return status;
}

static ssize_t camio_altoutput_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct camio_gpio_chip *camio_gc = platform_get_drvdata(pdev);
	unsigned int pins, select, reg, i, offset, ret;

	offset = 3;

	ret = sscanf(buf, "%x %x", &pins, &select);

	if (ret == 2) {
		reg = readl(camio_gc->mmchip.regs + CAMIO_GPIO_PCR);
		for (i = 0; i < 4; i++) {
			/* If pin was selected, set the output mux */
			if (pins & (1 << i)) {
				reg &= ~(0x3 << (i * 8 + offset));
				reg |= (select << (i * 8 + offset));
			}
		}
		writel(reg, camio_gc->mmchip.regs + CAMIO_GPIO_PCR);
	} else {
		size = -1;
	}

	return size;
}

static DEVICE_ATTR(altinput, 0666, camio_altinput_show,
			camio_altinput_store);
static DEVICE_ATTR(input_polarity, 0666, camio_altinput_show,
			camio_altinput_store);
static DEVICE_ATTR(altoutput, 0666, camio_altoutput_show,
			camio_altoutput_store);

static struct attribute *camio_attrs[] = {
	&dev_attr_altinput.attr,
	&dev_attr_input_polarity.attr,
	&dev_attr_altoutput.attr,
	NULL,
};
static struct attribute_group camio_attr_group = {
	.attrs = camio_attrs,
};

int camio_gpio_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	int id, reg, ret;
	struct camio_gpio_chip *camio_gc = devm_kzalloc(&pdev->dev,
				sizeof(*camio_gc), GFP_KERNEL);
	if (camio_gc == NULL) {
		ret = -ENOMEM;
		pr_err("%s: registration failed with status %d\n",
			node->full_name, ret);
		return ret;
	}
	camio_gc->irq = 0;

	spin_lock_init(&camio_gc->gpio_lock);

	id = pdev->id;

	camio_gc->mmchip.gc.ngpio = 4;

	camio_gc->mmchip.gc.direction_input	= camio_gpio_direction_input;
	camio_gc->mmchip.gc.direction_output	= camio_gpio_direction_output;
	camio_gc->mmchip.gc.get			= camio_gpio_get;
	camio_gc->mmchip.gc.set			= camio_gpio_set;
	camio_gc->mmchip.gc.to_irq		= camio_gpio_to_irq;
	camio_gc->mmchip.gc.owner		= THIS_MODULE;

	ret = of_mm_gpiochip_add(node, &camio_gc->mmchip);
	if (ret)
		goto err;

	platform_set_drvdata(pdev, camio_gc);

	if (of_get_property(node, "interrupts", &reg) == NULL)
		goto skip_irq;
	camio_gc->hwirq = irq_of_parse_and_map(node, 0);

	if (camio_gc->hwirq == NO_IRQ)
		goto skip_irq;

	camio_gc->irq = irq_domain_add_linear(node, camio_gc->mmchip.gc.ngpio,
				&camio_gpio_irq_ops, camio_gc);

	if (!camio_gc->irq) {
		ret = -ENODEV;
		goto dispose_irq;
	}

	irq_set_handler_data(camio_gc->hwirq, camio_gc);
	irq_set_chained_handler(camio_gc->hwirq, camio_gpio_irq_handler);

	ret = sysfs_create_group(&pdev->dev.kobj, &camio_attr_group);
	if (ret) {
		pr_err("%s: sysfs create group failed with status %d\n",
			node->full_name, ret);
		goto teardown;
	}

	return 0;

teardown:
	irq_domain_remove(camio_gc->irq);
dispose_irq:
	irq_dispose_mapping(camio_gc->hwirq);
	WARN_ON(gpiochip_remove(&camio_gc->mmchip.gc) < 0);

err:
	pr_err("%s: registration failed with status %d\n",
		node->full_name, ret);
	devm_kfree(&pdev->dev, camio_gc);

	return ret;
skip_irq:
	return 0;
}

static int camio_gpio_remove(struct platform_device *pdev)
{
	unsigned int irq, i;
	int status;
	struct camio_gpio_chip *camio_gc = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &camio_attr_group);

	status = gpiochip_remove(&camio_gc->mmchip.gc);

	if (status < 0)
		return status;

	if (camio_gc->irq) {
		irq_dispose_mapping(camio_gc->hwirq);

		for (i = 0; i < camio_gc->mmchip.gc.ngpio; i++) {
			irq = irq_find_mapping(camio_gc->irq, i);
			if (irq > 0)
				irq_dispose_mapping(irq);
		}

		irq_domain_remove(camio_gc->irq);
	}

	irq_set_handler_data(camio_gc->hwirq, NULL);
	irq_set_chained_handler(camio_gc->hwirq, NULL);
	devm_kfree(&pdev->dev, camio_gc);
	return -EIO;
}

#ifdef CONFIG_OF
static struct of_device_id camio_gpio_of_match[] = {
	{ .compatible = "cl,camio-1.0", },
	{},
};
MODULE_DEVICE_TABLE(of, camio_gpio_of_match);
#else
#define camio_gpio_of_match NULL
#endif

static struct platform_driver camio_gpio_driver = {
	.driver = {
		.name	= "camio_gpio",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(camio_gpio_of_match),
	},
	.probe		= camio_gpio_probe,
	.remove		= camio_gpio_remove,
};

static int __init camio_gpio_init(void)
{
	return platform_driver_register(&camio_gpio_driver);
}
subsys_initcall(camio_gpio_init);

static void __exit camio_gpio_exit(void)
{
	platform_driver_unregister(&camio_gpio_driver);
}
module_exit(camio_gpio_exit);

MODULE_DESCRIPTION("Camera GPIO driver");
MODULE_LICENSE("GPL");
