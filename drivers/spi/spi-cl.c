/*
 * Critical Link FPGA based SPI driver
 *
 * Copyright (C) 2014 Michael Williamson <micahel.williamson@criticallink.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#define DRV_NAME "spi_cl"

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/platform_data/spi-cl.h>

#define FPGA_SPI_TIMEOUT	(1*HZ)

#define MAX_FPGA_CHIP_SELECTS  8
#define MAX_FPGA_SPI_SPEED_HZ  250000000

/* Register Word (2 byte) Offsets */
#define VER_OFFSET		0
#define CSR_OFFSET		2
#define FWR_OFFSET		4
#define FDR_OFFSET		6
#define FRR_OFFSET		8
#define IER_OFFSET		10
#define IPR_OFFSET		12
#define DIV_OFFSET		14
#define DEL_OFFSET		16

#define CSR_RCVEN		(1 << 0)
#define CSR_SCLK_CTRL		(1 << 1)
#define CSR_CPHA		(1 << 2)
#define CSR_CPOL		(1 << 3)
#define CSR_DWIDTH_MASK		0x07
#define CSR_DWIDTH_SHIFT	4
#define CSR_LOOPBACK		(1 << 7)
#define CSR_GO			(1 << 8)
#define CSR_FIFO_RST		(1 << 9)
#define CSR_DELAY_CSMASK	(1 << 10)

#define FWR_WRITECNT_MASK	0x0FFF
#define FWR_WRITECNT_SHIFT	0
#define FWR_DEPTH_MASK		0x7
#define FWR_DEPTH_SHIFT		12

#define FRR_READCNT_MASK	0x0FFF
#define FRR_READCNT_SHIFT	0
#define FRR_DEPTH_MASK		0x7
#define FRR_DEPTH_SHIFT		12
#define FRR_EMPTY		(1 << 15)

#define IER_MOSI_TC		(1 << 0)
#define IER_MOSI_HF		(1 << 2)
#define IER_MISO_NEMPTY		(1 << 4)
#define IER_MISO_HF		(1 << 6)

#define IPR_MOSI_TC		(1 << 0)
#define IPR_MOSI_HF		(1 << 2)
#define IPR_MISO_NEMPTY		(1 << 4)
#define IPR_MISO_HF		(1 << 6)

#define DIV_DIVISOR_MASK	0x0FFF
#define DIV_DIVISOR_SHIFT	0
#define DIV_CS_EN_MASK		0x07
#define DIV_CS_EN_SHIFT		12

#define DEL_DELAY_MASK		0x7FFF
#define DEL_DELAY_SHIFT		0
#define DEL_DELAY_EN		0x8000

struct spi_cl_slave {
	int bytes_per_word;
};

/**
 *  this is the device driver specific parameters tied to each spi controller
 *  device in the system (the object data)
 */
struct spi_cl {
	struct spi_bitbang		bitbang;
	struct completion		done;
	struct spi_cl_slave		slave[MAX_FPGA_CHIP_SELECTS];
	int				remaining_bytes;
	int				fifo_depth;
	int				bytes_per_word;
	int				rx_remaining_bytes;
	int				delay;
	int				irq;
	const uint8_t			*tx_buf;
	uint8_t				*rx_buf;
	void				*baseaddr;
	struct spi_cl_platform_data	pdata;
};

static ssize_t spi_cl_csr_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct spi_cl *spi_cl =
		(struct spi_cl *)spi_master_get_devdata(dev_get_drvdata(dev));
	unsigned short *BaseReg = (unsigned short *)spi_cl->baseaddr;
	u16 CSR;

	CSR = ioread16(&BaseReg[CSR_OFFSET]);

	return sprintf(buf, "0x%X\n"
			"RE = %d\n"
			"SCLK = %d\n"
			"CPHA = %d\n"
			"CPOL = %d\n"
			"Width = %d\n"
			"LB = %d\n",
			CSR,
			(CSR & CSR_RCVEN) ? 1 : 0,
			(CSR & CSR_SCLK_CTRL) ? 1 : 0,
			(CSR & CSR_CPHA) ? 1 : 0,
			(CSR & CSR_CPOL) ? 1 : 0,
			1 << (((CSR >> CSR_DWIDTH_SHIFT) & CSR_DWIDTH_MASK)+2),
			(CSR & CSR_LOOPBACK) ? 1 : 0);
}
static DEVICE_ATTR(csr, S_IRUGO, spi_cl_csr_show, NULL);

static ssize_t spi_cl_state_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	ssize_t rv = 0;
	struct spi_cl *spi_cl =
		(struct spi_cl *)spi_master_get_devdata(dev_get_drvdata(dev));
	unsigned short *BaseReg = (unsigned short *)spi_cl->baseaddr;
	u16 CSR, FWR, FRR, IER, IPR, DIV, DEL;

	CSR = ioread16(&BaseReg[CSR_OFFSET]);
	FWR = ioread16(&BaseReg[FWR_OFFSET]);
	FRR = ioread16(&BaseReg[FRR_OFFSET]);
	IER = ioread16(&BaseReg[IER_OFFSET]);
	IPR = ioread16(&BaseReg[IPR_OFFSET]);
	DIV = ioread16(&BaseReg[DIV_OFFSET]);
	DEL = ioread16(&BaseReg[DEL_OFFSET]);
	rmb();

	rv += sprintf(buf, "CSR = 0x%X\n"
			"	RE = %d\n"
			"	SCLK = %d\n"
			"	CPHA = %d\n"
			"	CPOL = %d\n"
			"	Width = %d\n"
			"	LB = %d\n",
			CSR,
			(CSR & CSR_RCVEN) ? 1 : 0,
			(CSR & CSR_SCLK_CTRL) ? 1 : 0,
			(CSR & CSR_CPHA) ? 1 : 0,
			(CSR & CSR_CPOL) ? 1 : 0,
			1 << (((CSR >> CSR_DWIDTH_SHIFT) & CSR_DWIDTH_MASK)+2),
			(CSR & CSR_LOOPBACK) ? 1 : 0);

	rv += sprintf(&buf[rv], "FWR = 0x%X\n"
				"	write_cnt = %d\n"
				"	depth = %d\n",
				FWR,
				(FWR >> FWR_WRITECNT_SHIFT) & FWR_WRITECNT_MASK,
				1 << (((FWR >> FWR_DEPTH_SHIFT) &
					FWR_DEPTH_MASK) + 4));

	rv += sprintf(&buf[rv], "FRR = 0x%X\n"
				"	read_cnt = %d\n"
				"	depth = %d\n"
				"	empty = %d\n",
				FRR,
				(FRR >> FRR_READCNT_SHIFT) & FRR_READCNT_MASK,
				1 << (((FRR >> FRR_DEPTH_SHIFT) &
					FRR_DEPTH_MASK) + 4),
				(FRR & FRR_EMPTY) ? 1 : 0);

	rv += sprintf(&buf[rv], "IPR = 0x%X\n"
				"	mosi_tc = %d\n"
				"	mosi_hf = %d\n"
				"	miso_da = %d\n"
				"	miso_hf = %d\n",
				IPR,
				(IPR & IPR_MOSI_TC) ? 1 : 0,
				(IPR & IPR_MOSI_HF) ? 1 : 0,
				(IPR & IPR_MISO_NEMPTY) ? 1 : 0,
				(IPR & IPR_MISO_HF) ? 1 : 0);

	rv += sprintf(&buf[rv], "IER = 0x%X\n"
				"	mosi_tc = %d\n"
				"	mosi_hf = %d\n"
				"	miso_da = %d\n"
				"	miso_hf = %d\n",
				IER,
				(IER & IER_MOSI_TC) ? 1 : 0,
				(IER & IER_MOSI_HF) ? 1 : 0,
				(IER & IER_MISO_NEMPTY) ? 1 : 0,
				(IER & IER_MISO_HF) ? 1 : 0);

	rv += sprintf(&buf[rv], "DIV = 0x%X\n"
				"	divisor  = %d\n"
				"	chip_sel = %d\n",
				DIV,
				(DIV >> DIV_DIVISOR_SHIFT) & DIV_DIVISOR_MASK,
				(DIV >> DIV_CS_EN_SHIFT) & DIV_CS_EN_MASK);

	rv += sprintf(&buf[rv], "DEL = 0x%X\n"
				"	delay = %d\n"
				"	enable = %d\n",
				DEL,
				(DEL >> DEL_DELAY_SHIFT) & DEL_DELAY_MASK,
				(DEL & DEL_DELAY_EN) ? 1 : 0);
	return rv;
}
static DEVICE_ATTR(state, S_IRUGO, spi_cl_state_show, NULL);

static ssize_t spi_cl_remaining_bytes_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct spi_cl *spi_cl =
		(struct spi_cl *)spi_master_get_devdata(dev_get_drvdata(dev));
	return sprintf(buf, "%d\n", spi_cl->remaining_bytes);
}
static DEVICE_ATTR(remaining_bytes, S_IRUGO, spi_cl_remaining_bytes_show, NULL);

static ssize_t spi_cl_fifo_depth_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct spi_cl *spi_cl =
		(struct spi_cl *)spi_master_get_devdata(dev_get_drvdata(dev));
	return sprintf(buf, "%d\n", spi_cl->fifo_depth);
}
static DEVICE_ATTR(fifo_depth, S_IRUGO, spi_cl_fifo_depth_show, NULL);

static ssize_t spi_cl_delay_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct spi_cl *spi_cl =
		(struct spi_cl *)spi_master_get_devdata(dev_get_drvdata(dev));

	return sprintf(buf, "%d\n", spi_cl->delay);
}

static ssize_t spi_cl_delay_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int delay;
	u16 DEL;
	struct spi_cl *spi_cl =
		(struct spi_cl *)spi_master_get_devdata(dev_get_drvdata(dev));
	u16 *BaseAddr = (unsigned short *)spi_cl->baseaddr;
	DEL = 0;

	sscanf(buf, "%d", &delay);

	if (delay >= 0) {
		DEL &= ~(DEL_DELAY_MASK << DEL_DELAY_SHIFT);
		DEL |= (delay & DEL_DELAY_MASK) << DEL_DELAY_SHIFT;
		DEL |= DEL_DELAY_EN;
	}
	spi_cl->delay = delay;

	iowrite16(DEL, &BaseAddr[DEL_OFFSET]);

	return count;
}
static DEVICE_ATTR(delay, S_IRUGO | S_IWUSR, spi_cl_delay_show,
					     spi_cl_delay_set);

static struct attribute *spi_cl_attributes[] = {
		&dev_attr_fifo_depth.attr,
		&dev_attr_remaining_bytes.attr,
		&dev_attr_csr.attr,
		&dev_attr_state.attr,
		&dev_attr_delay.attr,
		NULL,
};

static struct attribute_group spi_cl_attr_group = {
		.attrs = spi_cl_attributes,
};

static void tx_word(struct spi_cl *spi_cl)
{
	unsigned short *BaseReg = (unsigned short *)spi_cl->baseaddr;
	u32 FDR;

	FDR = 0;

	if (spi_cl->tx_buf) {
		int i;
		for (i = 0; i < spi_cl->bytes_per_word; i++)
			FDR |= (unsigned) *spi_cl->tx_buf++ << (i * 8);
	}
	iowrite32(FDR, (uint32_t *) &BaseReg[FDR_OFFSET]);
	spi_cl->remaining_bytes -= spi_cl->bytes_per_word;
}

static void rx_word(struct spi_cl *spi_cl)
{
	unsigned short *BaseReg = (unsigned short *)spi_cl->baseaddr;
	u32 FDR;

	FDR = ioread32((uint32_t *) &BaseReg[FDR_OFFSET]);
	if (spi_cl->rx_buf) {
		int i;
		for (i = 0; i < spi_cl->bytes_per_word; i++) {
			*spi_cl->rx_buf++ = FDR & 0xFF;
			FDR >>= 8;
		}
		spi_cl->rx_remaining_bytes -= spi_cl->bytes_per_word;
	}
}

static void fill_tx_fifo(struct spi_cl *spi_cl)
{
	u16 FWR;
	unsigned short *BaseReg = (unsigned short *)spi_cl->baseaddr;
	int words_to_write;

	FWR = ioread16(&BaseReg[FWR_OFFSET]);
	/* while there is space in the FIFO... */
	words_to_write = spi_cl->fifo_depth - 2
			- ((FWR >> FWR_WRITECNT_SHIFT) & FWR_WRITECNT_MASK);

	while (words_to_write-- > 0) {
		/* if we are done, then quit */
		if (spi_cl->remaining_bytes <= 0)
			break;

		/* transfer a word (this will update remaining_bytes) */
		tx_word(spi_cl);
	}
}

/**
 * IRQ handler called when an SPI core is asserting an interrupt
 * condition.  This method is called within the context of an ISR, and
 * must be atomic.
 *
 * \param[in] dev the device issuing the interrupt.
 * \return 0
 */
static irqreturn_t spi_cl_irq(int irq, void *dev)
{
	struct spi_cl *spi_cl = dev;
	unsigned short *BaseReg;
	u16 IPR;
	u16 IER;

	BaseReg = (unsigned short *)spi_cl->baseaddr;

	IPR = ioread16(&BaseReg[IPR_OFFSET]);
	IER = ioread16(&BaseReg[IER_OFFSET]);

	if (spi_cl->remaining_bytes > 0)
		fill_tx_fifo(spi_cl);

	IPR = ioread16(&BaseReg[IPR_OFFSET]);
	IER = ioread16(&BaseReg[IER_OFFSET]);
	rmb();
	while (IPR & IPR_MISO_NEMPTY) {
		rx_word(spi_cl);
		IPR = ioread16(&BaseReg[IPR_OFFSET]);
		rmb();
	}

	/* no more data to transmit, disable FIFO level flags */
	if (spi_cl->remaining_bytes <= 0)
		/* disable helf empty threshold */
		IER &= ~IER_MOSI_HF;

	if (spi_cl->rx_remaining_bytes <= spi_cl->fifo_depth/2)
		IER &= ~IER_MISO_HF;

	if (spi_cl->rx_remaining_bytes <= 0)
		IER &= ~IER_MISO_NEMPTY;

	iowrite16(IER, &BaseReg[IER_OFFSET]);

	IPR = ioread16(&BaseReg[IPR_OFFSET]);

	/* if we are complete, then signal completion to waiting task */
	if ((IPR & IPR_MOSI_TC) && !(IPR & IPR_MISO_NEMPTY)) {
		/* disable interrupts */
		IER = 0;
		iowrite16(IER, &BaseReg[IER_OFFSET]);

		/* clear trasnmit completion status bit */
		iowrite16(IPR, &BaseReg[IPR_OFFSET]);

		/* signal to waiting task that we're done */
		complete(&spi_cl->done);
	}

	return IRQ_HANDLED;
}

/**
 * This routine is called when a device is removed from the FPGA bus.
 *
 * \param[in] dev pointer to the device being removed.
 */
static int spi_cl_remove(struct platform_device *pdev)
{
	int rv = 0;
	struct spi_cl *spi_cl;
	struct spi_master *master  = platform_get_drvdata(pdev);
	spi_cl = spi_master_get_devdata(master);

	rv = spi_bitbang_stop(&spi_cl->bitbang);

	sysfs_remove_group(&pdev->dev.kobj, &spi_cl_attr_group);

	spi_master_put(master);

	return rv;
}

static int spi_cl_setup_transfer(struct spi_device *spi, struct spi_transfer *t)
{
	struct spi_cl *spi_cl = spi_master_get_devdata(spi->master);
	unsigned short *BaseAddr;
	int divisor;
	u16 CSR, DIV;
	u8 bits_per_word = 0;
	u32 hz = 0;

	BaseAddr = (unsigned short *)spi_cl->baseaddr;

	if (t) {
		bits_per_word = t->bits_per_word;
		hz = t->speed_hz;
	}

	/* if bits_per_word is not set then set it default */
	if (!bits_per_word)
		bits_per_word = spi->bits_per_word;

	CSR = ioread16(&BaseAddr[CSR_OFFSET]);

	/* Our core only supports multiples of 4 between 4-32. */
	if (spi->bits_per_word > 32 || (spi->bits_per_word & 3) != 0)
		return -EINVAL;
	spi_cl->bytes_per_word = (spi->bits_per_word + 7) / 8;
	CSR &= ~(CSR_DWIDTH_MASK << CSR_DWIDTH_SHIFT);
	CSR |= ((spi->bits_per_word / 4 - 1) & CSR_DWIDTH_MASK)
		<< CSR_DWIDTH_SHIFT;

	if (!hz)
		hz = spi->max_speed_hz;
	DIV = ioread16(&BaseAddr[DIV_OFFSET]);
	divisor = 100000000 / (2 * hz); /* TODO */
	if (divisor == 0)
		divisor = 1;
	if ((spi_cl->pdata.master_ref_clk_hz / (2 * divisor)) > hz)
		divisor += 1;
	if (divisor > 0x0FFF)
		divisor = 0x0FFF;
	DIV &= ~(DIV_DIVISOR_MASK << DIV_DIVISOR_SHIFT);
	DIV |= (divisor << DIV_DIVISOR_SHIFT);

	/* we do not support LSB FIRST shifting */
	if (spi->mode & SPI_LSB_FIRST)
		return -EINVAL;

	if (spi->mode & SPI_CPOL)
		CSR |= CSR_CPOL;
	else
		CSR &= ~(CSR_CPOL);

	if (spi->mode & SPI_CPHA)
		CSR |= CSR_CPHA;
	else
		CSR &= ~(CSR_CPHA);

	if (spi->mode & SPI_LOOP)
		CSR |= CSR_LOOPBACK;
	else
		CSR &= ~(CSR_LOOPBACK);

	/* There's no SPI framework mode for this -- default to SYNC-gated,
	 * because that's what most SPI devices do */
	CSR |= CSR_SCLK_CTRL;

	/* Set rcv_en if there is a receive buffer requested */
	CSR &= ~(CSR_RCVEN);

	/* chip select */
	if (!(spi->mode & SPI_NO_CS)) {
		DIV &= ~(DIV_CS_EN_MASK << DIV_CS_EN_SHIFT);
		DIV |= (spi->chip_select << DIV_CS_EN_SHIFT);
	}

	iowrite16(DIV, &BaseAddr[DIV_OFFSET]);
	iowrite16(CSR, &BaseAddr[CSR_OFFSET]);

	return 0;
}

/*
 * This method is called when the SPI framework is cleaning up
 */
static void spi_cl_cleanup(struct spi_device *spi)
{
	/* struct spi_cl *spi_cl = spi_master_get_devdata(spi->master); */

	/* nothing to do yet... */
}

/*
 * This method is called when the SPI framework is configured for a given
 * chip device.
 */
static int spi_cl_setup(struct spi_device *spi)
{
	struct spi_cl *spi_cl;
	unsigned short *BaseAddr;

	spi_cl = spi_master_get_devdata(spi->master);
	BaseAddr = (unsigned short *)spi_cl->baseaddr;

	/* if bits per word length is zero then set it default 8 */
	if (!spi->bits_per_word)
		spi->bits_per_word = 8;

	if (!spi->max_speed_hz)
		spi->max_speed_hz = MAX_FPGA_SPI_SPEED_HZ;

	return 0;
}

static void spi_cl_chipselect(struct spi_device *spi, int value)
{
	/* struct spi_cl *spi_cl = spi_master_get_devdata(spi->master); */

	/* chip select controlled by transfers.. */
}

static int spi_cl_txrx_bufs(struct spi_device *spi, struct spi_transfer *t)
{
	struct spi_cl *spi_cl = spi_master_get_devdata(spi->master);
	unsigned short *BaseAddr = (unsigned short *)spi_cl->baseaddr;
	u16 IER, IPR, CSR, FRR;
	u32 temp;

	spi_cl = spi_master_get_devdata(spi->master);

	spi_cl->remaining_bytes = t->len;
	spi_cl->rx_buf = t->rx_buf;
	if (t->rx_buf)
		spi_cl->rx_remaining_bytes = t->len;
	else
		spi_cl->rx_remaining_bytes = 0;

	spi_cl->tx_buf = t->tx_buf;

	do {
		temp = ioread32((u32 *) &BaseAddr[FDR_OFFSET]);
		FRR = ioread16(&BaseAddr[FRR_OFFSET]);
		rmb();
	} while (!(FRR & FRR_EMPTY));

	/* clear any interrupt pending conditions */
	IPR = ioread16(&BaseAddr[IPR_OFFSET]);
	IPR |= (IPR_MOSI_TC);
	iowrite16(IPR, &BaseAddr[IPR_OFFSET]);

	IER = 0;
	iowrite16(IER, &BaseAddr[IER_OFFSET]);

	IPR = ioread16(&BaseAddr[IPR_OFFSET]);
	INIT_COMPLETION(spi_cl->done);

	/* reset fifos */
	CSR = ioread16(&BaseAddr[CSR_OFFSET]);
	CSR |= CSR_FIFO_RST;
	iowrite16(CSR, &BaseAddr[CSR_OFFSET]);
	wmb();

	/* fill the output fifo */
	fill_tx_fifo(spi_cl);

	/* start the outbound transfer by setting go bit */
	CSR = ioread16(&BaseAddr[CSR_OFFSET]);
	CSR |= CSR_GO;
	if (t->rx_buf)
		CSR |= CSR_RCVEN;
	else
		CSR &= ~(CSR_RCVEN);
	iowrite16(CSR, &BaseAddr[CSR_OFFSET]);

	/* enable interrupts to catch transfer completion, or
	 * FIFO emptying */
	IER = 0;
	IER |= IER_MOSI_TC;
	if (spi_cl->remaining_bytes)
		IER |= IER_MOSI_HF;
	if (spi_cl->rx_remaining_bytes >= spi_cl->fifo_depth/2)
		IER |= IER_MISO_HF;
	else if (spi_cl->rx_remaining_bytes > 0)
		IER |= IER_MISO_NEMPTY;
	iowrite16(IER, &BaseAddr[IER_OFFSET]);

	/* wait for completion */
	wait_for_completion(&spi_cl->done);

	return t->len - spi_cl->remaining_bytes;
}

#ifdef CONFIG_OF
static int spi_cl_of_get_pdata(struct platform_device *pdev,
				struct spi_cl *pspi_cl)
{
	struct device_node *np = pdev->dev.of_node;
	struct spi_cl_platform_data *pdata = &pspi_cl->pdata;
	unsigned int prop;

	if (of_property_read_u32(np, "bus-num", &prop)) {
		dev_err(&pdev->dev, "couldn't determine bus-num\n");
		return -ENXIO;
	}
	pdata->bus_num = prop;

	if (of_property_read_u32(np, "num-chipselect", &prop)) {
		dev_err(&pdev->dev, "couldn't determine num-chipselect\n");
		return -ENXIO;
	}
	pdata->num_chipselect = prop;

	if (of_property_read_u32(np, "master-ref-clk", &prop)) {
		dev_err(&pdev->dev, "couldn't determine master-ref-clk\n");
		return -ENXIO;
	}
	pdata->master_ref_clk_hz = prop;

	return 0;
}
#else
static int spi_cl_of_get_pdata(struct platform_device *pdev) { return -ENXIO; }
#endif

/**
 * The spi_probe routine is called after the spi driver is successfully
 * matched to an FPGA core with the same core ID.
 *
 * \param[in] dev device within an fpga_device structure.
 * return 0 on successful probe / initialization.
 */
static int spi_cl_probe(struct platform_device *pdev)
{
	int rv = 0;
	struct spi_cl_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct spi_cl *spi_cl;
	struct spi_master *master;
	struct resource *res;

	unsigned short *BaseReg;
	u16 FWR, DEL;

	rv = sysfs_create_group(&pdev->dev.kobj, &spi_cl_attr_group);
	if (rv) {
		dev_err(&pdev->dev,
			"spi_probe() failed to add attributes group - %d\n",
			rv);
		goto probe_bail;
	}

	master = spi_alloc_master(&pdev->dev, sizeof(struct spi_cl));
	if (master == NULL) {
		rv = -ENOMEM;
		goto probe_bail;
	}

	platform_set_drvdata(pdev, master);

	spi_cl = spi_master_get_devdata(master);
	if (spi_cl == NULL) {
		rv = -ENOENT;
		goto probe_bail_free_master;
	}

	if (dev_get_platdata(&pdev->dev)) {
		pdata = dev_get_platdata(&pdev->dev);
		spi_cl->pdata = *pdata;
	} else {
		/* try device tree, bail if we don't get what's needed */
		rv = spi_cl_of_get_pdata(pdev, spi_cl);
		if (rv < 0)
			goto probe_bail_free_master;
	}

	spi_cl->bitbang.master = spi_master_get(master);
	if (spi_cl->bitbang.master == NULL) {
		rv = -ENODEV;
		goto probe_bail_free_master;
	}

	/* find and map our resources */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "platform_get_resource failed\n");
		rv = -ENXIO;
		goto probe_bail_free_master;
	}

	if (!devm_request_mem_region(&pdev->dev, res->start,
		resource_size(res), pdev->name)) {
		dev_err(&pdev->dev, "request_mem_region failed\n");
		rv = -EBUSY;
		goto probe_bail_free_master;
	}

	spi_cl->baseaddr = devm_ioremap(&pdev->dev, res->start,
					resource_size(res));
	if (!spi_cl->baseaddr) {
		dev_err(&pdev->dev, "ioremap failed\n");
		rv = -ENOMEM;
		goto probe_bail_free_master;
	}

	BaseReg = (unsigned short *)spi_cl->baseaddr;

	spi_cl->bitbang.flags = SPI_CPOL | SPI_CPHA | SPI_LOOP;
	master->bus_num = spi_cl->pdata.bus_num;
	master->num_chipselect = spi_cl->pdata.num_chipselect;
	master->setup = spi_cl_setup;
	master->cleanup = spi_cl_cleanup;

	init_completion(&spi_cl->done);

	spi_cl->bitbang.chipselect = spi_cl_chipselect;
	spi_cl->bitbang.setup_transfer = spi_cl_setup_transfer;
	spi_cl->bitbang.txrx_bufs = spi_cl_txrx_bufs;
	spi_cl->bitbang.master->dev.of_node = pdev->dev.of_node;

	FWR = ioread16(&BaseReg[FWR_OFFSET]);
	spi_cl->fifo_depth = 1 <<
		(((FWR >> FWR_DEPTH_SHIFT) & FWR_DEPTH_MASK) + 4);

	DEL = ioread16(&BaseReg[DEL_OFFSET]);
	spi_cl->delay = (DEL >> DEL_DELAY_SHIFT) & DEL_DELAY_MASK;

	iowrite16(0, &BaseReg[IER_OFFSET]);

	/* IRQ */
	spi_cl->irq = platform_get_irq(pdev, 0);
	if (spi_cl->irq < 0) {
		dev_err(&pdev->dev, "No IRQ Assigned\n");
		rv = -EINVAL;
		goto probe_bail_free_master;
	}

	rv = devm_request_irq(&pdev->dev, spi_cl->irq, spi_cl_irq, 0,
			       pdev->name, spi_cl);
	if (rv) {
		dev_err(&pdev->dev, "Unable to allocated IRQ\n");
		goto probe_bail_free_master;
	}

	rv = spi_bitbang_start(&spi_cl->bitbang);
	if (rv)
		goto probe_bail_free_master;

	return rv;

probe_bail_free_master:
	spi_master_put(master);

probe_bail:
	sysfs_remove_group(&pdev->dev.kobj, &spi_cl_attr_group);
	return rv;
}

#ifdef CONFIG_OF
static const struct of_device_id spi_cl_match[] = {
	{ .compatible = "cl,spi-1.0", },
	{ .compatible = "cl,spi-1.0", },
	{},
};
MODULE_DEVICE_TABLE(of, spi_cl_match);
#endif /* CONFIG_OF */

static struct platform_driver spi_cl_driver = {
	.probe = spi_cl_probe,
	.remove = spi_cl_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = NULL,
		.of_match_table = of_match_ptr(spi_cl_match),
	},
};
module_platform_driver(spi_cl_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mike Williamson <michael.williamson@criticallink.com");
MODULE_DESCRIPTION("Driver for Critical Link SoC FPGA Based SPI Controller");

