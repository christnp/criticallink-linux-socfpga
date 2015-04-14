/*******************************************************************************
* STMMAC Ethernet Driver -- MDIO bus implementation
* Provides Bus interface for MII registers
*
*  Copyright (C) 2007-2009  STMicroelectronics Ltd

* This program is free software; you can redistribute it and/or modify it
* under the terms and conditions of the GNU General Public License,
* version 2, as published by the Free Software Foundation.
*
*  This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* The full GNU General Public License is included in this distribution in
* the file called "COPYING".
*
* Author: Carl Shaw <carl.shaw@st.com>
* Maintainer: Giuseppe Cavallaro <peppe.cavallaro@st.com>
*******************************************************************************/

#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>

#include <linux/io.h>

#include "stmmac.h"
#include "dwmac1000.h"

#define MII_BUSY 0x00000001
#define MII_WRITE 0x00000002

struct stmmac_mdio_priv {
	/* lock as we share register access with the mac */
	struct mutex lock;
	/* points to GMAC register base for this bus */
	void __iomem *ioaddr;
	struct mii_regs mii;    /* MII register Address offsets*/
};


static int stmmac_mdio_busy_wait(void __iomem *ioaddr, unsigned int mii_addr)
{
	unsigned long curr;
	unsigned long finish = jiffies + 3 * HZ;

	do {
		curr = jiffies;
		if (readl(ioaddr + mii_addr) & MII_BUSY)
			cpu_relax();
		else
			return 0;
	} while (!time_after_eq(curr, finish));

	return -EBUSY;
}

/**
 * stmmac_mdio_read
 * @bus: points to the mii_bus structure
 * @phyaddr: MII addr reg bits 15-11
 * @phyreg: MII addr reg bits 10-6
 * Description: it reads data from the MII register from within the phy device.
 * For the 7111 GMAC, we must set the bit 0 in the MII address register while
 * accessing the PHY registers.
 * Fortunately, it seems this has no drawback for the 7109 MAC.
 */
static int stmmac_mdio_read(struct mii_bus *bus, int phyaddr, int phyreg)
{
	struct stmmac_mdio_priv *priv = bus->priv;
	unsigned int mii_address = priv->mii.addr;
	unsigned int mii_data = priv->mii.data;

	int data = -EBUSY;
	u16 reg_value = (((phyaddr << 11) & (0x0000F800)) |
			((phyreg << 6) & (0x000007C0)));
	u32 clk_reg_val = 0;

	mutex_lock(&priv->lock);

	/* Read clock CSR instead of keeping it in priv - shared access */
	clk_reg_val = readl(priv->ioaddr + priv->mii.addr);
	clk_reg_val = clk_reg_val & (0xf << 2); /* mask the clock CSR bits */
	reg_value |= MII_BUSY | clk_reg_val;

	if (stmmac_mdio_busy_wait(priv->ioaddr, mii_address))
		goto out;

	writel(reg_value, priv->ioaddr + mii_address);

	if (stmmac_mdio_busy_wait(priv->ioaddr, mii_address))
		goto out;

	/* Read the data from the MII data register */
	data = (int)readl(priv->ioaddr + mii_data);

out:
	mutex_unlock(&priv->lock);
	return data;
}

/**
 * stmmac_mdio_write
 * @bus: points to the mii_bus structure
 * @phyaddr: MII addr reg bits 15-11
 * @phyreg: MII addr reg bits 10-6
 * @phydata: phy data
 * Description: it writes the data into the MII register from within the device.
 */
static int stmmac_mdio_write(struct mii_bus *bus, int phyaddr, int phyreg,
			     u16 phydata)
{
	struct stmmac_mdio_priv *priv = bus->priv;

	int rv = -EBUSY;
	u16 value =
	    (((phyaddr << 11) & (0x0000F800)) | ((phyreg << 6) & (0x000007C0)))
	    | MII_WRITE;
	u32 clk_reg_val = 0;

	mutex_lock(&priv->lock);
	/* change to read clock CSR instead of keeping it in priv */
	clk_reg_val = readl(priv->ioaddr + priv->mii.addr);
	clk_reg_val = clk_reg_val & (0xf << 2); /* mask the clock CSR bits */

	value |= MII_BUSY | clk_reg_val;

	/* Wait until any existing MII operation is complete */
	if (stmmac_mdio_busy_wait(priv->ioaddr, priv->mii.addr))
		goto out;

	/* Set the MII address register to write */
	writel(phydata, priv->ioaddr + priv->mii.data);
	writel(value, priv->ioaddr + priv->mii.addr);

	/* Wait until any existing MII operation is complete */
	rv =  stmmac_mdio_busy_wait(priv->ioaddr, priv->mii.addr);
out:
	mutex_unlock(&priv->lock);
	return rv;
}

static struct of_device_id stmmac_mdio_match[] = {
	{
		.compatible = "st,stmmac-mdio",
	},

	{},
};
MODULE_DEVICE_TABLE(of, stmmac_mdio_match);

static int stmmac_mdio_probe(struct platform_device *pdev)
{
	const struct of_device_id *id =
		of_match_device(stmmac_mdio_match, &pdev->dev);
	struct device_node *np = pdev->dev.of_node;
	struct resource res;
	struct stmmac_mdio_priv *priv = NULL;
	struct mii_bus *bus;
	int bus_id = -1;
	int err;
	int phy_loop;

	if (!id) {
		dev_err(&pdev->dev, "could not match device tree\n");
		return -EINVAL;
	}
	if (!np) {
		dev_err(&pdev->dev, "device does not have of_node\n");
		return -EINVAL;
	}

	dev_dbg(&pdev->dev, "found %s compatible node\n", id->compatible);

	bus = devm_mdiobus_alloc_size(&pdev->dev, sizeof(*priv));
	if (!bus) {
		dev_err(&pdev->dev, "failed to alloc mii bus\n");
		return -ENOMEM;
	}


	bus->parent = &pdev->dev;
	bus->name = "stmmac_mdio",
	bus->read = &stmmac_mdio_read;
	bus->write = &stmmac_mdio_write;

	bus->irq = devm_kmalloc_array(&pdev->dev, PHY_MAX_ADDR, sizeof(int),
				      GFP_KERNEL);
	if (!bus->irq)
		return -ENOMEM;

	for (phy_loop = 0; phy_loop < PHY_MAX_ADDR; phy_loop++)
		bus->irq[phy_loop] = PHY_POLL;

	bus_id = of_alias_get_id(np, "mdio");
	if (0 > bus_id)
		bus_id = 0;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-%x",
		 bus->name, bus_id);
	dev_dbg(&pdev->dev, "DRV: bus ID %s\n", bus->id);

	err = of_address_to_resource(np, 0, &res);
	if (err < 0) {
		dev_err(&pdev->dev, "could not obtain address information\n");
		goto error;
	}

	dev_info(&pdev->dev,
		 "%s@%llx", np->name, (unsigned long long)res.start);

	priv = bus->priv;
	priv->mii.addr = GMAC_MII_ADDR;
	priv->mii.data = GMAC_MII_DATA;
	priv->ioaddr = of_iomap(np, 0);
	if (!priv->ioaddr) {
		err = -ENOMEM;
		goto error;
	}

	/* Some device tree nodes represent only the MII registers, and
	 * others represent the MAC and MII registers.  The 'mii_offset' field
	 * contains the offset of the MII registers inside the mapped register
	 * space.
	 */
	if (priv->mii.addr > resource_size(&res)) {
		dev_err(&pdev->dev, "invalid register map\n");
		err = -EINVAL;
		goto error;
	}

#ifdef DT_PARSE_MAC_NODE
	struct device_node *mac_node;
	/* what we should do here is get the dt node for the mac we are
	 * using and pull needed data from that. Until then, its in the dt!
	 */
	mac_node = of_parse_phandle(np, "mac-handle", 0);
	if (!mac_node) {
		dev_err(pdev->dev, "unable to parse mac-handle\n");
		return -EINVAL;
	}
	of_node_get(mac_node);
	of_node_put(mac_node);
#endif /* DT_PARSE_MAC_NODE */

	mutex_init(&priv->lock);


	err = of_mdiobus_register(bus, np);
	if (err) {
		dev_err(&pdev->dev, "cannot register %s as MDIO bus\n",
			bus->name);
		goto error;
	}
	platform_set_drvdata(pdev, bus);

	return 0;

error:
	if (priv && priv->ioaddr)
		iounmap(priv->ioaddr);

	return err;
}


static int stmmac_mdio_remove(struct platform_device *pdev)
{
	struct device *device = &pdev->dev;
	struct mii_bus *bus = dev_get_drvdata(device);
	struct stmmac_mdio_priv *priv = bus->priv;

	mdiobus_unregister(bus);

	iounmap(priv->ioaddr);
	mdiobus_free(bus);

	return 0;
}

static struct platform_driver stmmac_mdio_driver = {
	.driver = {
		.name = "stmmac-mdio",
		.owner = THIS_MODULE,
		.of_match_table = stmmac_mdio_match,
	},
	.probe = stmmac_mdio_probe,
	.remove = stmmac_mdio_remove,
};

module_platform_driver(stmmac_mdio_driver);

MODULE_DESCRIPTION("MDIO driver for socfpga st micro mac");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:stmmac-mdio");

