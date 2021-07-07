/*
 * (C) Copyright 2017 Whitebox Systems / Northend Systems B.V.
 * S.J.R. van Schaik <stephan@whiteboxsystems.nl>
 * M.B.W. Wajer <merlijn@whiteboxsystems.nl>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <errno.h>
#include <spi.h>
#include <fdt_support.h>
#include <time.h>

#include <asm/bitops.h>
#include <asm/gpio.h>
#include <asm/io.h>

#include <asm/arch/clock.h>
#include <asm/arch/spi.h>

#define SUNXI_SPI_MAX_RATE (24 * 1000 * 1000)
#define SUNXI_SPI_MIN_RATE (3 * 1000)
#define SUNXI_SPI_MAX_CS_COUNT 4

#define SPI_CS_GPIOS_PREFERED 0x01

struct sunxi_spi_platdata {
	struct sunxi_spi_regs *regs;
	unsigned int activate_delay_us;
	unsigned int deactivate_delay_us;
	uint32_t freq;
};

struct sunxi_spi_priv {
	struct sunxi_spi_regs *regs;
	unsigned int max_freq;
	unsigned int last_transaction_us;
	unsigned int flags;
	struct gpio_desc cs_gpios[SUNXI_SPI_MAX_CS_COUNT];
};

DECLARE_GLOBAL_DATA_PTR;

static int sunxi_spi_parse_pins(struct udevice *dev)
{
	struct sunxi_spi_priv *priv = dev_get_priv(dev);
	const void *fdt = gd->fdt_blob;
	const char *pin_name;
	const fdt32_t *list;
	u32 phandle;
	int drive, pull = 0, pin, i, k;
	int offset;
	int size;
	struct gpio_desc desc;

	list = fdt_getprop(fdt, dev_of_offset(dev), "pinctrl-0", &size);
	if (!list) {
		printf("WARNING: sunxi_spi: cannot find pinctrl-0 node\n");
		return -EINVAL;
	}

	while (size) {
		phandle = fdt32_to_cpu(*list++);
		size -= sizeof(*list);

		offset = fdt_node_offset_by_phandle(fdt, phandle);
		if (offset < 0)
			return offset;

		drive = fdt_getprop_u32_default_node(fdt, offset, 0,
						     "drive-strength", 0);
		if (drive) {
			if (drive <= 10)
				drive = 0;
			else if (drive <= 20)
				drive = 1;
			else if (drive <= 30)
				drive = 2;
			else
				drive = 3;
		} else {
			drive = fdt_getprop_u32_default_node(fdt, offset, 0,
							     "allwinner,drive",
							      0);
			drive = min(drive, 3);
		}

		if (fdt_get_property(fdt, offset, "bias-disable", NULL))
			pull = 0;
		else if (fdt_get_property(fdt, offset, "bias-pull-up", NULL))
			pull = 1;
		else if (fdt_get_property(fdt, offset, "bias-pull-down", NULL))
			pull = 2;
		else
			pull = fdt_getprop_u32_default_node(fdt, offset, 0,
							    "allwinner,pull",
							     0);
		pull = min(pull, 2);

		for (i = 0; ; i++) {
			pin_name = fdt_stringlist_get(fdt, offset,
						      "pins", i, NULL);
			if (!pin_name) {
				pin_name = fdt_stringlist_get(fdt, offset,
							      "allwinner,pins",
							       i, NULL);
				if (!pin_name)
					break;
			}

			pin = name_to_gpio(pin_name);
			if (pin < 0)
				break;

			if (dm_gpio_lookup_name(pin_name, &desc) == 0) {
				for (k = 0; k < SUNXI_SPI_MAX_CS_COUNT; k++) {
					if (!dm_gpio_is_valid(&priv->cs_gpios[k])) continue;
					if (gpio_get_number(&desc) == gpio_get_number(&priv->cs_gpios[k])) break;
				}

				if (k < SUNXI_SPI_MAX_CS_COUNT) /* cs-gpio pin */
					continue;
			}

			sunxi_gpio_set_cfgpin(pin, SUNXI_GPC_SPI0);
			sunxi_gpio_set_drv(pin, drive);
			sunxi_gpio_set_pull(pin, pull);
		}
	}
	return 0;
}

static void sunxi_spi_enable_clock(struct udevice *bus)
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg * const)SUNXI_CCM_BASE;

#if defined(CONFIG_MACH_SUN6I) || defined(CONFIG_MACH_SUN8I) || \
	defined(CONFIG_MACH_SUN9I) || defined(CONFIG_MACH_SUN50I)
	setbits_le32(&ccm->ahb_reset0_cfg,
		(1 << AHB_GATE_OFFSET_SPI0));
#endif

	setbits_le32(&ccm->ahb_gate0, (1 << AHB_GATE_OFFSET_SPI0));
	writel((1 << 31), &ccm->spi0_clk_cfg);
}

static void sunxi_spi_disable_clock(void)
{
	struct sunxi_ccm_reg * const ccm =
		(struct sunxi_ccm_reg * const)SUNXI_CCM_BASE;

	writel(0, &ccm->spi0_clk_cfg);
	clrbits_le32(&ccm->ahb_gate0, (1 << AHB_GATE_OFFSET_SPI0));

#if defined(CONFIG_MACH_SUN6I) || defined(CONFIG_MACH_SUN8I) || \
	defined(CONFIG_MACH_SUN9I) || defined(CONFIG_MACH_SUN50I)
	clrbits_le32(&ccm->ahb_reset0_cfg,
		(1 << AHB_GATE_OFFSET_SPI0));
#endif
}

static void sunxi_spi_cs_activate(struct udevice *dev, unsigned int cs)
{
	struct udevice *bus = dev->parent;
	struct sunxi_spi_platdata *plat = dev_get_platdata(bus);
	struct sunxi_spi_priv *priv = dev_get_priv(bus);
	uint32_t reg;

	/* If it is too soon to perform another transaction, wait. */
	if (plat->deactivate_delay_us && priv->last_transaction_us) {
		unsigned int delay_us;

		delay_us = timer_get_us() - priv->last_transaction_us;

		if (delay_us < plat->deactivate_delay_us)
			udelay(plat->deactivate_delay_us - delay_us);
	}

	debug("%s: activate cs: %u, bus: '%s'\n", __func__, cs, bus->name);

	if (priv->flags & SPI_CS_GPIOS_PREFERED) {
		if (dm_gpio_is_valid(&priv->cs_gpios[cs])) {
			gpio_get_ops(priv->cs_gpios[cs].dev)->direction_output(priv->cs_gpios[cs].dev, priv->cs_gpios[cs].offset,
									       (priv->cs_gpios[cs].flags & GPIOD_ACTIVE_LOW) ? 1 : 0);
			ndelay(150);
		}
	}

	reg = readl(&priv->regs->xfer_ctl);
	reg &= ~(SUNXI_SPI_CTL_CS_MASK | SUNXI_SPI_CTL_CS_LEVEL);
	reg |= SUNXI_SPI_CTL_CS(cs);
	writel(reg, &priv->regs->xfer_ctl);

	if (plat->activate_delay_us)
		udelay(plat->activate_delay_us);
}

static void sunxi_spi_cs_deactivate(struct udevice *dev, unsigned int cs)
{
	struct udevice *bus = dev->parent;
	struct sunxi_spi_platdata *plat = dev_get_platdata(bus);
	struct sunxi_spi_priv *priv = dev_get_priv(bus);
	uint32_t reg;

	debug("%s: deactivate cs: %u, bus: '%s'\n", __func__, cs, bus->name);

	if (priv->flags & SPI_CS_GPIOS_PREFERED) {
		if (dm_gpio_is_valid(&priv->cs_gpios[cs])) {
			gpio_get_ops(priv->cs_gpios[cs].dev)->direction_output(priv->cs_gpios[cs].dev, priv->cs_gpios[cs].offset,
									       (priv->cs_gpios[cs].flags & GPIOD_ACTIVE_LOW) ? 0 : 1);
			ndelay(150);
		}
	}

	reg = readl(&priv->regs->xfer_ctl);
	reg &= ~SUNXI_SPI_CTL_CS_MASK;
	reg |= SUNXI_SPI_CTL_CS(cs);
	reg |= SUNXI_SPI_CTL_CS_LEVEL;
	writel(reg, &priv->regs->xfer_ctl);

	/* 
	 * Remember the time of this transaction so that we can honour the bus
	 * delay.
	 */
	if (plat->deactivate_delay_us)
		priv->last_transaction_us = timer_get_us();
}

static int sunxi_spi_ofdata_to_platdata(struct udevice *bus)
{
	struct sunxi_spi_platdata *plat = dev_get_platdata(bus);
	const void *blob = gd->fdt_blob;
	int node = dev_of_offset(bus);

	plat->regs = (struct sunxi_spi_regs *)devfdt_get_addr(bus);
	plat->activate_delay_us = fdtdec_get_int(
		blob, node, "spi-activate-delay", 0);
	plat->deactivate_delay_us = fdtdec_get_int(
		blob, node, "spi-deactivate-delay", 0);

	debug("%s: regs=%p, activate-delay=%u, deactivate-delay=%u\n",
		__func__, plat->regs, plat->activate_delay_us,
		plat->deactivate_delay_us);

	return 0;
}

static int sunxi_spi_probe(struct udevice *bus)
{
	struct sunxi_spi_platdata *plat = dev_get_platdata(bus);
	struct sunxi_spi_priv *priv = dev_get_priv(bus);
	int i, err;

	debug("%s: probe\n", __func__);

	priv->regs = plat->regs;
	priv->last_transaction_us = timer_get_us();

	err = gpio_request_list_by_name(bus, "cs-gpios", priv->cs_gpios,
					SUNXI_SPI_MAX_CS_COUNT, 0);
	if (err > 0) {
		priv->flags |= SPI_CS_GPIOS_PREFERED;

		for (i = 0; i < SUNXI_SPI_MAX_CS_COUNT; i++) {
			if (dm_gpio_is_valid(&priv->cs_gpios[i])) {
				dm_gpio_set_dir_flags(&priv->cs_gpios[i], GPIOD_IS_OUT);
				dm_gpio_set_value(&priv->cs_gpios[i], 1);
				ndelay(150);

				/* for using GPIO directly in SPI driver */
				dm_gpio_free(bus, &priv->cs_gpios[i]);
			} else {
				priv->cs_gpios[i].dev = NULL;
			}
		}
	} else {
		for (i = 0; i < SUNXI_SPI_MAX_CS_COUNT; i++) {
			priv->cs_gpios[i].dev = NULL;
		}
	}

	return 0;
}

static int sunxi_spi_claim_bus(struct udevice *dev)
{
	struct udevice *bus = dev->parent;
	struct sunxi_spi_priv *priv = dev_get_priv(bus);

	debug("%s: claiming bus\n", __func__);

	sunxi_spi_parse_pins(bus);
	sunxi_spi_enable_clock(bus);
	setbits_le32(&priv->regs->glb_ctl, SUNXI_SPI_CTL_MASTER |
		SUNXI_SPI_CTL_ENABLE | SUNXI_SPI_CTL_TP | SUNXI_SPI_CTL_SRST);

	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I))
		while (readl(&priv->regs->glb_ctl) & SUNXI_SPI_CTL_SRST)
			;

	setbits_le32(&priv->regs->xfer_ctl, SUNXI_SPI_CTL_CS_MANUAL |
		SUNXI_SPI_CTL_CS_LEVEL);
	setbits_le32(&priv->regs->fifo_ctl, SUNXI_SPI_CTL_RF_RST |
		SUNXI_SPI_CTL_TF_RST);

	return 0;
}

static int sunxi_spi_release_bus(struct udevice *dev)
{
	struct udevice *bus = dev->parent;
	struct sunxi_spi_priv *priv = dev_get_priv(bus);

	debug("%s: releasing bus\n", __func__);

	clrbits_le32(&priv->regs->glb_ctl, SUNXI_SPI_CTL_MASTER |
		SUNXI_SPI_CTL_ENABLE);
	sunxi_spi_disable_clock();

	return 0;
}

static void sunxi_spi_write(struct udevice *dev, const char *tx_buf,
	size_t nbytes)
{
	struct udevice *bus = dev->parent;
	struct sunxi_spi_priv *priv = dev_get_priv(bus);
	size_t i;
	char byte;

	if (!tx_buf)
		nbytes = 0;

	writel(SUNXI_SPI_XMIT_CNT(nbytes), &priv->regs->xmit_cnt);

	if (IS_ENABLED(CONFIG_SUNXI_GEN_SUN6I))
		writel(SUNXI_SPI_BURST_CNT(nbytes), &priv->regs->burst_ctl);

	for (i = 0; i < nbytes; ++i) {
		byte = *tx_buf++;
		writeb(byte, &priv->regs->tx_data);
	}
}

static int sunxi_spi_xfer(struct udevice *dev, unsigned int bitlen,
	const void *dout, void *din, unsigned long flags)
{
	struct udevice *bus = dev->parent;
	struct sunxi_spi_priv *priv = dev_get_priv(bus);
	struct dm_spi_slave_platdata *slave_plat = dev_get_parent_platdata(dev);
	const char *tx_buf = dout;
	char *rx_buf = din;
	size_t len = bitlen / 8;
	size_t i, nbytes;
	char byte;
	unsigned long timeout;
	int err = 0;

	if (bitlen % 8) {
		debug("%s: non byte-aligned SPI transfer.\n", __func__);
		return -1;
	}

	if (flags & SPI_XFER_BEGIN)
		sunxi_spi_cs_activate(dev, slave_plat->cs);

	while (len) {
		nbytes = min(len, (size_t)64 - 1);

		writel(SUNXI_SPI_BURST_CNT(nbytes), &priv->regs->burst_cnt);
		sunxi_spi_write(dev, tx_buf, nbytes);
		setbits_le32(&priv->regs->xfer_ctl, SUNXI_SPI_CTL_XCH);

		timeout = timer_get_us() + 200000;
		while (((readl(&priv->regs->fifo_sta) &
			SUNXI_SPI_FIFO_RF_CNT_MASK) >>
			SUNXI_SPI_FIFO_RF_CNT_BITS) < nbytes) {
			if (time_after(timer_get_us(), timeout)) {
				err = -1;
				break;
			}
		}
		if (err != 0) break;

		for (i = 0; i < nbytes; ++i) {
			byte = readb(&priv->regs->rx_data);

			if (rx_buf)
				*rx_buf++ = byte;
		}

		len -= nbytes;
		if (tx_buf)
			tx_buf += nbytes;
	}

	if (flags & SPI_XFER_END)
		sunxi_spi_cs_deactivate(dev, slave_plat->cs);

	return err;
}

static int sunxi_spi_set_speed(struct udevice *bus, uint speed)
{
	struct sunxi_spi_priv *priv = dev_get_priv(bus);
	unsigned int div;
	uint32_t reg;

	speed = min(speed, (unsigned int)SUNXI_SPI_MAX_RATE);
	speed = max((unsigned int)SUNXI_SPI_MIN_RATE, speed);

	div = SUNXI_SPI_MAX_RATE / (2 * speed);

	if (div <= (SUNXI_SPI_CLK_CTL_CDR2_MASK + 1)) {
		if (div > 0)
			div--;

		reg = SUNXI_SPI_CLK_CTL_CDR2(div) | SUNXI_SPI_CLK_CTL_DRS;
	} else {
		div = __ilog2(SUNXI_SPI_MAX_RATE) - __ilog2(speed);
		reg = SUNXI_SPI_CLK_CTL_CDR1(div);
	}

	writel(reg, &priv->regs->clk_ctl);

	debug("%s: speed=%u\n", __func__, speed);

	return 0;
}

static int sunxi_spi_set_mode(struct udevice *bus, uint mode)
{
	struct sunxi_spi_priv *priv = dev_get_priv(bus);
	uint32_t reg;

	reg = readl(&priv->regs->xfer_ctl);
	reg &= ~(SUNXI_SPI_CTL_CPOL | SUNXI_SPI_CTL_CPHA |
		SUNXI_SPI_CTL_CS_ACTIVE_LOW);

	if (mode & SPI_CPOL)
		reg |= SUNXI_SPI_CTL_CPOL;

	if (mode & SPI_CPHA)
		reg |= SUNXI_SPI_CTL_CPHA;

	if (!(mode & SPI_CS_HIGH))
		reg |= SUNXI_SPI_CTL_CS_ACTIVE_LOW;

	writel(reg, &priv->regs->xfer_ctl);

	debug("%s: mode=%d\n", __func__, mode);

	return 0;
}

static const struct dm_spi_ops sunxi_spi_ops = {
	.claim_bus	= sunxi_spi_claim_bus,
	.release_bus	= sunxi_spi_release_bus,
	.xfer		= sunxi_spi_xfer,
	.set_speed	= sunxi_spi_set_speed,
	.set_mode	= sunxi_spi_set_mode,
};

static const struct udevice_id sunxi_spi_ids[] = {
	{ .compatible = "allwinner,sun4i-a10-spi" },
	{ .compatible = "allwinner,sun6i-a31-spi" },
	{ .compatible = "allwinner,sun8i-h3-spi" },
	{ .compatible = "allwinner,sun50i-a64-spi" },
	{ }
};

U_BOOT_DRIVER(sunxi_spi) = {
	.name	= "sunxi_spi",
	.id	= UCLASS_SPI,
	.of_match = sunxi_spi_ids,
	.ops	= &sunxi_spi_ops,
	.ofdata_to_platdata = sunxi_spi_ofdata_to_platdata,
	.platdata_auto_alloc_size = sizeof(struct sunxi_spi_platdata),
	.priv_auto_alloc_size = sizeof(struct sunxi_spi_priv),
	.probe	= sunxi_spi_probe,
};
