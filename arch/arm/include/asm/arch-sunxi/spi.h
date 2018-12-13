/*
 * (C) Copyright 2017 Whitebox Systems / Northend Systems B.V.
 * S.J.R. van Schaik <stephan@whiteboxsystems.nl>
 * M.B.W. Wajer <merlijn@whiteboxsystems.nl>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _SUNXI_SPI_H
#define _SUNXI_SPI_H

#if defined(CONFIG_MACH_SUN6I) || defined(CONFIG_MACH_SUN8I) || \
	defined(CONFIG_MACH_SUN9I) || defined(CONFIG_MACH_SUN50I)
#include <asm/arch/spi_sun6i.h>
#else
#include <asm/arch/spi_sun4i.h>
#endif

#define SUNXI_SPI_BURST_CNT(cnt)	((cnt) & 0xffffff)
#define SUNXI_SPI_XMIT_CNT(cnt)		((cnt) & 0xffffff)

#define SUNXI_SPI_CLK_CTL_CDR2_MASK	0xff
#define SUNXI_SPI_CLK_CTL_CDR2(div)	((div) & SUNXI_SPI_CLK_CTL_CDR2_MASK)
#define SUNXI_SPI_CLK_CTL_CDR1_MASK	0xf
#define SUNXI_SPI_CLK_CTL_CDR1(div)		\
	(((div) & SUNXI_SPI_CLK_CTL_CDR1_MASK) << 8)
#define SUNXI_SPI_CLK_CTL_DRS		BIT(12)

#endif /* _SUNXI_SPI_H */
