/*
 * (C) Copyright 2017 Whitebox Systems / Northend Systems B.V.
 * S.J.R. van Schaik <stephan@whiteboxsystems.nl>
 * M.B.W. Wajer <merlijn@whiteboxsystems.nl>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _SUNXI_SPI_SUN4I_H
#define _SUNXI_SPI_SUN4I_H

struct sunxi_spi_regs {
	uint32_t rx_data;	/* 0x00 */
	uint32_t tx_data;	/* 0x04 */
	union {
		uint32_t glb_ctl;
		uint32_t xfer_ctl;
		uint32_t fifo_ctl;
		uint32_t burst_ctl;
	};			/* 0x08 */
	uint32_t int_ctl;	/* 0x0c */
	uint32_t int_sta;	/* 0x10 */
	uint32_t dma_ctl;	/* 0x14 */
	uint32_t wait;		/* 0x18 */
	uint32_t clk_ctl;	/* 0x1c */
	uint32_t burst_cnt;	/* 0x20 */
	uint32_t xmit_cnt;	/* 0x24 */
	uint32_t fifo_sta;	/* 0x28 */
};

#define SUNXI_SPI_CTL_SRST		0

#define SUNXI_SPI_CTL_ENABLE		BIT(0)
#define SUNXI_SPI_CTL_MASTER		BIT(1)
#define SUNXI_SPI_CTL_CPHA		BIT(2)
#define SUNXI_SPI_CTL_CPOL		BIT(3)
#define SUNXI_SPI_CTL_CS_ACTIVE_LOW	BIT(4)
#define SUNXI_SPI_CTL_TF_RST		BIT(8)
#define SUNXI_SPI_CTL_RF_RST		BIT(9)
#define SUNXI_SPI_CTL_XCH		BIT(10)
#define SUNXI_SPI_CTL_CS_MASK		0x3000
#define SUNXI_SPI_CTL_CS(cs)		(((cs) << 12) & SUNXI_SPI_CTL_CS_MASK)
#define SUNXI_SPI_CTL_DHB		BIT(15)
#define SUNXI_SPI_CTL_CS_MANUAL		BIT(16)
#define SUNXI_SPI_CTL_CS_LEVEL		BIT(17)
#define SUNXI_SPI_CTL_TP		BIT(18)

#define SUNXI_SPI_FIFO_RF_CNT_MASK	0x7f
#define SUNXI_SPI_FIFO_RF_CNT_BITS	0
#define SUNXI_SPI_FIFO_TF_CNT_MASK	0x7f
#define SUNXI_SPI_FIFO_TF_CNT_BITS	16

#endif /* _SUNXI_SPI_SUN4I_H */
