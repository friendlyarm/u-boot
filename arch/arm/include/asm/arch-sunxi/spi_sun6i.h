/*
 * (C) Copyright 2017 Whitebox Systems / Northend Systems B.V.
 * S.J.R. van Schaik <stephan@whiteboxsystems.nl>
 * M.B.W. Wajer <merlijn@whiteboxsystems.nl>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef _SUNXI_SPI_SUN6I_H
#define _SUNXI_SPI_SUN6I_H

struct sunxi_spi_regs {
	uint32_t unused0[1];
	uint32_t glb_ctl;	/* 0x04 */
	uint32_t xfer_ctl;	/* 0x08 */
	uint32_t unused1[1];
	uint32_t int_ctl;	/* 0x10 */
	uint32_t int_sta;	/* 0x14 */
	uint32_t fifo_ctl;	/* 0x18 */
	uint32_t fifo_sta;	/* 0x1c */
	uint32_t wait;		/* 0x20 */
	uint32_t clk_ctl;	/* 0x24 */
	uint32_t unused2[2];
	uint32_t burst_cnt;	/* 0x30 */
	uint32_t xmit_cnt;	/* 0x34 */
	uint32_t burst_ctl;	/* 0x38 */
	uint32_t unused3[113];
	uint32_t tx_data;	/* 0x200 */
	uint32_t unused4[63];
	uint32_t rx_data;	/* 0x300 */
};

#define SUNXI_SPI_CTL_ENABLE		BIT(0)
#define SUNXI_SPI_CTL_MASTER		BIT(1)
#define SUNXI_SPI_CTL_TP		BIT(7)
#define SUNXI_SPI_CTL_SRST		BIT(31)

#define SUNXI_SPI_CTL_CPHA		BIT(0)
#define SUNXI_SPI_CTL_CPOL		BIT(1)
#define SUNXI_SPI_CTL_CS_ACTIVE_LOW	BIT(2)
#define SUNXI_SPI_CTL_CS_MASK		0x30
#define SUNXI_SPI_CTL_CS(cs)		(((cs) << 4) & SUNXI_SPI_CTL_CS_MASK)
#define SUNXI_SPI_CTL_CS_MANUAL		BIT(6)
#define SUNXI_SPI_CTL_CS_LEVEL		BIT(7)
#define SUNXI_SPI_CTL_DHB		BIT(8)
#define SUNXI_SPI_CTL_XCH		BIT(31)

#define SUNXI_SPI_CTL_RF_RST		BIT(15)
#define SUNXI_SPI_CTL_TF_RST		BIT(31)

#define SUNXI_SPI_FIFO_RF_CNT_MASK	0x7f
#define SUNXI_SPI_FIFO_RF_CNT_BITS	0
#define SUNXI_SPI_FIFO_TF_CNT_MASK	0x7f
#define SUNXI_SPI_FIFO_TF_CNT_BITS	16

#endif /* _SUNXI_SPI_SUN6I_H */
