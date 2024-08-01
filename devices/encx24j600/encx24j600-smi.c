/*
 * Microchip ENCX24J600 ethernet driver (MAC + PHY)
 *
 * Copyright (C) 2014 modusoft GmbH
 * Author: Sascha Ittner <sascha.ittner@modusoft.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include "encx24j600.h"
#include "encx24j600_hw.h"

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/broadcom/bcm2835_smi.h>

#define DRV_NAME	"encx24j600-smi"
#define DRV_VERSION	"1.0"

#define SET_OFFSET	0x0100
#define CLR_OFFSET	0x0180

static const u16 memwin_regs[WIN_COUNT] = {
	EUDADATA,
	EGPDATA,
	ERXDATA
};

struct encx24j600_smi_ctx {
	struct encx24j600_priv priv;

	struct bcm2835_smi_instance *smi_inst;
	struct mutex lock;
};

static inline void select_reg(struct bcm2835_smi_instance *inst, u16 reg)
{
	u8 lo, hi;
	lo = reg & 0xff;
	hi = (reg >> 8) & 0x01;

	// latch address
	bcm2835_smi_set_address(inst, hi | 2); // set AS enable
	bcm2835_smi_write_buf(inst, &lo, 1);
	bcm2835_smi_set_address(inst, hi);
}

static void write_reg(struct bcm2835_smi_instance *inst, u16 reg, u16 val)
{
	u8 lo, hi;
	lo = val & 0xff;
	hi = (val >> 8) & 0xff;

	select_reg(inst, reg);
	bcm2835_smi_write_buf(inst, &lo, 1);
	select_reg(inst, reg + 1);
	bcm2835_smi_write_buf(inst, &hi, 1);
}

static u16 read_reg(struct bcm2835_smi_instance *inst, u16 reg)
{
	u8 lo, hi;

	select_reg(inst, reg);
	bcm2835_smi_read_buf(inst, &lo, 1);
	select_reg(inst, reg + 1);
	bcm2835_smi_read_buf(inst, &hi, 1);

	return lo | (hi << 8);
}

static u16 encx24j600_smi_read_reg(struct encx24j600_priv *priv, u8 reg)
{
	struct encx24j600_smi_ctx *ctx = container_of(priv, struct encx24j600_smi_ctx, priv);
	u16 val = 0;

	mutex_lock(&ctx->lock);
	val = read_reg(ctx->smi_inst, reg);
	mutex_unlock(&ctx->lock);
	return val;
}

static void encx24j600_smi_write_reg(struct encx24j600_priv *priv, u8 reg, u16 val)
{
	struct encx24j600_smi_ctx *ctx = container_of(priv, struct encx24j600_smi_ctx, priv);

	mutex_lock(&ctx->lock);
	write_reg(ctx->smi_inst, reg, val);
	mutex_unlock(&ctx->lock);
}

static void encx24j600_smi_clr_bits(struct encx24j600_priv *priv, u8 reg, u16 mask)
{
	struct encx24j600_smi_ctx *ctx = container_of(priv, struct encx24j600_smi_ctx, priv);

	mutex_lock(&ctx->lock);
	write_reg(ctx->smi_inst, reg + CLR_OFFSET, mask);
	mutex_unlock(&ctx->lock);
}

static void encx24j600_smi_set_bits(struct encx24j600_priv *priv, u8 reg, u16 mask)
{
	struct encx24j600_smi_ctx *ctx = container_of(priv, struct encx24j600_smi_ctx, priv);

	mutex_lock(&ctx->lock);
	write_reg(ctx->smi_inst, reg + SET_OFFSET, mask);
	mutex_unlock(&ctx->lock);
}

static void encx24j600_smi_cmd(struct encx24j600_priv *priv, enum encx24j600_byte_cmd cmd)
{
	struct encx24j600_smi_ctx *ctx = container_of(priv, struct encx24j600_smi_ctx, priv);
	struct bcm2835_smi_instance *inst = ctx->smi_inst;
	u16 val;

	mutex_lock(&ctx->lock);
	switch(cmd) {
		case CMD_SETETHRST:
			write_reg(inst, ECON2 + SET_OFFSET, ETHRST);
			break;
		case CMD_FCDISABLE:
			val = read_reg(inst, ECON1);
			val &= ~(FCOP1 | FCOP0);
			write_reg(inst, ECON1, val);
			break;
		case CMD_FCSINGLE:
			val = read_reg(inst, ECON1);
			val &= ~(FCOP1 | FCOP0);
			val |= FCOP0;
			write_reg(inst, ECON1, val);
			break;
		case CMD_FCMULTIPLE:
			val = read_reg(inst, ECON1);
			val &= ~(FCOP1 | FCOP0);
			val |= FCOP1;
			write_reg(inst, ECON1, val);
			break;
		case CMD_FCCLEAR:
			val = read_reg(inst, ECON1);
			val |= (FCOP1 | FCOP0);
			write_reg(inst, ECON1, val);
			break;
		case CMD_SETPKTDEC:
			write_reg(inst, ECON1 + SET_OFFSET, PKTDEC);
			break;
		case CMD_DMASTOP:
			write_reg(inst, ECON1 + CLR_OFFSET, DMAST);
			break;
		case CMD_DMACKSUM:
			val = read_reg(inst, ECON1);
			val &= ~(DMAST | DMACPY | DMACSSD | DMANOCS);
			val |= (DMAST);
			write_reg(inst, ECON1, val);
			break;
		case CMD_DMACKSUMS:
			val = read_reg(inst, ECON1);
			val &= ~(DMAST | DMACPY | DMACSSD | DMANOCS);
			val |= (DMAST | DMACSSD);
			write_reg(inst, ECON1, val);
			break;
		case CMD_DMACOPY:
			val = read_reg(inst, ECON1);
			val &= ~(DMAST | DMACPY | DMACSSD | DMANOCS);
			val |= (DMAST | DMACPY);
			write_reg(inst, ECON1, val);
			break;
		case CMD_DMACOPYS:
			val = read_reg(inst, ECON1);
			val &= ~(DMAST | DMACPY | DMACSSD | DMANOCS);
			val |= (DMAST | DMACPY | DMACSSD);
			write_reg(inst, ECON1, val);
			break;
		case CMD_SETTXRTS:
			write_reg(inst, ECON1 + SET_OFFSET, TXRTS);
			break;
		case CMD_ENABLERX:
			write_reg(inst, ECON1 + SET_OFFSET, RXEN);
			break;
		case CMD_DISABLERX:
			write_reg(inst, ECON1 + CLR_OFFSET, RXEN);
			break;
		case CMD_SETEIE:
			write_reg(inst, ESTAT + SET_OFFSET, INT);
			break;
		case CMD_CLREIE:
			write_reg(inst, ESTAT + CLR_OFFSET, INT);
			break;
		default:
			break;
	}
	mutex_unlock(&ctx->lock);
}

static void encx24j600_smi_read_mem(struct encx24j600_priv *priv, enum encx24j600_memwin win, u8 * data, size_t count)
{
	struct encx24j600_smi_ctx *ctx = container_of(priv, struct encx24j600_smi_ctx, priv);
	struct bcm2835_smi_instance *inst = ctx->smi_inst;
	size_t block;

	mutex_lock(&ctx->lock);

	// select window
	select_reg(inst, memwin_regs[win]);

	// transfer data (use 8 bit mode)
    if (priv->ecdev) {
	// avoid (IRQ dependend) DMA transfers
	while (count > 0) {
		block = count > DMA_THRESHOLD_BYTES ? DMA_THRESHOLD_BYTES : count;
		bcm2835_smi_read_buf(inst, data, block);
		data += block; count -= block;
	}
    } else {
	bcm2835_smi_read_buf(inst, data, count);
    }

	mutex_unlock(&ctx->lock);
}

static void encx24j600_smi_write_mem(struct encx24j600_priv *priv, enum encx24j600_memwin win, const u8 * data, size_t count)
{
	struct encx24j600_smi_ctx *ctx = container_of(priv, struct encx24j600_smi_ctx, priv);
	struct bcm2835_smi_instance *inst = ctx->smi_inst;
	size_t block;

	mutex_lock(&ctx->lock);

	// select window
	select_reg(inst, memwin_regs[win]);

	// transfer data (use 8 bit mode)
    if (priv->ecdev) {
	// avoid (IRQ dependend) DMA transfers
	while (count > 0) {
		block = count > DMA_THRESHOLD_BYTES ? DMA_THRESHOLD_BYTES : count;
		bcm2835_smi_write_buf(inst, data, block);
		data += block; count -= block;
	}
    } else {
	bcm2835_smi_write_buf(inst, data, count);
    }

	mutex_unlock(&ctx->lock);
}

static int encx24j600_smi_probe(struct platform_device *pdev)
{
	struct device_node *node, *smi_node;
	struct bcm2835_smi_instance *smi_inst;
	int irq;
	struct smi_settings *smi_settings;
	struct net_device *ndev;
	struct encx24j600_smi_ctx *ctx;

	node = pdev->dev.of_node;
	if (!node) {
		dev_err(&pdev->dev, "No device tree node supplied!");
		return -EINVAL;
	}

	smi_node = of_parse_phandle(node, "smi_handle", 0);
	if (!smi_node) {
		dev_err(&pdev->dev, "No device smi_handle supplied!");
		return -EINVAL;
	}

	// Request use of SMI peripheral
	smi_inst = bcm2835_smi_get(smi_node);
	if (!smi_inst) {
		dev_err(&pdev->dev, "Could not register with SMI.");
		return -EPROBE_DEFER;
	}

	// get interrupt
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
 		dev_err(&pdev->dev, "platform_get_irq failed.\n");
 		return irq;
 	}

	// Set SMI timing and bus width
	smi_settings = bcm2835_smi_get_settings_from_regs(smi_inst);

	smi_settings->data_width = SMI_WIDTH_8BIT;

	smi_settings->read_setup_time = 2;
	smi_settings->read_hold_time = 2;
	smi_settings->read_pace_time = 1;
	smi_settings->read_strobe_time = 10;

	smi_settings->write_setup_time = 2;
	smi_settings->write_hold_time = 2;
	smi_settings->write_pace_time = 5;
	smi_settings->write_strobe_time = 2;

	bcm2835_smi_set_regs_from_settings(smi_inst);

	ndev = alloc_etherdev(sizeof(struct encx24j600_smi_ctx));
	if (!ndev) {
		return -ENOMEM;
	}

	ctx = netdev_priv(ndev);
	ctx->priv.ndev = ndev;

	platform_set_drvdata(pdev, ctx);

	SET_NETDEV_DEV(ndev, &pdev->dev);
	ndev->irq = irq;
	ctx->smi_inst = smi_inst;
	mutex_init(&ctx->lock);

	ctx->priv.read_reg = encx24j600_smi_read_reg;
	ctx->priv.write_reg = encx24j600_smi_write_reg;
	ctx->priv.clr_bits = encx24j600_smi_clr_bits;
	ctx->priv.set_bits = encx24j600_smi_set_bits;
	ctx->priv.cmd = encx24j600_smi_cmd;
	ctx->priv.read_mem = encx24j600_smi_read_mem;
	ctx->priv.write_mem = encx24j600_smi_write_mem;

	return encx24j600_probe(&ctx->priv);
}

static int encx24j600_smi_remove(struct platform_device *pdev)
{
	struct encx24j600_smi_ctx *ctx = platform_get_drvdata(pdev);

	encx24j600_remove(&ctx->priv);

	return 0;
}

static const struct of_device_id encx24j600_smi_id_table[] = {
	{.compatible = "microchip,encx24j600-smi",},
	{ /* sentinel */ }
};
// prevent automatic loading
//MODULE_DEVICE_TABLE(of, encx24j600_smi_id_table);

static struct platform_driver encx24j600_smi_driver = {
	.probe = encx24j600_smi_probe,
	.remove = encx24j600_smi_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = encx24j600_smi_id_table,
	},
};
module_platform_driver(encx24j600_smi_driver);

MODULE_DESCRIPTION(DRV_NAME " ethernet driver (EtherCAT)");
MODULE_AUTHOR("Sascha Ittner <sascha.ittner@modusoft.de>");
MODULE_LICENSE("GPL");
