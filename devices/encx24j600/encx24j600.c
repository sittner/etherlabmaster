/**
 * Microchip ENCX24J600 ethernet driver
 *
 * Copyright (C) 2015 Gridpoint
 * Author: Jon Ringle <jringle@gridpoint.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include "encx24j600.h"
#include "encx24j600_hw.h"

#include <linux/slab.h>
#include <linux/ethtool.h>
#include <linux/skbuff.h>

#define DRV_NAME	"encx24j600"
#define DRV_VERSION	"1.0"

#define DEFAULT_MSG_ENABLE (NETIF_MSG_DRV | NETIF_MSG_PROBE | NETIF_MSG_LINK)

static int debug = -1;
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debug level (0=none,...,16=all)");

#define RESET_TIMEOUT_US  2000
#define PHY_TIMEOUT_US    1000
#define AUTONEG_TIMOUT_MS 2000

enum {
	RXFILTER_NORMAL,
	RXFILTER_MULTI,
	RXFILTER_PROMISC
};

static void dump_packet(const char *msg, int len, const char *data)
{
	pr_debug(DRV_NAME ": %s - packet len:%d\n", msg, len);
	print_hex_dump_bytes("pk data: ", DUMP_PREFIX_OFFSET, data, len);
}

static void encx24j600_dump_rsv(struct encx24j600_priv *priv, const char *msg,
				struct rsv *rsv)
{
	struct net_device *dev = priv->ndev;

	netdev_info(dev, "RX packet Len:%d\n", rsv->len);
	netdev_dbg(dev, "%s - NextPk: 0x%04x\n", msg, rsv->next_packet);
	netdev_dbg(dev, "RxOK: %d, DribbleNibble: %d\n",
		   RSV_GETBIT(rsv->rxstat, RSV_RXOK),
		   RSV_GETBIT(rsv->rxstat, RSV_DRIBBLENIBBLE));
	netdev_dbg(dev, "CRCErr:%d, LenChkErr: %d, LenOutOfRange: %d\n",
		   RSV_GETBIT(rsv->rxstat, RSV_CRCERROR),
		   RSV_GETBIT(rsv->rxstat, RSV_LENCHECKERR),
		   RSV_GETBIT(rsv->rxstat, RSV_LENOUTOFRANGE));
	netdev_dbg(dev, "Multicast: %d, Broadcast: %d, LongDropEvent: %d, CarrierEvent: %d\n",
		   RSV_GETBIT(rsv->rxstat, RSV_RXMULTICAST),
		   RSV_GETBIT(rsv->rxstat, RSV_RXBROADCAST),
		   RSV_GETBIT(rsv->rxstat, RSV_RXLONGEVDROPEV),
		   RSV_GETBIT(rsv->rxstat, RSV_CARRIEREV));
	netdev_dbg(dev, "ControlFrame: %d, PauseFrame: %d, UnknownOp: %d, VLanTagFrame: %d\n",
		   RSV_GETBIT(rsv->rxstat, RSV_RXCONTROLFRAME),
		   RSV_GETBIT(rsv->rxstat, RSV_RXPAUSEFRAME),
		   RSV_GETBIT(rsv->rxstat, RSV_RXUNKNOWNOPCODE),
		   RSV_GETBIT(rsv->rxstat, RSV_RXTYPEVLAN));
}

static u16 encx24j600_read_phy(struct encx24j600_priv *priv, u8 reg)
{
	unsigned long timeout;
	u16 val;

	mutex_lock(&priv->phy_lock);

	// Set the right address and start the register read operation
	priv->write_reg(priv, MIREGADR, MIREGADR_VAL | (reg & PHREG_MASK));
	priv->write_reg(priv, MICMD, MIIRD);

	// Loop to wait until the PHY register has been read through the MII
	// This requires 25.6us
	usleep_range(26, 100);
	timeout = jiffies + usecs_to_jiffies(PHY_TIMEOUT_US);
	while(priv->read_reg(priv, MISTAT) & BUSY) {
		if (time_after(jiffies, timeout)) {
			mutex_unlock(&priv->phy_lock);
			return 0;
		}
		cpu_relax();
	}

	// Stop reading
	priv->write_reg(priv, MICMD, 0);
	
	// Obtain results and return
	val = priv->read_reg(priv, MIRD);

	mutex_unlock(&priv->phy_lock);
	return val;
}

static void encx24j600_write_phy(struct encx24j600_priv *priv, u8 reg, u16 val)
{
	unsigned long timeout;

	mutex_lock(&priv->phy_lock);

	// Write the register address
	priv->write_reg(priv, MIREGADR,  MIREGADR_VAL | (reg & PHREG_MASK));
	
	// Write the data
	priv->write_reg(priv, MIWR, val);

	// Wait until the PHY register has been written
	usleep_range(26, 100);
	timeout = jiffies + usecs_to_jiffies(PHY_TIMEOUT_US);
	while(priv->read_reg(priv, MISTAT) & BUSY) {
		if (time_after(jiffies, timeout)) {
			mutex_unlock(&priv->phy_lock);
			return;
		}
		cpu_relax();
	}

	mutex_unlock(&priv->phy_lock);
}

static void encx24j600_update_phcon1(struct encx24j600_priv *priv)
{
	u16 phcon1 = encx24j600_read_phy(priv, PHCON1);
	if (priv->autoneg == AUTONEG_ENABLE) {
		phcon1 |= ANEN | RENEG;
	} else {
		phcon1 &= ~ANEN;
		if (priv->speed == SPEED_100)
			phcon1 |= SPD100;
		else
			phcon1 &= ~SPD100;

		if (priv->full_duplex)
			phcon1 |= PFULDPX;
		else
			phcon1 &= ~PFULDPX;
	}
	encx24j600_write_phy(priv, PHCON1, phcon1);
}

/* Waits for autonegotiation to complete. */
static int encx24j600_wait_for_autoneg(struct encx24j600_priv *priv)
{
	struct net_device *dev = priv->ndev;
	unsigned long timeout;
	u16 phstat1;
	u16 estat;

	phstat1 = encx24j600_read_phy(priv, PHSTAT1);
	timeout = jiffies + msecs_to_jiffies(AUTONEG_TIMOUT_MS);
	while ((phstat1 & ANDONE) == 0) {
		if (time_after(jiffies, timeout)) {
			u16 phstat3;

			netif_notice(priv, drv, dev, "timeout waiting for autoneg done\n");

			priv->autoneg = AUTONEG_DISABLE;
			phstat3 = encx24j600_read_phy(priv, PHSTAT3);
			priv->speed = (phstat3 & PHY3SPD100) ? SPEED_100 : SPEED_10;
			priv->full_duplex = (phstat3 & PHY3DPX) ? 1 : 0;
			encx24j600_update_phcon1(priv);
			netif_notice(priv, drv, dev, "Using parallel detection: %s/%s",
				     priv->speed == SPEED_100 ? "100" : "10",
				     priv->full_duplex ? "Full" : "Half");

			return -ETIMEDOUT;
		}
		cpu_relax();
		phstat1 = encx24j600_read_phy(priv, PHSTAT1);
	}

	estat = priv->read_reg(priv, ESTAT);
	if (estat & PHYDPX) {
		priv->set_bits(priv, MACON2, FULDPX);
		priv->write_reg(priv, MABBIPG, 0x15);
	} else {
		priv->clr_bits(priv, MACON2, FULDPX);
		priv->write_reg(priv, MABBIPG, 0x12);
		/* Max retransmittions attempt  */
		priv->write_reg(priv, MACLCON, 0x370f);
	}

	return 0;
}

/* Access the PHY to determine link status */
static void encx24j600_check_link_status(struct encx24j600_priv *priv)
{
	struct net_device *dev = priv->ndev;
	u16 estat;

	estat = priv->read_reg(priv, ESTAT);

	if (estat & PHYLNK) {
		if (priv->autoneg == AUTONEG_ENABLE)
			encx24j600_wait_for_autoneg(priv);

	    if (priv->ecdev) {
		ecdev_set_link(priv->ecdev, 1);
	    } else {
		netif_carrier_on(dev);
	    }

		netif_info(priv, ifup, dev, "link up\n");
	} else {
		netif_info(priv, ifdown, dev, "link down\n");

		/* Re-enable autoneg since we won't know what we might be
		 * connected to when the link is brought back up again.
		 */
		priv->autoneg = AUTONEG_ENABLE;
		priv->full_duplex = true;
		priv->speed = SPEED_100;

	    if (priv->ecdev) {
		ecdev_set_link(priv->ecdev, 0);
	    } else {
		netif_carrier_off(dev);
	    }
	}
}

static void encx24j600_int_link_handler(struct encx24j600_priv *priv)
{
	struct net_device *dev = priv->ndev;

	netif_dbg(priv, intr, dev, "%s", __func__);
	encx24j600_check_link_status(priv);
	priv->clr_bits(priv, EIR, LINKIF);
}

static void encx24j600_reset_hw_tx(struct encx24j600_priv *priv)
{
	priv->set_bits(priv, ECON2, TXRST);
	priv->clr_bits(priv, ECON2, TXRST);
}

static void encx24j600_hw_tx_start(struct encx24j600_priv *priv)
{
	struct sk_buff *skb = priv->tx_buf_xmit->skb;

	/* transmission still active ? */
	if (priv->tx_running) {
	    return;
	}

	if (priv->read_reg(priv, EIR) & TXABTIF)
		/* Last transmition aborted due to error. Reset TX interface */
		encx24j600_reset_hw_tx(priv);

	/* Clear the TXIF and TXABTIF flag if were previously set */
	priv->clr_bits(priv, EIR, TXIF | TXABTIF);

	/* check for packets to send */
	if (priv->tx_pending <= 0)
	    return;

	/* Program the Tx buffer start pointer */
	priv->write_reg(priv, ETXST, priv->tx_buf_xmit->hw_addr);

	/* Program the packet length */
	priv->write_reg(priv, ETXLEN, skb->len);

	/* Start the transmission */
	priv->tx_running = 1;
	priv->cmd(priv, CMD_SETTXRTS);

}

static void encx24j600_tx_complete(struct encx24j600_priv *priv, bool err)
{
	struct net_device *dev = priv->ndev;
	struct sk_buff *skb = priv->tx_buf_xmit->skb;

	priv->tx_running = 0;

	if (err)
		dev->stats.tx_errors++;
	else
		dev->stats.tx_packets++;

	dev->stats.tx_bytes += skb->len;

	netif_dbg(priv, tx_done, dev, "TX Done%s\n", err ? ": Err" : "");

	/* skip to next buffer */
	priv->tx_buf_xmit->skb = NULL;
	priv->tx_buf_xmit = priv->tx_buf_xmit->next;
	priv->tx_pending--;

	atomic_inc(&priv->tx_buf_free);
    if (!priv->ecdev) {
	dev_kfree_skb(skb);
	netif_wake_queue(dev);
    }

	encx24j600_hw_tx_start(priv);
}

static int encx24j600_receive_packet(struct encx24j600_priv *priv,
				     struct rsv *rsv)
{
	struct net_device *dev = priv->ndev;
	struct sk_buff *skb;

    if (priv->ecdev) {
	priv->read_mem(priv, WIN_RX, priv->ec_rx_buf, rsv->len);
	ecdev_receive(priv->ecdev, priv->ec_rx_buf, rsv->len);
    } else {
	skb = netdev_alloc_skb(dev, rsv->len + NET_IP_ALIGN);
	if (!skb) {
		pr_err_ratelimited("RX: OOM: packet dropped\n");
		dev->stats.rx_dropped++;
		return -ENOMEM;
	}
	skb_reserve(skb, NET_IP_ALIGN);
	priv->read_mem(priv, WIN_RX, skb_put(skb, rsv->len), rsv->len);

	if (netif_msg_pktdata(priv))
		dump_packet("RX", skb->len, skb->data);

	skb->dev = dev;
	skb->protocol = eth_type_trans(skb, dev);

	netif_rx(skb);
    }

	/* Maintain stats */
	dev->stats.rx_packets++;
	dev->stats.rx_bytes += rsv->len;

	return 0;
}

static void encx24j600_rx_packets(struct encx24j600_priv *priv, u8 packet_count)
{
	struct net_device *dev = priv->ndev;

	while (packet_count--) {
		struct rsv rsv;
		u16 newrxtail;

		priv->write_reg(priv, ERXRDPT, priv->next_packet);
		priv->read_mem(priv, WIN_RX, (u8 *) & rsv, sizeof(rsv));
		le16_to_cpus(&rsv.next_packet);
		le16_to_cpus(&rsv.len);
		le32_to_cpus(&rsv.rxstat);

		if (netif_msg_rx_status(priv))
			encx24j600_dump_rsv(priv, __func__, &rsv);

		if (!RSV_GETBIT(rsv.rxstat, RSV_RXOK) ||
		    (rsv.len > MAX_FRAMELEN)) {
			netif_err(priv, rx_err, dev, "RX Error %04x\n",
				  rsv.rxstat);
			dev->stats.rx_errors++;

			if (RSV_GETBIT(rsv.rxstat, RSV_CRCERROR))
				dev->stats.rx_crc_errors++;
			if (RSV_GETBIT(rsv.rxstat, RSV_LENCHECKERR))
				dev->stats.rx_frame_errors++;
			if (rsv.len > MAX_FRAMELEN)
				dev->stats.rx_over_errors++;
		} else {
			encx24j600_receive_packet(priv, &rsv);
		}

		priv->next_packet = rsv.next_packet;

		newrxtail = priv->next_packet - 2;
		if (newrxtail == RX_BUF_START)
			newrxtail = SRAM_SIZE - 2;

		priv->cmd(priv, CMD_SETPKTDEC);
		priv->write_reg(priv, ERXTAIL, newrxtail);
	}
}

static irqreturn_t encx24j600_irq(int irq, void *dev_id)
{
	struct encx24j600_priv *priv = dev_id;

	disable_irq_nosync(irq);
	kthread_queue_work(&priv->kworker, &priv->irq_work);

	return IRQ_HANDLED;
}

static void encx24j600_irq_proc(struct kthread_work *ws)
{
	struct encx24j600_priv *priv =
	    container_of(ws, struct encx24j600_priv, irq_work);

	struct net_device *dev = priv->ndev;
	int eir;

	eir = priv->read_reg(priv, EIR);

	if (eir & LINKIF)
		encx24j600_int_link_handler(priv);

	if (eir & TXIF)
		encx24j600_tx_complete(priv, false);

	if (eir & TXABTIF)
		encx24j600_tx_complete(priv, true);

	if (eir & RXABTIF) {
		if (eir & PCFULIF) {
			/* Packet counter is full */
			netif_err(priv, rx_err, dev, "Packet counter full\n");
		}
		dev->stats.rx_dropped++;
		priv->clr_bits(priv, EIR, RXABTIF);
	}

	if (eir & PKTIF) {
		u8 packet_count;

		packet_count = priv->read_reg(priv, ESTAT) & 0xff;
		while (packet_count) {
			encx24j600_rx_packets(priv, packet_count);
			packet_count = priv->read_reg(priv, ESTAT) & 0xff;
		}
	}

    if (!priv->ecdev) {
	enable_irq(dev->irq);
    }
}

static int encx24j600_soft_reset(struct encx24j600_priv *priv)
{
	unsigned long timeout;

	/* Write and verify a test value to EUDAST */
	timeout = jiffies + usecs_to_jiffies(RESET_TIMEOUT_US);
	while(1) {
		priv->write_reg(priv, EUDAST, EUDAST_TEST_VAL);
		if (priv->read_reg(priv, EUDAST) == EUDAST_TEST_VAL) {
			break;
		}
		if (time_after(jiffies, timeout)) {
			return -ETIMEDOUT;
		}
		cpu_relax();
	}

	/* Wait for CLKRDY to become set */
	timeout = jiffies + usecs_to_jiffies(RESET_TIMEOUT_US);
	while (!(priv->read_reg(priv, ESTAT) & CLKRDY)) {
		if (time_after(jiffies, timeout)) {
			return -ETIMEDOUT;
		}
		cpu_relax();
	}

	/* Issue a System Reset command */
	priv->cmd(priv, CMD_SETETHRST);
	usleep_range(25, 100);

	/* Confirm that EUDAST has 0000h after system reset */
	if (priv->read_reg(priv, EUDAST) != 0) {
		return -EINVAL;
	}

	/* Wait for PHY register and status bits to become available */
	usleep_range(256, 1000);

	return 0;
}

static void encx24j600_hw_init_tx(struct encx24j600_priv *priv)
{
	/* Reset TX */
	encx24j600_reset_hw_tx(priv);

	/* Clear the TXIF flag if were previously set */
	priv->clr_bits(priv, EIR, TXIF | TXABTIF);

	/* Write the Tx Buffer pointer */
	priv->write_reg(priv, EGPWRPT, TX_BUF_START);

	atomic_set(&priv->tx_buf_free, TX_BUF_COUNT);
	priv->tx_buf_input = priv->tx_buf;
	priv->tx_buf_prep = priv->tx_buf;
	priv->tx_buf_xmit = priv->tx_buf;
	priv->tx_running = 0;
	priv->tx_pending = 0;
}

static void encx24j600_hw_init_rx(struct encx24j600_priv *priv)
{
	priv->cmd(priv, CMD_DISABLERX);

	/* Set up RX packet start address in the SRAM */
	priv->write_reg(priv, ERXST, RX_BUF_START);

	/* Preload the RX Data pointer to the beginning of the RX area */
	priv->write_reg(priv, ERXRDPT, RX_BUF_START);
	priv->next_packet = RX_BUF_START;

	/* Set up RX end address in the SRAM */
	priv->write_reg(priv, ERXTAIL, SRAM_SIZE - 2);

	/* Reset the  user data pointers    */
	priv->write_reg(priv, EUDAST, SRAM_SIZE);
	priv->write_reg(priv, EUDAND, SRAM_SIZE + 1);

	/* Set Max Frame length */
	priv->write_reg(priv, MAMXFL, MAX_FRAMELEN);
}

static void encx24j600_dump_config(struct encx24j600_priv *priv,
				   const char *msg)
{
	pr_info(DRV_NAME ": %s\n", msg);

	/* CHIP configuration */
	pr_info(DRV_NAME " ECON1:   %04X\n", priv->read_reg(priv, ECON1));
	pr_info(DRV_NAME " ECON2:   %04X\n", priv->read_reg(priv, ECON2));
	pr_info(DRV_NAME " ERXFCON: %04X\n", priv->read_reg(priv,
								 ERXFCON));
	pr_info(DRV_NAME " ESTAT:   %04X\n", priv->read_reg(priv, ESTAT));
	pr_info(DRV_NAME " EIR:     %04X\n", priv->read_reg(priv, EIR));
	pr_info(DRV_NAME " EIDLED:  %04X\n", priv->read_reg(priv, EIDLED));

	/* MAC layer configuration */
	pr_info(DRV_NAME " MACON1:  %04X\n", priv->read_reg(priv, MACON1));
	pr_info(DRV_NAME " MACON2:  %04X\n", priv->read_reg(priv, MACON2));
	pr_info(DRV_NAME " MAIPG:   %04X\n", priv->read_reg(priv, MAIPG));
	pr_info(DRV_NAME " MACLCON: %04X\n", priv->read_reg(priv, MACLCON));
	pr_info(DRV_NAME " MABBIPG: %04X\n", priv->read_reg(priv, MABBIPG));

	/* PHY configuation */
	pr_info(DRV_NAME " PHCON1:  %04X\n", encx24j600_read_phy(priv, PHCON1));
	pr_info(DRV_NAME " PHCON2:  %04X\n", encx24j600_read_phy(priv, PHCON2));
	pr_info(DRV_NAME " PHANA:   %04X\n", encx24j600_read_phy(priv, PHANA));
	pr_info(DRV_NAME " PHANLPA: %04X\n", encx24j600_read_phy(priv, PHANLPA));
	pr_info(DRV_NAME " PHANE:   %04X\n", encx24j600_read_phy(priv, PHANE));
	pr_info(DRV_NAME " PHSTAT1: %04X\n", encx24j600_read_phy(priv, PHSTAT1));
	pr_info(DRV_NAME " PHSTAT2: %04X\n", encx24j600_read_phy(priv, PHSTAT2));
	pr_info(DRV_NAME " PHSTAT3: %04X\n", encx24j600_read_phy(priv, PHSTAT3));
}

static void encx24j600_set_rxfilter_mode(struct encx24j600_priv *priv)
{
	switch (priv->rxfilter) {
	case RXFILTER_PROMISC:
		priv->set_bits(priv, MACON1, PASSALL);
		priv->write_reg(priv, ERXFCON, UCEN | MCEN | NOTMEEN);
		break;
	case RXFILTER_MULTI:
		priv->clr_bits(priv, MACON1, PASSALL);
		priv->write_reg(priv, ERXFCON, UCEN | CRCEN | BCEN | MCEN);
		break;
	case RXFILTER_NORMAL:
	default:
		priv->clr_bits(priv, MACON1, PASSALL);
		priv->write_reg(priv, ERXFCON, UCEN | CRCEN | BCEN);
		break;
	}
}

static void encx24j600_hw_init(struct encx24j600_priv *priv)
{
	u16 macon2;

	priv->hw_enabled = false;

	/* PHY Leds: link status,
	 * LEDA: Link State + collision events
	 * LEDB: Link State + transmit/receive events
	 */
	priv->clr_bits(priv, EIDLED, 0xff00);
	priv->set_bits(priv, EIDLED, 0xcb00);

	/* Loopback disabled */
	priv->write_reg(priv, MACON1, 0x9);

	/* interpacket gap value */
	priv->write_reg(priv, MAIPG, 0x0c12);

	/* Write the auto negotiation pattern */
	encx24j600_write_phy(priv, PHANA, PHANA_DEFAULT);

	encx24j600_update_phcon1(priv);
	encx24j600_check_link_status(priv);

	macon2 = MACON2_RSV1 | TXCRCEN | PADCFG0 | PADCFG2 | MACON2_DEFER;
	if ((priv->autoneg == AUTONEG_DISABLE) && priv->full_duplex)
		macon2 |= FULDPX;

	priv->set_bits(priv, MACON2, macon2);

	priv->rxfilter = RXFILTER_NORMAL;
	encx24j600_set_rxfilter_mode(priv);

	/* Program the Maximum frame length */
	priv->write_reg(priv, MAMXFL, MAX_FRAMELEN);

	/* Init Tx pointers */
	encx24j600_hw_init_tx(priv);

	/* Init Rx pointers */
	encx24j600_hw_init_rx(priv);

	if (netif_msg_hw(priv))
		encx24j600_dump_config(priv, "Hw is initialized");
}

static void encx24j600_hw_enable(struct encx24j600_priv *priv)
{
	/* Clear the interrupt flags in case was set */
	priv->clr_bits(priv, EIR, (PCFULIF | RXABTIF | TXABTIF | TXIF |
					PKTIF | LINKIF));

	/* Enable the interrupts */
	priv->write_reg(priv, EIE, (PCFULIE | RXABTIE | TXABTIE | TXIE |
					 PKTIE | LINKIE));

	/* set global interrupt enable*/
    if (!priv->ecdev) {
	priv->set_bits(priv, EIE, INTIE);
    }

	/* Enable RX */
	priv->cmd(priv, CMD_ENABLERX);

	priv->hw_enabled = true;
}

static void encx24j600_hw_disable(struct encx24j600_priv *priv)
{
	/* Disable RX */
	priv->cmd(priv, CMD_DISABLERX);

	/* Disable all interrupts */
	priv->write_reg(priv, EIE, 0);

	priv->hw_enabled = false;
}

static int encx24j600_setlink(struct net_device *dev, u8 autoneg, u16 speed,
			      u8 duplex)
{
	struct encx24j600_priv *priv = netdev_priv(dev);
	int ret = 0;

	if (!priv->hw_enabled) {
		/* link is in low power mode now; duplex setting
		 * will take effect on next encx24j600_hw_init()
		 */
		if (speed == SPEED_10 || speed == SPEED_100) {
			priv->autoneg = (autoneg == AUTONEG_ENABLE);
			priv->full_duplex = (duplex == DUPLEX_FULL);
			priv->speed = (speed == SPEED_100);
		} else {
			netif_warn(priv, link, dev, "unsupported link speed setting\n");
			/*speeds other than SPEED_10 and SPEED_100 */
			/*are not supported by chip */
			ret = -EOPNOTSUPP;
		}
	} else {
		netif_warn(priv, link, dev, "Warning: hw must be disabled to set link mode\n");
		ret = -EBUSY;
	}
	return ret;
}

static void encx24j600_hw_get_macaddr(struct encx24j600_priv *priv,
				      unsigned char *ethaddr)
{
	unsigned short val;

	val = priv->read_reg(priv, MAADR1);

	ethaddr[0] = val & 0x00ff;
	ethaddr[1] = (val & 0xff00) >> 8;

	val = priv->read_reg(priv, MAADR2);

	ethaddr[2] = val & 0x00ffU;
	ethaddr[3] = (val & 0xff00U) >> 8;

	val = priv->read_reg(priv, MAADR3);

	ethaddr[4] = val & 0x00ffU;
	ethaddr[5] = (val & 0xff00U) >> 8;
}

/* Program the hardware MAC address from dev->dev_addr.*/
static int encx24j600_set_hw_macaddr(struct net_device *dev)
{
	struct encx24j600_priv *priv = netdev_priv(dev);

	if (priv->hw_enabled) {
		netif_info(priv, drv, dev, "Hardware must be disabled to set Mac address\n");
		return -EBUSY;
	}

	netif_info(priv, drv, dev, "%s: Setting MAC address to %pM\n",
		   dev->name, dev->dev_addr);

	priv->write_reg(priv, MAADR3, (dev->dev_addr[4] | dev->dev_addr[5] << 8));
	priv->write_reg(priv, MAADR2, (dev->dev_addr[2] | dev->dev_addr[3] << 8));
	priv->write_reg(priv, MAADR1, (dev->dev_addr[0] | dev->dev_addr[1] << 8));

	return 0;
}

/* Store the new hardware address in dev->dev_addr, and update the MAC.*/
static int encx24j600_set_mac_address(struct net_device *dev, void *addr)
{
	struct sockaddr *address = addr;

	if (netif_running(dev))
		return -EBUSY;
	if (!is_valid_ether_addr(address->sa_data))
		return -EADDRNOTAVAIL;

	eth_hw_addr_set(dev, address->sa_data);
	return encx24j600_set_hw_macaddr(dev);
}

static int encx24j600_open(struct net_device *dev)
{
	struct encx24j600_priv *priv = netdev_priv(dev);
	int ret;

	encx24j600_hw_disable(priv);
	encx24j600_hw_init(priv);

    if (!priv->ecdev) {
	ret = request_irq(dev->irq, encx24j600_irq, IRQF_TRIGGER_LOW, DRV_NAME, priv);
	if (unlikely(ret < 0)) {
		netdev_err(dev, "request irq %d failed (ret = %d)\n",
			   dev->irq, ret);
		return ret;
	}
    }

	encx24j600_hw_enable(priv);

    if (!priv->ecdev) {
	netif_start_queue(dev);
    }

	return 0;
}

static int encx24j600_stop(struct net_device *dev)
{
	struct encx24j600_priv *priv = netdev_priv(dev);

    if (!priv->ecdev) {
	netif_stop_queue(dev);
    }
	encx24j600_hw_disable(priv);
    if (!priv->ecdev) {
	free_irq(dev->irq, priv);
    }
	return 0;
}

static void encx24j600_ec_poll(struct net_device *dev)
{
	struct encx24j600_priv *priv = netdev_priv(dev);
	encx24j600_irq_proc(&priv->irq_work);
}

static void encx24j600_setrx_proc(struct kthread_work *ws)
{
	struct encx24j600_priv *priv =
	    container_of(ws, struct encx24j600_priv, setrx_work);

	encx24j600_set_rxfilter_mode(priv);
}

static void encx24j600_set_multicast_list(struct net_device *dev)
{
	struct encx24j600_priv *priv = netdev_priv(dev);
	int oldfilter = priv->rxfilter;

	if (dev->flags & IFF_PROMISC) {
		netif_dbg(priv, link, dev, "promiscuous mode\n");
		priv->rxfilter = RXFILTER_PROMISC;
	} else if ((dev->flags & IFF_ALLMULTI) || !netdev_mc_empty(dev)) {
		netif_dbg(priv, link, dev, "%smulticast mode\n",
			  (dev->flags & IFF_ALLMULTI) ? "all-" : "");
		priv->rxfilter = RXFILTER_MULTI;
	} else {
		netif_dbg(priv, link, dev, "normal mode\n");
		priv->rxfilter = RXFILTER_NORMAL;
	}

	if (oldfilter != priv->rxfilter)
		kthread_queue_work(&priv->kworker, &priv->setrx_work);
}

static void encx24j600_hw_tx(struct encx24j600_priv *priv)
{
	struct net_device *dev = priv->ndev;
	struct sk_buff *skb = priv->tx_buf_prep->skb;

	netif_info(priv, tx_queued, dev, "TX Packet Len:%d\n",
		   skb->len);

	if (netif_msg_pktdata(priv))
		dump_packet("TX", skb->len, skb->data);


	/* Set the data pointer to the TX buffer address in the SRAM */
	priv->write_reg(priv, EGPWRPT, priv->tx_buf_prep->hw_addr);

	/* Copy the packet into the SRAM */
	priv->write_mem(priv, WIN_GP, (u8 *) skb->data, skb->len);

	/* skip to next buffer */
	priv->tx_buf_prep = priv->tx_buf_prep->next;
	priv->tx_pending++;

	// start transmission
	encx24j600_hw_tx_start(priv);
}

static void encx24j600_tx_proc(struct kthread_work *ws)
{
	struct encx24j600_tx_buf *buf =
	    container_of(ws, struct encx24j600_tx_buf, work);
	struct encx24j600_priv *priv = buf->priv;

	encx24j600_hw_tx(priv);
}

static netdev_tx_t encx24j600_tx(struct sk_buff *skb, struct net_device *dev)
{
	struct encx24j600_priv *priv = netdev_priv(dev);

    if (priv->ecdev) {
	if (atomic_add_unless(&priv->tx_buf_free, -1, 0) == 0)
		return NETDEV_TX_BUSY;
    } else {
	if (atomic_dec_and_test(&priv->tx_buf_free))
	    netif_stop_queue(dev);
    }

	/* Remember the skb for deferred processing */
	priv->tx_buf_input->skb = skb;

    if (priv->ecdev) {
	encx24j600_hw_tx(priv);
    } else {
	/* save the timestamp */
	netif_trans_update(dev);

	/* start work */
	kthread_queue_work(&priv->kworker, &priv->tx_buf_input->work);
    }

	/* skip to next buffer */
	priv->tx_buf_input = priv->tx_buf_input->next;

	return NETDEV_TX_OK;
}

/* Deal with a transmit timeout */
static void encx24j600_tx_timeout(struct net_device *dev, unsigned int txqueue)
{
	struct encx24j600_priv *priv = netdev_priv(dev);

	netif_err(priv, tx_err, dev, "TX timeout at %ld, latency %ld\n",
		  jiffies, jiffies - dev_trans_start(dev));

	dev->stats.tx_errors++;
	encx24j600_hw_init_tx(priv);
	netif_wake_queue(dev);
	return;
}

static int encx24j600_get_regs_len(struct net_device *dev)
{
	return SFR_REG_COUNT;
}

static void encx24j600_get_regs(struct net_device *dev,
				struct ethtool_regs *regs, void *p)
{
	struct encx24j600_priv *priv = netdev_priv(dev);
	u16 *buff = p;
	u8 reg;

	regs->version = 1;
	for (reg = 0; reg < SFR_REG_COUNT; reg += 2) {
		buff[reg] = priv->read_reg(priv, reg);
	}
}

static void encx24j600_get_drvinfo(struct net_device *dev,
				   struct ethtool_drvinfo *info)
{
	strscpy(info->driver, DRV_NAME, sizeof(info->driver));
	strscpy(info->version, DRV_VERSION, sizeof(info->version));
	strscpy(info->bus_info, dev_name(dev->dev.parent),
		sizeof(info->bus_info));
}

static int encx24j600_get_link_ksettings(struct net_device *dev,
				   struct ethtool_link_ksettings *cmd)
{
	struct encx24j600_priv *priv = netdev_priv(dev);

	ethtool_convert_legacy_u32_to_link_mode(cmd->link_modes.supported,
	    SUPPORTED_10baseT_Half | SUPPORTED_10baseT_Full |
	    SUPPORTED_100baseT_Half | SUPPORTED_100baseT_Full |
	    SUPPORTED_Autoneg | SUPPORTED_TP);;

	cmd->base.speed = priv->speed;
	cmd->base.duplex = priv->full_duplex ? DUPLEX_FULL : DUPLEX_HALF;
	cmd->base.port = PORT_TP;
	cmd->base.autoneg = priv->autoneg ? AUTONEG_ENABLE : AUTONEG_DISABLE;

	return 0;
}

static int encx24j600_set_link_ksettings(struct net_device *dev,
				   const struct ethtool_link_ksettings *cmd)
{
	return encx24j600_setlink(dev, cmd->base.autoneg,
				  cmd->base.speed, cmd->base.duplex);
}

static u32 encx24j600_get_msglevel(struct net_device *dev)
{
	struct encx24j600_priv *priv = netdev_priv(dev);
	return priv->msg_enable;
}

static void encx24j600_set_msglevel(struct net_device *dev, u32 val)
{
	struct encx24j600_priv *priv = netdev_priv(dev);
	priv->msg_enable = val;
}

static const struct ethtool_ops encx24j600_ethtool_ops = {
	.get_link_ksettings = encx24j600_get_link_ksettings,
	.set_link_ksettings = encx24j600_set_link_ksettings,
	.get_drvinfo = encx24j600_get_drvinfo,
	.get_msglevel = encx24j600_get_msglevel,
	.set_msglevel = encx24j600_set_msglevel,
	.get_regs_len = encx24j600_get_regs_len,
	.get_regs = encx24j600_get_regs,
};

static const struct net_device_ops encx24j600_netdev_ops = {
	.ndo_open = encx24j600_open,
	.ndo_stop = encx24j600_stop,
	.ndo_start_xmit = encx24j600_tx,
	.ndo_set_rx_mode = encx24j600_set_multicast_list,
	.ndo_set_mac_address = encx24j600_set_mac_address,
	.ndo_tx_timeout = encx24j600_tx_timeout,
	.ndo_validate_addr = eth_validate_addr,
};

int encx24j600_probe(struct encx24j600_priv *priv)
{
	int ret;
	int i;
	struct encx24j600_tx_buf *buf;
	struct encx24j600_tx_buf *last_buf;
	u16 hw_addr;
	u16 eidled;
	u8 addr[ETH_ALEN];

	priv->msg_enable = netif_msg_init(debug, DEFAULT_MSG_ENABLE);

	/* Default configuration PHY configuration */
	priv->full_duplex = true;
	priv->autoneg = AUTONEG_ENABLE;
	priv->speed = SPEED_100;

	priv->ndev->netdev_ops = &encx24j600_netdev_ops;

	mutex_init(&priv->phy_lock);

	/* Reset device and check if it is connected */
	if (encx24j600_soft_reset(priv)) {
		netif_err(priv, probe, priv->ndev,
			  DRV_NAME ": Chip is not detected\n");
		ret = -EIO;
		goto out_free;
	}

	/* Initialize the device HW to the consistent state */
	encx24j600_hw_init(priv);

	kthread_init_worker(&priv->kworker);
	kthread_init_work(&priv->setrx_work, encx24j600_setrx_proc);
	kthread_init_work(&priv->irq_work, encx24j600_irq_proc);

	/* setup transmit buffer ring */
	buf = priv->tx_buf;
	last_buf = NULL;
	hw_addr = TX_BUF_START;
	for (i=0; i<TX_BUF_COUNT; i++) {
	    buf->priv = priv;
	    kthread_init_work(&buf->work, encx24j600_tx_proc);
	    buf->skb = NULL;

	    buf->hw_addr = hw_addr;
	    hw_addr += TX_BUF_SIZE;

	    if (last_buf != NULL) {
		last_buf->next = buf;
	    }
	    last_buf = buf;
	    buf++;
	}
	last_buf->next = priv->tx_buf;

	priv->kworker_task = kthread_run(kthread_worker_fn, &priv->kworker,
					 "encx24j600");
	if (IS_ERR(priv->kworker_task)) {
		ret = PTR_ERR(priv->kworker_task);
		goto out_free;
	}

	/* Get the MAC address from the chip */
	encx24j600_hw_get_macaddr(priv, addr);
	eth_hw_addr_set(priv->ndev, addr);

	priv->ndev->ethtool_ops = &encx24j600_ethtool_ops;

	eidled = priv->read_reg(priv, EIDLED);
	if (((eidled & DEVID_MASK) >> DEVID_SHIFT) != ENCX24J600_DEV_ID) {
		ret = -EINVAL;
		goto out_stop;
	}

	netif_info(priv, probe, priv->ndev, "Silicon rev ID: 0x%02x\n",
		   (eidled & REVID_MASK) >> REVID_SHIFT);

	netif_info(priv, drv, priv->ndev, "MAC address %pM\n", priv->ndev->dev_addr);

	// offer device to EtherCAT master module
	priv->ecdev = ecdev_offer(priv->ndev, encx24j600_ec_poll, THIS_MODULE);
    if (priv->ecdev) {
	priv->ec_rx_buf = kmalloc(TX_BUF_SIZE, GFP_KERNEL);
	if (!priv->ec_rx_buf) {
		ret = -ENOMEM;
		goto out_ecdev_withdraw;
	}

	ret = ecdev_open(priv->ecdev);
	if (ret) {
		goto out_ecdev_withdraw;
	}
    } else {
	ret = register_netdev(priv->ndev);
	if (unlikely(ret)) {
		netif_err(priv, probe, priv->ndev,
			  "Error %d initializing card encx24j600 card\n", ret);
		goto out_stop;
	}
    }

	return ret;

out_ecdev_withdraw:
	ecdev_withdraw(priv->ecdev);
out_stop:
	kthread_stop(priv->kworker_task);
out_free:
	free_netdev(priv->ndev);

	return ret;
}
//EXPORT_SYMBOL_GPL(encx24j600_probe);

void encx24j600_remove(struct encx24j600_priv *priv)
{
    if (priv->ecdev) {
	ecdev_close(priv->ecdev);
	ecdev_withdraw(priv->ecdev);
	kfree(priv->ec_rx_buf);
    } else {
	unregister_netdev(priv->ndev);
    }

	kthread_stop(priv->kworker_task);
	free_netdev(priv->ndev);
}
//EXPORT_SYMBOL_GPL(encx24j600_remove);

//MODULE_DESCRIPTION(DRV_NAME " ethernet driver (EtherCAT)");
//MODULE_AUTHOR("Jon Ringle <jringle@gridpoint.com>");
//MODULE_LICENSE("GPL");
