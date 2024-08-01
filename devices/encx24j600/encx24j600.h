/**
 * encx24j600.h
 *
 */

#ifndef _ENCX24J600_H
#define _ENCX24J600_H

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/kthread.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <linux/atomic.h>

#include "../ecdev.h"

#include "encx24j600_hw.h"

enum encx24j600_memwin {
	WIN_UDA,
	WIN_GP,
	WIN_RX,
	WIN_COUNT
};

enum encx24j600_byte_cmd {
	CMD_SETETHRST = 0,	/* System Reset */
	CMD_FCDISABLE,		/* Flow Control Disable */
	CMD_FCSINGLE,		/* Flow Control Single */
	CMD_FCMULTIPLE,		/* Flow Control Multiple */
	CMD_FCCLEAR,		/* Flow Control Clear */
	CMD_SETPKTDEC,		/* Decrement Packet Counter */
	CMD_DMASTOP,		/* DMA Stop */
	CMD_DMACKSUM,		/* DMA Start Checksum */
	CMD_DMACKSUMS,		/* DMA Start Checksum with Seed */
	CMD_DMACOPY,		/* DMA Start Copy */
	CMD_DMACOPYS,		/* DMA Start Copy and Checksum with Seed */
	CMD_SETTXRTS,		/* Request Packet Transmission */
	CMD_ENABLERX,		/* Enable RX */
	CMD_DISABLERX,		/* Disable RX */
	CMD_SETEIE,		/* Enable Interrupts */
	CMD_CLREIE,		/* Disable Interrupts */
	CMD_COUNT
};

struct encx24j600_priv;
struct encx24j600_tx_buf;

struct encx24j600_tx_buf {
	struct encx24j600_priv *priv;
	struct kthread_work work;
	struct sk_buff *skb;
	u16 hw_addr;
	struct encx24j600_tx_buf *next;
};

struct encx24j600_priv {
	struct net_device *ndev;
	struct mutex phy_lock;
	struct task_struct *kworker_task;
	struct kthread_worker kworker;
	struct kthread_work setrx_work;
	struct kthread_work irq_work;
	u16 next_packet;
	bool hw_enabled;
	bool full_duplex;
	bool autoneg;
	u16 speed;
	int rxfilter;
	u32 msg_enable;
	atomic_t tx_buf_free;
	bool tx_running;
	int tx_pending;

	struct encx24j600_tx_buf tx_buf[TX_BUF_COUNT];
	struct encx24j600_tx_buf *tx_buf_input;
	struct encx24j600_tx_buf *tx_buf_prep;
	struct encx24j600_tx_buf *tx_buf_xmit;

	u16 (*read_reg)(struct encx24j600_priv *priv, u8 reg);
	void (*write_reg)(struct encx24j600_priv *priv, u8 reg, u16 val);
	void (*clr_bits)(struct encx24j600_priv *priv, u8 reg, u16 mask);
	void (*set_bits)(struct encx24j600_priv *priv, u8 reg, u16 mask);
	void (*cmd)(struct encx24j600_priv *priv, enum encx24j600_byte_cmd cmd);
	void (*read_mem)(struct encx24j600_priv *priv, enum encx24j600_memwin win, u8 * data, size_t count);
	void (*write_mem)(struct encx24j600_priv *priv, enum encx24j600_memwin win, const u8 * data, size_t count);

	ec_device_t *ecdev;
	void *ec_rx_buf;
};

int encx24j600_probe(struct encx24j600_priv *priv);
void encx24j600_remove(struct encx24j600_priv *priv);

#endif
