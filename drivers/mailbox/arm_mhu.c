/*
 * Copyright (C) 2013-2015 Fujitsu Semiconductor Ltd.
 * Copyright (C) 2015 Linaro Ltd.
 * Author: Jassi Brar <jaswinder.singh@linaro.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/amba/bus.h>
#include <linux/mailbox_controller.h>

//adv start
/*
	Essendo le prime define tutte nulle, tutto quello che viene fatto su 3 registri in tx e, 
	separatamente in rx, finisce su STAT.
	In particolare vengono fatti i SET e CLEAR.
	Le normali set e clear vengono quindi sostituite da altre che vanno direttamente a modificare
	il contenuto dei registri STAT di stato, però prendono gli stessi ingressi delle write e read perchè
	tanto puntano sempre a STAT.
*/
#define INTR_STAT_OFS	0x0
#define INTR_SET_OFS	0x0
#define INTR_CLR_OFS	0x0

#define MHU_LP_OFFSET	0x0 	//questo è realmente utilizzato
#define MHU_HP_OFFSET	0x8 	//sotto la mailbox mappo i due canali che non dovrebbero essere utilizzati
#define MHU_SEC_OFFSET	0x16 	//ma nel caso in cui lo siano almeno non sono sovrapposti a LP.
#define TX_REG_OFFSET	0x4 	/*grazie alle modifiche fatte sotto in tx_reg ci finisce l'indice di rx
							  	mentre in rx_reg ci finisce tx_reg+ questo offset. In questo modo abbiamo prima tx e poi rx
							  	che corrispondono alla configurazione Doorbell (tx) e poi Completion (rx) della 
							  	mailbox generata con RegGen*/

#define MHU_CHANS	3
//adv end

struct mhu_link {
	unsigned irq;
	void __iomem *tx_reg;
	void __iomem *rx_reg;
};

struct arm_mhu {
	void __iomem *base;
	struct mhu_link mlink[MHU_CHANS];
	struct mbox_chan chan[MHU_CHANS];
	struct mbox_controller mbox;
};

//adv start
static void intr_set(u32 value, volatile void __iomem *addr)
{
	u32 read_value;

	//printk("intr_set: val = %x",value);
	read_value = readl_relaxed(addr);
	value = value | read_value;
	writel_relaxed(value, addr);
}

static void intr_clr(u32 value, volatile void __iomem *addr)
{
	u32 read_value;

	//printk("intr_clr: val = %x",value);
	value = value ^ (0xFFFFFFFF);
	read_value = readl_relaxed(addr);
	value = value & read_value;
	writel_relaxed(value, addr);
}
//adv end

static irqreturn_t mhu_rx_interrupt(int irq, void *p)
{
	
	struct mbox_chan *chan = p;
	struct mhu_link *mlink = chan->con_priv;
	u32 val;


	val = readl_relaxed(mlink->rx_reg + INTR_STAT_OFS);

	// if (!val)
	// 	return IRQ_NONE;

	mbox_chan_received_data(chan, (void *)&val);

	// // //adv start
	// // //writel_relaxed(val, mlink->rx_reg + INTR_CLR_OFS);
	intr_clr(val, mlink->rx_reg + INTR_CLR_OFS);
	// // //adv end
	return IRQ_HANDLED;
}

static bool mhu_last_tx_done(struct mbox_chan *chan)
{
	struct mhu_link *mlink = chan->con_priv;
	u32 val = readl_relaxed(mlink->tx_reg + INTR_STAT_OFS);

	return (val == 0);
}

static int mhu_send_data(struct mbox_chan *chan, void *data)
{
	struct mhu_link *mlink = chan->con_priv;
	u32 *arg = data;

	//adv start
	// printk("mhu_send data to addr: 0x%x", mlink->tx_reg + INTR_SET_OFS);
	//writel_relaxed(*arg, mlink->tx_reg + INTR_SET_OFS);
	intr_set(*arg, mlink->tx_reg + INTR_SET_OFS);
	//adv end

	return 0;
}

static int mhu_startup(struct mbox_chan *chan)
{
	// printk("mhu startup");
	struct mhu_link *mlink = chan->con_priv;
	u32 val;
	int ret;

	val = readl_relaxed(mlink->tx_reg + INTR_STAT_OFS);
	//adv start
	//writel_relaxed(val, mlink->tx_reg + INTR_CLR_OFS);
	intr_clr(val, mlink->tx_reg + INTR_CLR_OFS);
	//adv end

	//adv start 
	// printk("IRQ is not acquired in mhu_startup on purpose, otherwise it will return error");
	// printk("mhu_startup: requested MHU irq %d", mlink->irq);
	// printk("mhu_startup: for mhu with rx reg %d", mlink->rx_reg);

	ret = request_irq(mlink->irq, mhu_rx_interrupt,
			  IRQF_SHARED, "mhu_link", chan);
	if (ret) {
		dev_err(chan->mbox->dev,
			"Unable to acquire IRQ %d\n", mlink->irq);
		return ret;
	}
	// printk("mhu_startup: OK");
	//adv end
	return 0;
}

static void mhu_shutdown(struct mbox_chan *chan)
{
	struct mhu_link *mlink = chan->con_priv;

	free_irq(mlink->irq, chan);
}

static const struct mbox_chan_ops mhu_ops = {
	.send_data = mhu_send_data,
	.startup = mhu_startup,
	.shutdown = mhu_shutdown,
	.last_tx_done = mhu_last_tx_done,
};

static int mhu_probe(struct amba_device *adev, const struct amba_id *id)
{
	int i, err;
	struct arm_mhu *mhu;
	struct device *dev = &adev->dev;
	int mhu_reg[MHU_CHANS] = {MHU_LP_OFFSET, MHU_HP_OFFSET, MHU_SEC_OFFSET};

	/* Allocate memory for device */
	mhu = devm_kzalloc(dev, sizeof(*mhu), GFP_KERNEL);
	if (!mhu)
		return -ENOMEM;

	mhu->base = devm_ioremap_resource(dev, &adev->res);
	if (IS_ERR(mhu->base)) {
		dev_err(dev, "ioremap failed\n");
		return PTR_ERR(mhu->base);
	}

	for (i = 0; i < MHU_CHANS; i++) {
		mhu->chan[i].con_priv = &mhu->mlink[i];
		mhu->mlink[i].irq = adev->irq[i];
		//adv start in questo modo tx/doorbell viene prima di rx/completion rispecchiando la struttura 
		//della mailbox di reggen
		mhu->mlink[i].tx_reg = mhu->base + mhu_reg[i];
		// printk("mhu tx reg = %d", mhu->mlink[i].tx_reg);

		mhu->mlink[i].rx_reg = mhu->mlink[i].tx_reg + TX_REG_OFFSET;
		// printk("mhu rx reg = %d", mhu->mlink[i].rx_reg);
		//adv end
	}

	mhu->mbox.dev = dev;
	mhu->mbox.chans = &mhu->chan[0];
	mhu->mbox.num_chans = MHU_CHANS;
	mhu->mbox.ops = &mhu_ops;
	mhu->mbox.txdone_irq = false;
	mhu->mbox.txdone_poll = true;
	mhu->mbox.txpoll_period = 1;

	amba_set_drvdata(adev, mhu);

	err = mbox_controller_register(&mhu->mbox);
	if (err) {
		dev_err(dev, "Failed to register mailboxes %d\n", err);
		return err;
	}

	dev_info(dev, "ARM MHU Mailbox registered\n");
	return 0;
}

static int mhu_remove(struct amba_device *adev)
{
	struct arm_mhu *mhu = amba_get_drvdata(adev);

	mbox_controller_unregister(&mhu->mbox);

	return 0;
}

static struct amba_id mhu_ids[] = {
	{
		.id	= 0x1bb098,
		.mask	= 0xffffff,
	},
	{ 0, 0 },
};
MODULE_DEVICE_TABLE(amba, mhu_ids);

static struct amba_driver arm_mhu_driver = {
	.drv = {
		.name	= "mhu",
	},
	.id_table	= mhu_ids,
	.probe		= mhu_probe,
	.remove		= mhu_remove,
};
module_amba_driver(arm_mhu_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ARM MHU Driver");
MODULE_AUTHOR("Jassi Brar <jassisinghbrar@gmail.com>");
