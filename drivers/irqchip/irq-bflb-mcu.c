// SPDX-License-Identifier: GPL-2.0
/*
 * Hardware state machine:
 *  - Set status = 1 when all of the below conditions are met:
 *     a) trigger == 1
 *     b) mask == 0
 *     c) last pass trigger == 0 OR last pass mask == 1
 *  - Set status = 0 when writing clear = 1
 *
 * This means:
 *  - Mask without ack does nothing.
 *  - Unmask retriggers level interrupts.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>

#define BFLB_MCU_STATUS(n)		(0x00 + 4 * (n))
#define BFLB_MCU_MASK(n)		(0x08 + 4 * (n))
#define BFLB_MCU_CLEAR(n)		(0x10 + 4 * (n))

#define BFLB_MCU_NR_IRQS		64

static void bflb_mcu_irq_ack(struct irq_data *d)
{
	irq_hw_number_t	hwirq = irqd_to_hwirq(d);
	void __iomem *regs = irq_data_get_irq_chip_data(d);
	void __iomem *reg = regs + BFLB_MCU_CLEAR(hwirq / 32);
	unsigned int bit = BIT(hwirq % 32);

	writel_relaxed(bit, reg);
}

static void bflb_mcu_irq_mask(struct irq_data *d)
{
	irq_hw_number_t	hwirq = irqd_to_hwirq(d);
	void __iomem *regs = irq_data_get_irq_chip_data(d);
	void __iomem *reg = regs + BFLB_MCU_MASK(hwirq / 32);
	unsigned int bit = BIT(hwirq % 32);

	writel_relaxed(readl_relaxed(reg) | bit, reg);
}

static void bflb_mcu_irq_mask_ack(struct irq_data *d)
{
	irq_hw_number_t	hwirq = irqd_to_hwirq(d);
	void __iomem *regs = irq_data_get_irq_chip_data(d);
	void __iomem *mask_reg = regs + BFLB_MCU_MASK(hwirq / 32);
	void __iomem *ack_reg = regs + BFLB_MCU_CLEAR(hwirq / 32);
	unsigned int bit = BIT(hwirq % 32);

	writel_relaxed(readl_relaxed(mask_reg) | bit, mask_reg);
	writel_relaxed(bit, ack_reg);
}

static void bflb_mcu_irq_unmask(struct irq_data *d)
{
	irq_hw_number_t	hwirq = irqd_to_hwirq(d);
	void __iomem *regs = irq_data_get_irq_chip_data(d);
	void __iomem *reg = regs + BFLB_MCU_MASK(hwirq / 32);
	unsigned int bit = BIT(hwirq % 32);

	writel_relaxed(readl_relaxed(reg) & ~bit, reg);
}

static const struct irq_chip bflb_mcu_irq_chip = {
	.name		= "BFLB MCU",
	.irq_ack	= bflb_mcu_irq_ack,
	.irq_mask	= bflb_mcu_irq_mask,
	.irq_mask_ack	= bflb_mcu_irq_mask_ack,
	.irq_unmask	= bflb_mcu_irq_unmask,
	.flags		= IRQCHIP_SKIP_SET_WAKE,
};

static int bflb_mcu_domain_map(struct irq_domain *domain, unsigned int virq,
			       irq_hw_number_t hwirq)
{
	irq_domain_set_info(domain, virq, hwirq, &bflb_mcu_irq_chip,
			    domain->host_data, handle_level_irq, NULL, NULL);

	return 0;
}

static const struct irq_domain_ops bflb_mcu_domain_ops = {
	.map		= bflb_mcu_domain_map,
	.xlate		= irq_domain_xlate_onecell,
};

static void bflb_mcu_handle_irq(struct irq_desc *desc)
{
	struct irq_domain *domain = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	void __iomem *regs = domain->host_data;

	chained_irq_enter(chip, desc);

	for (unsigned int i = 0; i < BITS_TO_U32(BFLB_MCU_NR_IRQS); ++i) {
		unsigned long status = readl_relaxed(regs + BFLB_MCU_STATUS(i));
		int bit;

		for_each_set_bit(bit, &status, 32)
			generic_handle_domain_irq(domain, 32 * i + bit);
	}

	chained_irq_exit(chip, desc);
}

static int __init bflb_mcu_init(struct device_node *node,
				       struct device_node *parent)
{
	struct irq_domain *domain;
	void __iomem *regs;
	int irq, ret;

	regs = of_io_request_and_map(node, 0, NULL);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	irq = irq_of_parse_and_map(node, 0);
	if (irq <= 0) {
		ret = -EINVAL;
		goto out_iounmap;
	}

	domain = irq_domain_add_linear(node, BFLB_MCU_NR_IRQS,
				       &bflb_mcu_domain_ops, regs);
	if (!domain) {
		ret = -ENOMEM;
		goto out_unmap_irq;
	}

	/* Mask and ack all interrupts. */
	writel_relaxed(0xffffffff, regs + BFLB_MCU_MASK(0));
	writel_relaxed(0xffffffff, regs + BFLB_MCU_MASK(1));
	writel_relaxed(0xffffffff, regs + BFLB_MCU_CLEAR(0));
	writel_relaxed(0xffffffff, regs + BFLB_MCU_CLEAR(1));

	irq_set_chained_handler_and_data(irq, bflb_mcu_handle_irq, domain);

	return 0;

out_unmap_irq:
	irq_dispose_mapping(irq);
out_iounmap:
	iounmap(regs);

	return ret;
}

IRQCHIP_DECLARE(bflb_mcu, "bflb,bl808-mcu-irq", bflb_mcu_init);
