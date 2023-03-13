// SPDX-License-Identifier: GPL-2.0+

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/io.h>
#include <linux/of.h>

#include "phy-generic.h"

/* 0x500 : usb_ctl */
#define PDS_USB_CTL_OFFSET		(0)

#define PDS_REG_USB_SW_RST_N	(1 << 0)
#define PDS_REG_USB_EXT_SUSP_N	(1 << 1)
#define PDS_REG_USB_WAKEUP      (1 << 2)
#define PDS_REG_USB_L1_WAKEUP   (1 << 3)
#define PDS_REG_USB_DRVBUS_POL  (1 << 4)
#define PDS_REG_USB_IDDIG       (1 << 5)

/* 0x504 : usb_phy_ctrl */
#define PDS_USB_PHY_CTRL_OFFSET		(4)

#define PDS_REG_USB_PHY_PONRST     (1 << 0)
#define PDS_REG_USB_PHY_OSCOUTEN   (1 << 1)
#define PDS_REG_USB_PHY_XTLSEL     (2 << 2)
#define PDS_REG_USB_PHY_OUTCLKSEL  (1 << 4)
#define PDS_REG_USB_PHY_PLLALIV    (1 << 5)
#define PDS_REG_PU_USB20_PSW       (1 << 6)

struct bl808_usbphy {
	struct usb_phy_generic	usb_phy_gen;
	void __iomem			*base;
};

static int bl808_usbphy_init(struct usb_phy *phy)
{
	u32 val;
	struct bl808_usbphy *b_phy = dev_get_drvdata(phy->dev);

	val = readl(b_phy->base + PDS_USB_PHY_CTRL_OFFSET);
	val &= ~PDS_REG_USB_PHY_XTLSEL;
	writel(val, b_phy->base + PDS_USB_PHY_CTRL_OFFSET);

	val = readl(b_phy->base + PDS_USB_PHY_CTRL_OFFSET);
	val |= PDS_REG_PU_USB20_PSW;
	writel(val, b_phy->base + PDS_USB_PHY_CTRL_OFFSET);

	val = readl(b_phy->base + PDS_USB_PHY_CTRL_OFFSET);
	val |= PDS_REG_USB_PHY_PONRST;
	writel(val, b_phy->base + PDS_USB_PHY_CTRL_OFFSET);

	udelay(1);

	val = readl(b_phy->base + PDS_USB_CTL_OFFSET);
	val &= ~PDS_REG_USB_SW_RST_N;
	writel(val, b_phy->base + PDS_USB_CTL_OFFSET);

	udelay(1);

	val = readl(b_phy->base + PDS_USB_CTL_OFFSET);
	val |= PDS_REG_USB_EXT_SUSP_N;
	writel(val, b_phy->base + PDS_USB_CTL_OFFSET);

	mdelay(3);

	val = readl(b_phy->base + PDS_USB_CTL_OFFSET);
	val |= PDS_REG_USB_SW_RST_N;
	writel(val, b_phy->base + PDS_USB_CTL_OFFSET);

	mdelay(2);

	return 0;
}

static void bl808_usbphy_shutdown(struct usb_phy *phy)
{
	u32 val;
	struct bl808_usbphy *b_phy = dev_get_drvdata(phy->dev);

	// clrbits_le32(b_phy->base + PDS_USB_PHY_CTRL_OFFSET,
	// 	     PDS_REG_USB_PHY_PONRST_MSK);
	val = readl(b_phy->base + PDS_USB_PHY_CTRL_OFFSET);
	val &= ~PDS_REG_USB_PHY_PONRST;
	writel(val, b_phy->base + PDS_USB_PHY_CTRL_OFFSET);

	// clrbits_le32(b_phy->base + PDS_USB_PHY_CTRL_OFFSET,
	// 	     PDS_REG_PU_USB20_PSW_MSK);
	val = readl(b_phy->base + PDS_USB_PHY_CTRL_OFFSET);
	val &= ~PDS_REG_PU_USB20_PSW;
	writel(val, b_phy->base + PDS_USB_PHY_CTRL_OFFSET);

}

static int bl808_usbphy_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct bl808_usbphy	*b_phy;
	int ret;

	b_phy = devm_kzalloc(dev, sizeof(*b_phy), GFP_KERNEL);
	if (!b_phy)
		return -ENOMEM;

	b_phy->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(b_phy->base))
		return PTR_ERR(b_phy->base);

	ret = usb_phy_gen_create_phy(dev, &b_phy->usb_phy_gen);
	if (ret)
		return ret;

	b_phy->usb_phy_gen.phy.init = bl808_usbphy_init;
	b_phy->usb_phy_gen.phy.shutdown = bl808_usbphy_shutdown;

	platform_set_drvdata(pdev, b_phy);

	return usb_add_phy_dev(&b_phy->usb_phy_gen.phy);
}

static int bl808_usbphy_remove(struct platform_device *pdev)
{
	struct bl808_usbphy *b_phy = platform_get_drvdata(pdev);

	usb_remove_phy(&b_phy->usb_phy_gen.phy);

	return 0;
}

static const struct of_device_id bl808_usbphy_ids[] = {
	{ .compatible = "bflb,bl808-usb-phy" },
	{ }
};

static struct platform_driver bl808_usbphy_driver = {
	.probe          = bl808_usbphy_probe,
	.remove         = bl808_usbphy_remove,
	.driver         = {
		.name   = "bl808_usbphy",
		.of_match_table = bl808_usbphy_ids,
	},
};

module_platform_driver(bl808_usbphy_driver);

MODULE_ALIAS("platform:bl808_usbphy");
MODULE_AUTHOR("Madushan Nishantha");
MODULE_DESCRIPTION("BL808 USB phy driver");
MODULE_LICENSE("GPL v2");
