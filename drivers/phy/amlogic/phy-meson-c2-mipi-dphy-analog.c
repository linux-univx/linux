// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Amlogic MIPI RX Analog PHY driver
 *
 * Copyright (C) 2020 RTAVS, Ltd.
 *
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define G12B_DPHY_MUX_BASE              (0xff63c34c)

#define HI_CSI_PHY_CNTL0                    0x00
#define  HI_CSI_PHY_CNTL0_HS_SKEW_ENABLE	BIT(27)
#define  HI_CSI_PHY_CNTL0_HS_CML		BIT(26)
#define HI_CSI_PHY_CNTL1                    0x04
#define  HI_CSI_PHY_CNTL1_CSI_TTL_EN		BIT(31)
#define HI_CSI_PHY_CNTL2                    0x08
#define HI_CSI_PHY_CNTL3                    0x0c


/*
 * Design: generic phy driver
 *
 * phy_configure: set dphy configuration state, for ui_val
 * phy_set_mode_ext:  set dphy mux and aphy mode, submode for csi2_phymux
 *
 * csi_dphy_analog: aphy_rx@0 {
 * 	compatible = "amlogic,axg-csi-aphy-rx"";
 * 	reg = <0x0 0x0 0x0 0x4>;
 * };
 *
 * csi_dphy_rx: dphy_rx@ff63c34c {
 *	compatible = "amlogic,axg-csi-dphy-rx";
 *	reg = <0x0 0xff644000 0x0 0x1c>;
 *	//resets = <&reset RESET_PCIE_PHY>;
 *	phys = <&csi_aphy_rx PHY_TYPE_MIPI>;
 *	phy-names = "analog";
 * };
 */

enum csi2_phymux {
	// phy-analog <--> dphy_rx0 <--> csi_host0  1/2/4 lanes
	CSI2_PHYMUX_A = 0,

	// phy-analog <--> dphy_rx1 <--> csi_host1  1/2/4 lanes
	CSI2_PHYMUX_B,

	// phy-analog_clka <--> dphy_rx0 <--> csi_host0  1/2 lanes
	// phy-analog_clkb <--> dphy_rx1 <--> csi_host1  1/2 lanes
	CSI2_PHYMUX_AB,

	CSI2_PHYMUX_MAX
};


struct csi_aphy {
	struct phy *phy;
	struct regmap *regmap;
	struct phy_configure_opts_mipi_dphy config;

	enum csi2_phymux phymux;
	u32 ui_val;
};

static const struct regmap_config csi_aphy_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = HI_CSI_PHY_CNTL3,
};

static int csi_aphy_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct csi_aphy *priv = phy_get_drvdata(phy);
	struct phy_configure_opts_mipi_dphy *config = &opts->mipi_dphy;
	unsigned int ui_val = 0;
	u64 data_rate_mbps;

	/* pass with phy_mipi_dphy_get_default_config (with pixel rate?) */
	ret = phy_mipi_dphy_config_validate(config);
	if (ret)
		return ret;

	data_rate_mbps = div_u64(config->hs_clk_rate, 1000 * 1000);

	dev_dbg(priv->dev, "lanes %d - data_rate_mbps %llu\n",
		config->lanes, data_rate_mbps);

	// FIXME:
	ui_val = 1000 / data_rate_mbps;

	if ((ui_val % data_rate_mbps) != 0) {
		ui_val += 1;
	}

	priv->ui_val = ui_val;
	priv->config = *config;
	return 0;
}


static int csi_aphy_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct csi_aphy *priv = phy_get_drvdata(phy);

	if (mode != PHY_MODE_MIPI_DPHY)
	return -EINVAL;

	if (submode >= CSI2_PHYMUX_MAX)
		return -EINVAL;

	dev_info(&phy->dev, "Changing phymux to %d\n", submode);
	priv->phymux = submode;

	return 0;
}


static int csi_aphy_power_on(struct phy *phy)
{
	struct csi_aphy *priv = phy_get_drvdata(phy);

#if MESON_C308X
	if (priv->ui_val <= 1)
		regmap_write(priv->regmap, HI_CSI_PHY_CNTL0, 0x0b440585);
	else
		regmap_write(priv->regmap, HI_CSI_PHY_CNTL0, 0x0b440581);
	regmap_write(priv->regmap, HI_CSI_PHY_CNTL1, 0x803f0000);
#elif MESON_C305X
	regmap_write(priv->regmap, HI_CSI_PHY_CNTL0, 0x0f820701);
	regmap_write(priv->regmap, HI_CSI_PHY_CNTL1, 0x003f1221);
#else
	dev_info(&phy->dev, "FIXME!!!\n");
#endif

	if (priv->phymux == CSI2_PHYMUX_A) {
		regmap_write(priv->regmap, HI_CSI_PHY_CNTL3, 0x0002);
	} else if (priv->phymux == CSI2_PHYMUX_B) {
		regmap_write(priv->regmap, HI_CSI_PHY_CNTL3, 0xc002);
	} else if (priv->phymux == CSI2_PHYMUX_AB) {
		dev_info(&phy->dev, "FIXME!!!\n");
	}

	return 0;
}

static int csi_aphy_power_off(struct phy *phy)
{
	//struct csi_aphy *priv = phy_get_drvdata(phy);
	return 0;
}

static int csi_aphy_init(struct phy *phy)
{
	return 0;
}

static int csi_aphy_exit(struct phy *phy)
{
	return 0;
}

static const struct phy_ops csi_aphy_ops = {
	.init	= csi_aphy_init,
	.exit	= csi_aphy_exit,
	.power_on	= csi_aphy_power_on,
	.power_off	= csi_aphy_power_off,
	.set_mode   = csi_aphy_set_mode,
	.configure	= csi_aphy_configure,
	.owner	= THIS_MODULE,
};

static const struct of_device_id csi_aphy_dt_ids[] = {
	{
		.compatible = "amlogic,axg-csi-phy-analog",
	},
	{}
};
MODULE_DEVICE_TABLE(of, csi_aphy_dt_ids);


static int csi_aphy_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	struct csi_aphy *priv;
	struct resource *res;
	void __iomem *base;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		return PTR_ERR(base);
	}

	priv->regmap = devm_regmap_init_mmio(&pdev->dev, base,
						 &csi_aphy_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "Couldn't create the DPHY regmap\n");
		return PTR_ERR(priv->regmap);
	}

	priv->phy = devm_phy_create(dev, np, &csi_aphy_ops);
	if (IS_ERR(priv->phy)) {
		dev_err(dev, "failed to create phy\n");
		return PTR_ERR(priv->phy);
	}

	phy_set_drvdata(priv->phy, priv);
	dev_set_drvdata(dev, priv);

	// override a xlate hook ?
	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static struct platform_driver csi_aphy_driver = {
	.probe = csi_aphy_probe,
	.driver = {
		.name	= "amlogic-csi-phy-anlogic",
		.of_match_table = csi_aphy_dt_ids,
	},
};
module_platform_driver(csi_aphy_driver);

MODULE_AUTHOR("Kaspter Ju <camus@rtavs.com>");
MODULE_DESCRIPTION("Amlogic Analog PHY RX driver");
MODULE_LICENSE("Dual MIT/GPL");
