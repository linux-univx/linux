// SPDX-License-Identifier: (GPL-2.0+ OR MIT)
/*
 * Amlogic MIPI DPHY RX driver
 *
 * Copyright (C) 2020 RTAVS, Ltd.
 *
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/*
 * Design: generic phy driver with no video node
 *
 * phy_configure: set dphy configuration state, eg clk_miss/hs_settle/...
 * phy_set_mode_ext:  set dphy mux and aphy mode, submode for csi2_phymux
 *
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

#define MIPI_PHY_CTRL	          0x00
#define MIPI_PHY_CLK_LANE_CTRL	  0x04
#define MIPI_PHY_DATA_LANE_CTRL	  0x08
#define MIPI_PHY_DATA_LANE_CTRL1  0x0C
#define MIPI_PHY_TCLK_MISS	0x10
#define MIPI_PHY_TCLK_SETTLE	  0x14
#define MIPI_PHY_THS_EXIT	      0x18
#define MIPI_PHY_THS_SKIP	      0x1C
#define MIPI_PHY_THS_SETTLE	      0x20
#define MIPI_PHY_TINIT	          0x24
#define MIPI_PHY_TULPS_C	      0x28
#define MIPI_PHY_TULPS_S	      0x2C
#define MIPI_PHY_TMBIAS		      0x30
#define MIPI_PHY_TLP_EN_W	      0x34
#define MIPI_PHY_TLPOK	          0x38
#define MIPI_PHY_TWD_INIT	      0x3C
#define MIPI_PHY_TWD_HS		      0x40
#define MIPI_PHY_AN_CTRL0	      0x44
#define MIPI_PHY_AN_CTRL1	      0x48
#define MIPI_PHY_AN_CTRL2	      0x4C
#define MIPI_PHY_CLK_LANE_STS	  0x50
#define MIPI_PHY_DATA_LANE0_STS	  0x54
#define MIPI_PHY_DATA_LANE1_STS	  0x58
#define MIPI_PHY_DATA_LANE2_STS	  0x5C
#define MIPI_PHY_DATA_LANE3_STS	  0x60
#define MIPI_PHY_INT_STS	      0x6C
#define MIPI_PHY_MUX_CTRL0        0x184
#define MIPI_PHY_MUX_CTRL1	      0x188





struct csi_dphy_drv_data {
	const char * const *clks;
	unsigned int num_clks;
};

struct csi_dphy {
	struct device *dev;

	struct phy *phy;
	struct phy *analog;
	struct regmap *regmap;
	//struct reset_control *reset;

	struct clk_bulk_data *clks;

	const struct csi_dphy_drv_data *drv_data;

	struct phy_configure_opts_mipi_dphy config;

	enum csi2_phymux phymux;

	u8 hsfreq;
};

static const struct regmap_config csi_dphy_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = MIPI_PHY_MUX_CTRL1,
	.name = "mipi-dphy-rx",
};


static void csi_dphy_enable(struct csi_dphy *priv)
{

#if 0
	// see phy_configure_opts_mipi_dphy
	// continue mode
	regmap_write(priv->regmap, MIPI_PHY_CLK_LANE_CTRL, 0x3d8);

	// clk_miss = 50 ns --(clk_miss < 60 ns)
	regmap_write(priv->regmap, MIPI_PHY_TCLK_MISS, 0x9);


	// during which the HS receiver
	// should ignore any Clock Lane HS transitions, starting from
	// the beginning of @clk_prepare.
	// clk_settle = 160 ns --( 95ns< clk_settle < 300 ns)
	regmap_write(priv->regmap, MIPI_PHY_TCLK_SETTLE, 0x1f);

	// Time, in picoseconds, that the transmitter drives LP-11 following a HS burst
	// hs_exit = 160 ns --(x>100ns)
	regmap_write(priv->regmap, MIPI_PHY_THS_EXIT, 0x1f);

	/**
	 * @hs_skip:
	 *
	 * Time interval, in picoseconds, during which the HS-RX
	 * should ignore any transitions on the Data Lane, following a
	 * HS burst. The end point of the interval is defined as the
	 * beginning of the LP-11 state following the HS burst.
	 *
	 * Minimum value: 40000 ps
	 * Maximum value: 55000 ps + 4 * @hs_clk_rate period in ps
	 */
	// hs_skip = 55 ns --(40ns <x < 55ns + 4 * UI)
	regmap_write(priv->regmap, MIPI_PHY_THS_SKIP, 0xa);

	/**
	 * @hs_settle:
	 *
	 * Time interval, in picoseconds, during which the HS receiver
	 * shall ignore any Data Lane HS transitions, starting from
	 * the beginning of @hs_prepare.
	 *
	 * Minimum value: 85000 ps + 6 * @hs_clk_rate period in ps
	 * Maximum value: 145000 ps + 10 * @hs_clk_rate period in ps
	 */
	// hs_settle = 160ns --(85 ns + 6*UI<x<145 ns + 10*UI)
	regmap_write(priv->regmap, MIPI_PHY_THS_SETTLE, priv->hsfreq);

	/**
	 * @init:
	 *
	 * Time, in microseconds for the initialization period to
	 * complete.
	 *
	 * Minimum value: 100 us
	 */
	// > 100us
	regmap_write(priv->regmap, MIPI_PHY_TINIT, 0x4e20);

	regmap_write(priv->regmap, MIPI_PHY_TMBIAS, 0x100);
	regmap_write(priv->regmap, MIPI_PHY_TULPS_C, 0x1000);
	regmap_write(priv->regmap, MIPI_PHY_TULPS_S, 0x100);
	regmap_write(priv->regmap, MIPI_PHY_TLP_EN_W, 0x0c);
	regmap_write(priv->regmap, MIPI_PHY_TLPOK, 0x100);

	// watch dog for init and hs transfer
	regmap_write(priv->regmap, MIPI_PHY_TWD_INIT, 0x400000);
	regmap_write(priv->regmap, MIPI_PHY_TWD_HS, 0x400000);

	// enable data lanes pipe line and hs sync bit err.
	regmap_write(priv->regmap, MIPI_PHY_DATA_LANE_CTRL, 0x0);
	regmap_write(priv->regmap, MIPI_PHY_DATA_LANE_CTRL1, 0x3 | (0x1f << 2) | (0x3 << 7));

	// dphy mux
	if (priv->phymux == CSI2_PHYMUX_A) {
	    // mux A
	    regmap_write(priv->regmap, MIPI_PHY_MUX_CTRL0, 0x00000123);
	    regmap_write(priv->regmap, MIPI_PHY_MUX_CTRL1, 0x00000123);
	} else if (priv->phymux == CSI2_PHYMUX_B) {
	    // mux B
	    regmap_write(priv->regmap, MIPI_PHY_MUX_CTRL0, 0x000123ff);
	    regmap_write(priv->regmap, MIPI_PHY_MUX_CTRL1, 0x0001ff01);
	} else if (priv->phymux == CSI2_PHYMUX_AB) {
	    // A + B
	    ;
	}

	regmap_write(priv->regmap, MIPI_PHY_CTRL, 0);
#endif
}





static int csi_dphy_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct csi_dphy *priv = phy_get_drvdata(phy);
	const struct csi_dphy_drv_data *drv_data = priv->drv_data;
	struct phy_configure_opts_mipi_dphy *config = &opts->mipi_dphy;
	unsigned int hsfreq = 0;
	unsigned int i;
	u64 data_rate_mbps;
	int ret;

	ret = phy_configure(priv->analog, opts);
	if (ret != 0)
		return ret;

	/* pass with phy_mipi_dphy_get_default_config (with pixel rate?) */
	ret = phy_mipi_dphy_config_validate(config);
	if (ret)
		return ret;

	data_rate_mbps = div_u64(config->hs_clk_rate, 1000 * 1000);

	dev_dbg(priv->dev, "lanes %d - data_rate_mbps %llu\n",
		config->lanes, data_rate_mbps);

	u32 ui_val = 1000 / data_rate_mbps;

	if ((ui_val % data_rate_mbps) != 0) {
	    ui_val += 1;
	}

	hsfreq = ((85 + 145 + (16 * ui_val)) / 2) / 5;

	priv->hsfreq = hsfreq;
	priv->config = *config;
	return 0;
}


static int csi_dphy_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct csi_dphy *priv = phy_get_drvdata(phy);
	int ret;

	if (mode != PHY_MODE_MIPI_DPHY)
		return -EINVAL;

	if (submode >= CSI2_PHYMUX_MAX)
		return -EINVAL;

	ret = phy_set_mode_ext(priv->analog, mode, submode);
	if (ret != 0)
		return ret;

	dev_info(&phy->dev, "Changing phymux to %d\n", submode);
	priv->phymux = submode;

	return 0;
}


static int csi_dphy_power_on(struct phy *phy)
{
	struct csi_dphy *priv = phy_get_drvdata(phy);
	int ret;

	ret = phy_power_on(priv->analog);
	if (ret != 0)
		return ret;

	ret = clk_bulk_enable(priv->drv_data->num_clks, priv->clks);
	if (ret)
		return ret;

	csi_dphy_enable(priv);

	return 0;
}

static int csi_dphy_power_off(struct phy *phy)
{
	struct csi_dphy *priv = phy_get_drvdata(phy);
	int ret;

	ret = phy_power_off(priv->analog);
	if (ret != 0)
		return ret;

	regmap_write(priv->regmap, GRF_DPHY_RX0_ENABLE, 0);
	clk_bulk_disable(priv->drv_data->num_clks, priv->clks);
	return 0;
}

static int csi_dphy_init(struct phy *phy)
{
	struct csi_dphy *priv = phy_get_drvdata(phy);
	int ret;

	ret = phy_init(priv->analog);
	if (ret != 0)
		return ret;

	return clk_bulk_prepare(priv->drv_data->num_clks, priv->clks);
}

static int csi_dphy_exit(struct phy *phy)
{
	struct csi_dphy *priv = phy_get_drvdata(phy);

	int ret;

	ret = phy_exit(priv->analog);
	if (ret != 0)
		return ret;

	clk_bulk_unprepare(priv->drv_data->num_clks, priv->clks);
	return 0;
}


static const struct phy_ops csi_dphy_ops = {
	.init		= csi_dphy_init,
	.exit		= csi_dphy_exit,
	.power_on	= csi_dphy_power_on,
	.power_off	= csi_dphy_power_off,
	.set_mode   	= csi_dphy_set_mode,
	.configure	= csi_dphy_configure,
	.owner		= THIS_MODULE,
};


static const char * const c2_dphy_clks[] = {
	"dphy-ref",
	"dphy-cfg",
};

static const struct csi_dphy_drv_data a311d_dphy_rx_drv_data = {
	.clks = c2_dphy_clks,
	.num_clks = ARRAY_SIZE(c2_dphy_clks),
};

static const struct of_device_id csi_dphy_dt_ids[] = {
	{
		.compatible = "amlogic,c308x-csi-dphy",
		.data = &a311d_dphy_rx_drv_data,
	},
	{}
};
MODULE_DEVICE_TABLE(of, csi_dphy_dt_ids);

static int csi_dphy_probe(struct platform_device *pdev)
{
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	const struct csi_dphy_drv_data *drv_data;
	struct csi_dphy *priv;
	struct resource *res;
	void __iomem *base;

	struct phy *phy;
	unsigned int i;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->phy = devm_phy_create(dev, np, &csi_dphy_ops);
	if (IS_ERR(priv->phy)) {
		dev_err(dev, "failed to create phy\n");
		return PTR_ERR(priv->phy);
	}

	drv_data = of_device_get_match_data(&pdev->dev);
	if (!drv_data)
		return -EINVAL;
	priv->drv_data = drv_data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		return PTR_ERR(base);
	}

	priv->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					     &csi_dphy_regmap_config);
	if (IS_ERR(priv->regmap)) {
		dev_err(dev, "Couldn't create the DPHY regmap\n");
		return PTR_ERR(priv->regmap);
	}

	priv->clks = devm_kcalloc(&pdev->dev, drv_data->num_clks,
				  sizeof(*priv->clks), GFP_KERNEL);
	if (!priv->clks)
		return -ENOMEM;

	for (i = 0; i < drv_data->num_clks; i++)
		priv->clks[i].id = drv_data->clks[i];

	ret = devm_clk_bulk_get(&pdev->dev, drv_data->num_clks, priv->clks);
	if (ret)
		return ret;

	priv->analog = devm_phy_optional_get(dev, "analog");
	if (IS_ERR(priv->analog))
		return PTR_ERR(priv->analog);

	phy_set_drvdata(phy, priv);
	dev_set_drvdata(dev, priv);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static struct platform_driver csi_dphy_driver = {
	.probe = csi_dphy_probe,
	.driver = {
		.name	= "amlogic-csi-dphy",
		.of_match_table = csi_dphy_dt_ids,
	},
};
module_platform_driver(csi_dphy_driver);

MODULE_AUTHOR("Kaspter Ju <camus@rtavs.com>");
MODULE_DESCRIPTION("Amlogic MIPI DPHY RX driver");
MODULE_LICENSE("Dual MIT/GPL");
