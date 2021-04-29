// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Frieder Schrempf <frieder.schrempf@kontron.de>
 *
 * Based on tc358762.c by
 *  Marek Vasut <marex@denx.de>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/i2c.h>

#include <video/mipi_display.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

/* PPI layer registers */
#define PPI_STARTPPI		0x0104 /* START control bit */
#define PPI_LPTXTIMECNT		0x0114 /* LPTX timing signal */
#define PPI_LANEENABLE		0x0134
#define PPI_TX_RX_TA		0x013c
#define PPI_D0S_CLRSIPOCOUNT	0x0164 /* Assertion timer for Lane 0 */
#define PPI_D1S_CLRSIPOCOUNT	0x0168 /* Assertion timer for Lane 1 */
#define PPI_D2S_CLRSIPOCOUNT	0x016c /* Assertion timer for Lane 2 */
#define PPI_D3S_CLRSIPOCOUNT	0x0170 /* Assertion timer for Lane 3 */

#define PPI_START_FUNCTION	1

/* DSI layer registers */
#define DSI_STARTDSI		0x0204 /* START control bit of DSI-TX */
#define DSI_LANEENABLE		0x0210 /* Enables each lane */

#define DSI_RX_START		1

/* Parallel Out Registers */
#define POCTRL			0x0448

#define VPCTRL			0x0450
#define HTIM01			0x0454
#define VTIM01			0x046c
#define VFUEN0			0x0464

/* System Controller Registers */
#define SYSCTRL			0x0510

#define SYSCTRL_DP0_VIDSRC_DSI	1

/* PLL registers */
#define DP0_PLLCTRL		0x0900
#define PXL_PLLCTRL		0x0908
#define PXL_PLLPARAM	0x0914
#define SYS_PLLPARAM	0x0918

struct tc358x67_dsi_dpi {
	struct device *dev;
	struct drm_bridge bridge;
	struct drm_connector connector;
	struct drm_bridge *panel_bridge;
	struct gpio_desc *reset_gpio;
	struct mipi_dsi_device	*dsi;
	struct device_node	*dsi_host_node;
	struct i2c_client *client;
};

static int tc358x67_dsi_dpi_i2c_write(struct i2c_client *client, u16 reg, u32 val)
{
	u8 data[] = {reg >> 8, reg, val, val >> 8, val >> 16, val >> 24};
	struct i2c_msg msg;
	int ret;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret != 1) {
		dev_err(&client->dev, "tc358x67_dsi_dpi I2C write failed (reg=0x%04x, val=0x%08x) ret: %d\n",
				reg, val, ret);
	}

	return ret;
}

static inline struct tc358x67_dsi_dpi *bridge_to_tc358x67_dsi_dpi(struct drm_bridge *bridge)
{
	return container_of(bridge, struct tc358x67_dsi_dpi, bridge);
}

static void tc358x67_dsi_dpi_pre_enable(struct drm_bridge *bridge)
{
	struct tc358x67_dsi_dpi *ctx = bridge_to_tc358x67_dsi_dpi(bridge);

	tc358x67_dsi_dpi_i2c_write(ctx->client, POCTRL, 0x80);
	tc358x67_dsi_dpi_i2c_write(ctx->client, SYS_PLLPARAM, 0x00000300);
	tc358x67_dsi_dpi_i2c_write(ctx->client, PXL_PLLPARAM, 0x0027011b);

	tc358x67_dsi_dpi_i2c_write(ctx->client, DP0_PLLCTRL, 0x1);
	msleep(5);
	tc358x67_dsi_dpi_i2c_write(ctx->client, DP0_PLLCTRL, 0x5);

	tc358x67_dsi_dpi_i2c_write(ctx->client, PXL_PLLCTRL, 0x1);
	msleep(5);
	tc358x67_dsi_dpi_i2c_write(ctx->client, PXL_PLLCTRL, 0x5);

	tc358x67_dsi_dpi_i2c_write(ctx->client, PPI_TX_RX_TA, 0x000A000C);
	tc358x67_dsi_dpi_i2c_write(ctx->client, PPI_LPTXTIMECNT,0x0000008);
	tc358x67_dsi_dpi_i2c_write(ctx->client, PPI_D0S_CLRSIPOCOUNT, 0x9);
	tc358x67_dsi_dpi_i2c_write(ctx->client, PPI_D1S_CLRSIPOCOUNT, 0x9);
	tc358x67_dsi_dpi_i2c_write(ctx->client, PPI_D2S_CLRSIPOCOUNT, 0x9);
	tc358x67_dsi_dpi_i2c_write(ctx->client, PPI_D3S_CLRSIPOCOUNT, 0x9);

	tc358x67_dsi_dpi_i2c_write(ctx->client, PPI_LANEENABLE, 0x1f);
	tc358x67_dsi_dpi_i2c_write(ctx->client, DSI_LANEENABLE, 0x1f);

	tc358x67_dsi_dpi_i2c_write(ctx->client, PPI_STARTPPI, PPI_START_FUNCTION);
	tc358x67_dsi_dpi_i2c_write(ctx->client, DSI_STARTDSI, DSI_RX_START);
	tc358x67_dsi_dpi_i2c_write(ctx->client, SYSCTRL, SYSCTRL_DP0_VIDSRC_DSI);
}

static int tc358x67_dsi_dpi_attach(struct drm_bridge *bridge)
{
	struct tc358x67_dsi_dpi *ctx = bridge_to_tc358x67_dsi_dpi(bridge);
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	int ret;

	const struct mipi_dsi_device_info info = {
		.type = "tc358x67_dsi_dpi",
		.channel = 0,
		.node = NULL,
	};

	host = of_find_mipi_dsi_host_by_node(ctx->dsi_host_node);
	if (!host) {
		dev_err(ctx->dev, "failed to find dsi host\n");
		return -EPROBE_DEFER;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) {
		dev_err(ctx->dev, "failed to create dsi device\n");
		ret = PTR_ERR(dsi);
		return -ENODEV;
	}

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE;

	ctx->dsi = dsi;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		dev_err(ctx->dev, "failed to attach dsi to host\n");
		mipi_dsi_device_unregister(dsi);
	}

	return drm_bridge_attach(bridge->encoder, ctx->panel_bridge,
				 bridge);
}

static const struct drm_bridge_funcs tc358x67_dsi_dpi_bridge_funcs = {
	.pre_enable = tc358x67_dsi_dpi_pre_enable,
	.attach = tc358x67_dsi_dpi_attach,
};

static int tc358x67_dsi_dpi_parse_dt(struct tc358x67_dsi_dpi *ctx)
{
	struct drm_bridge *panel_bridge;
	struct device *dev = ctx->dev;
	struct device_node *in_ep;
	struct drm_panel *panel;
	int ret;

	/* Reset GPIO is optional */
	ctx->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->reset_gpio))
		return PTR_ERR(ctx->reset_gpio);

	in_ep = of_graph_get_endpoint_by_regs(dev->of_node, 0, -1);
	if (!in_ep)
		return PTR_ERR(in_ep);

	ctx->dsi_host_node = of_graph_get_remote_port_parent(in_ep);
	of_node_put(in_ep);
	if (!ctx->dsi_host_node)
			return PTR_ERR(ctx->dsi_host_node);

	ret = drm_of_find_panel_or_bridge(dev->of_node, 1, 0, &panel, NULL);
	if (ret)
		return ret;

	panel_bridge = devm_drm_panel_bridge_add(dev, panel, DRM_MODE_CONNECTOR_DPI);

	if (IS_ERR(panel_bridge))
		return PTR_ERR(panel_bridge);

	ctx->panel_bridge = panel_bridge;

	return 0;
}

static int tc358x67_dsi_dpi_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tc358x67_dsi_dpi *ctx;
	int ret;

	ctx = devm_kzalloc(dev, sizeof(struct tc358x67_dsi_dpi), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	i2c_set_clientdata(client, ctx);

	ctx->dev = dev;
	ctx->client = client;

	ret = tc358x67_dsi_dpi_parse_dt(ctx);
	if (ret < 0)
		return ret;

	if (ctx->reset_gpio) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		usleep_range(5000, 10000);
	}

	ctx->bridge.funcs = &tc358x67_dsi_dpi_bridge_funcs;
	ctx->bridge.of_node = dev->of_node;

	drm_bridge_add(&ctx->bridge);

	return ret;
}

static int tc358x67_dsi_dpi_remove(struct i2c_client *client)
{
	struct tc358x67_dsi_dpi *ctx = i2c_get_clientdata(client);

	mipi_dsi_detach(ctx->dsi);
	drm_bridge_remove(&ctx->bridge);

	return 0;
}

static const struct of_device_id tc358x67_dsi_dpi_of_match[] = {
	{ .compatible = "toshiba,tc358x67-dsi-dpi" },
	{ }
};
MODULE_DEVICE_TABLE(of, tc358x67_dsi_dpi_of_match);

static struct i2c_driver tc358x67_dsi_dpi_driver = {
	.probe = tc358x67_dsi_dpi_probe,
	.remove = tc358x67_dsi_dpi_remove,
	.driver = {
		.name = "tc358x67-dsi-dpi",
		.of_match_table = tc358x67_dsi_dpi_of_match,
	},
};
module_i2c_driver(tc358x67_dsi_dpi_driver);

MODULE_AUTHOR("Frieder Schrempf <frieder.schrempf@kontron-electronics.de");
MODULE_DESCRIPTION("I2C based Driver for TC358x67 Bridge in DSI/DPI Mode");
MODULE_LICENSE("GPL v2");
