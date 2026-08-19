// SPDX-License-Identifier: GPL-2.0+
/*
 * DRM driver for Sitronix ST7365P panels as fitted to the PicoCalc
 *
 * Copyright 2025 Calculinux
 *
 * Adapted from the PicoCalc ILI9488 TinyDRM driver
 * (drivers/staging/picocalc/picocalc_lcd_drm/ili9488_drm.c)
 * Copyright 2025 nekocharm <jumba.jookiba@outlook.com>
 *
 * Some code copied from ili9486.c
 * Copyright 2020 Kamlesh Gurudasani <kamlesh.gurudasani@gmail.com>
 */

 #include <linux/backlight.h>
 #include <linux/delay.h>
 #include <linux/gpio/consumer.h>
 #include <linux/module.h>
 #include <linux/property.h>
 #include <linux/spi/spi.h>

 #include <video/mipi_display.h>

 #include <drm/drm_atomic_helper.h>
 #include <drm/drm_damage_helper.h>
 #include <drm/drm_drv.h>
 #include <drm/drm_framebuffer.h>
 #include <drm/drm_fb_helper.h>
 #include <drm/drm_format_helper.h>
 #include <drm/drm_gem_framebuffer_helper.h>
 #include <drm/drm_gem_atomic_helper.h>
 #include <drm/drm_gem_dma_helper.h>
 #include <drm/drm_managed.h>
 #include <drm/drm_mipi_dbi.h>
 #include <drm/drm_modeset_helper.h>

/*
 * ST7365P register set. The ST7365P shares the ILI9488 register map, so the
 * command numbers below match the ones used by the ILI9488 driver; the names
 * follow the ST7365P datasheet.
 */
#define ST7365P_SLPOUT				0x11	/* Sleep out */
#define ST7365P_INVON				0x21	/* Display inversion on */
#define ST7365P_DISPON				0x29	/* Display on */
#define ST7365P_MADCTL				0x36	/* Memory data access control */
#define ST7365P_COLMOD				0x3A	/* Interface pixel format */
#define ST7365P_IFMODE				0xB0	/* Interface mode control */
#define ST7365P_FRMCTR1				0xB1	/* Frame rate control, normal mode */
#define ST7365P_INVTR				0xB4	/* Display inversion control */
#define ST7365P_DISCTRL				0xB6	/* Display function control */
#define ST7365P_ETMOD				0xB7	/* Entry mode set */
#define ST7365P_PWCTRL1				0xC0	/* Power control 1 */
#define ST7365P_PWCTRL2				0xC1	/* Power control 2 */
#define ST7365P_VMCTRL				0xC5	/* VCOM control */
#define ST7365P_PGAMCTRL			0xE0	/* Positive gamma correction */
#define ST7365P_NGAMCTRL			0xE1	/* Negative gamma correction */
#define ST7365P_IMGFUNC				0xE9	/* Set image function */
#define ST7365P_ADJCTRL3			0xF7	/* Adjust control 3 */

/* MADCTL bits, panel is wired so that MX | BGR gives the right orientation */
#define ST7365P_MADCTL_BGR			BIT(3)
#define ST7365P_MADCTL_MV			BIT(5)
#define ST7365P_MADCTL_MX			BIT(6)
#define ST7365P_MADCTL_MY			BIT(7)

/*
 * mipi-dbi runs register writes at 10MHz and pixel data at spi->max_speed_hz,
 * so the bus can be pushed well past what the init sequence tolerates. Setting
 * this overrides the spi-max-frequency from the device tree, which makes it
 * possible to find the panel's ceiling without reflashing a device tree. The
 * SPI core still clamps to the controller ceiling: 50MHz on stock
 * spi-rockchip, 100MHz with kernel-patches/0001 applied.
 */
static unsigned int spi_speed_hz;
module_param(spi_speed_hz, uint, 0444);
MODULE_PARM_DESC(spi_speed_hz,
		 "Override the SPI clock used for pixel data, in Hz (0: use spi-max-frequency)");

static int dummy_backlight_update_status(struct backlight_device *bd)
{
	return 0;
}

static const struct backlight_ops dummy_backlight_ops = {
	.update_status = dummy_backlight_update_status,
};

static void st7365p_enable(struct drm_simple_display_pipe *pipe,
			     struct drm_crtc_state *crtc_state,
			     struct drm_plane_state *plane_state)
{
	struct mipi_dbi_dev *dbidev = drm_to_mipi_dbi_dev(pipe->crtc.dev);
	struct mipi_dbi *dbi = &dbidev->dbi;
	int ret, idx;

	if (!drm_dev_enter(pipe->crtc.dev, &idx))
		return;

	DRM_DEBUG_KMS("\n");

	ret = mipi_dbi_poweron_conditional_reset(dbidev);
	if (ret < 0)
		goto out_exit;

	mipi_dbi_command(dbi, ST7365P_PGAMCTRL,
					 0x00, 0x03, 0x09, 0x08, 0x16, 0x0A,
					 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08,
					 0x16, 0x1A, 0x0F);
	mipi_dbi_command(dbi, ST7365P_NGAMCTRL,
					 0x00, 0x16, 0x19, 0x03, 0x0F, 0x05,
					 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D,
					 0x35, 0x37, 0x0F);
	mipi_dbi_command(dbi, ST7365P_PWCTRL1, 0x17, 0x15);
	mipi_dbi_command(dbi, ST7365P_PWCTRL2, 0x41);
	mipi_dbi_command(dbi, ST7365P_VMCTRL, 0x00, 0x12, 0x80);
	mipi_dbi_command(dbi, ST7365P_MADCTL,
			 ST7365P_MADCTL_MX | ST7365P_MADCTL_BGR);
	/* The ST7365P takes RGB565 straight over SPI, no RGB666 padding needed */
	mipi_dbi_command(dbi, ST7365P_COLMOD, MIPI_DCS_PIXEL_FMT_16BIT << 4 |
					      MIPI_DCS_PIXEL_FMT_16BIT);
	mipi_dbi_command(dbi, ST7365P_IFMODE, 0x00);
	/* fosc/2, 17 clocks per line */
	mipi_dbi_command(dbi, ST7365P_FRMCTR1, 0xD0, 0x11);
	mipi_dbi_command(dbi, ST7365P_INVON);
	mipi_dbi_command(dbi, ST7365P_INVTR, 0x02);
	mipi_dbi_command(dbi, ST7365P_DISCTRL, 0x02, 0x02, 0x3B);
	mipi_dbi_command(dbi, ST7365P_ETMOD, 0xC6);
	mipi_dbi_command(dbi, ST7365P_IMGFUNC, 0x00);
	mipi_dbi_command(dbi, ST7365P_ADJCTRL3, 0xA9, 0x51, 0x2C, 0x82);
	mipi_dbi_command(dbi, ST7365P_SLPOUT);
	msleep(120);
	mipi_dbi_enable_flush(dbidev, crtc_state, plane_state);
	msleep(50);
	mipi_dbi_command(dbi, ST7365P_DISPON);

out_exit:
	drm_dev_exit(idx);
}

static const struct drm_simple_display_pipe_funcs st7365p_pipe_funcs = {
	.mode_valid = mipi_dbi_pipe_mode_valid,
	.enable = st7365p_enable,
	.disable = mipi_dbi_pipe_disable,
	.update = mipi_dbi_pipe_update,
};

static const struct drm_display_mode st7365p_mode = {
	DRM_SIMPLE_MODE(320, 320, 49, 49),
};

DEFINE_DRM_GEM_DMA_FOPS(st7365p_fops);

static const struct drm_driver st7365p_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &st7365p_fops,
	DRM_GEM_DMA_DRIVER_OPS_VMAP,
	.debugfs_init		= mipi_dbi_debugfs_init,
	.name			= "st7365p_picocalc",
	.desc			= "Sitronix ST7365P TinyDRM",
	.date			= "20250819",
	.major			= 1,
	.minor			= 0,
};

static const struct of_device_id st7365p_of_match[] = {
	{ .compatible = "sitronix,st7365p" },
	{ .compatible = "picocalc,st7365p" },
	{},
};
MODULE_DEVICE_TABLE(of, st7365p_of_match);

static const struct spi_device_id st7365p_id[] = {
	{ "st7365p", 0 },
	{},
};
MODULE_DEVICE_TABLE(spi, st7365p_id);

static int st7365p_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mipi_dbi_dev *dbidev;
	struct drm_device *drm;
	struct mipi_dbi *dbi;
	struct gpio_desc *dc;
	struct backlight_properties props;
	u32 rotation = 0;
	int ret;

	dbidev = devm_drm_dev_alloc(dev, &st7365p_driver,
				    struct mipi_dbi_dev, drm);
	if (IS_ERR(dbidev))
		return PTR_ERR(dbidev);

	dbi = &dbidev->dbi;
	drm = &dbidev->drm;

	dbi->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(dbi->reset))
		return dev_err_probe(dev, PTR_ERR(dbi->reset), "Failed to get GPIO 'reset'\n");

	dc = devm_gpiod_get(dev, "dc", GPIOD_OUT_LOW);
	if (IS_ERR(dc))
		return dev_err_probe(dev, PTR_ERR(dc), "Failed to get GPIO 'dc'\n");

	dbidev->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(dbidev->backlight))
	{
		memset(&props, 0, sizeof(props));
		props.type = BACKLIGHT_RAW;
		props.max_brightness = 1;
		dbidev->backlight = devm_backlight_device_register(dev, "dummy-backlight",
										dev, NULL, &dummy_backlight_ops, &props);
		if (IS_ERR(dbidev->backlight))
			return dev_err_probe(dev, PTR_ERR(dbidev->backlight), "Failed to register dummy-backlight\n");
	}

	device_property_read_u32(dev, "rotation", &rotation);

	if (spi_speed_hz)
		spi->max_speed_hz = spi_speed_hz;

	ret = mipi_dbi_spi_init(spi, dbi, dc);
	if (ret)
		return ret;

	dbi->read_commands = NULL;

	ret = mipi_dbi_dev_init(dbidev, &st7365p_pipe_funcs,
		&st7365p_mode, rotation);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;

	spi_set_drvdata(spi, drm);

	dev_info(dev, "ST7365P on SPI at %u Hz for pixel data\n",
		 spi->max_speed_hz);

	drm_fbdev_generic_setup(drm, 0);
	return 0;
}

static void st7365p_remove(struct spi_device *spi)
{
	struct drm_device *drm = spi_get_drvdata(spi);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void st7365p_shutdown(struct spi_device *spi)
{
	drm_atomic_helper_shutdown(spi_get_drvdata(spi));
}

static struct spi_driver st7365p_spi_driver = {
	.driver = {
		.name = "st7365p_picocalc",
		.of_match_table = st7365p_of_match,
	},
	.id_table = st7365p_id,
	.probe = st7365p_probe,
	.remove = st7365p_remove,
	.shutdown = st7365p_shutdown,
};
module_spi_driver(st7365p_spi_driver);

MODULE_DESCRIPTION("Sitronix ST7365P TinyDRM driver for PicoCalc");
MODULE_AUTHOR("Calculinux");
MODULE_LICENSE("GPL");
