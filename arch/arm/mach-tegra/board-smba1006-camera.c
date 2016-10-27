/*
 * arch/arm/mach-tegra/board-smba1006-camera.c
 *
 * Copyright (C) 2011 Eduardo Jose Tagle <ejtagle@tutopia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#if 0
#include <linux/console.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_data/tegra_usb.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/reboot.h>
#include <linux/memblock.h>
#include <linux/earlysuspend.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <media/tegra_v4l2_camera.h>
#include <media/soc_camera.h>
#include <media/s5k4cdgx.h>

#include <mach/io.h>
#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/nand.h>
#include <mach/iomap.h>

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "board.h"
#include "board-smba1006.h"
#include "clock.h"
#include "gpio-names.h"
#include "devices.h"

#define S5K4CDGX_POWER_PIN TEGRA_GPIO_PBB5
#define S5K4CDGX_RESET_PIN TEGRA_GPIO_PD2

// TODO: clean these up into a common header
#define S5K4CDGX_MCLK_FREQ 24000000

static struct dentry *dir = 0;
static u32 on = 0;

static int clink_s5k4cdgx_set_power(struct device *dev, int power_on) {
  return smba_s5k4cdgx_set_power(power_on);
}

struct s5k4cdgx_platform_data smba_s5k4cdgx_data = {
	.mclk_frequency = S5K4CDGX_MCLK_FREQ,
	.bus_type = V4L2_MBUS_PARALLEL,
	.gpio_stby = { 
		.gpio = S5K4CDGX_POWER_PIN, 
		.level = 0, // active-low
	},
	.gpio_reset = { 
		.gpio = S5K4CDGX_RESET_PIN,
		.level = 0, // active-low
	},
	.nlanes = 1,
	.horiz_flip = true,
	.vert_flip = true,
};

static struct i2c_board_info smba_i2c3_board_info_camera = {
  I2C_BOARD_INFO("S5K4CDGX",  0x3c),
  // platform data will get overwritten here with soc_camera_device
};

static struct soc_camera_link clink_s5k4cdgx = {
  .board_info = &smba_i2c3_board_info_camera,
  .i2c_adapter_id = 3,
  .power = &clink_s5k4cdgx_set_power,
  .priv = &smba_s5k4cdgx_data,
  // TODO: move s5k4cdgx regulators here
};

static struct platform_device smba_tegra_s5k4cdgx_device = {
  .name   = "soc-camera-pdrv",
  .id     = 0,
  .dev    = {
    .platform_data = &clink_s5k4cdgx,
  },
};
#if 0
static void camera_suspend(struct device *dev, pm_message_t state)
{
	pr_info("%s\n", __func__);
	smba_s5k4cdgx_set_power(0);
}

static int __devinit camera_probe(struct platform_device *pdev) {
	return 0;
}

static int __devexit camera_remove(struct platform_device *pdev) {
	return 0;
}

static struct platform_driver tegra_camera_power_device_driver = {
	.probe		= camera_probe,
	.remove		= camera_remove,
	.driver		= {
		.name	= "camera_power"
	},
    .suspend = camera_suspend,
};
#endif

static struct platform_device tegra_camera_power_device = {
  // note the underscore
  .name   = "tegra_camera",
  .id     = 0,
};

#ifdef CONFIG_HAS_EARLYSUSPENDx
/* put early_suspend/late_resume handlers here for the display in order
 * to keep the code out of the display driver, keeping it closer to upstream
 */
struct early_suspend camera_early_suspender;

#endif


/* In theory we might want to use this callback to reference the 
   tegra_camera driver from the soc_camera host driver instead of
   the i2c client driver */
static int smba_enable_camera(struct nvhost_device *ndev)
{
	// struct soc_camera_host *ici = to_soc_camera_host(&ndev->dev);

	dev_dbg(&ndev->dev, "%s\n", __func__);

	return 0;
}

static void smba_disable_camera(struct nvhost_device *ndev)
{
	dev_dbg(&ndev->dev, "%s\n", __func__);
}

#ifdef CONFIG_HAS_EARLYSUSPENDx
static void camera_early_suspend(struct early_suspend *h)
{
	pr_info("%s\n", __func__);
	smba_s5k4cdgx_set_power(0);
}

static void camera_late_resume(struct early_suspend *h)
{
	pr_info("%s\n", __func__);
}
#endif


static int add_read_op(void *data, u64 *value)
{
	*value = on;
	return 0;
}

static int add_write_op(void *data, u64 value)
{
	int set = value == 0? 0 : 1;
	smba_s5k4cdgx_set_power(set);
	on = set;
	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(add_fops, add_read_op, add_write_op, "%llu\n");

static struct tegra_camera_platform_data smba_camera_pdata = {
  .enable_camera = &smba_enable_camera,
  .disable_camera = &smba_disable_camera,
  .flip_h = 0,
  .flip_v = 0,
};

int __init smba_camera_register_devices(void)
{
  int ret;
  struct dentry *junk;


#ifdef CONFIG_HAS_EARLYSUSPENDx
	camera_early_suspender.suspend = camera_early_suspend;
	camera_early_suspender.resume = camera_late_resume;
	camera_early_suspender.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	register_early_suspend(&camera_early_suspender);
#endif

  //platform_driver_register(&tegra_camera_power_device_driver);

  tegra_camera_device.dev.platform_data = &smba_camera_pdata;

  ret = platform_device_register(&tegra_camera_power_device);
  if(ret)
    return ret;

  ret = platform_device_register(&smba_tegra_s5k4cdgx_device);
  if(ret)
    return ret;

  ret = nvhost_device_register(&tegra_camera_device);
  if(ret)
    return ret;
    
    dir = debugfs_create_dir("camera", 0);
    if (!dir) {
        printk(KERN_ALERT "debugfs: failed to create /sys/kernel/debug/camera\n");
        return -1;
    }

	junk = debugfs_create_file("power", 0666, dir, NULL, &add_fops);
	if (!junk) {
		printk(KERN_ALERT "debugfs: Error creating file /sys/kernel/debug/camera/power");
		return -1;
	}

  return 0;
}
#endif