/*
 *  MiSTer_fb.c -- MiSTer Frame Reader driver
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/uaccess.h>
#include <linux/console.h>
#include <linux/fbcon.h>

#define PALETTE_SIZE         256
#define DRIVER_NAME          "MiSTer_fb"
#define VSYNC_TIMEOUT_MSEC   50

static u32 width = 0, height = 0, stride = 0, format = 0, rb = 1, frame_count = 0, res_count = 0;
module_param(width,  uint, 0444);
module_param(height, uint, 0444);
module_param(stride, uint, 0444);
module_param(format, uint, 0444);
module_param(rb,     uint, 0444);
module_param(frame_count, uint, 0444);
module_param(res_count, uint, 0444);

struct fb_dev {
	struct platform_device *pdev;
	struct fb_info info;
	struct resource *fb_res;
	void __iomem *fb_base;
	int irq;
};

static wait_queue_head_t vs_wait;
static irqreturn_t irq_handler(int irq, void *par)
{
	frame_count++;
	wake_up_interruptible(&vs_wait);
	return IRQ_HANDLED;
}

static struct fb_dev *p_fbdev = 0;

static int fb_setcolreg(unsigned regno, unsigned red, unsigned green,
			   unsigned blue, unsigned transp, struct fb_info *info)
{
	if (regno > 255) return 1;

	red >>= 8;
	green >>= 8;
	blue >>= 8;

	red &= 255;
	green &= 255;
	blue &= 255;

	if (regno < 256)
	{
		if(format==1555)
		{
			red >>=3; green >>=3; blue >>=3;
			((u32 *)info->pseudo_palette)[regno] = rb ? (red << 10) | (green << 5) | blue : (blue << 10) | (green << 5) | red;
		}
		else if(format==565)
		{
			red >>=3; green >>=2; blue >>=3;
			((u32 *)info->pseudo_palette)[regno] = rb ? (red << 11) | (green << 5) | blue : (blue << 11) | (green << 5) | red;
		}
		else if(format==8)
		{
			((u32 *)info->pseudo_palette)[regno] = (red << 16) | (green << 8) | blue;
		}
		else
		{
			((u32 *)info->pseudo_palette)[regno] = rb ? (red << 16) | (green << 8) | blue : (blue << 16) | (green << 8) | red;
		}
	}

	return 0;
}

static int fb_wait_for_vsync(u32 arg)
{
	u32 count;
	int ret;

	if (arg != 0) return -ENODEV;

	count = frame_count;
	ret = wait_event_interruptible_timeout(vs_wait,
				       count != frame_count,
				       msecs_to_jiffies(VSYNC_TIMEOUT_MSEC));

	return (!ret) ? -ETIMEDOUT : 0;
}

static int fb_ioctl(struct fb_info *info, unsigned int cmd, unsigned long arg)
{
	int ret;
	u32 ioarg;

	switch (cmd)
	{
	case FBIO_WAITFORVSYNC:
		if (get_user(ioarg, (u32 __user *)arg))
		{
			ret = -EFAULT;
			break;
		}

		ret = fb_wait_for_vsync(ioarg);
		break;
	default:
		ret = -ENOTTY;
	}

	return ret;
}

static struct fb_ops ops = {
	.owner = THIS_MODULE,
	.fb_fillrect = sys_fillrect,
	.fb_copyarea = sys_copyarea,
	.fb_imageblit = sys_imageblit,
	.fb_setcolreg = fb_setcolreg,
	.fb_ioctl = fb_ioctl,
};

static int setup_fb_info(struct fb_dev *fbdev)
{
	struct fb_info *info = &fbdev->info;

	if(!width) width = 640;
	if(!height) height = 480;
	if(!stride) stride = (width*4 + 255) & ~255;

	info->var.xres = width;
	info->var.yres = height;
	info->fix.line_length = stride;

	strcpy(info->fix.id, DRIVER_NAME);
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = (format == 8) ? FB_VISUAL_PSEUDOCOLOR : FB_VISUAL_TRUECOLOR;
	info->fix.accel = FB_ACCEL_NONE;

	info->fbops = &ops;
	info->var.activate = FB_ACTIVATE_NOW;
	info->var.height = -1;
	info->var.width = -1;
	info->var.vmode = FB_VMODE_NONINTERLACED;

	info->var.xres_virtual = info->var.xres,
	info->var.yres_virtual = info->var.yres;

	info->var.red.msb_right = 0;
	info->var.green.msb_right = 0;
	info->var.blue.msb_right = 0;

	if(format==1555)
	{
		/* settings for 16bit pixels */
		info->var.bits_per_pixel = 16;

		info->var.red.offset = 0;
		info->var.green.offset = 5;
		info->var.blue.offset = 10;

		info->var.red.length = 5;
		info->var.green.length = 5;
		info->var.blue.length = 5;
	}
	else if(format==565)
	{
		/* settings for 16bit pixels */
		info->var.bits_per_pixel = 16;

		info->var.red.offset = 0;
		info->var.green.offset = 5;
		info->var.blue.offset = 11;

		info->var.red.length = 5;
		info->var.green.length = 6;
		info->var.blue.length = 5;
	}
	else if(format==8)
	{
		/* settings for 256 color mode */
		info->var.bits_per_pixel = 8;

		info->var.red.offset = 0;
		info->var.green.offset = 0;
		info->var.blue.offset = 0;

		info->var.red.length = 8;
		info->var.green.length = 8;
		info->var.blue.length = 8;
	}
	else
	{
		/* settings for 32bit pixels */
		info->var.bits_per_pixel = 32;

		info->var.red.offset = 0;
		info->var.green.offset = 8;
		info->var.blue.offset = 16;

		info->var.red.length = 8;
		info->var.green.length = 8;
		info->var.blue.length = 8;
		format = 8888;
	}

	if(rb)
	{
		u32 tmp = info->var.red.offset;
		info->var.red.offset = info->var.blue.offset;
		info->var.blue.offset = tmp;
	}

	dev_info(&fbdev->pdev->dev, "width = %u, height = %u, format=%u\n", info->var.xres, info->var.yres, format);

	info->fix.smem_len = info->fix.line_length * info->var.yres;

	info->pseudo_palette = fbdev->fb_base;
	info->flags = FBINFO_FLAG_DEFAULT;

	return 0;
}

static int fb_probe(struct platform_device *pdev)
{
	int retval;
	struct fb_dev *fbdev;

	fbdev = devm_kzalloc(&pdev->dev, sizeof(*fbdev), GFP_KERNEL);
	if (!fbdev)	return -ENOMEM;

	fbdev->pdev = pdev;

	fbdev->fb_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!fbdev->fb_res) return -ENODEV;

	fbdev->fb_base = memremap(fbdev->fb_res->start, resource_size(fbdev->fb_res), MEMREMAP_WT);
	if (IS_ERR(fbdev->fb_base))
	{
		dev_err(&pdev->dev, "devm_ioremap_resource fb failed\n");
		return PTR_ERR(fbdev->fb_base);
	}

	retval = setup_fb_info(fbdev);
	fbdev->info.fix.smem_start = fbdev->fb_res->start+4096;
	fbdev->info.screen_base = fbdev->fb_base+4096;

	retval = fb_alloc_cmap(&fbdev->info.cmap, PALETTE_SIZE, 0);
	if (retval < 0) return retval;

	platform_set_drvdata(pdev, fbdev);

	retval = register_framebuffer(&fbdev->info);
	if (retval < 0) goto err_dealloc_cmap;

	dev_info(&pdev->dev, "fb%d: %s frame buffer device at 0x%x+0x%x, virt: 0x%p\n",
		 fbdev->info.node, fbdev->info.fix.id,
		 (unsigned)fbdev->info.fix.smem_start,
		 fbdev->info.fix.smem_len, fbdev->info.screen_base);

	fbdev->irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	dev_info(&pdev->dev, "fb%d: IRQ=%d\n", fbdev->info.node, fbdev->irq);

	init_waitqueue_head(&vs_wait);

	if (fbdev->irq == NO_IRQ || request_irq(fbdev->irq, &irq_handler, IRQF_SHARED, "MiSTer_fb", fbdev))
	{
		dev_err(&pdev->dev, "request_irq failed. fb won't use IRQ.\n");
	}

	p_fbdev = fbdev;
	return 0;

err_dealloc_cmap:
	fb_dealloc_cmap(&fbdev->info.cmap);
	return retval;
}

static int fb_remove(struct platform_device *dev)
{
	struct fb_dev *fbdev = platform_get_drvdata(dev);

	if (fbdev)
	{
		unregister_framebuffer(&fbdev->info);
		fb_dealloc_cmap(&fbdev->info.cmap);
		if(fbdev->irq != NO_IRQ) free_irq(fbdev->irq, dev);
		fbdev->irq = NO_IRQ;
	}
	return 0;
}

void fb_set(struct fb_info *info)
{
	struct fb_videomode mode;
	int ret = 0;

	fb_pan_display(info, &info->var);
	fb_set_cmap(&info->cmap, info);
	fb_var_to_videomode(&mode, &info->var);

	if (info->modelist.prev && info->modelist.next && !list_empty(&info->modelist))
	{
		ret = fb_add_videomode(&mode, &info->modelist);
	}

	if (!ret)
	{
		/*
		struct fb_event event;
		info->flags &= ~FBINFO_MISC_USEREVENT;
		event.info = info;
		event.data = &mode;
		fb_notifier_call_chain(FB_EVENT_MODE_CHANGE_ALL, &event);
		*/
		fbcon_update_vcs(info, true);
	}
}

static int mode_set(const char *val, const struct kernel_param *kp)
{
	if(p_fbdev)
	{
		console_lock();
		lock_fb_info(&p_fbdev->info);

		memset(p_fbdev->fb_base, 0, resource_size(p_fbdev->fb_res));

		sscanf(val, "%u %u %u %u %u", &format, &rb, &width, &height, &stride);
		setup_fb_info(p_fbdev);

		fb_set(&p_fbdev->info);
		unlock_fb_info(&p_fbdev->info);

		res_count++;
		console_unlock();
	}
	return 0;
}

static int mode_get(char *val, const struct kernel_param *kp)
{
	if(p_fbdev)
	{
		sprintf(val, "%u %u %u %u %u", format, rb, width, height, stride);
		return strlen(val);
	}
	return 0;
}

static const struct kernel_param_ops param_ops = {
	.set = mode_set,
	.get = mode_get,
};

module_param_cb(mode, &param_ops, NULL, 0664);


static struct of_device_id fb_match[] = {
	{ .compatible = "MiSTer_fb" },
	{},
};
MODULE_DEVICE_TABLE(of, fb_match);

static struct platform_driver fb_driver = {
	.probe = fb_probe,
	.remove = fb_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
		.of_match_table = fb_match,
	},
};
module_platform_driver(fb_driver);

MODULE_DESCRIPTION("MiSTer framebuffer driver");
MODULE_AUTHOR("Sorgelig@MiSTer");
MODULE_LICENSE("GPL v2");

