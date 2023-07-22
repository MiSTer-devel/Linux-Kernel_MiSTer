// SPDX-License-Identifier: GPL-2.0+
/*
 * HID driver for Nintendo Gamecube Controller Adapters
 *
 * Copyright (c) 2020 François-Xavier Carton <fx.carton91@gmail.com>
 *
 * This driver is based on:
 *   https://github.com/ToadKing/wii-u-gc-adapter
 *   drivers/hid/hid-wiimote-core.c
 *   drivers/hid/hid-steam.c
 *
 */

#include "hid-ids.h"
#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/rcupdate.h>
#include <linux/usb.h>
#include "usbhid/usbhid.h"

enum gamecube_output {
	GC_CMD_INIT = 0x13,
	GC_CMD_RUMBLE = 0x11
};

enum gamecube_input {
	GC_INPUT_REPORT = 0x21
};

#define GC_INPUT_REPORT_SIZE 37

enum gamecube_ctrl_flags {
	GC_FLAG_EXTRAPOWER = BIT(2),
	GC_TYPE_NORMAL = BIT(4),
	GC_TYPE_WAVEBIRD = BIT(5),
	GC_TYPES = GC_TYPE_NORMAL | GC_TYPE_WAVEBIRD
};

enum gamecube_btn {
	GC_BTN_START = BIT(0),
	GC_BTN_Z = BIT(1),
	GC_BTN_R = BIT(2),
	GC_BTN_L = BIT(3),
	GC_BTN_A = BIT(8),
	GC_BTN_B = BIT(9),
	GC_BTN_X = BIT(10),
	GC_BTN_Y = BIT(11),
	GC_BTN_DPAD_LEFT = BIT(12),
	GC_BTN_DPAD_RIGHT = BIT(13),
	GC_BTN_DPAD_DOWN = BIT(14),
	GC_BTN_DPAD_UP = BIT(15),
};

struct gamecube_ctrl {
	struct input_dev __rcu *input;
	struct gamecube_adpt *adpt;
	enum gamecube_ctrl_flags flags;
	u8 axis_min[6];
	u8 axis_max[6];
	u8 rumble;
	struct work_struct work_connect;
	spinlock_t flags_lock;
	spinlock_t rumble_lock;
};

struct gamecube_adpt {
	struct gamecube_ctrl ctrls[4];
	struct hid_device *hdev;
	struct work_struct work_rumble;
	u8 rumble;
};

static int gamecube_hid_send(struct hid_device *hdev, const u8 *data, size_t n)
{
	u8 *buf;
	int ret;

	buf = kmemdup(data, n, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	ret = hid_hw_output_report(hdev, buf, n);
	kfree(buf);
	return ret;
}

static int gamecube_send_cmd_init(struct hid_device *hdev)
{
	const u8 initcmd[] = {GC_CMD_INIT};

	return gamecube_hid_send(hdev, initcmd, sizeof(initcmd));
}

#ifdef CONFIG_HID_GAMECUBE_ADAPTER_FF
static int gamecube_send_cmd_rumble(struct gamecube_adpt *adpt)
{
	struct gamecube_ctrl *ctrls = adpt->ctrls;
	u8 cmd[5] = {GC_CMD_RUMBLE};
	unsigned long flags;
	unsigned int i;
	int rumble_ok;
	u8 rumble = 0;

	for (i = 0; i < 4; i++) {
		spin_lock_irqsave(&ctrls[i].flags_lock, flags);
		rumble_ok = (ctrls[i].flags & GC_TYPES)
			&& (ctrls[i].flags & GC_FLAG_EXTRAPOWER);
		spin_unlock_irqrestore(&ctrls[i].flags_lock, flags);
		if (!rumble_ok)
			continue;
		spin_lock_irqsave(&ctrls[i].rumble_lock, flags);
		cmd[i + 1] = ctrls[i].rumble;
		rumble |= (ctrls[i].rumble & 1U) << i;
		spin_unlock_irqrestore(&ctrls[i].rumble_lock, flags);
	}
	if (rumble == adpt->rumble)
		return 0;
	adpt->rumble = rumble;
	return gamecube_hid_send(adpt->hdev, cmd, sizeof(cmd));
}

static void gamecube_rumble_worker(struct work_struct *work)
{
	struct gamecube_adpt *adpt = container_of(work, struct gamecube_adpt,
						  work_rumble);

	gamecube_send_cmd_rumble(adpt);
}

static int gamecube_rumble_play(struct input_dev *dev, void *data,
							 struct ff_effect *eff)
{
	struct gamecube_ctrl *ctrl = input_get_drvdata(dev);
	struct gamecube_adpt *adpt = ctrl->adpt;
	unsigned long flags;

	if (eff->type != FF_RUMBLE)
		return 0;

	spin_lock_irqsave(&ctrl->rumble_lock, flags);
	ctrl->rumble = (eff->u.rumble.strong_magnitude
			|| eff->u.rumble.weak_magnitude);
	spin_unlock_irqrestore(&ctrl->rumble_lock, flags);
	schedule_work(&adpt->work_rumble);
	return 0;
}
#endif

static const unsigned int gamecube_buttons[] = {
	BTN_START, BTN_TR2, BTN_TR, BTN_TL,
	BTN_SOUTH, BTN_WEST, BTN_EAST, BTN_NORTH,
	BTN_DPAD_LEFT, BTN_DPAD_RIGHT, BTN_DPAD_DOWN, BTN_DPAD_UP
};

static const unsigned int gamecube_axes[] = {
	ABS_X, ABS_Y, ABS_RX, ABS_RY, ABS_Z, ABS_RZ
};

static const char *gamecube_ctrl_name(u8 type)
{
	switch (type) {
	case GC_TYPE_NORMAL:
		return "Standard Gamecube Controller";
	case GC_TYPE_WAVEBIRD:
		return "Wavebird Gamecube Controller";
	}
	return NULL;
}

static int gamecube_ctrl_create(struct gamecube_ctrl *ctrl, u8 type)
{
	struct input_dev *input;
	struct hid_device *hdev = ctrl->adpt->hdev;
	const char *name;
	unsigned int i;
	int ret;

	name = gamecube_ctrl_name(type);
	if (!name) {
		unsigned int num = ctrl - ctrl->adpt->ctrls + 1U;

		hid_warn(hdev, "port %u: unknown controller plugged in\n", num);
		return -EINVAL;
	}

	input = input_allocate_device();
	if (!input)
		return -ENOMEM;

	input_set_drvdata(input, ctrl);
	input->id.bustype = hdev->bus;
	input->id.vendor = hdev->vendor;
	input->id.product = hdev->product;
	input->id.version = hdev->version;
	input->name = name;

	for (i = 0; i < ARRAY_SIZE(gamecube_buttons); i++)
		input_set_capability(input, EV_KEY, gamecube_buttons[i]);
	for (i = 0; i < ARRAY_SIZE(gamecube_axes); i++)
		input_set_abs_params(input, gamecube_axes[i], 0, 255, 0, 0);
#ifdef CONFIG_HID_GAMECUBE_ADAPTER_FF
	if (type == GC_TYPE_NORMAL) {
		input_set_capability(input, EV_FF, FF_RUMBLE);
		if (input_ff_create_memless(input, NULL, gamecube_rumble_play))
			hid_warn(hdev, "failed to create ff memless\n");
	}
#endif

	ret = input_register_device(input);
	if (ret)
		goto err_free_device;

	rcu_assign_pointer(ctrl->input, input);
	return 0;

err_free_device:
	input_free_device(input);
	return ret;
}

static void gamecube_ctrl_destroy(struct gamecube_ctrl *ctrl)
{
	struct input_dev *input;

	rcu_read_lock();
	input = rcu_dereference(ctrl->input);
	rcu_read_unlock();
	if (!input)
		return;
	RCU_INIT_POINTER(ctrl->input, NULL);
	synchronize_rcu();
	input_unregister_device(input);
}

static void gamecube_work_connect_cb(struct work_struct *work)
{
	struct gamecube_ctrl *ctrl = container_of(work, struct gamecube_ctrl,
						work_connect);
	struct input_dev *input;
	unsigned long irq_flags;
	unsigned int num = ctrl - ctrl->adpt->ctrls + 1U;
	u8 type;

	spin_lock_irqsave(&ctrl->flags_lock, irq_flags);
	type = ctrl->flags & GC_TYPES;
	spin_unlock_irqrestore(&ctrl->flags_lock, irq_flags);

	rcu_read_lock();
	input = rcu_dereference(ctrl->input);
	rcu_read_unlock();

	if (type && input) {
		hid_info(ctrl->adpt->hdev, "port %u: already connected\n", num);
	} else if (type) {
		hid_info(ctrl->adpt->hdev, "port %u: controller plugged in\n", num);
		gamecube_ctrl_create(ctrl, type);
	} else if (input) {
		hid_info(ctrl->adpt->hdev, "port %u: controller unplugged\n", num);
		gamecube_ctrl_destroy(ctrl);
	}
}

static void gamecube_ctrl_handle_report(struct gamecube_ctrl *ctrl, u8 *data)
{
	struct input_dev *dev;
	u16 btns = data[1] << 8 | data[2];
	u8 old_flags, new_flags = data[0];
	unsigned long irq_flags;
	unsigned int i;

	spin_lock_irqsave(&ctrl->flags_lock, irq_flags);
	old_flags = ctrl->flags;
	ctrl->flags = new_flags;
	spin_unlock_irqrestore(&ctrl->flags_lock, irq_flags);

	if ((new_flags & GC_TYPES) != (old_flags & GC_TYPES)) {
		// Reset min/max values. The default values were obtained empirically
		for (i = 0; i < 6; i++) {
			ctrl->axis_min[i] = 45; // max across all axes of min values
			ctrl->axis_max[i] = 215; // min across all axes of max values
		}
		schedule_work(&ctrl->work_connect);
		return;
	}
	if (!(new_flags & GC_TYPES))
		return;

	rcu_read_lock();
	dev = rcu_dereference(ctrl->input);
	if (!dev)
		goto unlock;

	input_report_key(dev, BTN_START, btns & GC_BTN_START);
	input_report_key(dev, BTN_TR2, btns & GC_BTN_Z);
	input_report_key(dev, BTN_TR, btns & GC_BTN_R);
	input_report_key(dev, BTN_TL, btns & GC_BTN_L);
	input_report_key(dev, BTN_SOUTH, btns & GC_BTN_A);
	input_report_key(dev, BTN_WEST, btns & GC_BTN_B);
	input_report_key(dev, BTN_EAST, btns & GC_BTN_X);
	input_report_key(dev, BTN_NORTH, btns & GC_BTN_Y);
	input_report_key(dev, BTN_DPAD_LEFT, btns & GC_BTN_DPAD_LEFT);
	input_report_key(dev, BTN_DPAD_RIGHT, btns & GC_BTN_DPAD_RIGHT);
	input_report_key(dev, BTN_DPAD_DOWN, btns & GC_BTN_DPAD_DOWN);
	input_report_key(dev, BTN_DPAD_UP, btns & GC_BTN_DPAD_UP);
	for (i = 0; i < 6; i++) {
		u8 a, b, v = data[3 + i];

		a = ctrl->axis_min[i] = min(ctrl->axis_min[i], v);
		b = ctrl->axis_max[i] = max(ctrl->axis_max[i], v);
		v = 255U * (v - a) / (b - a);
		if (gamecube_axes[i] == ABS_Y || gamecube_axes[i] == ABS_RY)
			v = 255U - v;
		input_report_abs(dev, gamecube_axes[i], v);
	}
	input_sync(dev);

unlock:
	rcu_read_unlock();
}

static int gamecube_hid_event(struct hid_device *hdev,
			struct hid_report *report, u8 *raw_data, int size)
{
	struct gamecube_adpt *adpt = hid_get_drvdata(hdev);
	u8 *ctrl_data;
	unsigned int i;

	if (size < 1)
		return -EINVAL;
	if (size == GC_INPUT_REPORT_SIZE && raw_data[0] == GC_INPUT_REPORT) {
		for (i = 0; i < 4; i++) {
			ctrl_data = raw_data + 1 + 9 * i;
			gamecube_ctrl_handle_report(adpt->ctrls + i, ctrl_data);
		}
	} else {
		hid_warn(hdev, "unhandled event\n");
	}

	return 0;
}

static struct gamecube_adpt *gamecube_adpt_create(struct hid_device *hdev)
{
	struct gamecube_adpt *adpt;
	struct gamecube_ctrl *ctrl;
	unsigned int i;

	adpt = kzalloc(sizeof(*adpt), GFP_KERNEL);
	if (!adpt)
		return NULL;

	adpt->hdev = hdev;
	hid_set_drvdata(hdev, adpt);

	for (i = 0; i < 4; i++) {
		ctrl = adpt->ctrls + i;
		ctrl->adpt = adpt;
		INIT_WORK(&ctrl->work_connect, gamecube_work_connect_cb);
		spin_lock_init(&ctrl->flags_lock);
		spin_lock_init(&ctrl->rumble_lock);
	}
#ifdef CONFIG_HID_GAMECUBE_ADAPTER_FF
	INIT_WORK(&adpt->work_rumble, gamecube_rumble_worker);
	adpt->rumble = 0;
#endif

	return adpt;
}

static void gamecube_adpt_destroy(struct gamecube_adpt *adpt)
{
	unsigned int i;

	for (i = 0; i < 4; i++)
		gamecube_ctrl_destroy(adpt->ctrls + i);
#ifdef CONFIG_HID_GAMECUBE_ADAPTER_FF
	cancel_work_sync(&adpt->work_rumble);
#endif
	hid_hw_close(adpt->hdev);
	hid_hw_stop(adpt->hdev);
	kfree(adpt);
}

/* This is needed, as by default the URB buffer size is set to 38, which is
 * one byte too long and will result in EOVERFLOW failures.
 */
static int gamecube_fixup_urb_in(struct gamecube_adpt *adpt)
{
	struct hid_device *hdev = adpt->hdev;
	struct usbhid_device *usbhid;

	if (!hid_is_using_ll_driver(hdev, &usb_hid_driver))
		return -EINVAL;
	usbhid = hdev->driver_data;
	if (usbhid->urbin->transfer_buffer_length < GC_INPUT_REPORT_SIZE)
		return -EINVAL;
	usbhid->urbin->transfer_buffer_length = GC_INPUT_REPORT_SIZE;
	return 0;
}

static int gamecube_hid_probe(struct hid_device *hdev,
				const struct hid_device_id *id)
{
	struct gamecube_adpt *adpt;
	int ret;

	adpt = gamecube_adpt_create(hdev);
	if (!adpt) {
		hid_err(hdev, "Can't alloc device\n");
		return -ENOMEM;
	}

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "HID parse failed\n");
		goto err;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret) {
		hid_err(hdev, "HW start failed\n");
		goto err;
	}

	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "cannot start hardware I/O\n");
		goto err_stop;
	}

	ret = gamecube_fixup_urb_in(adpt);
	if (ret) {
		hid_err(hdev, "failed to fix input URB\n");
		goto err_close;
	}

	ret = gamecube_send_cmd_init(hdev);
	if (ret < 0) {
		hid_err(hdev, "failed to send init command\n");
		goto err_close;
	}

	hid_info(hdev, "new adapter registered\n");
	return 0;

err_close:
	hid_hw_close(hdev);
err_stop:
	hid_hw_stop(hdev);
err:
	kfree(adpt);
	return ret;
}

#ifdef CONFIG_PM
static int gamecube_resume(struct hid_device *hdev)
{
	gamecube_send_cmd_init(hdev);
	return 0;
}
#endif

static void gamecube_hid_remove(struct hid_device *hdev)
{
	struct gamecube_adpt *adpt = hid_get_drvdata(hdev);

	hid_info(hdev, "adapter removed\n");
	gamecube_adpt_destroy(adpt);
}

static const struct hid_device_id gamecube_hid_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_NINTENDO,
			 USB_DEVICE_ID_NINTENDO_GAMECUBE_ADAPTER) },
	{ }
};
MODULE_DEVICE_TABLE(hid, gamecube_hid_devices);

static struct hid_driver gamecube_hid_driver = {
	.name		= "gamecube-adapter",
	.id_table	= gamecube_hid_devices,
	.probe		= gamecube_hid_probe,
	.remove		= gamecube_hid_remove,
	.raw_event	= gamecube_hid_event,
#ifdef CONFIG_PM
	.reset_resume	= gamecube_resume,
#endif
};
module_hid_driver(gamecube_hid_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("François-Xavier Carton <fx.carton91@gmail.com>");
MODULE_DESCRIPTION("Driver for Nintendo Gamecube Controller Adapters");
