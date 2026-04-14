// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Flydigi Vader 4 Pro D-Input support
 *  fixes weird button map over Bluetooth connection
 *
 *  Copyright (c) 2026 Alexey Melnikov
 */

#include <linux/hid.h>
#include <linux/input.h>
#include <linux/module.h>

static int vader4pro_event(struct hid_device *hdev, struct hid_field *field,
                           struct hid_usage *usage, __s32 value)
{
	struct input_dev *input = field->hidinput->input;

	if (usage->type == EV_KEY) {
		switch (usage->code) {
		case KEY_ENTER:
			input_report_key(input, BTN_Z, value);
			input_sync(input);
			return 1;

		case BTN_0:
			input_report_key(input, BTN_C, value);
			input_sync(input);
			return 1;

		case KEY_CHANNELUP:
			input_report_key(input, BTN_GRIPL2, value);
			input_sync(input);
			return 1;

		case KEY_TV:
			input_report_key(input, BTN_GRIPL, value);
			input_sync(input);
			return 1;

		case KEY_CHANNELDOWN:
			input_report_key(input, BTN_GRIPR, value);
			input_sync(input);
			return 1;

		case KEY_EJECTCD:
			input_report_key(input, BTN_GRIPR2, value);
			input_sync(input);
			return 1;

		case KEY_CAMERA:
			input_report_key(input, BTN_TRIGGER_HAPPY3, value);
			input_sync(input);
			return 1;

		case KEY_RED:
			input_report_key(input, BTN_MODE, value);
			input_sync(input);
			return 1;
		}
	}
	return 0;
}

static int vader4pro_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct hid_input *hidinput;
	int ret = hid_parse(hdev);
	if (ret) return ret;

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) return ret;

	list_for_each_entry(hidinput, &hdev->inputs, list) {
		struct input_dev *input = hidinput->input;
		set_bit(EV_KEY, input->evbit);
		set_bit(BTN_C, input->keybit);
		set_bit(BTN_Z, input->keybit);
		set_bit(BTN_GRIPL, input->keybit);
		set_bit(BTN_GRIPL2, input->keybit);
		set_bit(BTN_GRIPR, input->keybit);
		set_bit(BTN_GRIPR2, input->keybit);
		set_bit(BTN_TRIGGER_HAPPY3, input->keybit);
		set_bit(BTN_MODE, input->keybit);
	}
	return 0;
}

static const struct hid_device_id vader4pro_devices[] = {
	{ HID_BLUETOOTH_DEVICE(0xd7d7, 0x0041) },
	{ }
};
MODULE_DEVICE_TABLE(hid, vader4pro_devices);

static struct hid_driver vader4pro_driver = {
	.name = "vader4pro",
	.id_table = vader4pro_devices,
	.probe = vader4pro_probe,
	.event = vader4pro_event,
};
module_hid_driver(vader4pro_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexey Melnikov");
MODULE_DESCRIPTION("Flydigi Vader 4 Pro D-Input mode button remapper");

