// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2021 Severin von Wnuck <severinvonw@outlook.de>
 */

#include <linux/module.h>
#include <linux/hrtimer.h>
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>

#include "../bus/bus.h"

#define GIP_HS_NAME "Microsoft X-Box One headset"

#define GIP_HS_CONFIG_DELAY msecs_to_jiffies(1000)
#define GIP_HS_POWER_ON_DELAY msecs_to_jiffies(1000)

static const struct snd_pcm_hardware gip_headset_pcm_hw = {
	.info = SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_BATCH |
		SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_CONTINUOUS,
	.periods_min = 2,
	.periods_max = 1024,
};

struct gip_headset {
	struct gip_client *client;

	struct delayed_work config_work;
	struct delayed_work power_on_work;
	struct work_struct register_work;
	bool registered;

	struct hrtimer timer;
	void *buffer;

	struct gip_headset_stream {
		struct snd_pcm_substream *substream;
		snd_pcm_uframes_t pointer;
		snd_pcm_uframes_t period;
	} playback, capture;

	struct snd_card *card;
	struct snd_pcm *pcm;
};

static int gip_headset_pcm_open(struct snd_pcm_substream *sub)
{
	struct gip_headset *headset = snd_pcm_substream_chip(sub);
	struct gip_audio_config *cfg;
	struct snd_pcm_hardware hw = gip_headset_pcm_hw;

	if (sub->stream == SNDRV_PCM_STREAM_PLAYBACK)
		cfg = &headset->client->audio_config_out;
	else
		cfg = &headset->client->audio_config_in;

	hw.rate_min = cfg->sample_rate;
	hw.rate_max = cfg->sample_rate;
	hw.channels_min = cfg->channels;
	hw.channels_max = cfg->channels;
	hw.buffer_bytes_max = cfg->buffer_size * 8;
	hw.period_bytes_min = cfg->buffer_size;
	hw.period_bytes_max = cfg->buffer_size * 8;

	sub->runtime->hw = hw;

	return 0;
}

static int gip_headset_pcm_close(struct snd_pcm_substream *sub)
{
	return 0;
}

static int gip_headset_pcm_hw_params(struct snd_pcm_substream *sub,
				     struct snd_pcm_hw_params *params)
{
	return snd_pcm_lib_alloc_vmalloc_buffer(sub,
						params_buffer_bytes(params));
}

static int gip_headset_pcm_hw_free(struct snd_pcm_substream *sub)
{
	return snd_pcm_lib_free_vmalloc_buffer(sub);
}

static int gip_headset_pcm_prepare(struct snd_pcm_substream *sub)
{
	return 0;
}

static int gip_headset_pcm_trigger(struct snd_pcm_substream *sub, int cmd)
{
	struct gip_headset *headset = snd_pcm_substream_chip(sub);
	struct gip_headset_stream *stream;

	if (sub->stream == SNDRV_PCM_STREAM_PLAYBACK)
		stream = &headset->playback;
	else
		stream = &headset->capture;

	stream->pointer = 0;
	stream->period = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		stream->substream = sub;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		stream->substream = NULL;
		break;
	default:
		return -EINVAL;
	}

	if (!stream->substream && sub->stream == SNDRV_PCM_STREAM_PLAYBACK)
		memset(headset->buffer, 0,
		       headset->client->audio_config_out.buffer_size);

	return 0;
}

static snd_pcm_uframes_t gip_headset_pcm_pointer(struct snd_pcm_substream *sub)
{
	struct gip_headset *headset = snd_pcm_substream_chip(sub);
	struct gip_headset_stream *stream;

	if (sub->stream == SNDRV_PCM_STREAM_PLAYBACK)
		stream = &headset->playback;
	else
		stream = &headset->capture;

	return bytes_to_frames(sub->runtime, stream->pointer);
}

static const struct snd_pcm_ops gip_headset_pcm_ops = {
	.open = gip_headset_pcm_open,
	.close = gip_headset_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = gip_headset_pcm_hw_params,
	.hw_free = gip_headset_pcm_hw_free,
	.prepare = gip_headset_pcm_prepare,
	.trigger = gip_headset_pcm_trigger,
	.pointer = gip_headset_pcm_pointer,
	.page = snd_pcm_lib_get_vmalloc_page,
};

static bool gip_headset_copy_playback(struct gip_headset_stream *stream,
				      unsigned char *data, int len)
{
	struct snd_pcm_runtime *runtime = stream->substream->runtime;
	size_t buf_size = snd_pcm_lib_buffer_bytes(stream->substream);
	size_t remaining = buf_size - stream->pointer;

	if (len <= remaining) {
		memcpy(data, runtime->dma_area + stream->pointer, len);
	} else {
		memcpy(data, runtime->dma_area + stream->pointer, remaining);
		memcpy(data + remaining, runtime->dma_area, len - remaining);
	}

	stream->pointer += len;
	if (stream->pointer >= buf_size)
		stream->pointer -= buf_size;

	stream->period += len;
	if (stream->period >= runtime->period_size) {
		stream->period -= runtime->period_size;
		return true;
	}

	return false;
}

static bool gip_headset_copy_capture(struct gip_headset_stream *stream,
				     unsigned char *data, int len)
{
	struct snd_pcm_runtime *runtime = stream->substream->runtime;
	size_t buf_size = snd_pcm_lib_buffer_bytes(stream->substream);
	size_t remaining = buf_size - stream->pointer;

	if (len <= remaining) {
		memcpy(runtime->dma_area + stream->pointer, data, len);
	} else {
		memcpy(runtime->dma_area + stream->pointer, data, remaining);
		memcpy(runtime->dma_area, data + remaining, len - remaining);
	}

	stream->pointer += len;
	if (stream->pointer >= buf_size)
		stream->pointer -= buf_size;

	stream->period += len;
	if (stream->period >= runtime->period_size) {
		stream->period -= runtime->period_size;
		return true;
	}

	return false;
}

static enum hrtimer_restart gip_headset_send_samples(struct hrtimer *timer)
{
	struct gip_headset *headset = container_of(timer, typeof(*headset),
						   timer);
	struct gip_headset_stream *stream = &headset->playback;
	struct gip_audio_config *cfg = &headset->client->audio_config_out;
	struct snd_pcm_substream *sub = stream->substream;
	bool elapsed = false;
	int err;
	unsigned long flags;

	if (sub) {
		snd_pcm_stream_lock_irqsave(sub, flags);

		if (sub->runtime && snd_pcm_running(sub))
			elapsed = gip_headset_copy_playback(stream,
							    headset->buffer,
							    cfg->buffer_size);

		snd_pcm_stream_unlock_irqrestore(sub, flags);

		if (elapsed)
			snd_pcm_period_elapsed(sub);
	}

	/* retry if driver runs out of buffers */
	err = gip_send_audio_samples(headset->client, headset->buffer);
	if (err && err != -ENOSPC)
		return HRTIMER_NORESTART;

	hrtimer_forward_now(timer, ms_to_ktime(GIP_AUDIO_INTERVAL));

	return HRTIMER_RESTART;
}

static int gip_headset_init_card(struct gip_headset *headset)
{
	struct snd_card *card;
	int err;

	err = snd_card_new(&headset->client->dev, SNDRV_DEFAULT_IDX1,
			   SNDRV_DEFAULT_STR1, THIS_MODULE, 0, &card);
	if (err)
		return err;

	strscpy(card->driver, "GIP Headset", sizeof(card->driver));
	strscpy(card->shortname, GIP_HS_NAME, sizeof(card->shortname));
	snprintf(card->longname, sizeof(card->longname), "%s at %s",
		 GIP_HS_NAME, dev_name(&headset->client->dev));

	headset->card = card;

	return 0;
}

static int gip_headset_init_pcm(struct gip_headset *headset)
{
	struct gip_client *client = headset->client;
	struct snd_pcm *pcm;
	int err;

	err = snd_pcm_new(headset->card, "GIP Headset", 0, 1, 1, &pcm);
	if (err)
		return err;

	strscpy(pcm->name, "GIP Headset", sizeof(pcm->name));
	pcm->private_data = headset;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &gip_headset_pcm_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &gip_headset_pcm_ops);

	headset->buffer = devm_kzalloc(&client->dev,
				       client->audio_config_out.buffer_size,
				       GFP_KERNEL);
	if (!headset->buffer)
		return -ENOMEM;

	headset->pcm = pcm;

	return snd_card_register(headset->card);
}

static int gip_headset_start_audio(struct gip_headset *headset)
{
	struct gip_client *client = headset->client;
	int err;

	/* set headset volume to maximum */
	err = gip_fix_audio_volume(client);
	if (err)
		return err;

	err = gip_init_audio_out(client);
	if (err)
		return err;

	hrtimer_start(&headset->timer, 0, HRTIMER_MODE_REL);

	return 0;
}

static void gip_headset_config(struct work_struct *work)
{
	struct gip_headset *headset = container_of(to_delayed_work(work),
						   typeof(*headset),
						   config_work);
	struct gip_client *client = headset->client;
	struct gip_info_element *fmts = client->audio_formats;
	int err;

	dev_dbg(&client->dev, "%s: format=0x%02x/0x%02x\n", __func__,
		fmts->data[0], fmts->data[1]);

	/* suggest initial audio format */
	err = gip_suggest_audio_format(client, fmts->data[0], fmts->data[1]);
	if (err)
		dev_err(&client->dev, "%s: suggest format failed: %d\n",
			__func__, err);
}

static void gip_headset_power_on(struct work_struct *work)
{
	struct gip_headset *headset = container_of(to_delayed_work(work),
						   typeof(*headset),
						   power_on_work);
	struct gip_client *client = headset->client;
	int err;

	err = gip_set_power_mode(client, GIP_PWR_ON);
	if (err)
		dev_err(&client->dev, "%s: set power mode failed: %d\n",
			__func__, err);
}

static void gip_headset_register(struct work_struct *work)
{
	struct gip_headset *headset = container_of(work, typeof(*headset),
						   register_work);
	struct device *dev = &headset->client->dev;
	int err;

	err = gip_headset_init_card(headset);
	if (err) {
		dev_err(dev, "%s: init card failed: %d\n", __func__, err);
		return;
	}

	err = gip_headset_init_pcm(headset);
	if (err) {
		dev_err(dev, "%s: init PCM failed: %d\n", __func__, err);
		goto err_free_card;
	}

	err = gip_headset_start_audio(headset);
	if (err) {
		dev_err(dev, "%s: start audio failed: %d\n", __func__, err);
		goto err_free_card;
	}

	return;

err_free_card:
	snd_card_free(headset->card);
	headset->card = NULL;
}

static int gip_headset_op_audio_ready(struct gip_client *client)
{
	struct gip_headset *headset = dev_get_drvdata(&client->dev);

	schedule_delayed_work(&headset->power_on_work, GIP_HS_POWER_ON_DELAY);

	return 0;
}

static int gip_headset_op_audio_volume(struct gip_client *client,
				       int in, int out)
{
	struct gip_headset *headset = dev_get_drvdata(&client->dev);

	/* headset reported initial volume, start audio I/O */
	if (!headset->registered) {
		schedule_work(&headset->register_work);
		headset->registered = true;
	}

	/* ignore hardware volume, let software handle volume changes */
	return 0;
}

static int gip_headset_op_audio_samples(struct gip_client *client,
					void *data, int len)
{
	struct gip_headset *headset = dev_get_drvdata(&client->dev);
	struct gip_headset_stream *stream = &headset->capture;
	struct snd_pcm_substream *sub = stream->substream;
	bool elapsed = false;
	unsigned long flags;

	if (!sub)
		return 0;

	snd_pcm_stream_lock_irqsave(sub, flags);

	if (sub->runtime && snd_pcm_running(sub))
		elapsed = gip_headset_copy_capture(stream, data, len);

	snd_pcm_stream_unlock_irqrestore(sub, flags);

	if (elapsed)
		snd_pcm_period_elapsed(sub);

	return 0;
}

static int gip_headset_probe(struct gip_client *client)
{
	struct gip_headset *headset;
	struct gip_info_element *fmts = client->audio_formats;
	int err;

	if (!fmts || !fmts->count)
		return -ENODEV;

	headset = devm_kzalloc(&client->dev, sizeof(*headset), GFP_KERNEL);
	if (!headset)
		return -ENOMEM;

	headset->client = client;

	INIT_DELAYED_WORK(&headset->config_work, gip_headset_config);
	INIT_DELAYED_WORK(&headset->power_on_work, gip_headset_power_on);
	INIT_WORK(&headset->register_work, gip_headset_register);

	hrtimer_init(&headset->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	headset->timer.function = gip_headset_send_samples;

	err = gip_enable_audio(client);
	if (err)
		return err;

	err = gip_init_audio_in(client);
	if (err) {
		gip_disable_audio(client);
		return err;
	}

	dev_set_drvdata(&client->dev, headset);

	/* delay to prevent response from being dropped */
	schedule_delayed_work(&headset->config_work, GIP_HS_CONFIG_DELAY);

	return 0;
}

static void gip_headset_remove(struct gip_client *client)
{
	struct gip_headset *headset = dev_get_drvdata(&client->dev);

	cancel_delayed_work_sync(&headset->config_work);
	cancel_delayed_work_sync(&headset->power_on_work);
	cancel_work_sync(&headset->register_work);
	hrtimer_cancel(&headset->timer);
	gip_disable_audio(client);

	if (headset->card) {
		snd_card_disconnect(headset->card);
		snd_card_free_when_closed(headset->card);
	}

	dev_set_drvdata(&client->dev, NULL);
}

static struct gip_driver gip_headset_driver = {
	.name = "xone-gip-headset",
	.class = "Windows.Xbox.Input.Headset",
	.ops = {
		.audio_ready = gip_headset_op_audio_ready,
		.audio_volume = gip_headset_op_audio_volume,
		.audio_samples = gip_headset_op_audio_samples,
	},
	.probe = gip_headset_probe,
	.remove = gip_headset_remove,
};
module_gip_driver(gip_headset_driver);

MODULE_ALIAS("gip:Windows.Xbox.Input.Headset");
MODULE_AUTHOR("Severin von Wnuck <severinvonw@outlook.de>");
MODULE_DESCRIPTION("xone GIP headset driver");
MODULE_VERSION("#VERSION#");
MODULE_LICENSE("GPL");
