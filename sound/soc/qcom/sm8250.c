// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020, Linaro Limited

#include <dt-bindings/sound/qcom,q6afe.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/sizes.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/soundwire/sdw.h>
#include <sound/jack.h>
#include <linux/input-event-codes.h>
#include "qdsp6/q6afe.h"
#include "common.h"
#include "usb_offload_utils.h"
#include "sdw.h"

#define MI2S_BCLK_RATE		1536000

struct sm8250_snd_data {
	bool stream_prepared[AFE_PORT_MAX];
	struct snd_soc_card *card;
	struct snd_soc_jack jack;
	struct snd_soc_jack usb_offload_jack;
	bool usb_offload_jack_setup;
	struct snd_soc_jack dp_jack;
	bool jack_setup;
	bool mius_rx_test;
	bool sweet_pri_clock_enabled;
};

static int sm8250_mius_rx_get(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(card);

	guard(mutex)(&card->pcm_mutex);
	ucontrol->value.integer.value[0] = data->mius_rx_test;
	return 0;
}

static int sm8250_mius_rx_put(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_runtime *rtd;
	bool enable = ucontrol->value.integer.value[0];

	guard(mutex)(&card->pcm_mutex);
	if (data->mius_rx_test == enable)
		return 0;
	for_each_card_rtds(card, rtd) {
		struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

		if (cpu_dai->id == PRIMARY_MI2S_RX && snd_soc_dai_active(cpu_dai))
			return -EBUSY;
	}
	data->mius_rx_test = enable;
	return 1;
}

static const struct snd_kcontrol_new sm8250_mius_controls[] = {
	SOC_SINGLE_BOOL_EXT("MIUS RX 96kHz Test Switch", 0,
			    sm8250_mius_rx_get, sm8250_mius_rx_put),
};

static int sm8250_snd_init(struct snd_soc_pcm_runtime *rtd)
{
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	switch (cpu_dai->id) {
	case DISPLAY_PORT_RX:
		return qcom_snd_dp_jack_setup(rtd, &data->dp_jack, 0);
	case USB_RX:
		return qcom_snd_usb_offload_jack_setup(rtd, &data->usb_offload_jack,
						       &data->usb_offload_jack_setup);
	default:
		return qcom_snd_wcd_jack_setup(rtd, &data->jack, &data->jack_setup);
	}
}

static void sm8250_snd_exit(struct snd_soc_pcm_runtime *rtd)
{
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	if (cpu_dai->id == USB_RX)
		qcom_snd_usb_offload_jack_remove(rtd,
						 &data->usb_offload_jack_setup);

}

static int sm8250_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				     struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);

	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;
	if (of_machine_is_compatible("xiaomi,sweet") &&
	    cpu_dai->id == TX_CODEC_DMA_TX_4) {
		rate->min = rate->max = 96000;
		channels->min = channels->max = 1;
	}
	if (data->mius_rx_test && cpu_dai->id == PRIMARY_MI2S_RX) {
		rate->min = rate->max = 96000;
		/* Sweet's OEM I2S backend is stereo S24_LE, unlike its mono FE. */
		channels->min = channels->max = 2;
		snd_mask_none(fmt);
		snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S24_LE);
		return 0;
	}
	snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);

	return 0;
}

static int sm8250_sweet_pri_clock_disable(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	int ret;

	if (!of_machine_is_compatible("xiaomi,sweet") ||
	    cpu_dai->id != PRIMARY_MI2S_RX || !data->sweet_pri_clock_enabled)
		return 0;

	ret = snd_soc_dai_set_sysclk(cpu_dai, Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT,
				     0, SNDRV_PCM_STREAM_PLAYBACK);
	if (!ret)
		data->sweet_pri_clock_enabled = false;
	else
		dev_err(rtd->dev, "Failed to release sweet PRI_MI2S clock: %d\n", ret);

	return ret;
}

static int sm8250_snd_startup(struct snd_pcm_substream *substream)
{
	unsigned int fmt = SND_SOC_DAIFMT_BP_FP;
	unsigned int codec_dai_fmt = SND_SOC_DAIFMT_BC_FC;
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai;
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	unsigned int bclk = MI2S_BCLK_RATE;
	unsigned int clock_id;
	bool sweet_primary = of_machine_is_compatible("xiaomi,sweet") &&
			     cpu_dai->id == PRIMARY_MI2S_RX;
	int i, ret;

	/* A failed release must not be hidden by acquiring a second clock vote. */
	if (sweet_primary && data->sweet_pri_clock_enabled)
		return -EBUSY;

	switch (cpu_dai->id) {
	case PRIMARY_MI2S_RX:
		clock_id = Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT;
		if (data->mius_rx_test)
			/* S24_LE occupies two 32-bit I2S slots at 96 kHz. */
			bclk = 96000 * 2 * 32;
		break;
	case SECONDARY_MI2S_RX:
		clock_id = Q6AFE_LPASS_CLK_ID_SEC_MI2S_IBIT;
		break;
	case TERTIARY_MI2S_RX:
		clock_id = Q6AFE_LPASS_CLK_ID_TER_MI2S_IBIT;
		break;
	case QUINARY_MI2S_RX:
		clock_id = Q6AFE_LPASS_CLK_ID_QUI_MI2S_IBIT;
		break;
	default:
		return qcom_snd_sdw_startup(substream);
	}

	ret = snd_soc_dai_set_sysclk(cpu_dai, clock_id, bclk,
				     SNDRV_PCM_STREAM_PLAYBACK);
	if (ret)
		return ret;
	if (sweet_primary)
		data->sweet_pri_clock_enabled = true;

	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret)
		goto err_clock;

	codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		ret = snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
		if (ret && ret != -ENOTSUPP)
			goto err_clock;
	}

	ret = qcom_snd_sdw_startup(substream);
	if (!ret)
		return 0;

err_clock:
	sm8250_sweet_pri_clock_disable(rtd);
	return ret;
}

static void sm8250_snd_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);

	qcom_snd_sdw_shutdown(substream);
	/* ASoC has already called the CPU DAI shutdown and stopped its AFE port. */
	sm8250_sweet_pri_clock_disable(rtd);
}

static int sm8250_snd_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);

	return qcom_snd_sdw_prepare(substream, &data->stream_prepared[cpu_dai->id]);
}

static int sm8250_snd_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	return qcom_snd_sdw_hw_free(substream, &data->stream_prepared[cpu_dai->id]);
}

static const struct snd_soc_ops sm8250_be_ops = {
	.startup = sm8250_snd_startup,
	.shutdown = sm8250_snd_shutdown,
	.hw_free = sm8250_snd_hw_free,
	.prepare = sm8250_snd_prepare,
};

/* Diagnostic hostless RX: local PCM bookkeeping, no Q6ASM/ADM session. */
static const struct snd_pcm_hardware sm8250_mius_hostless_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rates = SNDRV_PCM_RATE_96000,
	.rate_min = 96000,
	.rate_max = 96000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = SZ_64K,
	.period_bytes_min = 1024,
	.period_bytes_max = SZ_32K,
	.periods_min = 2,
	.periods_max = 64,
};

static int sm8250_mius_hostless_open(struct snd_soc_component *component,
				   struct snd_pcm_substream *substream)
{
	snd_soc_set_runtime_hwparams(substream, &sm8250_mius_hostless_hw);
	return 0;
}

static int sm8250_mius_hostless_construct(struct snd_soc_component *component,
					struct snd_soc_pcm_runtime *rtd)
{
	return snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_VMALLOC,
					     NULL, SZ_64K, SZ_64K);
}

static snd_pcm_uframes_t
sm8250_mius_hostless_pointer(struct snd_soc_component *component,
			    struct snd_pcm_substream *substream)
{
	/* No host DMA: a priming frame stays local and is never sent to DSP. */
	return 0;
}

static const struct snd_soc_component_driver sm8250_mius_hostless_component = {
	.name = "sweet-mius-hostless",
	.open = sm8250_mius_hostless_open,
	.pcm_new = sm8250_mius_hostless_construct,
	.pointer = sm8250_mius_hostless_pointer,
};

static int sm8250_mius_hostless_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);

	/* Explicit opt-in, and the shared PRI clock owner rejects overlap. */
	if (!data->mius_rx_test)
		return -EPERM;
	return sm8250_snd_startup(substream);
}

static int sm8250_mius_hostless_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	if (params_rate(params) != 96000 || params_channels(params) != 2 ||
	    params_format(params) != SNDRV_PCM_FORMAT_S24_LE)
		return -EINVAL;
	return 0;
}

static const struct snd_soc_ops sm8250_mius_hostless_ops = {
	.hw_params = sm8250_mius_hostless_hw_params,
	.startup = sm8250_mius_hostless_startup,
	.shutdown = sm8250_snd_shutdown,
	.prepare = sm8250_snd_prepare,
	.hw_free = sm8250_snd_hw_free,
};

static int sm8250_add_mius_hostless(struct snd_soc_card *card)
{
	struct snd_soc_dai_link_component *platform;
	struct snd_soc_dai_link *links, *primary = NULL, *link;
	int i, ret;

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm && link->id == PRIMARY_MI2S_RX) {
			primary = link;
			break;
		}
	}
	if (!primary)
		return -EINVAL;

	links = devm_kcalloc(card->dev, card->num_links + 1,
			     sizeof(*links), GFP_KERNEL);
	platform = devm_kzalloc(card->dev, sizeof(*platform), GFP_KERNEL);
	if (!links || !platform)
		return -ENOMEM;
	memcpy(links, card->dai_link, card->num_links * sizeof(*links));
	link = &links[card->num_links];
	/* Reuse the physical DAIs, not the routing/ASM platform. */
	link->cpus = primary->cpus;
	link->num_cpus = primary->num_cpus;
	link->codecs = primary->codecs;
	link->num_codecs = primary->num_codecs;
	platform->of_node = card->dev->of_node;
	link->platforms = platform;
	link->num_platforms = 1;
	link->name = "MIUS Hostless RX";
	link->stream_name = "MIUS Hostless RX";
	link->id = PRIMARY_MI2S_RX;
	link->playback_only = 1;
	link->nonatomic = 1;
	link->ignore_pmdown_time = 1;
	link->ops = &sm8250_mius_hostless_ops;
	ret = devm_snd_soc_register_component(card->dev,
					     &sm8250_mius_hostless_component,
					     NULL, 0);
	if (ret)
		return ret;
	card->dai_link = links;
	card->num_links++;
	return 0;
}

static void sm8250_add_be_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm == 1) {
			link->init = sm8250_snd_init;
			link->exit = sm8250_snd_exit;
			link->be_hw_params_fixup = sm8250_be_hw_params_fixup;
			link->ops = &sm8250_be_ops;
		}
	}
}

static int sm8250_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct sm8250_snd_data *data;
	struct device *dev = &pdev->dev;
	int ret;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card)
		return -ENOMEM;

	card->owner = THIS_MODULE;
	/* Allocate the private data */
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	card->dev = dev;
	dev_set_drvdata(dev, card);
	snd_soc_card_set_drvdata(card, data);
	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;

	card->driver_name = of_device_get_match_data(dev);
	if (of_machine_is_compatible("xiaomi,sweet")) {
		card->controls = sm8250_mius_controls;
		card->num_controls = ARRAY_SIZE(sm8250_mius_controls);
	}
	sm8250_add_be_ops(card);
	if (of_machine_is_compatible("xiaomi,sweet")) {
		ret = sm8250_add_mius_hostless(card);
		if (ret)
			return ret;
	}
	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id snd_sm8250_dt_match[] = {
	{ .compatible = "fairphone,fp4-sndcard", .data = "sm7225" },
	{ .compatible = "fairphone,fp5-sndcard", .data = "qcm6490" },
	{ .compatible = "qcom,qrb2210-sndcard", .data = "qcm2290" },
	{ .compatible = "qcom,qrb4210-rb2-sndcard", .data = "sm4250" },
	{ .compatible = "qcom,qrb5165-rb5-sndcard", .data = "sm8250" },
	{ .compatible = "qcom,sm8250-sndcard", .data = "sm8250" },
	{}
};

MODULE_DEVICE_TABLE(of, snd_sm8250_dt_match);

static struct platform_driver snd_sm8250_driver = {
	.probe  = sm8250_platform_probe,
	.driver = {
		.name = "snd-sm8250",
		.of_match_table = snd_sm8250_dt_match,
	},
};
module_platform_driver(snd_sm8250_driver);
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_DESCRIPTION("SM8250 ASoC Machine Driver");
MODULE_LICENSE("GPL");
