// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2024, Danila Tikhonov <danila@jiaxyga.com>
 */

#include <linux/devm-helpers.h>
#include <linux/iio/consumer.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>

/* BATT offsets */
#define QG_SUBTYPE_REG			0x05
#define QG_ADC_IBAT_5A			0x03
#define QG_ADC_IBAT_10A			0x04
#define QG_STATUS2_REG			0x09
#define QG_GOOD_OCV_BIT			BIT(1)
#define QG_S3_GOOD_OCV_V_DATA0_REG		0x74 /* 2-byte 0x74-0x75 */
#define QG_S2_NORMAL_AVG_V_DATA0_REG	0x80 /* 2-byte 0x80-0x81 */
#define QG_S2_NORMAL_AVG_I_DATA0_REG	0x82 /* 2-byte 0x82-0x83 */
#define QG_LAST_ADC_V_DATA0_REG		0xc0 /* 2-byte 0xc0-0xc1 */
#define QG_LAST_ADC_I_DATA0_REG		0xc2 /* 2-byte 0xc2-0xc3 */

#define QG_STATUS3_REG			0x0a
#define QG_DATA_CTL1_REG			0x41
#define QG_MASTER_HOLD_BIT		BIT(0)
#define QG_MEAS_CTL2_REG			0x51
#define QG_ACCUM_V_REG			0x88
#define QG_V_FIFO_REG			0x90
#define QG_I_FIFO_REG			0xa0
#define QG_FIFO_RESET			0x8000
#define QG_MAX_FIFO			8
#define QG_CAPTURE_PERIOD_MS		300000

/* SRAM offsets */
#define QG_SDAM_LEARNED_CAPACITY_OFFSET	0x68 /* 2-byte 0x68-0x69 */

struct qcom_qg_chip {
	struct device *dev;
	struct regmap *regmap;
	unsigned int base;
	unsigned int current_factor;

	struct iio_channel *batt_therm_chan;

	struct nvmem_device *sdam;

	struct power_supply *batt_psy;
	struct power_supply_battery_info *batt_info;

	struct mutex lock;
	struct delayed_work capture_work;
	bool ready;
	bool estimator_enabled;
	bool anchored;
	bool data_gap;
	s64 charge_uams;
	s64 full_uams;
	ktime_t last_capture;
	ktime_t ocv_time;
	int ocv_uv;
	u64 captured_ms;
	u64 captures;
	u64 gaps;
	s64 last_elapsed_ms;
	s64 last_delta_uams;
	u32 last_duration_ms;
	u32 sample_interval_ms;
	u32 fifo_count;
	u32 accum_count;
};

static bool fifo_estimator = true;
module_param(fifo_estimator, bool, 0444);
MODULE_PARM_DESC(fifo_estimator, "Use FIFO charge estimation when an OCV table is supplied");

struct qcom_qg_snapshot {
	s64 charge_uams;
	u32 duration_ms;
	u32 full_fifo_ms;
	u32 interval_ms;
	u32 fifo_count;
	u32 accum_count;
};

static int qcom_qg_get_current(struct qcom_qg_chip *chip, u8 offset, int *val)
{
	s16 temp;
	u8 readval[2];
	int ret;

	ret = regmap_bulk_read(chip->regmap, chip->base + offset, readval, 2);
	if (ret) {
		dev_err(chip->dev, "Failed to read current: %d\n", ret);
		return ret;
	}

	temp = (s16)(readval[1] << 8 | readval[0]);
	if (temp == (s16)QG_FIFO_RESET)
		return -ENODATA;
	*val = div_s64((s64)temp * chip->current_factor, 1000);

	/*
	 * PSY API expects charging batteries to report a positive current, which is inverted
	 * to what the PMIC reports.
	 */
	*val = -*val;

	return 0;
}

static int qcom_qg_get_voltage(struct qcom_qg_chip *chip, u8 offset, int *val)
{
	int ret, temp;
	u8 readval[2];

	ret = regmap_bulk_read(chip->regmap, chip->base + offset, readval, 2);
	if (ret) {
		dev_err(chip->dev, "Failed to read voltage: %d\n", ret);
		return ret;
	}

	temp = readval[1] << 8 | readval[0];
	if (temp == QG_FIFO_RESET)
		return -ENODATA;
	*val = div_u64((u64)temp * 194637, 1000);

	return 0;
}

static int qcom_qg_read_ocv(struct qcom_qg_chip *chip, int *val)
{
	unsigned int status;
	int ret;

	/* SDAM may contain an OCV left by a previous Android boot. */
	ret = regmap_read(chip->regmap, chip->base + QG_STATUS2_REG, &status);
	if (ret)
		return ret;
	if (!(status & QG_GOOD_OCV_BIT))
		return -ENODATA;

	return qcom_qg_get_voltage(chip, QG_S3_GOOD_OCV_V_DATA0_REG, val);
}

/* Hold and release also start a new FIFO epoch, preventing double counting. */
static int qcom_qg_capture(struct qcom_qg_chip *chip,
			   struct qcom_qg_snapshot *snapshot)
{
	u8 config[2], v_fifo[QG_MAX_FIFO * 2], i_fifo[QG_MAX_FIFO * 2];
	u8 accum[7];
	unsigned int count, i;
	u32 samples, length, accum_count, raw_v;
	s64 raw_i = 0;
	int ret, release_ret;

	ret = regmap_update_bits(chip->regmap, chip->base + QG_DATA_CTL1_REG,
				 QG_MASTER_HOLD_BIT, 0);
	if (ret)
		goto release;
	ret = regmap_update_bits(chip->regmap, chip->base + QG_DATA_CTL1_REG,
				 QG_MASTER_HOLD_BIT, QG_MASTER_HOLD_BIT);
	if (ret)
		goto release;

	ret = regmap_bulk_read(chip->regmap, chip->base + QG_MEAS_CTL2_REG,
			       config, sizeof(config));
	if (ret)
		goto release;
	length = ((config[0] >> 3) & 7) + 1;
	samples = 1U << ((config[0] & 7) + 1);
	/* PM6150's QG clock is 32764 Hz, not the nominal 32000 Hz. */
	snapshot->interval_ms = DIV_ROUND_CLOSEST(config[1] * 10 * 32000, 32764);
	if (!snapshot->interval_ms) {
		ret = -EINVAL;
		goto release;
	}
	snapshot->full_fifo_ms = length * samples * snapshot->interval_ms;

	ret = regmap_read(chip->regmap, chip->base + QG_STATUS3_REG, &count);
	if (ret)
		goto release;
	count &= 0xf;
	if (count > length) {
		ret = -EINVAL;
		goto release;
	}
	ret = regmap_bulk_read(chip->regmap, chip->base + QG_V_FIFO_REG,
			       v_fifo, sizeof(v_fifo));
	if (ret)
		goto release;
	ret = regmap_bulk_read(chip->regmap, chip->base + QG_I_FIFO_REG,
			       i_fifo, sizeof(i_fifo));
	if (ret)
		goto release;
	for (i = 0; i < count; i++) {
		if (get_unaligned_le16(v_fifo + 2 * i) == QG_FIFO_RESET ||
		    get_unaligned_le16(i_fifo + 2 * i) == QG_FIFO_RESET) {
			ret = -ENODATA;
			goto release;
		}
		raw_i += (s64)(s16)get_unaligned_le16(i_fifo + 2 * i) * samples;
	}

	ret = regmap_bulk_read(chip->regmap, chip->base + QG_ACCUM_V_REG,
			       accum, sizeof(accum));
	if (ret)
		goto release;
	accum_count = accum[6];
	if (accum_count >= samples) {
		ret = -EINVAL;
		goto release;
	}
	if (accum_count) {
		raw_v = accum[0] | accum[1] << 8 | accum[2] << 16;
		if (!raw_v || raw_v / accum_count == QG_FIFO_RESET) {
			ret = -ENODATA;
			goto release;
		}
		raw_i += sign_extend32(accum[3] | accum[4] << 8 | accum[5] << 16, 23);
	}
	snapshot->charge_uams = -div_s64(raw_i * chip->current_factor *
					 snapshot->interval_ms, 1000);
	snapshot->duration_ms = (count * samples + accum_count) * snapshot->interval_ms;
	snapshot->fifo_count = count;
	snapshot->accum_count = accum_count;
release:
	/* Release even if reading failed: leaving master held stops measurements. */
	release_ret = regmap_update_bits(chip->regmap, chip->base + QG_DATA_CTL1_REG,
					QG_MASTER_HOLD_BIT, 0);
	if (release_ret)
		return release_ret;
	return ret;
}

static int qcom_qg_ocv_capacity(struct qcom_qg_chip *chip, int voltage)
{
	int temp, ret;

	ret = iio_read_channel_processed(chip->batt_therm_chan, &temp);
	if (ret < 0)
		return ret;
	return power_supply_batinfo_ocv2cap(chip->batt_info, voltage, temp / 1000);
}

static void qcom_qg_anchor(struct qcom_qg_chip *chip, int percent)
{
	chip->charge_uams = div_s64(chip->full_uams * clamp(percent, 0, 100), 100);
	chip->anchored = true;
	chip->data_gap = false;
}

static void qcom_qg_capture_work(struct work_struct *work)
{
	struct qcom_qg_chip *chip = container_of(to_delayed_work(work),
						struct qcom_qg_chip, capture_work);
	struct qcom_qg_snapshot snapshot = {};
	union power_supply_propval status;
	ktime_t now = ktime_get_boottime();
	s64 elapsed, target, correction;
	int ret, ocv_ret, voltage, batt_current, percent;
	unsigned long delay = msecs_to_jiffies(QG_CAPTURE_PERIOD_MS);

	mutex_lock(&chip->lock);
	elapsed = ktime_ms_delta(now, chip->last_capture);
	ocv_ret = qcom_qg_read_ocv(chip, &voltage);
	ret = qcom_qg_capture(chip, &snapshot);
	chip->last_capture = ktime_get_boottime();
	if (ret || snapshot.duration_ms > elapsed + elapsed / 10 + 1000) {
		chip->data_gap = true;
		chip->gaps++;
		dev_warn_ratelimited(chip->dev, "Rejected QG capture: %d, sampled %u elapsed %lld ms\n",
				     ret, snapshot.duration_ms, elapsed);
		if (ret)
			delay = msecs_to_jiffies(10000);
	} else {
		/* A FIFO can wrap while the CPU sleeps; do not invent the lost charge. */
		if (elapsed >= snapshot.full_fifo_ms ||
		    elapsed > snapshot.duration_ms + snapshot.interval_ms * 20 + 1000) {
			chip->data_gap = true;
			chip->gaps++;
		}
		chip->last_elapsed_ms = elapsed;
		chip->last_duration_ms = snapshot.duration_ms;
		chip->last_delta_uams = snapshot.charge_uams;
		chip->charge_uams = clamp(chip->charge_uams + snapshot.charge_uams,
					 0LL, chip->full_uams);
		chip->captured_ms += snapshot.duration_ms;
		chip->captures++;
		chip->sample_interval_ms = snapshot.interval_ms;
		chip->fifo_count = snapshot.fifo_count;
		chip->accum_count = snapshot.accum_count;
	}

	if (!ocv_ret && voltage > 2000000 && voltage < 5000000) {
		/* Consume the sticky event; equal subsequent OCVs are new measurements. */
		ret = regmap_write(chip->regmap, chip->base + QG_STATUS2_REG, 0);
		if (!ret) {
			chip->ocv_uv = voltage;
			chip->ocv_time = now;
			percent = qcom_qg_ocv_capacity(chip, voltage);
			if (percent >= 0) {
				if (!chip->anchored || chip->data_gap) {
					qcom_qg_anchor(chip, percent);
				} else {
					/* Limit drift correction after the first anchor. */
					target = div_s64(chip->full_uams * percent, 100);
					correction = chip->full_uams / 1000;
					chip->charge_uams += clamp(target - chip->charge_uams,
								  -correction, correction);
				}
			}
		}
	}

	ret = power_supply_get_property_from_supplier(chip->batt_psy,
						     POWER_SUPPLY_PROP_STATUS, &status);
	if (!ret && status.intval == POWER_SUPPLY_STATUS_FULL &&
	    !qcom_qg_get_voltage(chip, QG_LAST_ADC_V_DATA0_REG, &voltage) &&
	    !qcom_qg_get_current(chip, QG_LAST_ADC_I_DATA0_REG, &batt_current) &&
	    voltage >= chip->batt_info->voltage_max_design_uv - 50000 &&
	    abs(batt_current) < 100000)
		qcom_qg_anchor(chip, 100);
	mutex_unlock(&chip->lock);

	power_supply_changed(chip->batt_psy);
	queue_delayed_work(system_freezable_wq, &chip->capture_work, delay);
}

static ssize_t soc_state_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct qcom_qg_chip *chip = dev_get_drvdata(dev);
	ssize_t ret;

	if (!chip->estimator_enabled)
		return sysfs_emit(buf, "enabled=0\n");
	mutex_lock(&chip->lock);
	ret = sysfs_emit(buf,
			"enabled=1 anchored=%u gap=%u captures=%llu gaps=%llu sampled_ms=%llu interval_ms=%u fifo=%u accum=%u charge_uah=%lld ocv_uv=%d elapsed_ms=%lld duration_ms=%u delta_uah=%lld\n",
			chip->anchored, chip->data_gap, chip->captures, chip->gaps,
			chip->captured_ms, chip->sample_interval_ms, chip->fifo_count,
			chip->accum_count, div_s64(chip->charge_uams, 3600000), chip->ocv_uv,
			chip->last_elapsed_ms, chip->last_duration_ms,
			div_s64(chip->last_delta_uams, 3600000));
	mutex_unlock(&chip->lock);
	return ret;
}
static DEVICE_ATTR_RO(soc_state);

static struct attribute *qcom_qg_attrs[] = {
	&dev_attr_soc_state.attr,
	NULL,
};
ATTRIBUTE_GROUPS(qcom_qg);

/* Preserve the legacy voltage estimate for boards without an OCV profile. */
static int qcom_qg_get_capacity(struct qcom_qg_chip *chip, int *val)
{
	int ret, voltage_now;
	int voltage_min = chip->batt_info->voltage_min_design_uv;
	int voltage_max = chip->batt_info->voltage_max_design_uv;

	if (chip->estimator_enabled) {
		mutex_lock(&chip->lock);
		*val = div64_s64(chip->charge_uams * 100 + chip->full_uams / 2,
				chip->full_uams);
		mutex_unlock(&chip->lock);
		return 0;
	}

	ret = qcom_qg_get_voltage(chip,
				QG_S2_NORMAL_AVG_V_DATA0_REG, &voltage_now);
	if (ret) {
		dev_err(chip->dev, "Failed to get current voltage: %d\n", ret);
		return ret;
	}

	if (voltage_now <= voltage_min)
		*val = 0;
	else if (voltage_now >= voltage_max)
		*val = 100;
	else
		*val = (((voltage_now - voltage_min) * 100) /
						(voltage_max - voltage_min));

	return 0;
}

static enum power_supply_property qcom_qg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static int qcom_qg_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct qcom_qg_chip *chip = power_supply_get_drvdata(psy);
	u8 learned_capacity[2];
	int ret;

	/* Pair with probe's release: publish fully initialized estimation state. */
	if (!smp_load_acquire(&chip->ready))
		return -EAGAIN;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = power_supply_get_property_from_supplier(psy, psp, val);
		if (ret == -ENODEV)
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		else if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = chip->batt_info->voltage_max_design_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = chip->batt_info->voltage_min_design_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = qcom_qg_get_voltage(chip,
				QG_LAST_ADC_V_DATA0_REG, &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = qcom_qg_get_voltage(chip,
				QG_S2_NORMAL_AVG_V_DATA0_REG, &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		if (!chip->estimator_enabled) {
			ret = qcom_qg_read_ocv(chip, &val->intval);
			if (ret)
				return ret;
		} else {
			mutex_lock(&chip->lock);
			ret = chip->ocv_uv &&
			      ktime_ms_delta(ktime_get_boottime(), chip->ocv_time) <=
			      QG_CAPTURE_PERIOD_MS ? 0 : -ENODATA;
			val->intval = chip->ocv_uv;
			mutex_unlock(&chip->lock);
			if (ret)
				return ret;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = qcom_qg_get_current(chip,
				QG_LAST_ADC_I_DATA0_REG, &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = qcom_qg_get_current(chip,
				QG_S2_NORMAL_AVG_I_DATA0_REG, &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (chip->batt_info->charge_full_design_uah <= 0)
			return -ENODATA;
		val->intval = chip->batt_info->charge_full_design_uah;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = nvmem_device_read(chip->sdam,
				QG_SDAM_LEARNED_CAPACITY_OFFSET,
				sizeof(learned_capacity), learned_capacity);
		if (ret < 0)
			return ret;
		if (ret != sizeof(learned_capacity))
			return -EIO;
		val->intval = (learned_capacity[0] |
			      learned_capacity[1] << 8) * 1000; /* mAh to uAh */
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (!chip->estimator_enabled)
			return -ENODATA;
		mutex_lock(&chip->lock);
		val->intval = div_s64(chip->charge_uams, 3600000);
		mutex_unlock(&chip->lock);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		mutex_lock(&chip->lock);
		val->intval = chip->estimator_enabled && chip->anchored && !chip->data_gap ?
			POWER_SUPPLY_CAPACITY_LEVEL_NORMAL : POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
		mutex_unlock(&chip->lock);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = qcom_qg_get_capacity(chip, &val->intval);
		if (ret)
			return ret;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = iio_read_channel_processed
					(chip->batt_therm_chan, &val->intval);
		if (ret < 0)
			return ret;
		val->intval /= 100; /* 1/1000 °C (millidegC) to 1/10 °C */
		break;
	default:
		dev_err(chip->dev, "invalid property: %d\n", psp);
		return -EINVAL;
	}
	return 0;
}

static struct power_supply_desc batt_psy_desc = {
	.name = "qcom_qg",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = qcom_qg_props,
	.num_properties = ARRAY_SIZE(qcom_qg_props),
	.get_property = qcom_qg_get_property,
	.external_power_changed = power_supply_changed,
};

static int qcom_qg_probe(struct platform_device *pdev)
{
	struct qcom_qg_chip *chip;
	struct power_supply_config psy_cfg = {};
	unsigned int subtype;
	struct qcom_qg_snapshot snapshot = {};
	int ret, voltage, percent;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	mutex_init(&chip->lock);
	platform_set_drvdata(pdev, chip);

	/* Regmap */
	chip->regmap = dev_get_regmap(chip->dev->parent, NULL);
	if (!chip->regmap)
		return dev_err_probe(chip->dev, -ENODEV,
				     "Failed to locate the regmap\n");

	/* Get base address */
	ret = device_property_read_u32(chip->dev, "reg", &chip->base);
	if (ret < 0)
		return dev_err_probe(chip->dev, ret,
				     "Couldn't read base address\n");

	ret = regmap_read(chip->regmap, chip->base + QG_SUBTYPE_REG, &subtype);
	if (ret)
		return dev_err_probe(chip->dev, ret, "Couldn't read QGauge subtype\n");
	switch (subtype) {
	case QG_ADC_IBAT_5A:
		chip->current_factor = 152588;
		break;
	case QG_ADC_IBAT_10A:
		chip->current_factor = 305176;
		break;
	default:
		return dev_err_probe(chip->dev, -ENODEV,
				     "Unsupported QGauge subtype %#x\n", subtype);
	}

	/* ADC for thermal channel */
	chip->batt_therm_chan = devm_iio_channel_get(chip->dev, "batt-therm");
	if (IS_ERR(chip->batt_therm_chan))
		return dev_err_probe(chip->dev, PTR_ERR(chip->batt_therm_chan),
				     "Couldn't get batt-therm IIO channel\n");

	/* NVMEM for SDAM access */
	chip->sdam = devm_nvmem_device_get(chip->dev, NULL);
	if (IS_ERR(chip->sdam))
		return dev_err_probe(chip->dev, PTR_ERR(chip->sdam),
				     "Couldn't get SDAM nvmem device\n");

	psy_cfg.drv_data = chip;
	psy_cfg.fwnode = dev_fwnode(chip->dev);

	/* Power supply */
	chip->batt_psy =
		devm_power_supply_register(chip->dev, &batt_psy_desc, &psy_cfg);
	if (IS_ERR(chip->batt_psy))
		return dev_err_probe(chip->dev, PTR_ERR(chip->batt_psy),
				     "Failed to register power supply\n");

	/* Battery info */
	ret = power_supply_get_battery_info(chip->batt_psy, &chip->batt_info);
	if (ret)
		return dev_err_probe(chip->dev, ret,
				     "Failed to get battery info\n");

	if (fifo_estimator && chip->batt_info->ocv_table[0] &&
	    chip->batt_info->charge_full_design_uah > 0) {
		/* A terminal-voltage seed is provisional until rested OCV or full charge. */
		ret = qcom_qg_get_voltage(chip, QG_S2_NORMAL_AVG_V_DATA0_REG, &voltage);
		if (ret)
			return ret;
		percent = qcom_qg_ocv_capacity(chip, voltage);
		if (percent < 0)
			return percent;
		chip->full_uams = (s64)chip->batt_info->charge_full_design_uah * 3600000;
		chip->charge_uams = div_s64(chip->full_uams * percent, 100);
		/* Discard pre-probe data and any sticky event from a previous OS. */
		ret = regmap_write(chip->regmap, chip->base + QG_STATUS2_REG, 0);
		if (ret)
			return ret;
		ret = qcom_qg_capture(chip, &snapshot);
		if (ret)
			return dev_err_probe(chip->dev, ret, "Couldn't start QG capture epoch\n");
		chip->last_capture = ktime_get_boottime();
		ret = devm_delayed_work_autocancel(chip->dev, &chip->capture_work,
						 qcom_qg_capture_work);
		if (ret)
			return ret;
		chip->estimator_enabled = true;
		queue_delayed_work(system_freezable_wq, &chip->capture_work,
				   msecs_to_jiffies(QG_CAPTURE_PERIOD_MS));
	}
	/* Property readers can run as soon as the power supply is registered. */
	smp_store_release(&chip->ready, true);
	power_supply_changed(chip->batt_psy);

	return 0;
}

static const struct of_device_id qcom_qg_of_match[] = {
	{ .compatible = "qcom,pm6150-qg", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, qcom_qg_of_match);

static struct platform_driver qcom_qg_driver = {
	.driver = {
		.name = "qcom,qcom_qg",
		.of_match_table = qcom_qg_of_match,
		.dev_groups = qcom_qg_groups,
	},
	.probe = qcom_qg_probe,
};

module_platform_driver(qcom_qg_driver);

MODULE_AUTHOR("Danila Tikhonov <danila@jiaxyga.com>");
MODULE_DESCRIPTION("Qualcomm PMIC QGauge (QG) driver");
MODULE_LICENSE("GPL");
