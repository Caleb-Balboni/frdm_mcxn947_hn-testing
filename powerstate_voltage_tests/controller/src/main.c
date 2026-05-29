#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

#define INA745_ADDR_0          0x40
#define INA745_ADDR_1          0x41

#define INA745_CONFIG          0x00
#define INA745_ADC_CONFIG      0x01
#define INA745_VBUS            0x05
#define INA745_DIETEMP         0x06
#define INA745_CURRENT         0x07
#define INA745_POWER           0x08
#define INA745_DIAG_ALRT       0x0B
#define INA745_MANUFACTURER_ID 0x3E

#define INA745_EXPECTED_MFG_ID 0x5449

/* Fixed conversion factors (EZShunt integrated 800uohm shunt; no SHUNT_CAL/
 * ADCRANGE on this part - these LSBs are fixed in silicon). */
#define INA745_VBUS_LSB_UV    3125.0   /* 3.125 mV/LSB */
#define INA745_DIETEMP_LSB_MC 125.0    /* 125 m degC/LSB */
#define INA745_CURRENT_LSB_UA 1200.0   /* 1.2 mA/LSB */
#define INA745_POWER_LSB_UW   240.0    /* 240 uW/LSB */

#define INA745_CONFIG_RST_BIT (1u << 15)

/* DIAG_ALRT (0x0B) flags */
#define INA745_DIAG_CNVRF     (1u << 1)   /* conversion ready (set when done) */
#define INA745_DIAG_MATHOF    (1u << 9)   /* arithmetic overflow -> I/P invalid */

/* ADC_CONFIG (0x01) value tuned for maximum POWER accuracy:
 *   MODE   = Fh  -> continuous temperature + current + bus voltage
 *                   (only continuous mode that produces a POWER result)
 *   VBUSCT = 7h  -> 4120 us bus-voltage conversion  (lowest bus noise)
 *   VSENCT = 7h  -> 4120 us shunt conversion         (lowest current noise)
 *   TCT    = 0h  ->   50 us temperature conversion   (temp unused for power,
 *                                                      minimized so it doesn't
 *                                                      bloat the cycle time)
 *   AVG    = 7h  -> 1024x hardware averaging          (strongest noise lever)
 * One averaged POWER update every (4120+4120+50)us * 1024 ~= 8.49 s. */
#define INA745_ADC_CONFIG_VAL 0xFFC7u

static const struct device *const bus = DEVICE_DT_GET(DT_NODELABEL(i3c1));

static int ina_read_u16(uint8_t addr, uint8_t reg, uint16_t *out)
{
	uint8_t buf[2];
	int rc = i2c_burst_read(bus, addr, reg, buf, sizeof(buf));
	if (rc < 0) {
		return rc;
	}
	*out = ((uint16_t)buf[0] << 8) | buf[1];
	return 0;
}

static int ina_read_u24(uint8_t addr, uint8_t reg, uint32_t *out)
{
	uint8_t buf[3];
	int rc = i2c_burst_read(bus, addr, reg, buf, sizeof(buf));
	if (rc < 0) {
		return rc;
	}
	*out = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
	return 0;
}

static int ina_write_u16(uint8_t addr, uint8_t reg, uint16_t val)
{
	uint8_t buf[3] = { reg, (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
	return i2c_write(bus, buf, sizeof(buf), addr);
}

static int ina_print_cfg(uint8_t addr);

static int ina_init(uint8_t addr)
{
	uint16_t mfg = 0;
	if (ina_read_u16(addr, INA745_MANUFACTURER_ID, &mfg) < 0) {
		printk("ina@0x%02X: no response on bus\n", addr);
		return -1;
	}
	if (mfg != INA745_EXPECTED_MFG_ID) {
		printk("ina@0x%02X: bad mfg id 0x%04X (expected 0x%04X)\n",
		       addr, mfg, INA745_EXPECTED_MFG_ID);
		return -1;
	}
	if (ina_write_u16(addr, INA745_CONFIG, INA745_CONFIG_RST_BIT) < 0) {
		printk("ina@0x%02X: soft reset failed\n", addr);
		return -1;
	}
	k_msleep(2);

	/* Apply the high-accuracy ADC configuration and read it back to confirm
	 * the write took (a silently dropped write would leave the part at its
	 * FB68h default and quietly cost accuracy). */
	if (ina_write_u16(addr, INA745_ADC_CONFIG, INA745_ADC_CONFIG_VAL) < 0) {
		printk("ina@0x%02X: adc_config write failed\n", addr);
		return -1;
	}
	uint16_t adc_cfg = 0;
	if (ina_read_u16(addr, INA745_ADC_CONFIG, &adc_cfg) < 0) {
		printk("ina@0x%02X: adc_config readback failed\n", addr);
		return -1;
	}
	if (adc_cfg != INA745_ADC_CONFIG_VAL) {
		printk("ina@0x%02X: adc_config mismatch (got 0x%04X want 0x%04X)\n",
		       addr, adc_cfg, INA745_ADC_CONFIG_VAL);
		return -1;
	}

	printk("ina@0x%02X: ready (mfg=0x%04X)\n", addr, mfg);
	ina_print_cfg(addr);
	return 0;
}

// One raw sample of all four channels. Returns 0 on success, <0 on bus error.
static int ina_sample_raw(uint8_t addr, int16_t *vbus_s, int16_t *curr_s,
			  int16_t *temp_s, uint32_t *pwr_raw)
{
	uint16_t vbus_raw = 0, curr_raw = 0, temp_raw = 0;
	uint32_t pwr = 0;
	if (ina_read_u16(addr, INA745_VBUS,    &vbus_raw) < 0 ||
	    ina_read_u16(addr, INA745_CURRENT, &curr_raw) < 0 ||
	    ina_read_u16(addr, INA745_DIETEMP, &temp_raw) < 0 ||
	    ina_read_u24(addr, INA745_POWER,   &pwr)      < 0) {
		return -1;
	}
	*vbus_s  = (int16_t)vbus_raw;
	*curr_s  = (int16_t)curr_raw;
	*temp_s  = (int16_t)temp_raw >> 4;
	*pwr_raw = pwr;
	return 0;
}

/* Conversion-time field codes 0h..7h -> microseconds (shared by VBUSCT,
 * VSENCT, TCT) and AVG field codes 0h..7h -> averaging count. */
static const uint16_t ina_conv_time_us[8] = {
	50, 84, 150, 280, 540, 1052, 2074, 4120,
};
static const uint16_t ina_avg_count[8] = {
	1, 4, 16, 64, 128, 256, 512, 1024,
};

static int ina_print_cfg(uint8_t addr)
{
	uint16_t cfg_reg = 0;
	int rc = ina_read_u16(addr, INA745_ADC_CONFIG, &cfg_reg);
	if (rc < 0) {
		printk("ina@0x%02X: failed to read ADC_CONFIG\n", addr);
		return rc;
	}

	uint8_t mode   = (cfg_reg >> 12) & 0xF;
	uint8_t vbusct = (cfg_reg >> 9)  & 0x7;
	uint8_t vsenct = (cfg_reg >> 6)  & 0x7;
	uint8_t tct    = (cfg_reg >> 3)  & 0x7;
	uint8_t avg    =  cfg_reg        & 0x7;

	/* Per-conversion cycle covers whichever channels MODE enables; for the
	 * accuracy-relevant current+bus path that is VBUSCT + VSENCT (+ TCT,
	 * since the only continuous current+bus mode also runs temperature). */
	uint32_t cycle_us = (uint32_t)ina_conv_time_us[vbusct] +
			    ina_conv_time_us[vsenct] +
			    ina_conv_time_us[tct];
	uint32_t update_us = cycle_us * ina_avg_count[avg];

	printk("ina@0x%02X: ADC_CONFIG=0x%04X MODE=0x%X VBUSCT=%uus VSENCT=%uus "
	       "TCT=%uus AVG=%ux -> update ~%u.%03u s\n",
	       addr, cfg_reg, mode,
	       ina_conv_time_us[vbusct], ina_conv_time_us[vsenct],
	       ina_conv_time_us[tct], ina_avg_count[avg],
	       update_us / 1000000u, (update_us / 1000u) % 1000u);
	return 0;
}

// Report one fresh, hardware-averaged sample if the device has completed a new
// conversion since we last read it. The chip already averages 1024 conversions
// internally, so the right thing is to consume each result exactly once rather
// than re-reading the same latched value thousands of times (which adds no
// noise reduction). Returns:
//   0  printed a fresh sample
//   1  no new conversion ready yet (nothing printed, not an error)
//  <0  bus error
static int ina_poll_report(uint8_t addr, uint32_t ts_ms)
{
	uint16_t diag = 0;
	if (ina_read_u16(addr, INA745_DIAG_ALRT, &diag) < 0) {
		return -1;
	}
	if (!(diag & INA745_DIAG_CNVRF)) {
		return 1;   /* averaging window not finished yet */
	}

	int16_t vbus_s, curr_s, temp_s;
	uint32_t pwr_raw;
	if (ina_sample_raw(addr, &vbus_s, &curr_s, &temp_s, &pwr_raw) < 0) {
		return -1;
	}

	double v = (vbus_s * INA745_VBUS_LSB_UV)    / 1000000.0;
	double a = (curr_s * INA745_CURRENT_LSB_UA) / 1000000.0;
	double w = (pwr_raw * INA745_POWER_LSB_UW)  / 1000000.0;
	double t = (temp_s * INA745_DIETEMP_LSB_MC) / 1000.0;

	/* MATHOF means the internal current/power math overflowed -> those two
	 * values are not trustworthy. Flag it rather than logging bad data. */
	const char *flag = (diag & INA745_DIAG_MATHOF) ? "  [MATHOF: I/P INVALID]" : "";

	printk("[%9u ms] 0x%02X  V=%.4f V  I=%.4f A  P=%.4f W  T=%.2f C%s\n",
	       ts_ms, addr, v, a, w, t, flag);
	return 0;
}

int main(void)
{
	printk("\n========================================\n");
	printk("  POWERSTATE_TEST controller (frdm_mcxn947)\n");
	printk("  Reading INA745 over i3c1@i2c-compat\n");
	printk("========================================\n");

	if (!device_is_ready(bus)) {
		printk("i3c1 not ready\n");
		return -1;
	}

	bool have0 = (ina_init(INA745_ADDR_0) == 0);
	bool have1 = (ina_init(INA745_ADDR_1) == 0);
	if (!have0 && !have1) {
		printk("no INA745 devices responded; will keep polling 0x40 in case the host comes up.\n");
	}

	while (1) {
		uint32_t ts = k_uptime_get_32();
		if (have0) {
			if (ina_poll_report(INA745_ADDR_0, ts) < 0) {
				have0 = false;   // lost device; try to re-init next pass
			}
		} else {
			have0 = (ina_init(INA745_ADDR_0) == 0);
		}
		if (have1) {
			if (ina_poll_report(INA745_ADDR_1, ts) < 0) {
				have1 = false;
			}
		}

		/* Both devices average for ~8.5 s per result, so polling fast buys
		 * nothing. Sleep keeps the I2C bus and CPU idle between checks while
		 * still catching each new conversion promptly. */
		k_msleep(50);
	}
	return 0;
}
