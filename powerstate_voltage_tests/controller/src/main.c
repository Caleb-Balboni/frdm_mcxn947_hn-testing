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

#define INA745_VBUS_LSB_UV    3125.0f
#define INA745_DIETEMP_LSB_MC 125.0f
#define INA745_CURRENT_LSB_UA 1200.0f
#define INA745_POWER_LSB_UW   240.0f

#define INA745_CONFIG_RST_BIT (1u << 15)

#define SAMPLES_PER_AVG       25000

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
	printk("ina@0x%02X: ready (mfg=0x%04X)\n", addr, mfg);
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

// Collect SAMPLES_PER_AVG readings from the INA745 at addr, then print the
// mean of each channel. Failed reads are skipped; if every read fails the
// function prints a "no response" line and returns < 0.
static int ina_sample_avg(uint8_t addr, uint32_t ts_ms)
{
	double v_sum = 0.0, a_sum = 0.0, w_sum = 0.0, t_sum = 0.0;
	uint32_t ok = 0;

	for (uint32_t i = 0; i < SAMPLES_PER_AVG; i++) {
		int16_t vbus_s, curr_s, temp_s;
		uint32_t pwr_raw;
		if (ina_sample_raw(addr, &vbus_s, &curr_s, &temp_s, &pwr_raw) < 0) {
			continue;
		}
		v_sum += (vbus_s * INA745_VBUS_LSB_UV)    / 1000000.0;
		a_sum += (curr_s * INA745_CURRENT_LSB_UA) / 1000000.0;
		w_sum += (pwr_raw * INA745_POWER_LSB_UW)  / 1000000.0;
		t_sum += (temp_s * INA745_DIETEMP_LSB_MC) / 1000.0;
		ok++;
	}

	if (ok == 0) {
		printk("[%9u ms] 0x%02X  no response (0/%u samples)\n",
		       ts_ms, addr, SAMPLES_PER_AVG);
		return -1;
	}

	double n = (double)ok;
	printk("[%9u ms] 0x%02X  avg of %u/%u  V=%.4f V  I=%.4f A  P=%.4f W  T=%.2f C\n",
	       ts_ms, addr, ok, SAMPLES_PER_AVG,
	       v_sum / n, a_sum / n, w_sum / n, t_sum / n);
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
			if (ina_sample_avg(INA745_ADDR_0, ts) < 0) {
				have0 = false;   // lost device; try to re-init next pass
			}
		} else {
			have0 = (ina_init(INA745_ADDR_0) == 0);
		}
		if (have1) {
			if (ina_sample_avg(INA745_ADDR_1, ts) < 0) {
				have1 = false;
			}
		}
	}
	return 0;
}
