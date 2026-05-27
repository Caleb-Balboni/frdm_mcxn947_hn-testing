#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

/* Direct handle to the I3C controller, used via its I2C-API shim. No
 * bridge layer in between — this is a clean slave-side bus scan. */
static const struct device *const i3c_dev =
	DEVICE_DT_GET(DT_NODELABEL(i3c0));

/* Probe with a 0-length WRITE (matches SMBus-quick semantics that
 * i2cdetect uses for most addresses). Controller emits START + addr +
 * W and should NACK if no device is present. */
static int probe_write_quick(uint8_t addr)
{
	uint8_t dummy = 0;
	struct i2c_msg msg = {
		.buf   = &dummy,
		.len   = 0,
		.flags = I2C_MSG_WRITE | I2C_MSG_STOP,
	};
	return i2c_transfer(i3c_dev, &msg, 1, addr);
}

/* Probe with a 1-byte READ. Controller emits START + addr + R, samples
 * ACK on the 9th clock, reads (or NACKs) the byte. Most devices ACK
 * their address on a blind read; missing devices NACK cleanly. */
static int probe_read_byte(uint8_t addr)
{
	uint8_t dummy = 0;
	struct i2c_msg msg = {
		.buf   = &dummy,
		.len   = 1,
		.flags = I2C_MSG_READ | I2C_MSG_STOP,
	};
	return i2c_transfer(i3c_dev, &msg, 1, addr);
}

static void run_scan(const char *label, int (*probe)(uint8_t))
{
	printk("\n--- Scan via %s ---\n", label);
	printk("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");

	int found = 0;
	for (uint8_t row = 0; row < 8; row++) {
		printk("%02x:", row * 16);
		for (uint8_t col = 0; col < 16; col++) {
			uint8_t addr = (row * 16) + col;

			if (addr < 0x08 || addr > 0x77) {
				printk("   ");
				continue;
			}

			int ret = probe(addr);
			if (ret == 0) {
				printk(" %02x", addr);
				found++;
			} else {
				printk(" --");
			}
			k_msleep(2);
		}
		printk("\n");
	}

	printk("Result: %d address(es) ACKed via %s.\n", found, label);
}

int main(void)
{
	printk("\n");
	printk("==========================================\n");
	printk("  i3c0 direct I2C scan (no bridge)\n");
	printk("  Comparing 0-byte WRITE vs 1-byte READ\n");
	printk("==========================================\n");

	if (!device_is_ready(i3c_dev)) {
		printk("i3c0 device not ready\n");
		return -1;
	}
	printk("i3c0 ready: %s\n", i3c_dev->name);
	printk("Probing legacy I2C addresses 0x08..0x77...\n");

	/* SMBus-quick equivalent: 0-byte write. This is what the
	 * Linux i2cdetect uses for most addresses, and the suspected
	 * source of the spurious ACKs in the bridge scan. */
	run_scan("0-byte WRITE (SMBus-quick)", probe_write_quick);

	/* 1-byte read: a deeper probe that actually clocks data on
	 * the wire. i2cdetect uses this for the "fragile" ranges
	 * 0x30-0x37 and 0x50-0x5F. */
	run_scan("1-byte READ", probe_read_byte);

	printk("\n==========================================\n");
	printk("Comparison done.\n");
	printk("  - If WRITE result has many more ACKs than READ:\n");
	printk("    the I3C controller's 0-byte write path is\n");
	printk("    spuriously succeeding without checking NACK.\n");
	printk("  - If both results match: the bridge layer is\n");
	printk("    the source of spurious ACKs, not the controller.\n");
	printk("==========================================\n");

	while (1) {
		k_msleep(60000);
	}
	return 0;
}
