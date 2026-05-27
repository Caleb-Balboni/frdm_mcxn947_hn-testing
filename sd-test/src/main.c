/*
 * Native SD card read/write throughput test (Zephyr / NXP USDHC).
 *
 * Bypasses the spi2c protocol entirely — talks directly to the SD card via
 * disk_access_*. Useful for isolating USDHC performance from the master <->
 * slave SPI link.
 *
 * Layout:
 *   - Init the disk, print sec_size and sec_count.
 *   - For a set of total-byte targets, run write+read+verify, time each phase,
 *     and print MB/s.
 *
 * Writes are NON-DESTRUCTIVE only if you point WRITE_START_SECTOR at unused
 * space. Default targets sectors at offset 1 GB from the start of the device,
 * which is in the middle of an ext4 partition on a card with a normal layout
 * — DO NOT run this against a card you care about without changing the
 * offset to a known-empty region.
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/storage/disk_access.h>
#include <string.h>
#include <stdint.h>

#define DISK_NAME        "SD"
#define SECTOR_BYTES     512
#define BUF_SECTORS      64                        /* 32 KB per call — matches sd-bus */
#define BUF_BYTES        (BUF_SECTORS * SECTOR_BYTES)

/* WARNING — this overwrites the sectors at this offset. Set this to a region
 * of the card that's safe to clobber. 0x200000 sectors = 1 GB offset on a
 * 512-byte-sector card. Adjust if your partition layout is different. */
#define WRITE_START_SECTOR 0x200000

/* How much data to write/read for each test pass. */
static const size_t test_sizes_mb[] = { 1, 4, 16 };

static uint8_t write_buf[BUF_BYTES] __aligned(4);
static uint8_t read_buf[BUF_BYTES] __aligned(4);

/* Deterministic PRNG so the read-back can recompute expected bytes without
 * holding all the data in RAM. */
static uint64_t rng_state;
static uint64_t xorshift64(void) {
	uint64_t x = rng_state;
	x ^= x << 13;
	x ^= x >> 7;
	x ^= x << 17;
	rng_state = x;
	return x;
}

static void fill_buf(uint8_t *buf, size_t len) {
	for (size_t i = 0; i + sizeof(uint64_t) <= len; i += sizeof(uint64_t)) {
		uint64_t v = xorshift64();
		memcpy(buf + i, &v, sizeof(uint64_t));
	}
}

static int run_pass(uint32_t total_sectors, uint32_t start_sector) {
	const uint64_t seed = 0xCAFEF00DDEADBEEFULL;
	uint32_t calls = total_sectors / BUF_SECTORS;
	if (calls == 0) {
		printk("test size too small (need >= %u sectors)\n", BUF_SECTORS);
		return -1;
	}

	size_t total_bytes = (size_t)total_sectors * SECTOR_BYTES;
	size_t total_kb    = total_bytes / 1024;

	printk("\n--- pass: %zu KB (%u sectors, %u calls of %u sectors each) ---\n",
	       total_kb, total_sectors, calls, BUF_SECTORS);

	/* ---- WRITE ---- */
	rng_state = seed;
	int64_t t_start = k_uptime_get();
	for (uint32_t i = 0; i < calls; i++) {
		fill_buf(write_buf, BUF_BYTES);
		int rc = disk_access_write(DISK_NAME, write_buf,
		                           start_sector + i * BUF_SECTORS,
		                           BUF_SECTORS);
		if (rc) {
			printk("write failed at call %u: %d\n", i, rc);
			return rc;
		}
	}
	int64_t t_write_ms = k_uptime_get() - t_start;

	/* ---- READ + VERIFY ---- */
	rng_state = seed;
	t_start = k_uptime_get();
	uint32_t mismatches = 0;
	int32_t  first_bad_call = -1;
	for (uint32_t i = 0; i < calls; i++) {
		fill_buf(write_buf, BUF_BYTES);  /* expected pattern */
		int rc = disk_access_read(DISK_NAME, read_buf,
		                          start_sector + i * BUF_SECTORS,
		                          BUF_SECTORS);
		if (rc) {
			printk("read failed at call %u: %d\n", i, rc);
			return rc;
		}
		if (memcmp(write_buf, read_buf, BUF_BYTES) != 0) {
			mismatches++;
			if (first_bad_call < 0) first_bad_call = (int32_t)i;
		}
	}
	int64_t t_read_ms = k_uptime_get() - t_start;

	/* ---- REPORT ---- */
	uint32_t per_call_w_us = (uint32_t)((t_write_ms * 1000) / calls);
	uint32_t per_call_r_us = (uint32_t)((t_read_ms  * 1000) / calls);

	/* throughput = total_kb / time_ms * 1000 = KB/s, then /1024 = MB/s */
	/* compute in fixed point: kbs * 100 = (total_kb * 100000) / t_ms */
	uint32_t w_kbs_x100 = t_write_ms ? (uint32_t)((total_kb * 100000ULL) / t_write_ms) : 0;
	uint32_t r_kbs_x100 = t_read_ms  ? (uint32_t)((total_kb * 100000ULL) / t_read_ms ) : 0;

	printk("WRITE: %lld ms total, %u us/call (%u sec/call), %u.%02u KB/s\n",
	       t_write_ms, per_call_w_us, BUF_SECTORS,
	       w_kbs_x100 / 100, w_kbs_x100 % 100);
	printk("READ:  %lld ms total, %u us/call (%u sec/call), %u.%02u KB/s\n",
	       t_read_ms,  per_call_r_us, BUF_SECTORS,
	       r_kbs_x100 / 100, r_kbs_x100 % 100);
	if (mismatches) {
		printk("VERIFY: %u/%u calls mismatched (first at call %d)  FAIL\n",
		       mismatches, calls, first_bad_call);
		return -EIO;
	}
	printk("VERIFY: all %u calls match  PASS\n", calls);
	return 0;
}

int main(void) {
	printk("\n");
	printk("==============================================\n");
	printk("  Native SD throughput test (disk_access API)\n");
	printk("==============================================\n");

	int rc = disk_access_init(DISK_NAME);
	if (rc) {
		printk("disk_access_init(%s) failed: %d\n", DISK_NAME, rc);
		return rc;
	}

	uint32_t sec_size = 0, sec_count = 0;
	rc = disk_access_ioctl(DISK_NAME, DISK_IOCTL_GET_SECTOR_SIZE, &sec_size);
	if (rc) {
		printk("GET_SECTOR_SIZE failed: %d\n", rc);
		return rc;
	}
	rc = disk_access_ioctl(DISK_NAME, DISK_IOCTL_GET_SECTOR_COUNT, &sec_count);
	if (rc) {
		printk("GET_SECTOR_COUNT failed: %d\n", rc);
		return rc;
	}

	uint64_t total_mb = ((uint64_t)sec_count * sec_size) / (1024ULL * 1024ULL);
	printk("disk: sec_size=%u  sec_count=%u  (%llu MB)\n",
	       sec_size, sec_count, total_mb);
	printk("write_start_sector = 0x%X (byte offset 0x%llX)\n",
	       WRITE_START_SECTOR,
	       (unsigned long long)WRITE_START_SECTOR * sec_size);
	if (sec_size != SECTOR_BYTES) {
		printk("UNEXPECTED: sec_size != %u, aborting\n", SECTOR_BYTES);
		return -EINVAL;
	}
	if (WRITE_START_SECTOR + (test_sizes_mb[ARRAY_SIZE(test_sizes_mb) - 1]
	                          * 1024 * 1024 / SECTOR_BYTES) > sec_count) {
		printk("UNEXPECTED: write range exceeds card capacity, aborting\n");
		return -EINVAL;
	}

	for (size_t i = 0; i < ARRAY_SIZE(test_sizes_mb); i++) {
		uint32_t sectors = test_sizes_mb[i] * 1024 * 1024 / SECTOR_BYTES;
		rc = run_pass(sectors, WRITE_START_SECTOR);
		if (rc) {
			printk("pass at %zu MB failed: %d — stopping\n",
			       test_sizes_mb[i], rc);
			return rc;
		}
	}

	printk("\nALL PASSES DONE — idling.\n");
	while (1) {
		k_msleep(60000);
	}
	return 0;
}
