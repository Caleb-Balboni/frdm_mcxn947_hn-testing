#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>

/* Set to 1 to skip SPI bursts. The LPSPI peripheral still claims
 * the SCK/MOSI/MISO/CS pads (so they sit at their idle states per
 * CPOL/CPHA), but no transactions occur. Useful for measuring DC
 * voltages on the SPI lines with a multimeter while the firmware
 * is running. */
#define DISABLE_SPI_BURST 0

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec sirq =
	GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, sirq_gpios);
static const struct gpio_dt_spec srdy =
	GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, srdy_gpios);

/* SPI mode 3 (CPOL=1, CPHA=1) to match the spi2c slave config. */
#define SPI_OP (SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8) | \
		SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_LINES_SINGLE)

static const struct spi_dt_spec spi_test =
	SPI_DT_SPEC_GET(DT_NODELABEL(spi_test_dev), SPI_OP, 0);

static uint8_t tx_buf[] = { 0xA5, 0x5A, 0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE };

static int __maybe_unused spi_clock_burst(uint32_t iter)
{
	struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
	struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
	int ret = spi_write_dt(&spi_test, &tx_set);
	if (ret != 0) {
		printk("iter %u: spi_write failed: %d\n", iter, ret);
	} else {
		printk("iter %u: spi clocked %u bytes\n",
		       iter, (unsigned)sizeof(tx_buf));
	}
	return ret;
}

int main(void)
{
	int ret;

	if (!gpio_is_ready_dt(&sirq)) {
		printk("SIRQ GPIO controller not ready\n");
		return -1;
	}
	if (!gpio_is_ready_dt(&srdy)) {
		printk("SRDY GPIO controller not ready\n");
		return -1;
	}
	if (!spi_is_ready_dt(&spi_test)) {
		printk("SPI bus not ready\n");
		return -1;
	}

	ret = gpio_pin_configure_dt(&sirq, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		printk("Failed to configure SIRQ as output: %d\n", ret);
		return ret;
	}
	ret = gpio_pin_configure_dt(&srdy, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		printk("Failed to configure SRDY as output: %d\n", ret);
		return ret;
	}

	printk("== sirq/srdy + spi master cross-test ==\n");
#if DISABLE_SPI_BURST
	printk("SPI BURSTS DISABLED — LPSPI idle for DC measurements\n");
#else
	printk("spi freq: %u Hz, burst: %u bytes\n",
	       spi_test.config.frequency, (unsigned)sizeof(tx_buf));
#endif

	bool sirq_state = false;
	bool srdy_state = false;
	bool toggle_sirq_next = true;
	uint32_t iter = 0;

	while (1) {
		/* Toggle one GPIO so SIRQ and SRDY each alternate at 1 Hz
		 * (each one flips once per 1000 ms cycle). */
		if (toggle_sirq_next) {
			sirq_state = !sirq_state;
			gpio_pin_set_dt(&sirq, sirq_state);
			printk("iter %u: SIRQ -> %s\n",
			       iter, sirq_state ? "HIGH" : "LOW");
		} else {
			srdy_state = !srdy_state;
			gpio_pin_set_dt(&srdy, srdy_state);
			printk("iter %u: SRDY -> %s\n",
			       iter, srdy_state ? "HIGH" : "LOW");
		}
		toggle_sirq_next = !toggle_sirq_next;

#if DISABLE_SPI_BURST
		/* No SPI burst — leave LPSPI idle so SCK/CS sit at their
		 * CPOL idle states and the user can DMM the lines. */
		k_msleep(500);
#else
		/* 100 ms gap so the scope can clearly see the GPIO edge,
		 * then the SPI burst that follows. */
		k_msleep(100);
		spi_clock_burst(iter);

		/* Remaining 400 ms to round out the 500 ms phase. */
		k_msleep(400);
#endif
		iter++;
	}
	return 0;
}
