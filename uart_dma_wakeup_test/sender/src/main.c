#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

/*
 * Transmit-only test program.
 *
 * Drives bytes out flexcomm2_lpuart2 (the board "arduino_serial" header),
 * which is enabled by default on frdm_mcxn947_hn. flexcomm4_lpuart4 is the
 * board console (printk), so the data line stays independent of console output.
 */
const struct device* uart_dev = DEVICE_DT_GET(DT_NODELABEL(flexcomm2_lpuart2));

static const uint8_t msg[] = "hello from sender\r\n";

int main(void) {
	if (!device_is_ready(uart_dev)) {
		printk("uart device failed to init\n");
		return -1;
	}

	printk("sender up, transmitting on flexcomm5_lpuart5\n");

	for (;;) {
		for (size_t i = 0; i < sizeof(msg) - 1; i++) {
			uart_poll_out(uart_dev, msg[i]);
		}
		printk("sent %u bytes\n", (unsigned int)(sizeof(msg) - 1));
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
