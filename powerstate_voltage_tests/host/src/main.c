#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <cmsis_core.h>
#include <fsl_clock.h>
#include "mcxn-pm.h"

// MCXN947 register peeks for diagnostics. Advance pin is now P3_18, so
// we look at gpio3 (base 0x4009_c000) and its NVIC IRQs 23 and 24.
#define CMC_CKSTAT_ADDR  0x40048014u
#define GPIO3_BASE       0x4009c000u
#define GPIO3_ISFR0_ADDR (GPIO3_BASE + 0xA0u)
#define GPIO3_PDIR_ADDR  (GPIO3_BASE + 0x40u)
#define GPIO3_ICR_BASE   (GPIO3_BASE + 0x80u)
#define GPIO3_PIN18_ICR  (GPIO3_ICR_BASE + 18 * 4)
#define GPIO3_IRQ0       23
#define GPIO3_IRQ1       24
#define OR_IRQn          0   // MCXN947 system OR-gated combined interrupt

static void dump_post_wake(const char *tag)
{
	uint32_t ckstat = *(volatile uint32_t *)CMC_CKSTAT_ADDR;
	uint32_t icr18  = *(volatile uint32_t *)GPIO3_PIN18_ICR;
	uint32_t isfr0  = *(volatile uint32_t *)GPIO3_ISFR0_ADDR;
	uint32_t pdir   = *(volatile uint32_t *)GPIO3_PDIR_ADDR;
	uint32_t iser0  = NVIC->ISER[0];
	uint32_t ispr0  = NVIC->ISPR[0];
	uint32_t primask = __get_PRIMASK();
	uint32_t basepri = __get_BASEPRI();
	printk("  [%s] CKSTAT=0x%08x ICR18=0x%08x ISFR0=0x%08x PDIR.18=%u\n",
	       tag, ckstat, icr18, isfr0, (pdir >> 18) & 1);
	printk("  [%s] ISER0=0x%08x ISPR0=0x%08x PRIMASK=%u BASEPRI=0x%02x (IRQ23=%u IRQ24=%u)\n",
	       tag, iser0, ispr0, primask, basepri,
	       (iser0 >> GPIO3_IRQ0) & 1, (iser0 >> GPIO3_IRQ1) & 1);
}

// Two separate buttons:
//   wakeup_pin   = P0_19 = WUU input 3 : ONLY used to wake the chip from sleep
//   advance_pin  = P0_30              : plain Zephyr GPIO IRQ, used while the
//                                        chip is active to advance to the next state
// Decoupling the two roles keeps the WUU's post-wake state out of the advance path.
static struct gpio_dt_spec wake_gpio    = GPIO_DT_SPEC_GET(DT_NODELABEL(wakeup_pin), gpios);
static struct gpio_dt_spec advance_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(advance_pin), gpios);
static const uint8_t wake_pin_num = 3;

#define DEBOUNCE_MS 150
static int64_t last_wake_ms;

// WUU pin cb. Its only job is to exist so the WUU IRQ has a handler -- the
// hardware wakes the chip out of __WFI; this cb is just a diagnostic ping.
static void wake_cb(void *user_data)
{
	(void)user_data;
	int64_t now = k_uptime_get();
	if (now - last_wake_ms < DEBOUNCE_MS) {
		return;
	}
	last_wake_ms = now;
	printk("wuu wake fired\n");
}

static const struct external_pin_cfg WAKE_PIN_CFG = {
	.event = EXTERNAL_PIN_WAKEUP_INTERRUPT,
	.edge  = EXTERNAL_PIN_EDGE_ANY,
	.pm    = EXTERNAL_PIN_ALL_POWER_MODES,
};

// Poll the advance pin. Wire idles HIGH (pull-up); touching it to GND
// drives it LOW = "pressed". Sample every 20 ms with a short debounce.
static void await_press(const char *next_state)
{
	dump_post_wake("await_press");
	printk(">>> ACTIVE -- touch ADVANCE wire to GND to enter %s\n", next_state);

	// Wait until the pin reads low (wire on GND).
	while (gpio_pin_get_raw(advance_gpio.port, advance_gpio.pin) == 1) {
		printk("pin is inactive\n");
		k_msleep(20);
	}
	k_msleep(50);
	printk("pin went active\n");
	if (gpio_pin_get_raw(advance_gpio.port, advance_gpio.pin) == 1) {
		await_press(next_state);   // glitch; redo
		return;
	}
	printk("advance triggered (polled)\n");

	// Wait for release (wire pulled away from GND).
	while (gpio_pin_get_raw(advance_gpio.port, advance_gpio.pin) == 0) {
		k_msleep(20);
	}
	k_msleep(50);
}

// Re-arm the WUU pin right before going to sleep. Per RM 24.3.2, the WUU
// can be left "immediately disabled" after recovery from Power-Down-class
// modes; rewriting PE/PDC/PMC and clearing the pin flag puts it back in a
// known armed state.
static void rearm_wuu(void)
{
	wuu_cfg_external_pin(wake_pin_num, (struct external_pin_cfg *)&WAKE_PIN_CFG);
}

int main(void)
{
	printk("\n========================================\n");
	printk("  POWERSTATE_TEST host (frdm_mcxn947_hn)\n");
	printk("========================================\n");

	// SRS (System Reset Status, RM 38.7.1.8) records what caused the
	// most recent reset. If the chip reboots mid-test, the bits here
	// tell us why: LPACK (LP-mode peripheral didn't ack), WDOG, etc.
	uint32_t srs  = *(volatile uint32_t *)0x40048080u;
	uint32_t ssrs = *(volatile uint32_t *)0x40048088u;
	printk("  boot: SRS=0x%08x SSRS=0x%08x\n", srs, ssrs);
	if (srs & (1u << 10)) printk("  -> LPACK reset (LP-mode peripheral didn't ack within 65536 cycles)\n");
	if (srs & (1u << 11)) printk("  -> RSTACK reset (reset timeout)\n");
	if (srs & (1u << 13)) printk("  -> WWDT0 reset\n");
	if (srs & (1u << 14)) printk("  -> SW reset\n");
	if (srs & (1u << 15)) printk("  -> LOCKUP reset\n");
	if (srs & (1u << 0))  printk("  -> WAKEUP (cold reset wake from deep-power-down)\n");
	// Clear the sticky status so future cycles only show new bits.
	*(volatile uint32_t *)0x40048088u = ssrs;

	if (!gpio_is_ready_dt(&wake_gpio) || !gpio_is_ready_dt(&advance_gpio)) {
		printk("gpio pin(s) not ready\n");
		return -1;
	}
	if (gpio_pin_configure_dt(&wake_gpio, GPIO_INPUT) ||
	    gpio_pin_configure_dt(&advance_gpio, GPIO_INPUT | GPIO_PULL_UP)) {
		printk("failed to configure gpio pins as inputs\n");
		return -1;
	}

	// WUU on wakeup_pin -- only role is to pull the chip out of cmc_*()
	wuu_external_pin_enable_interrupt(1);
	wuu_external_pin_attach_cb(wake_pin_num, wake_cb, NULL);
	wuu_cfg_external_pin(wake_pin_num, (struct external_pin_cfg *)&WAKE_PIN_CFG);

	// --- ACTIVE 1 --> SLEEP ---
	await_press("SLEEP");
	rearm_wuu();
	dump_post_wake("pre-sleep");
	printk(">>> entering SLEEP\n");
	cmc_sleep();
	printk(">>> woke from SLEEP\n");
	dump_post_wake("post-sleep");

	// --- ACTIVE 2 --> DEEP SLEEP ---
	await_press("DEEP SLEEP");
	rearm_wuu();
	printk(">>> entering DEEP SLEEP\n");
	cmc_deep_sleep();
	printk(">>> woke from DEEP SLEEP\n");

	// --- ACTIVE 3 --> POWER DOWN ---
	await_press("POWER DOWN");
	rearm_wuu();
	printk(">>> entering POWER DOWN\n");
	k_msleep(20);
	CLOCK_DisableClock(kCLOCK_LPFlexComm4);
	cmc_power_down();
	CLOCK_EnableClock(kCLOCK_LPFlexComm4);
	printk(">>> woke from POWER DOWN\n");

	// --- ACTIVE 4 --> DEEP POWER DOWN ---
	// wake from DPD resets the whole chip; we won't return from this call.
	// after the next press, main() will start over from the banner above.
	await_press("DEEP POWER DOWN (chip will reset on wake)");
	rearm_wuu();
	printk(">>> entering DEEP POWER DOWN\n");
	k_msleep(20);
	CLOCK_DisableClock(kCLOCK_LPFlexComm4);
	cmc_deep_power_down();

	printk(">>> unexpected: returned from DEEP POWER DOWN\n");
	while (1) {
		k_sleep(K_SECONDS(60));
	}
	return 0;
}
