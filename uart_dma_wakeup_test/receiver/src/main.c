#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <fsl_edma.h>
#include <fsl_clock.h>
#include <fsl_cmc.h>
#include <fsl_wuu.h>
#include <fsl_spc.h>

#define MCXN_WAKEUP_DELAY	DT_PROP_OR(DT_NODELABEL(spc), wakeup_delay, 0)
#define MCXN_WUU_ADDR		(WUU_Type *)DT_REG_ADDR(DT_INST(0, nxp_wuu))
#define MCXN_CMC_ADDR		(CMC_Type *)DT_REG_ADDR(DT_INST(0, nxp_cmc))
#define MCXN_SPC_ADDR		(SPC_Type *)DT_REG_ADDR(DT_INST(0, nxp_spc))

enum power_state {
	PWR_STATE_SLEEP,
	PWR_STATE_DEEP_SLEEP,
	PWR_STATE_POWER_DOWN,
};

static void pre_enter_power_state_hook(void) {

	CMC_SetPowerModeProtection(MCXN_CMC_ADDR, kCMC_AllowAllLowPowerModes);
	CMC_EnableDebugOperation(MCXN_CMC_ADDR, false);
	CMC_ConfigFlashMode(MCXN_CMC_ADDR, true, false);
	//WUU_SetInternalWakeUpModulesConfig(MCXN_WUU_ADDR, module_idx, kWUU_InternalModuleInterrupt);
}

static void post_enter_power_state_hook(void) {
	if ((SCB->SCR & SCB_SCR_SLEEPDEEP_Msk) == SCB_SCR_SLEEPDEEP_Msk) {
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	}
	__enable_irq();
	__ISB();
	SPC_ClearPowerDomainLowPowerRequestFlag(MCXN_SPC_ADDR, kSPC_PowerDomain0);
	SPC_ClearPowerDomainLowPowerRequestFlag(MCXN_SPC_ADDR, kSPC_PowerDomain1);
	SPC_ClearLowPowerRequest(MCXN_SPC_ADDR);
}

static void enter_power_state(enum power_state state) {
	pre_enter_power_state_hook();
	__enable_irq();
	__set_BASEPRI(0);
	switch (state) {
	case PWR_STATE_SLEEP:
		CMC_SetClockMode(MCXN_CMC_ADDR, kCMC_GateCoreClock);
		CMC_SetMAINPowerMode(MCXN_CMC_ADDR, kCMC_ActiveOrSleepMode);
		CMC_SetWAKEPowerMode(MCXN_CMC_ADDR, kCMC_ActiveOrSleepMode);
		__WFI();
		break;

	case PWR_STATE_DEEP_SLEEP:
		CMC_SetClockMode(MCXN_CMC_ADDR, kCMC_GateCoreClock);
		//CMC_SetClockMode(MCXN_CMC_ADDR, kCMC_GateAllSystemClocksEnterLowPowerMode);
		//CLOCK_EnableClock(kCLOCK_LPFlexComm5);
		CMC_SetMAINPowerMode(MCXN_CMC_ADDR, kCMC_DeepSleepMode);
		CMC_SetWAKEPowerMode(MCXN_CMC_ADDR, kCMC_DeepSleepMode);
		__WFI();
		break;

	case PWR_STATE_POWER_DOWN:
		SPC_SetLowPowerWakeUpDelay(SPC0, MCXN_WAKEUP_DELAY);
		CMC_SetClockMode(MCXN_CMC_ADDR, kCMC_GateAllSystemClocksEnterLowPowerMode);
		CMC_SetMAINPowerMode(MCXN_CMC_ADDR, kCMC_PowerDownMode);
		CMC_SetWAKEPowerMode(MCXN_CMC_ADDR, kCMC_PowerDownMode);
		__WFI();
		break;

	default:
		break;
	}
	post_enter_power_state_hook();
}

const struct device* uart_dev = DEVICE_DT_GET(DT_NODELABEL(flexcomm5_lpuart5));

static uint8_t rx_cb_bufs[2][64];

static uint8_t rx_cb_buf_idx = 0;

RING_BUF_DECLARE(user_rx_buf, 256);

static void uart_rx_callback(const struct device* dev, struct uart_event* event, void* user_data) {
	switch(event->type) {
		case  UART_RX_BUF_REQUEST:
			uart_rx_buf_rsp(dev, rx_cb_bufs[rx_cb_buf_idx], sizeof(rx_cb_bufs[rx_cb_buf_idx]));
			rx_cb_buf_idx = rx_cb_buf_idx ? 0 : 1;
			break;
		case UART_RX_BUF_RELEASED:
			break;
		case UART_RX_RDY:
			ring_buf_put(&user_rx_buf, event->data.rx.buf + event->data.rx.offset, event->data.rx.len);
			break;
	}
}

int main(void) {
	if (!device_is_ready(uart_dev)) {
		printk("uart device failed to init\n");
		return -1;
	}
	uart_callback_set(uart_dev, uart_rx_callback, NULL);
	EDMA_EnableAsyncRequest(DMA0, 1, true);
	uart_rx_enable(uart_dev, rx_cb_bufs[0], sizeof(rx_cb_bufs[0]), 1000);
	for (;;) {
		printk("going to sleep\n");
		enter_power_state(PWR_STATE_DEEP_SLEEP);
		printk("woke!\n");
		uint32_t size = ring_buf_size_get(&user_rx_buf);
		if (size <= 0) {
			printk("ring buffer was empty, going back to sleep\n");
			continue;
		}
		uint8_t buf[64];
		ring_buf_get(&user_rx_buf, buf, size);
		printk("got data: ");
		for (int i = 0; i < size; i++) {
			printk("%c", (char)buf[i]);
		}
		printk("\n");
	}
	return 0;
}
