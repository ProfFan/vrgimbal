#include "systick.h"
#include "iwdg.h"
#include "delay.h"

volatile uint32 systick_uptime_millis;
volatile uint32 uart1_lic_millis;
volatile uint32 uart2_lic_millis;
volatile uint32 uart3_lic_millis;
volatile uint32 uart4_lic_millis;
volatile uint32 uart5_lic_millis;
volatile uint32 uart6_lic_millis;

static void (*systick_user_callback)(void);

/**
 * @brief Initialize and enable SysTick.
 *
 * Clocks the system timer with the core clock, turns it on, and
 * enables interrupts.
 *
 * @param reload_val Appropriate reload counter to tick every 1 ms.
 */
void systick_init(uint32 reload_val) {
    SysTick->LOAD = reload_val;
    systick_enable();
}

/**
 * Clock the system timer with the core clock, but don't turn it
 * on or enable interrupt.
 */
void systick_disable() {
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
}

/**
 * Clock the system timer with the core clock and turn it on;
 * interrupt every 1 ms, for systick_timer_millis.
 */
void systick_enable() {
    /* re-enables init registers without changing reload val */
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
						SysTick_CTRL_TICKINT_Msk |
						SysTick_CTRL_ENABLE_Msk;				                                  
}

/**
 * @brief Attach a callback to be called from the SysTick exception handler.
 *
 * To detach a callback, call this function again with a null argument.
 */
void systick_attach_callback(void (*callback)(void)) {
    systick_user_callback = callback;
}

/*
 * SysTick ISR
 */
void SysTick_Handler(void) {
    systick_uptime_millis++;
    uart1_lic_millis++;
    uart2_lic_millis++;
    uart3_lic_millis++;
    uart4_lic_millis++;
	
    if (systick_user_callback) {
        systick_user_callback();
    }
}
