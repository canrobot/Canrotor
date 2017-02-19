#include <Board.h>

static volatile uint32_t msTicks = 0;


void HAL_SYSTICK_Callback(void)
{
	msTicks++;
}

uint32_t micros(void)
{
	 register uint32_t ms, cycle_cnt;
	    do {
	        ms = msTicks;
	        cycle_cnt = SysTick->VAL;
	    } while (ms != msTicks);
	    return (ms * 1000) + (168 * 1000 - cycle_cnt) / 168;
}
