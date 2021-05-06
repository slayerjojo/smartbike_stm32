#include "driver_stm32.h"
#include "sigma_log.h"

#if defined(PLATFORM_STM32)

#include <assert.h>
#include <sys/time.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

uint32_t stm32_ticks(void)
{
    struct timeval tv;
    gettimeofday(&tv, 0);
    return (uint64_t)tv.tv_sec * 1000 + (uint64_t)tv.tv_usec / 1000;
}

uint32_t stm32_ticks_from(uint32_t start)
{
	uint32_t now;

    if (!start)
        return 0;
    
    now = os_ticks();
    if (now >= start)
        return now - start;
    return 0xffffffffL - start + now;
}
/*
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}
*/
#endif
