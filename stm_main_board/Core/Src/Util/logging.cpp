#include "Util/logging.h"

extern "C"
{
  int _write(int file, char *ptr, int len)
  {
    for (int DataIdx = 0; DataIdx < len; DataIdx++)
      // ITM_SendChar(*ptr++);
        HAL_UART_Transmit(&huart3, (uint8_t*) ptr++, 1, HAL_MAX_DELAY);
    return len;
  }
}
