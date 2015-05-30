#include "retarget.h"

#include "simple_uart.h"
#include "boards.h"

int _write(int fd, char * str, int len)
{
    for (int i = 0; i < len; i++)
    {
        simple_uart_put(str[i]);
    }
    return len;
}

void retarget_init(void)
{
    //simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, HWFC);
    simple_uart_config(RTS_PIN_NUMBER, 9, CTS_PIN_NUMBER, 11, HWFC);
}
