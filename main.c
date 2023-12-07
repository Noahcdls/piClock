#include <stdio.h>
#include "include/ili9341.h"

int main()
{
    ili9341_spi_init();
    ili9341_set_fgcolor565(ili9341_color565(40, 40, 40));
    ili9341_set_window(0, 0, 320, 240);
    ili9341_fillScreen();
}