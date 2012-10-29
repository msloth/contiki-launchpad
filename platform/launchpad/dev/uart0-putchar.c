#include <stdio.h>
#include "dev/uart0.h"

int
putchar(int c)
{
  uart0_writeb((char)c);
  return c;
}
