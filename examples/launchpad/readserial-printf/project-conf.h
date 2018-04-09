#ifndef __PROJECT_CONF_H__
#define __PROJECT_CONF_H__
/*---------------------------------------------------------------------------*/
#ifndef F_CPU
#define F_CPU                   16000000uL
#endif /* F_CPU */

#ifndef _MCU_
#define _MCU_                   2553
#endif /* _MCU_ */

#ifndef USE_SERIAL
#define USE_SERIAL              1
#endif /* USE_SERIAL */

#ifndef UART_SPEED
#define UART_SPEED              4800
// #define UART_SPEED              9600
#endif /* UART_SPEED */
/*---------------------------------------------------------------------------*/
#endif  /* __PROJECT_CONF_H__ */
