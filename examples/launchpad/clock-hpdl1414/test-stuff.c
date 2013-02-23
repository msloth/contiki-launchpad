#include <stdio.h>
#include <stdint.h>


static char hpdlbuf[5];
static volatile uint8_t seconds = 0;
static volatile uint8_t minutes = 0;
static volatile uint8_t hours = 0;
/*---------------------------------------------------------------------------*/
/* converts an uint8_t < 100 to ASCII; if <10, the char for tens is replaced
with zero_tens_char. */
void
byte_to_ascii(char *buf, uint8_t val, uint8_t zero_tens_char)
{
  uint8_t tens = 0;
  if(val < 10) {
    buf[0] = zero_tens_char;
    goto fix_ones;
  } else if(val < 20) {
    buf[0] = '1';
    val -= 10;
  } else if(val < 30) {
    buf[0] = '2';
    val -= 20;
  } else if(val < 40) {
    buf[0] = '3';
    val -= 30;
  } else if(val < 50) {
    buf[0] = '4';
    val -= 40;
  } else if(val < 60) {
    buf[0] = '5';
    val -= 50;
  } else if(val < 70) {
    buf[0] = '6';
    val -= 60;
  } else if(val < 80) {
    buf[0] = '7';
    val -= 70;
  } else if(val < 90) {
    buf[0] = '8';
    val -= 80;
  } else {
    buf[0] = '9';
    val -= 90;
  }

fix_ones:
  buf[1] = '0' + val;
}
/*---------------------------------------------------------------------------*/
int 
main(int argc, char *argv[])
{
  uint32_t time;

#if 0
  /* tests the byte_to_ascii function */
  uint8_t temp;
  char b[3];
  b[2] = 0;
  for(temp = 0; temp < 100; temp += 1) {
    byte_to_ascii(b, temp, '0');
    printf("%s\n", b);
  }
  return 0;
#endif /* if 0; commented out code */




  /* init the buffer */
  hpdlbuf[4] = 0;
  byte_to_ascii(&(hpdlbuf[0]), hours, '0');
  byte_to_ascii(&(hpdlbuf[2]), minutes, '0');

  /* run for two days, and set the ascii buffer accordingly */
  for(time = 0; time < 60*60*48; time += 1) {
    seconds++;
    if(seconds >= 60) {
      seconds = 0;
      minutes++;
      if(minutes >= 60) {
        minutes = 0;
        hours++;
        if(hours >= 24) {
          hours = 0;
        }
        byte_to_ascii(&(hpdlbuf[0]), hours, '0');
      }
      byte_to_ascii(&(hpdlbuf[2]), minutes, '0');
      printf("Time is %u.%u, or: %s\n", hours, minutes, hpdlbuf);
    }
  }
  
  return 0;
}
/*---------------------------------------------------------------------------*/
