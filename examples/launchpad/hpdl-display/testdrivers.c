#include <stdio.h>
#include <string.h>
#include <stdint.h>


/*---------------------------------------------------------------------------*/
static uint8_t
ascii_to_hpdl(uint8_t k)
{
  if(k >= 'a' && k <= 'z') {
    return 'A' + k - 'a';
  }
  if(k > 0x5f || k < 0x20) {
    /* for invalid characters, just print a space (ie nothing) */
    return ' ';
  }
  return k;
}

/*---------------------------------------------------------------------------*/
/*  Returns a string with the argument byte written in binary.
    Example usage:
      printf("Port1: %s\n", char2bin(P1IN)); */
static uint8_t b[9];
static uint8_t *
char2bin(uint8_t x)
{
  uint8_t z;
  b[8] = '\0';
  for (z = 0; z < 8; z++) {
    b[7-z] = (x & (1 << z)) ? '1' : '0';
  }
  return b;
}

/*---------------------------------------------------------------------------*/
#define HPDL_A0_PIN   (1<<4)
#define HPDL_A1_PIN   (1<<5)
uint8_t ad = 0;
void
hpdl_write_char(uint8_t pos, uint8_t ch)
{
  uint8_t tkn;

  /* sanity check */
  if(pos < 1 || pos > 4) {
    return;
  }
  
  tkn = ascii_to_hpdl(ch);

  /* set address; acc to datasheet should be before clearing WR.
   * The address follows this convention: char #1 is the leftmost, closest to
   * pin 1, then #2, #3 and #4 is on the outer right, like so: [1 2 3 4]
   * The corresponding from the datasheet is instead: [3 2 1 0]
   */
  switch(pos) {
  case 1:
    ad |= (HPDL_A0_PIN | HPDL_A1_PIN);
    break;
  case 2:
    ad &= ~(HPDL_A0_PIN);
    ad |= (HPDL_A1_PIN);
    break;
  case 3:
    ad &= ~(HPDL_A1_PIN);
    ad |= (HPDL_A0_PIN);
    break;
  case 4:
    ad &= ~(HPDL_A0_PIN | HPDL_A1_PIN);
    break;
  }
  printf("-------Address: %s\n", char2bin(ad));

  /* set writing conditions */
  printf("WR low\n");

  /* set digits */
  printf("Digits: %s\n", char2bin(tkn));
  if(tkn & (1<<6)) {
    printf("EP high\n");
  } else {
    printf("EP low\n");
  }

  /* clear writing conditions */
  printf("WR high\n");
/*  asm("NOP;");*/
}
/*---------------------------------------------------------------------------*/
/* clear the display */
void
hpdl_clear(void)
{
  hpdl_write_char(1, ' ');
  hpdl_write_char(2, ' ');
  hpdl_write_char(3, ' ');
  hpdl_write_char(4, ' ');

#if HPDL_USE_SCROLL
  hpdl_scroll_stop();
#endif    /* HPDL_USE_SCROLL */
}
/*---------------------------------------------------------------------------*/
void
hpdl_write_string(char *s)
{
  uint8_t z;
  if(s == NULL) {
    return;
  }
  printf("---------------------------------------------------------------\n");
  printf("'%s'\n", s);

  /* just print up until an null termination, then print spaces after that */
  for(z = 0; z < 4; z += 1) {
    if(s[z] != 0) {
      hpdl_write_char(z+1, s[z]);
    } else {
      for(; z < 4; z += 1) {
        hpdl_write_char(z+1, ' ');
      }
      return;
    }
  }
}

/*---------------------------------------------------------------------------*/
int 
main(int argc, char *argv[])
{
  hpdl_write_string("Hej ");
/*  hpdl_write_string("1337");*/
/*  hpdl_write_string("/(*+");*/
/*  hpdl_write_char(3, ')');*/
/*  hpdl_write_char(4, '\\');*/
/*  hpdl_clear();*/

  return 0;
}
/*---------------------------------------------------------------------------*/
