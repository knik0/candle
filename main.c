/****************************************************************************
    LED candle
    Copyright (C) 2014  Krzysztof Nikiel

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
****************************************************************************/

#include "tiny13a.h"

enum
{ FALSE = 0, TRUE = 1 };

static volatile uint8_t g_tcnt;

#define M(bit) (1<<(bit))
#define CBI(port,val) asm volatile ("cbi %0,%1"::"I" (&(port)-32), "I"(val))
#define SBI(port,val) asm volatile ("sbi %0,%1"::"I" (&(port)-32), "I"(val))

// PORTB:
enum
{ PB_LED1 = 0, PB_LED2 = 1 };

static uint16_t g_lfsr;
static inline uint16_t rand()
{
  // Based on: http://en.wikipedia.org/wiki/Linear_feedback_shift_register
  /* taps: 16 14 13 11; characteristic polynomial: x^16 + x^14 + x^13 + x^11 + 1 */
#define RNDMASK 0xB400u

  uint8_t lbs = g_lfsr;
  g_lfsr = (g_lfsr >> 1);
  if (lbs & 1)
    g_lfsr ^= RNDMASK;
  return g_lfsr;
}

static void candle(void)
{
  // OCA&OCB PWM enabled; fast PWM mode 3
  TCCR0A = M(COM0A1) | M(COM0B1) | M(COM0B0) | M(WGM01) | M(WGM00);
  TCCR0B = M(CS00);		// CK/1

  void out1(uint8_t v)
  {
    OCR0A = 0xff - v;
  }
  void out2(uint8_t v)
  {
    OCR0B = v;
  }

  out1(0);
  out2(0);

  // LED state:
  int16_t pos = 0;
  int16_t posad = 0;
  int16_t pow = 0;
  int16_t powad = 0;
  while (1)
  {
    enum
    { POWMID = 0xb0 };
    int16_t powr, posr;
    uint16_t r;
    uint16_t pow2;

    r = rand();

    // flame power
    powr = ((r >> 2) & 0x1f) + POWMID - 0x0f;
    // state variable bandpass filter
    powad += powr - pow;
    pow += powad / 4;
    pow -= (pow - POWMID) / 8;

    if (PINB & M(PINB3))
    {
      // PB3=1 -> mode 1

      // flame position
      posr = ((r >> 6) & 0x1ff) - 0x80;
      // bandpass filter
      posad += posr - pos;
      pos += posad / 16;
      pos -= (pos - 0x80) / 8;
    }
    else
    {
      // PB3=0 -> mode 0

      // flame position
      posr = ((r >> 6) & 0x7f) + 0x40;
      // bandpass filter
      posad += posr - pos;
      pos += posad / 8;
      pos -= (pos - 0x80) / 64;
    }

    enum
    { POWMIN = 0x40, POWMAX = 0xe0 };
    if (pow < POWMIN)
      pow = POWMIN;
    if (pow > POWMAX)
      pow = POWMAX;

    enum
    { POSMIN = 0x10, POSMAX = 0xf0 };
    if (pos < POSMIN)
      pos = POSMIN;
    if (pos > POSMAX)
      pos = POSMAX;

    pow2 = ((uint16_t) pow * (uint16_t) pow) >> 8;

    // wait for sync
    while (g_tcnt)
      asm volatile ("sleep");
    // set delay
    if (PINB & M(PINB4))
      // PB4=1 -> slow
      g_tcnt = 20;
    else
      // PB4=0 -> fast
      g_tcnt = 16;

    out1(((uint16_t) pos * pow2) >> 8);
    out2(((uint16_t) (0xff - pos) * pow2) >> 8);
  }
}


int main(void) __attribute__ ((noreturn));
int main(void)
{
  // init:
#define PORTBMASK (M(PB_LED1)|M(PB_LED2))
  DDRB = PORTBMASK;
  PORTB = ((uint8_t) ~ PORTBMASK) & ~(M(PINB3) | M(PINB4));

  // system clock prescaler:
  CLKPR = M(CLKPCE);
  //CLKPR = 6;                  // osc/64 ~150kHz
  CLKPR = 5;			// osc/32 ~300kHz

  TCCR0B = M(CS00);		// clk/1

  // OVF interrupt
  g_tcnt = 0;
  TIMSK0 = M(TOIE0);
  asm volatile ("sei");
  // enable sleep
  MCUCR = M(SE);

  // disable comparator:
  SBI(ACSR, ACD);
  // shut down ADC:
  PRR = M(PRADC);

  g_lfsr = 1;
  g_tcnt = 16;
  candle();

  while (1);
}

static void __vector_ovf(void) __attribute__ ((signal, used));
static void __vector_ovf(void)
{
  if (g_tcnt)
    g_tcnt--;
}


asm(".text");
asm("__do_copy_data:");
asm("__do_clear_bss:");

asm(".section .ctors");
// ATtiny13 verctors:
// 0x0000 RESET      External Pin, Power-on Reset, Brown-out Reset, Watchdog Reset
asm("rjmp __startup__");
// 0x0001 INT0       External Interrupt Request 0
asm("reti");
// 0x0002 PCINT0     Pin Change Interrupt Request 0
asm("reti");
// 0x0003 TIM0_OVF   Timer/Counter Overflow
asm("rjmp __vector_ovf");
// 0x0004 EE_RDY     EEPROM Ready
// 0x0005 ANA_COMP   Analog Comparator
// 0x0006 TIM0_COMPA Timer/Counter Compare Match A
// 0x0007 TIM0_COMPB Timer/Counter Compare Match B
// 0x0008 WDT        Watchdog Time-out
// 0x0009 ADC        ADC Conversion Complete

asm("__startup__:");
asm("clr __zero_reg__");
asm("ldi r28,lo8(__stack - 0)");
asm("out __SP_L__,r28");
asm("out __SREG__,__zero_reg__");
asm("rjmp main");
