#ifndef __ATTINY13A_H__
#define __ATTINY13A_H__

/*
	Generated automatically from Datasheet Register Summary
	Sun Aug  3 17:46:15 2014
*/

#include "stdint.h"

/* 03 ADCSRB – ACME – – – ADTS2 ADTS1 ADTS0 */
#define ADCSRB	(*(volatile uint8_t *)(0x23))
#define ADTS0	0
#define ADTS1	1
#define ADTS2	2
#define ACME	6

/* 04 ADCL ADC Data Register Low Byte */
#define ADC	(*(volatile uint16_t *)(0x24))
#define ADCL	(*(volatile uint8_t *)(0x24))

/* 05 ADCH ADC Data Register High Byte */
#define ADCH	(*(volatile uint8_t *)(0x25))

/* 06 ADCSRA ADEN ADSC ADATE ADIF ADIE ADPS2 ADPS1 ADPS0 */
#define ADCSRA	(*(volatile uint8_t *)(0x26))
#define ADPS0	0
#define ADPS1	1
#define ADPS2	2
#define ADIE	3
#define ADIF	4
#define ADATE	5
#define ADSC	6
#define ADEN	7

/* 07 ADMUX – REFS0 ADLAR – – – MUX1 MUX0 */
#define ADMUX	(*(volatile uint8_t *)(0x27))
#define MUX0	0
#define MUX1	1
#define ADLAR	5
#define REFS0	6

/* 08 ACSR ACD ACBG ACO ACI ACIE – ACIS1 ACIS0 */
#define ACSR	(*(volatile uint8_t *)(0x28))
#define ACIS0	0
#define ACIS1	1
#define ACIE	3
#define ACI	4
#define ACO	5
#define ACBG	6
#define ACD	7

/* 14 DIDR0 – – ADC0D ADC2D ADC3D ADC1D AIN1D AIN0D */
#define DIDR0	(*(volatile uint8_t *)(0x34))
#define AIN0D	0
#define AIN1D	1
#define ADC1D	2
#define ADC3D	3
#define ADC2D	4
#define ADC0D	5

/* 15 PCMSK – – PCINT5 PCINT4 PCINT3 PCINT2 PCINT1 PCINT0 */
#define PCMSK	(*(volatile uint8_t *)(0x35))
#define PCINT0	0
#define PCINT1	1
#define PCINT2	2
#define PCINT3	3
#define PCINT4	4
#define PCINT5	5

/* 16 PINB – – PINB5 PINB4 PINB3 PINB2 PINB1 PINB0 */
#define PINB	(*(volatile uint8_t *)(0x36))
#define PINB0	0
#define PINB1	1
#define PINB2	2
#define PINB3	3
#define PINB4	4
#define PINB5	5

/* 17 DDRB – – DDB5 DDB4 DDB3 DDB2 DDB1 DDB0 */
#define DDRB	(*(volatile uint8_t *)(0x37))
#define DDB0	0
#define DDB1	1
#define DDB2	2
#define DDB3	3
#define DDB4	4
#define DDB5	5

/* 18 PORTB – – PORTB5 PORTB4 PORTB3 PORTB2 PORTB1 PORTB0 */
#define PORTB	(*(volatile uint8_t *)(0x38))
#define PORTB0	0
#define PORTB1	1
#define PORTB2	2
#define PORTB3	3
#define PORTB4	4
#define PORTB5	5

/* 1C EECR – – EEPM1 EEPM0 EERIE EEMPE EEPE EERE */
#define EECR	(*(volatile uint8_t *)(0x3c))
#define EERE	0
#define EEPE	1
#define EEMPE	2
#define EERIE	3
#define EEPM0	4
#define EEPM1	5

/* 1D EEDR EEPROM Data Register */
#define EEDR	(*(volatile uint8_t *)(0x3d))

/* 1E EEARL – – EEPROM Address Register */
#define EEARL	(*(volatile uint8_t *)(0x3e))

/* 21 WDTCR WDTIF WDTIE WDP3 WDCE WDE WDP2 WDP1 WDP0 */
#define WDTCR	(*(volatile uint8_t *)(0x41))
#define WDP0	0
#define WDP1	1
#define WDP2	2
#define WDE	3
#define WDCE	4
#define WDP3	5
#define WDTIE	6
#define WDTIF	7

/* 25 PRR – – – – – – PRTIM0 PRADC */
#define PRR	(*(volatile uint8_t *)(0x45))
#define PRADC	0
#define PRTIM0	1

/* 26 CLKPR CLKPCE – – – CLKPS3 CLKPS2 CLKPS1 CLKPS0 */
#define CLKPR	(*(volatile uint8_t *)(0x46))
#define CLKPS0	0
#define CLKPS1	1
#define CLKPS2	2
#define CLKPS3	3
#define CLKPCE	7

/* 28 GTCCR TSM – – – – – – PSR10 */
#define GTCCR	(*(volatile uint8_t *)(0x48))
#define PSR10	0
#define TSM	7

/* 29 OCR0B Timer/Counter – Output Compare Register B */
#define OCR0B	(*(volatile uint8_t *)(0x49))

/* 2E DWDR DWDR[7:0] */
#define DWDR	(*(volatile uint8_t *)(0x4e))
#define DWDR0	0
#define DWDR1	1
#define DWDR2	2
#define DWDR3	3
#define DWDR4	4
#define DWDR5	5
#define DWDR6	6
#define DWDR7	7

/* 2F TCCR0A COM0A1 COM0A0 COM0B1 COM0B0 – – WGM01 WGM00 */
#define TCCR0A	(*(volatile uint8_t *)(0x4f))
#define WGM00	0
#define WGM01	1
#define COM0B0	4
#define COM0B1	5
#define COM0A0	6
#define COM0A1	7

/* 30 BODCR – – – – – – BODS BODSE */
#define BODCR	(*(volatile uint8_t *)(0x50))
#define BODSE	0
#define BODS	1

/* 31 OSCCAL Oscillator Calibration Register */
#define OSCCAL	(*(volatile uint8_t *)(0x51))

/* 32 TCNT0 Timer/Counter (8-bit) */
#define TCNT0	(*(volatile uint8_t *)(0x52))

/* 33 TCCR0B FOC0A FOC0B – – WGM02 CS02 CS01 CS00 */
#define TCCR0B	(*(volatile uint8_t *)(0x53))
#define CS00	0
#define CS01	1
#define CS02	2
#define WGM02	3
#define FOC0B	6
#define FOC0A	7

/* 34 MCUSR – – – – WDRF BORF EXTRF PORF */
#define MCUSR	(*(volatile uint8_t *)(0x54))
#define PORF	0
#define EXTRF	1
#define BORF	2
#define WDRF	3

/* 35 MCUCR – PUD SE SM1 SM0 – ISC01 ISC00 */
#define MCUCR	(*(volatile uint8_t *)(0x55))
#define ISC00	0
#define ISC01	1
#define SM0	3
#define SM1	4
#define SE	5
#define PUD	6

/* 36 OCR0A Timer/Counter – Output Compare Register A */
#define OCR0A	(*(volatile uint8_t *)(0x56))

/* 37 SPMCSR – – – CTPB RFLB PGWRT PGERS SELFPRGEN */
#define SPMCSR	(*(volatile uint8_t *)(0x57))
#define SELFPRGEN	0
#define PGERS	1
#define PGWRT	2
#define RFLB	3
#define CTPB	4

/* 38 TIFR0 – – – – OCF0B OCF0A TOV0 – */
#define TIFR0	(*(volatile uint8_t *)(0x58))
#define TOV0	1
#define OCF0A	2
#define OCF0B	3

/* 39 TIMSK0 – – – – OCIE0B OCIE0A TOIE0 – */
#define TIMSK0	(*(volatile uint8_t *)(0x59))
#define TOIE0	1
#define OCIE0A	2
#define OCIE0B	3

/* 3A GIFR – INTF0 PCIF – – – – – */
#define GIFR	(*(volatile uint8_t *)(0x5a))
#define PCIF	5
#define INTF0	6

/* 3B GIMSK – INT0 PCIE – – – – – */
#define GIMSK	(*(volatile uint8_t *)(0x5b))
#define PCIE	5
#define INT0	6

/* 3D SPL SP[7:0] */
#define SPL	(*(volatile uint8_t *)(0x5d))
#define SP0	0
#define SP1	1
#define SP2	2
#define SP3	3
#define SP4	4
#define SP5	5
#define SP6	6
#define SP7	7

/* 3F SREG I T H S V N Z C */
#define SREG	(*(volatile uint8_t *)(0x5f))
#define SREGC	0
#define SREGZ	1
#define SREGN	2
#define SREGV	3
#define SREGS	4
#define SREGH	5
#define SREGT	6
#define SREGI	7

#endif /* __ATTINY13A_H__ */
