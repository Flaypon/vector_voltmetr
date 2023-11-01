#ifndef __AVR_ATtiny13A__
    #define __AVR_ATtiny13A__
#endif

// #ifndef __AVR_ATmega328P__
//     #define __AVR_ATmega328P__
// #endif

#include <avr/io.h>
#include <util/delay.h>

#define F_CPU 1200000UL

// DDRx - port (A,B,C etc..)
int main()
{
    DDRB |= 1 << PINB5;
    PORTB = 0b00000000;
    while (1)
    {
      PORTB |= (1 << PINB5);
      _delay_ms(1000);

      PORTB &= ~(1 << PINB5);
      _delay_ms(1000);
    }
    return 0;
}
