// #ifndef __AVR_ATtiny13A__
//     #define __AVR_ATtiny13A__
// #endif

#ifndef __AVR_ATmega328P__
    #define __AVR_ATmega328P__
#endif

#include <avr/io.h>
#include <util/delay.h>

// #define F_CPU 8000000UL

// DDRx - port (A,B,C etc..)
int main()
{
    DDRB |= 1 << PINB5;
    PORTB = 0b00000000;
    while (1)
    {
      PORTB = 0b00000001;
      _delay_ms(500);
      PORTB = 0b00000000;
      _delay_ms(500);
    }
    return 0;
}
