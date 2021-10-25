#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/delay.h>

// The on board LED is connected to PB5
#define INIT_LED (DDRB |= (1 << DDB5))
#define SET_LED (PORTB |= (1 << PORTB5))
#define CLEAR_LED (PORTB &= ~(1 << PORTB5))
#define READ_LED (PORTB & (1 << PORTB5))

int init(void)
{
    cli();    // disable global interrupts
    INIT_LED; // Set  bit in Data Direction Port B Register.

    // Timer/Counter 0,  8 bit
    TCCR0A = 0;             // set entire TCCR1A register to 0
    TCCR0B = 0;             // set entire TCCR1B register to 0
                            // (as we do not know the initial  values)
    TCCR0A |= (1 << WGM01); // CTC mode
    TIMSK0 |= 1 << OCIE0A;  // Output compare match interrupt enable
    OCR0A = 249;            //(FCPU=16000000)/(prescaler=256)/(OCR0+1=250)=250 ticks per second (every 4ms)
    TCCR0B |= (1 << CS02);  // Start Timer 0 with prescaler=256, f=62.5KHz

    // Timer/Counter 1, 16 bit
    //
    // Used to generate the 10µs Trigger pulse and
    // to measure the pulse width of the Echo response pulse.
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); // No clock source, timer stopped.
    TCCR1B |= (1 << ICES1);                               // Input capture on rising (positive) edge.
    // TIMSK1 |= (1 << ICIE1);                               // Enable Input Capture Interrupt Enable for Timer/Counter 1.
    // OCR1A = 19;                                           // 19+1 cycles=10µs (1 tick=0.5µs)

    TCCR1A |= (1 << COM1A1) | (1 << COM1A0); // Configure PB1 alternate function as OC1A.
                                             // Set OC1A on Compare Match,
                                             // clear OC1A at BOTTOM (inverting mode).
    // /**
    // | Mode | WGM13 | WGM12 | WGM11 | WGM10 | Timer/Counter Mode of  Operation | TOP  | Update  of  OCR1x at | TOV1 Flag Set on |
    // |------+-------+-------+-------+-------+----------------------------------+------+----------------------+------------------|
    // |   14 |     1 |     1 |     1 |     0 | Fast PWM                         | ICR1 | BOTTOM               | TOP              |
    // **/
    TCCR1B |= (1 << WGM13) | (1 << WGM12); // Fast PWM mode 14
    TCCR1A |= (1 << WGM11);
    TCCR1A &= ~(1 << WGM10);
    OCR1A = (0xFFFF - 18); // Set MATCH to be 10µs behind MAX.
    ICR1 = 1;              // Set TOP to 0. This effectively prevents any further pulses from happening.
                           // To trigger one 10µs pulse, set TCNT1 to a value between (ICR1 + 1) and (OCR1A -1).

    // The trigger signal will be generated on PB1.
    DDRB |= (1 << DDB1); // Configure PB1 as output.

    // TIMSK1 |= (1 << TOIE1);
    TCCR1B |= (1 << CS11); // Start Timer 1 with prescaler=8. Tick every 0.5µs.

    sei();

    return 0;
}

int main(void)
{
    init();
    while (1)
    {
    }

    return 0;
}

void toggle_led_pin(void)
{
    // if (READ_LED)
    // {
    //     CLEAR_LED;
    // }
    // else
    // {
    //     SET_LED;
    // }
}

void trigger_pulse(void)
{
    // One shot pulse.
    // Any value between (ICR1 + 1) and (OCR1A -1).
    TCNT1 = (OCR1A - 19);
}

// Interrupt handler for Timer/Counter 0 compare A.
// Called every 4ms
ISR(TIMER0_COMPA_vect)
{
    static uint8_t cnt = 0;
    cnt++;
    if (cnt == 25)
    { // 100ms mark
        cnt = 0;
        trigger_pulse();
    }
}

ISR(TIMER1_OVF_vect)
{
    toggle_led_pin();
}