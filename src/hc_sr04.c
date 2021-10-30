#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/delay.h>
#include "uart/uart.h"

// The on board LED is connected to PB5
#define INIT_LED (DDRB |= (1 << DDB5))
#define SET_LED (PORTB |= (1 << PORTB5))
#define CLEAR_LED (PORTB &= ~(1 << PORTB5))
#define READ_LED (PORTB & (1 << PORTB5))

// The trigger signal will be generated on pin PB2 (Arduino nano D10)
#define INIT_TRIGGER_PIN (DDRB |= (1 << DDB2))

#define TRIGGER_PULSE_CLK_COUNT 20
#define MATCH_VALUE (0xFFFF - (TRIGGER_PULSE_CLK_COUNT - 1))

#define UART_BAUD_RATE 9600

void stop_timer1(void)
{
    // No clock source, timer stopped.
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
}

int setup_sampling_timer(void)
{
    // Timer/Counter 0,  8 bit
    TCCR0A = 0;             // set entire TCCR1A register to 0
    TCCR0B = 0;             // set entire TCCR1B register to 0
                            // (as we do not know the initial  values)
    TCCR0A |= (1 << WGM01); // CTC mode
    TIMSK0 |= 1 << OCIE0A;  // Output compare match interrupt enable
    OCR0A = 249;            //(FCPU=16000000)/(prescaler=256)/(OCR0+1=250)=250 ticks per second (every 4ms)
    TCCR0B |= (1 << CS02);  // Start Timer 0 with prescaler=256, f=62.5KHz

    return 0;
}

/**
 * This routine leaves Timer1 stopped.
 */
int setup_hs_sr04_timer(void)
{
    // Timer/Counter 1, 16 bit
    //
    // Used to generate the 10µs Trigger pulse and
    // to measure the pulse width of the Echo response pulse.

    // Timer1 stopped and set to normal mode for immediate upate of OCR1x.
    TCCR1B = 0x00;
    TCCR1A = 0x00;

    /**
    | Mode | WGM13 | WGM12 | WGM11 | WGM10 | Timer/Counter Mode of  Operation | TOP   | Update  of  OCR1x at | TOV1 Flag Set on |
    |------+-------+-------+-------+-------+----------------------------------+-------+----------------------+------------------|
    |   14 |     1 |     1 |     1 |     0 | Fast PWM                         | ICR1  | BOTTOM               | TOP              |
    |   15 |     1 |     1 |     1 |     1 | Fast PWM                         | OCR1A | BOTTOM               | TOP              |
    **/
    OCR1B = MATCH_VALUE; // Set MATCH to be 10µs behind MAX.
    OCR1A = 0x0000;      // Set TOP to 0. This effectively prevents any further pulses from happening when in mode 15.
                         // To trigger one 10µs pulse, set TCNT1 to a value between (OCR1A + 1) and (OCR1B -1).
    TCNT1 = 0x0000;

    // Configure PB2 alternate function as OC1B.
    // Set OC1B on Compare Match,
    // clear OC1B at BOTTOM (inverting mode).
    // Fast PWM mode 15
    TCCR1A = (1 << COM1B1) | (1 << COM1B0) | (1 << WGM11) | (1 << WGM10);
    TCCR1B |= (1 << WGM13) | (1 << WGM12);

    return 0;

    // Normal mode with TOP set to 0xFFFF.
    TCCR1A = 0x00;
    TCCR1B = 0x00;

    // TCCR1B |= (1 << ICES1); // Input capture on rising (positive) edge.
    // TIMSK1 |= (1 << ICIE1); // Enable Input Capture Interrupt Enable for Timer/Counter 1.
}

int init(void)
{
    cli();

    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));
    uart_attach_stdout();

    stop_timer1();

    INIT_LED;
    INIT_TRIGGER_PIN;

    setup_sampling_timer();
    setup_hs_sr04_timer();

    sei();

    return 0;
}

int main(void)
{
    init();
    printf("Initialization done ...\n\r");
    while (1)
    {
    }

    return 0;
}

void toggle_led_pin(void)
{
    if (READ_LED)
    {
        CLEAR_LED;
    }
    else
    {
        SET_LED;
    }
}

void trigger_pulse(void)
{
    // One shot pulse.
    // Any value between (OCR1A + 1) and (OCR1B -1).
    TCNT1 = MATCH_VALUE - 1;
    TIFR1 |= (1 << TOV1);   // Clear Overflow interrupt flag
    TIMSK1 |= (1 << TOIE1); // Enable Timer 1 Overflow interrup
    TCCR1B |= (1 << CS11);  // Start Timer 1 with prescaler=8. Tick every 0.5µs.
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
    TIFR1 |= (1 << TOV1);    // Clear Overflow interrupt flag
    TIMSK1 &= ~(1 << TOIE1); // Disable Timer 1 Overflow interrup
    stop_timer1();
    toggle_led_pin();
}