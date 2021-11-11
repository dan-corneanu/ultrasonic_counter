#include "hardware/hc_sr04.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define SPEED_OF_SOUND 343 // m / s

// The trigger signal will be generated on pin PB2 (Arduino nano D10)
#define INIT_TRIGGER_PIN (DDRB |= (1 << DDB2))

// The echo signal will be sampled on pin PB0 (Arduino nano D08)
#define INIT_ECHO_PIN (DDRB &= ~(1 << DDB0))

#define TRIGGER_PULSE_CLK_COUNT 20
#define MATCH_VALUE (0xFFFF - (TRIGGER_PULSE_CLK_COUNT - 1))

#define MAX_INSTANCES 1
static hc_sr04_t *instances[MAX_INSTANCES] = {0};

/**
 * Set Normal Mode of operation with Top 0xFFFF.
 *
 * Set Input Capture Edge Select to rising edge.
 *
 * Start Timer 1 with prescaler=8. Generate one tick every 0.5µs.
 */
void hc_sr04_measure_echo(hc_sr04_t *instance)
{
    TCNT1 = 0x0000;
    TIFR1 |= (1 << ICF1);   // Clear Input Capture flag.
    TIMSK1 |= (1 << ICIE1); // Enable Timer 1 Input Capture interrupt.

    TCCR1A = 0x00;
    TCCR1B = (1 << ICES1) | (1 << CS11);
}

/**
 * Configure PB2 alternate function as OC1B.
 *
 * Set OC1B on Compare Match,
 *  clear OC1B at BOTTOM (inverting mode).
 *
 * Configure Fast PWM mode 15.
 *
 * Start Timer 1 with prescaler=8. Generate one tick every 0.5µs.
 */
static int setup_hc_sr04_timer(hc_sr04_t *instance)
{
    // Timer/Counter 1,  16 bit
    // Used to generate the 10µs Trigger pulse and
    // to measure the pulse width of the Echo response pulse.
    TCCR1A = 0x00;
    TCCR1B = 0x00;

    OCR1B = MATCH_VALUE; // Set MATCH to be 10µs behind MAX.
    OCR1A = 0x0000;      // Set TOP to 0. This effectively prevents any further pulses from happening when in mode 15.
                         // To trigger one 10µs pulse, set TCNT1 to a value between (OCR1A + 1) and (OCR1B -1).
    TCNT1 = 0x0000;

    TIFR1 = 0x00;  // Clear all interrupt flags.
    TIMSK1 = 0x00; // Disable all interrupts.

    return 0;
}

hc_sr04_err_t hc_sr04_init(hc_sr04_t *instance, hc_sr04_callback_t callback)
{
    instance->echo_timeout = 0xFFFF;
    instance->callback = callback;
    instance->echo_end = 0;
    instance->state = HC_SR04_IDLING;

    instances[0] = instance;

    INIT_TRIGGER_PIN;
    INIT_ECHO_PIN;
    setup_hc_sr04_timer(instance);

    return HC_SR04_ERR_OK;
}

hc_sr04_err_t hc_sr04_trigger(hc_sr04_t *instance)
{
    // One shot pulse.
    // Any value between (OCR1A + 1) and (OCR1B -1).
    TCNT1 = MATCH_VALUE - 1;
    TIFR1 |= (1 << TOV1);   // Clear Overflow interrupt flag
    TIMSK1 |= (1 << TOIE1); // Enable Timer 1 Overflow interrup

    /**
    | Mode | WGM13 | WGM12 | WGM11 | WGM10 | Timer/Counter Mode of  Operation | TOP   | Update  of  OCR1x at | TOV1 Flag Set on |
    |------+-------+-------+-------+-------+----------------------------------+-------+----------------------+------------------|
    |   14 |     1 |     1 |     1 |     0 | Fast PWM                         | ICR1  | BOTTOM               | TOP              |
    |   15 |     1 |     1 |     1 |     1 | Fast PWM                         | OCR1A | BOTTOM               | TOP              |
    **/
    TCCR1A = (1 << COM1B1) | (1 << COM1B0) | (1 << WGM11) | (1 << WGM10);
    TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11);

    return HC_SR04_ERR_OK;
}

hc_sr04_state_t hc_sr04_get_state(hc_sr04_t *instance)
{
    return HC_SR04_ERR_OK;
}

unsigned long hc_sr04_get_value_mm(hc_sr04_t *instance)
{
    // Each count is 0.5µs
    // Time to travel one way in µs.
    unsigned long time_to_travel = (instance->echo_end >> 2);
    return time_to_travel * SPEED_OF_SOUND / 1000;
}

// Interrupt service routines

ISR(TIMER1_OVF_vect)
{
    // Stop Timer1
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TIFR1 |= (1 << TOV1);    // Clear Overflow flag
    TIMSK1 &= ~(1 << TOIE1); // Disable Timer 1 Overflow interrup

    if (instances[0]->state == HC_SR04_MEASURING_ECHO)
    {
        instances[0]->state = HC_SR04_ECHO_FAULT;
    }
    else
    {
        instances[0]->state = HC_SR04_MEASURING_ECHO;
        hc_sr04_measure_echo(instances[0]);
    }
}

ISR(TIMER1_CAPT_vect)
{
    if (TCCR1B & (1 << ICES1))
    {
        // This was the rising edge.
        TCCR1B &= ~(1 << ICES1);
        TCNT1 = 0x0000;
    }
    else
    {
        // This was the falling edge.
        instances[0]->echo_end = ICR1;
        
        TCCR1A = 0x00;
        TCCR1B = 0x00;
        TIFR1 |= (1 << ICF1);    // Clear Input Capture flag
        TIMSK1 &= ~(1 << ICIE1); // Disable Timer 1 Input Capture interrup

        instances[0]->state = HC_SR04_IDLING;
        instances[0]->callback();
    }
}