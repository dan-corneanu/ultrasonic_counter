#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/delay.h>
#include "uart/uart.h"

#define SPEED_OF_SOUND 343 // m / s

// The on board LED is connected to PB5
#define INIT_LED (DDRB |= (1 << DDB5))
#define SET_LED (PORTB |= (1 << PORTB5))
#define CLEAR_LED (PORTB &= ~(1 << PORTB5))
#define READ_LED (PORTB & (1 << PORTB5))

// The trigger signal will be generated on pin PB2 (Arduino nano D10)
#define INIT_TRIGGER_PIN (DDRB |= (1 << DDB2))

// The echo signal will be sampled on pin PB0 (Arduino nano D08)
#define INIT_ECHO_PIN (DDRB &= ~(1 << DDB0))

#define TRIGGER_PULSE_CLK_COUNT 20
#define MATCH_VALUE (0xFFFF - (TRIGGER_PULSE_CLK_COUNT - 1))

#define UART_BAUD_RATE 9600

volatile enum main_states {
    MAIN_IDLING
} main_state = MAIN_IDLING;

#define MAIN_SIGNAL_TRIGGER_NEW_HC_SR04_SAMPLE 0
#define MAIN_SIGNAL_HC_SR04_NEW_SAMPLE 1
#define MAIN_SIGNAL_HC_SR04_ERROR 2

volatile uint8_t main_signals = 0x00;

volatile enum hc_sr04_states {
    HC_SR04_IDLING,
    HC_SR04_TRIGGERING,
    HC_SR04_MEASURING_ECHO,
    HC_SR04_ECHO_FAULT
} hc_sr04_state = HC_SR04_IDLING;

#define HC_SR04_SIGNAL_TRIGGER_DONE 0
#define HC_SR04_SIGNAL_ECHO_OVERFLOW 1
#define HC_SR04_SIGNAL_ECHO_DONE 2
#define HC_SR04_SIGNAL_START 3

volatile uint8_t hc_sr04_signals = 0x00;

uint8_t hc_sr04_error = 1;
unsigned long distance_mm = 0;

// ------ BUZZER --------

//  Buzzer pin 1 and 2
#define INIT_BUZZER_PIN_1 (DDRB |= (1 << DDB3))
#define INIT_BUZZER_PIN_2 (DDRB |= (1 << DDB4))
#define TOGGLE_BUZZER_PINS (PORTB ^= ((1 << PORTB3) | (1 << PORTB4)))

static uint8_t buzzer_started = 0;

void buzzer_start(void)
{
    buzzer_started = 1;
    PORTB |= (1 << PORTB3);
    PORTB &= ~(1 << PORTB4);
}

void buzzer_stop(void)
{
    buzzer_started = 0;
    // Set both pins to 0.
    // The pizo transducer does not like DC current.
    PORTB &= ~((1 << PORTB3) | (1 << PORTB4));
}

int buzzer_init(void)
{
    INIT_BUZZER_PIN_1;
    INIT_BUZZER_PIN_2;

    buzzer_stop();

    return 0;
}

void buzzer_tick(void)
{
    if (buzzer_started)
    {
        TOGGLE_BUZZER_PINS;
    }
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

volatile uint16_t echo_end = 0;

int setup_main_timer(void)
{
    // Timer/Counter 0,  8 bit
    TCCR0A = 0;
    TCCR0B = 0;
    OCR0A = 249;            //(FCPU=16000000)/(prescaler=256)/(OCR0+1=250)=250 ticks per second (every 4ms)
    TCCR0A |= (1 << WGM01); // CTC mode
    TIMSK0 |= 1 << OCIE0A;  // Output compare match interrupt enable
    TCCR0B |= (1 << CS02);  // Start Timer 0 with prescaler=256, f=62.5KHz

    return 0;
}

int setup_hc_sr04_timer(void)
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

int hc_sr04_trigger(void)
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

    return 0;
}

/**
 * Set Normal Mode of operation with Top 0xFFFF.
 *
 * Set Input Capture Edge Select to rising edge.
 *
 * Start Timer 1 with prescaler=8. Generate one tick every 0.5µs.
 */
int hc_sr04_measure_echo(void)
{
    TCNT1 = 0x0000;
    TIFR1 |= (1 << ICF1);   // Clear Input Capture flag.
    TIMSK1 |= (1 << ICIE1); // Enable Timer 1 Input Capture interrupt.

    TCCR1A = 0x00;
    TCCR1B = (1 << ICES1) | (1 << CS11);

    return 0;
}

int hc_sr04_compute_distance(void)
{
    toggle_led_pin();

    // Each count is 0.5µs
    // Time to travel one way in µs.
    unsigned long time_to_travel = (echo_end >> 2);
    distance_mm = time_to_travel * SPEED_OF_SOUND / 1000;
    // Signal main_sm that a new value is available.
    main_signals |= (1 << MAIN_SIGNAL_HC_SR04_NEW_SAMPLE);
    echo_end = 0;

    return 0;
}

int init(void)
{
    cli();

    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));
    uart_attach_stdout();

    INIT_LED;
    INIT_TRIGGER_PIN;
    INIT_ECHO_PIN;

    setup_main_timer();
    setup_hc_sr04_timer();
    buzzer_init();
    
    sei();

    return 0;
}

int main(void)
{
    init();
    printf("Initialization done ...\n\r");

    while (1)
    {
        // // main state machine
        switch (main_state)
        {
        case MAIN_IDLING:
            if (main_signals & (1 << MAIN_SIGNAL_TRIGGER_NEW_HC_SR04_SAMPLE))
            {
                main_signals &= ~(1 << MAIN_SIGNAL_TRIGGER_NEW_HC_SR04_SAMPLE);
                hc_sr04_signals |= (1 << HC_SR04_SIGNAL_START);
            }
            else if (main_signals & (1 << MAIN_SIGNAL_HC_SR04_NEW_SAMPLE))
            {
                main_signals &= ~(1 << MAIN_SIGNAL_HC_SR04_NEW_SAMPLE);
                hc_sr04_error = 0;
                printf("%lu mm\n\r", distance_mm);
            }
            else if (main_signals & (1 << MAIN_SIGNAL_HC_SR04_ERROR))
            {
                main_signals &= ~(1 << MAIN_SIGNAL_HC_SR04_ERROR);
                hc_sr04_error = 1;
                printf("No object detected.\n\r");
            }
            break;

        default:
            break;
        }

        // // hc_sr04 state machine
        switch (hc_sr04_state)
        {
        case HC_SR04_IDLING:
            if (hc_sr04_signals & (1 << HC_SR04_SIGNAL_START))
            {
                hc_sr04_signals &= ~(1 << HC_SR04_SIGNAL_START);
                hc_sr04_state = HC_SR04_TRIGGERING;
                hc_sr04_trigger();
            }
            break;
        case HC_SR04_TRIGGERING:
            if (hc_sr04_signals & (1 << HC_SR04_SIGNAL_TRIGGER_DONE))
            {
                hc_sr04_signals &= ~(1 << HC_SR04_SIGNAL_TRIGGER_DONE);
                hc_sr04_state = HC_SR04_MEASURING_ECHO;
                hc_sr04_measure_echo();
            };
            break;
        case HC_SR04_MEASURING_ECHO:
            if (hc_sr04_signals & (1 << HC_SR04_SIGNAL_ECHO_DONE))
            {
                hc_sr04_signals &= ~(1 << HC_SR04_SIGNAL_ECHO_DONE);
                hc_sr04_compute_distance();
                hc_sr04_state = HC_SR04_IDLING;
            }
            else if (hc_sr04_signals & (1 << HC_SR04_SIGNAL_ECHO_OVERFLOW))
            {
                hc_sr04_signals &= ~(1 << HC_SR04_SIGNAL_ECHO_OVERFLOW);
                // Signal main_sm that we have failed to acquire a new sample.
                main_signals |= (1 << MAIN_SIGNAL_HC_SR04_ERROR);
                hc_sr04_state = HC_SR04_IDLING;
            };
            break;
        default:
            break;
        }
    }

    return 0;
}

// Interrupt handler for Timer/Counter 0 compare A.
// Called every 4ms
ISR(TIMER0_COMPA_vect)
{
    static uint8_t cnt = 0;
    cnt++;
    if (!hc_sr04_error && distance_mm < 200 && !buzzer_started)
    {
        buzzer_start();
    }
    else if (!hc_sr04_error && distance_mm >= 200 && buzzer_started)
    {
        buzzer_stop();
    }
    buzzer_tick();

    if (cnt == 25)
    { // 100ms mark
        cnt = 0;
        main_signals |= (1 << MAIN_SIGNAL_TRIGGER_NEW_HC_SR04_SAMPLE);
    }
}

ISR(TIMER1_OVF_vect)
{
    // Stop Timer1
    TCCR1A = 0x00;
    TCCR1B = 0x00;
    TIFR1 |= (1 << TOV1);    // Clear Overflow flag
    TIMSK1 &= ~(1 << TOIE1); // Disable Timer 1 Overflow interrup

    if (hc_sr04_state == HC_SR04_MEASURING_ECHO)
    {
        hc_sr04_signals |= (1 << HC_SR04_SIGNAL_ECHO_OVERFLOW);
    }
    else
    {
        hc_sr04_signals |= (1 << HC_SR04_SIGNAL_TRIGGER_DONE);
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
        echo_end = ICR1;
        hc_sr04_signals |= (1 << HC_SR04_SIGNAL_ECHO_DONE);

        TCCR1A = 0x00;
        TCCR1B = 0x00;
        TIFR1 |= (1 << ICF1);    // Clear Input Capture flag
        TIMSK1 &= ~(1 << ICIE1); // Disable Timer 1 Input Capture interrup
    }
}