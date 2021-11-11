#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <util/delay.h>
#include "uart/uart.h"
#include "hardware/hc_sr04.h"

// The on board LED is connected to PB5
#define INIT_LED (DDRB |= (1 << DDB5))
#define SET_LED (PORTB |= (1 << PORTB5))
#define CLEAR_LED (PORTB &= ~(1 << PORTB5))
#define READ_LED (PORTB & (1 << PORTB5))

#define UART_BAUD_RATE 9600

volatile enum main_states {
    MAIN_IDLING
} main_state = MAIN_IDLING;

#define MAIN_SIGNAL_TRIGGER_NEW_HC_SR04_SAMPLE 0
#define MAIN_SIGNAL_HC_SR04_NEW_SAMPLE 1
#define MAIN_SIGNAL_HC_SR04_ERROR 2

volatile uint8_t main_signals = 0x00;

hc_sr04_t hc_sr04;
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

void hc_sr04_new_value_callback(void)
{
    if (HC_SR04_ECHO_FAULT == hc_sr04_get_state(&hc_sr04))
    {
        main_signals |= (1 << MAIN_SIGNAL_HC_SR04_ERROR);
    }
    else
    {
        distance_mm = hc_sr04_get_value_mm(&hc_sr04);
        main_signals |= (1 << MAIN_SIGNAL_HC_SR04_NEW_SAMPLE);
    }
}

int init(void)
{
    cli();

    uart_init(UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU));
    uart_attach_stdout();

    INIT_LED;
    hc_sr04_init(&hc_sr04, hc_sr04_new_value_callback);

    setup_main_timer();
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
                hc_sr04_trigger(&hc_sr04);
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