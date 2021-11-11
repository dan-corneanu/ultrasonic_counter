#ifndef __HARDWARE_HC_SRO4_H_INCLUDED__
#define __HARDWARE_HC_SRO4_H_INCLUDED__

#include <inttypes.h>

typedef enum hc_sr04_err
{
    HC_SR04_ERR_OK
} hc_sr04_err_t;

typedef enum hc_sr04_state
{
    HC_SR04_IDLING,
    HC_SR04_TRIGGERING,
    HC_SR04_MEASURING_ECHO,
    HC_SR04_ECHO_FAULT
} hc_sr04_state_t;

typedef void (*hc_sr04_callback_t)(void);

typedef struct
{
    uint16_t echo_timeout;
    hc_sr04_callback_t callback;
    volatile uint16_t echo_end;
    hc_sr04_state_t state;
} hc_sr04_t;

hc_sr04_err_t hc_sr04_init(
    hc_sr04_t *instance,
    hc_sr04_callback_t callback);

hc_sr04_err_t hc_sr04_trigger(hc_sr04_t *);

hc_sr04_state_t hc_sr04_get_state(hc_sr04_t *);

unsigned long hc_sr04_get_value_mm(hc_sr04_t *);

#endif