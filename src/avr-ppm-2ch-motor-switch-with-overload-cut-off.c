/*
 *
 * avr-ppm-2ch-motor-switch-with-overload-cut-off.c
 *
 * Copyright (C) 2025  Barnard33
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This code is only intended to be used for educational purposes.
 * It is not production stable and must under no circumstance be used
 * in any kind of radio controlled machinery, e.g., planes, cars, boats, etc.
 *
 * Created: 2025-12-28 13:54
 *
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#ifndef F_CPU
#error F_CPU not defined in make file or as compiler command argument
#endif

#ifndef NOP
#define NOP asm("NOP")
#endif

#define SERVO_IN_1 (1<<PD2)
#define SERVO_IN_2 (1<<PD3)

// ticks at 8MHz IO clock with a timer prescaler of 256
// 3.2e-05 seconds per timer tick
#define TICKS_MIN 31 // 0.000992 ms
#define TICKS_B 43   // 0.001376 ms, difference to next: 0.000128 ms
#define TICKS_N 47   // 0.001504 ms, difference to next: 0.000128 ms
#define TICKS_F 51   // 0.001632 ms
#define TICKS_MAX 63 // 0.002016 ms

// change this value according to your requirements
#define MAX_VOLTAGE_ONE_MOTOR 0.160

// change this value according to your requirements
#define ADDITIONAL_VOLTAGE_2ND_MOTOR 0.04

#define VOLTAGE_TO_ADC_TICKS(MAX_VALUE) ((MAX_VALUE * 1024.0) / 5.0)

typedef enum { FALSE, TRUE } bool_t;

/**
 * Defines the direction of motor rotation.
 * * [CW]: clockwise
 * * [CCW]: counter clockwise
 */
typedef enum { CW, CCW } direction_t;

typedef enum { MOTOR_1, MOTOR_2 } motor_t;

typedef enum { LOW, HIGH } pin_state_t;

typedef enum { OFF, ON } led_state_t;

typedef enum { PULL_UP_DISABLED, PULL_UP_ENABLED } pull_up_state_t;

typedef enum { SERVO_INVALID, SERVO_B, SERVO_N, SERVO_F } servo_state_t;

/**
 * Hardware abstraction type for output port pins.
 */
typedef struct {
    volatile uint8_t* port;
    volatile uint8_t* ddr;
    volatile uint8_t pin_mask;
} pin_hal_t;

/**
 * Hardware abstraction type for input port pins.
 */
typedef struct {
    pin_hal_t pin_hal;
    volatile uint8_t* pin;
} in_pin_hal_t;

/**
 * Hardware abstraction type for controlling one motor.
 * One output pin for clockwise rotation and one output pin for counter
 * clockwise rotation.
 */
typedef struct {
    pin_hal_t cw_pin_hal;
    pin_hal_t ccw_pin_hal;
} motor_hal_t;

/**
 * Defines the output port pins for controlling the motors.
 */
motor_hal_t motor_hals[] = {
    {
        {
            &PORTD,
            &DDRD,
            (1<<PD4)
        },
        {
            &PORTD,
            &DDRD,
            (1<<PD5)
        }

    },
    {
        {
            &PORTD,
            &DDRD,
            (1<<PD6)
        },
        {
            &PORTD,
            &DDRD,
            (1<<PD7)
        }
    }
};

/**
 * Defines the output port pin for the onboard LED.
 */
pin_hal_t led_hal = {
    &PORTB,
    &DDRB,
    (1<<PB5)
};

/**
 * Defines the input port pin for the servo signal controlling motor one.
 */
in_pin_hal_t int0_hal = {
    {
        &PORTD,
        &DDRD,
        (1 << PD2)
    },
    &PIND
};

/**
 * Defines the input port pin for the servo signal controlling motor two.
 */
in_pin_hal_t int1_hal = {
    {
        &PORTD,
        &DDRD,
        (1 << PD3)
    },
    &PIND
};

inline void out_pin_on(pin_hal_t pin_hal)
{
    *(pin_hal.port) |= pin_hal.pin_mask;
}

inline void out_pin_off(pin_hal_t pin_hal)
{
    *(pin_hal.port) &= ~(pin_hal.pin_mask);
}

inline bool_t is_out_pin_on(pin_hal_t pin_hal)
{
    if(*(pin_hal.port) & pin_hal.pin_mask) return TRUE;
    return FALSE;
}

inline void configure_out_pin(pin_hal_t pin_hal)
{
    *(pin_hal.ddr) |= pin_hal.pin_mask;
    out_pin_off(pin_hal);
}

inline void configure_in_pin(in_pin_hal_t in_pin_hal, pull_up_state_t pull_up_state)
{
    *(in_pin_hal.pin_hal.ddr) &= ~(in_pin_hal.pin_hal.pin_mask);
    if(pull_up_state) {
        *(in_pin_hal.pin_hal.port) |= in_pin_hal.pin_hal.pin_mask;
    } else {
        *(in_pin_hal.pin_hal.port) &= ~(in_pin_hal.pin_hal.pin_mask);
    }
}

inline pin_state_t read_pin(in_pin_hal_t in_pin_hal)
{
    if ((*(in_pin_hal.pin)) & (in_pin_hal.pin_hal.pin_mask)) {
        return HIGH;
    } else {
        return LOW;
    }
}

inline void motor_on(motor_t motor, direction_t direction)
{
    motor_hal_t motor_hal = motor_hals[motor];
    switch(direction) {
    case CW:
        out_pin_off(motor_hal.ccw_pin_hal);
        out_pin_on(motor_hal.cw_pin_hal);
        break;
    case CCW:
        out_pin_off(motor_hal.cw_pin_hal);
        out_pin_on(motor_hal.ccw_pin_hal);
        break;
    }
}

inline void motor_off(motor_t motor)
{
    motor_hal_t motor_hal = motor_hals[motor];
    out_pin_off(motor_hal.cw_pin_hal);
    out_pin_off(motor_hal.ccw_pin_hal);
}

inline bool_t is_motor_on(motor_t motor)
{
    motor_hal_t motor_hal = motor_hals[motor];
    pin_hal_t cw_pin_hal = motor_hal.cw_pin_hal;
    pin_hal_t ccw_pin_hal = motor_hal.ccw_pin_hal;
    return is_out_pin_on(cw_pin_hal) || is_out_pin_on(ccw_pin_hal);
}

inline void configure_motor(motor_t motor)
{
    motor_hal_t motor_hal = motor_hals[motor];
    configure_out_pin(motor_hal.cw_pin_hal);
    configure_out_pin(motor_hal.ccw_pin_hal);
    motor_off(motor);
}

inline void led(led_state_t state)
{
    if(state) {
        out_pin_on(led_hal);
    } else {
        out_pin_off(led_hal);
    }
}

inline void configure_adc(void)
{
    // set reference voltage to AVcc (5V)
    ADMUX = (1 << REFS0);

    // activate ADC and set prescaler to 128 (8MHz / 64 = 125kHz sampling rate)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);
}

inline uint16_t read_adc(void)
{
    // start measurement
    ADCSRA |= (1 << ADSC);

    // wait until measurement is complete (ADSC becomes 0)
    while (ADCSRA & (1 << ADSC));

    return ADC; // return 10-bit result (0 - 1023)
}

inline uint8_t read_timer_0(void)
{
    return TCNT0;
}

inline bool_t has_timer_0_overflow(void)
{
    if(TIFR0 & (1 << TOV0)) return TRUE;
    else return FALSE;
}

inline void reset_timer_0(void)
{
    TCNT0 = 0;
    TIFR0 |= (1 << TOV0);  // reset overflow flag by writing 1 to it
}

inline void start_timer_0(void)
{
    // use prescaler 256 -> results in 62 ticks per 2 ms, good enough to recognize fwd, neutral, bwd servo positions
    TCCR0B |= (1 << CS02);
}

inline void stop_timer_0(void)
{
    TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
}

inline uint8_t read_timer_2(void)
{
    return TCNT2;
}

inline bool_t has_timer_2_overflow(void)
{
    if(TIFR2 & (1 << TOV2)) return TRUE;
    else return FALSE;
}

inline void reset_timer_2(void)
{
    TCNT2 = 0;
    TIFR2 |= (1 << TOV2); // reset overflow flag by writing 1 to it
}

inline void stop_timer_2(void)
{
    TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20));
}

inline void start_timer_2(void)
{
    // use prescaler 256 -> results in 62 ticks per 2 ms, good enough to recognize fwd, neutral, bwd servo positions
    TCCR2B |= ((1 << CS22) | (1 << CS21));
}

inline void configure_int_0_rising_edge(void)
{
    EICRA |= (1 << ISC01) | (1 << ISC00);
}

inline void configure_int_0_falling_edge(void)
{
    // EICRA |= (1 << ISC01); // not necessary, shall already be set if rising edge was configured before
    EICRA &= ~(1 << ISC00);
}

inline void configure_int_0(void)
{
    EIMSK |= (1 << INT0); // enable int 0
    configure_int_0_rising_edge();
    configure_in_pin(int0_hal, PULL_UP_DISABLED);
}

inline uint8_t is_int0_rising_edge(void)
{
    return EICRA & (1 << ISC00);
}

inline void configure_int_1_rising_edge(void)
{
    EICRA |= (1 << ISC11) | (1 << ISC10);
}

inline void configure_int_1_falling_edge(void)
{
    // EICRA |= (1 << ISC01); // not necessary, shall already be set if rising edge was configured before
    EICRA &= ~(1 << ISC10);
}

inline void configure_int_1(void)
{
    EIMSK |= (1 << INT1); // enable int 1
    configure_int_1_rising_edge();
    configure_in_pin(int1_hal, PULL_UP_DISABLED);
}

inline uint8_t is_int1_rising_edge(void)
{
    return EICRA & (1 << ISC10);
}

inline servo_state_t evaluate_servo_state(uint8_t timer_ticks)
{
    if(timer_ticks > TICKS_MAX) return SERVO_INVALID;
    else if(timer_ticks > TICKS_F) return SERVO_F;
    else if(timer_ticks > TICKS_B) return SERVO_N;
    else if(timer_ticks > TICKS_MIN) return SERVO_B;
    else return SERVO_INVALID;
}

inline static void handle_motor(motor_t motor, servo_state_t servo_in, bool_t inhibited)
{
    if(!inhibited) {
        switch(servo_in) {
        case SERVO_B:
            motor_on(motor, CCW);
            // wait a moment, as the motor's starting current is so high
            // that the motor would be switched off again during the 
            // next ADC measurement
            _delay_ms(20); 
            break;
        case SERVO_F:
            motor_on(motor, CW);
            // wait a moment, as the motor's starting current is so high
            // that the motor would be switched off again during the 
            // next ADC measurement
            _delay_ms(20);
            break;
        default:
            motor_off(motor);
            break;
        }
    }
}

volatile servo_state_t servo_in_0 = SERVO_N;

volatile servo_state_t servo_in_1 = SERVO_N;

/**
 * Interrupt handler for servo signal input pin on int0.
 */
ISR(INT0_vect)
{
    if(is_int0_rising_edge()) {
        reset_timer_0();
        start_timer_0();
        configure_int_0_falling_edge();
    } else {
        stop_timer_0();
        if(has_timer_0_overflow()) {
            // reset timer and overflow flag after an overflow,
            // which indicates an invalid signal
            reset_timer_0();
        } else {
            servo_in_0 = evaluate_servo_state(read_timer_0());
        }
        configure_int_0_rising_edge();
    }
}

/**
 * Interrupt handler for servo signal input pin on int1.
 */
ISR(INT1_vect)
{
    if(is_int1_rising_edge()) {
        reset_timer_2();
        start_timer_2();
        configure_int_1_falling_edge();
    } else {
        stop_timer_2();
        if(has_timer_2_overflow()) {
            // reset timer and overflow flag after an overflow,
            // which indicates an invalid signal
            reset_timer_2();
        } else {
            servo_in_1 = evaluate_servo_state(read_timer_2());
        }
        configure_int_1_rising_edge();
    }
}

// defined as constants, so that the double evaluation is done by during 
// compile time and linking the floating point library is not necessary
const uint16_t ADC_MAX_VALUE_ONE_MOTOR = VOLTAGE_TO_ADC_TICKS(MAX_VOLTAGE_ONE_MOTOR);
const uint16_t ADC_MAX_VALUE_TWO_MOTORS = VOLTAGE_TO_ADC_TICKS(MAX_VOLTAGE_ONE_MOTOR) + VOLTAGE_TO_ADC_TICKS(ADDITIONAL_VOLTAGE_2ND_MOTOR);

int main(void)
{
    // switch CPU and IO clock down to 8 MHz
    CLKPR |= (1 << CLKPCE);
    CLKPR |= (1 << CLKPS0);

    configure_motor(MOTOR_1);
    configure_motor(MOTOR_2);

    configure_out_pin(led_hal);

    configure_adc();

    configure_int_0();
    configure_int_1();

    led(ON);
    _delay_ms(1000);
    led(OFF);

    sei();

    bool_t inhibited = FALSE;

    while (1) {
        // read the shunt voltage and decide on overload cut-off
        bool_t motor_1_on = is_motor_on(MOTOR_1);
        bool_t motor_2_on = is_motor_on(MOTOR_2);
        if(motor_1_on || motor_2_on) {
            uint16_t max_adc_value = ADC_MAX_VALUE_ONE_MOTOR;
            if(motor_1_on && motor_2_on) {
                max_adc_value = ADC_MAX_VALUE_TWO_MOTORS;
            }

            uint16_t adc_value = read_adc();
            if (adc_value > max_adc_value) {
                motor_off(MOTOR_1);
                motor_off(MOTOR_2);
                inhibited = TRUE;
                led(ON);
            }
        }

        // reset inhibition flag when both servo signals go to off
        if(inhibited && servo_in_0 == SERVO_N && servo_in_1 == SERVO_N) {
            inhibited = FALSE;
            led(OFF);
        }

        // start/stop the motors according to the servo signals
        handle_motor(MOTOR_1, servo_in_0, inhibited);
        handle_motor(MOTOR_2, servo_in_1, inhibited);
    }
    return 0;
}
