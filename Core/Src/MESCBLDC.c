/*
 **
 ******************************************************************************
 * @file           : MESCBLDC.c
 * @brief          : BLDC running code
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 David Molony.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************

 * MESCBLDC.c
 *
 *  Created on: 25 Jul 2020
 *      Author: David Molony
 */

#include "MESCBLDC.h"

#include "MESCfoc.h"
#include "MESChw_setup.h"
#include "MESCmotor_state.h"
#include "stm32f3xx_hal.h"

#define PWM_FREQUENCY      37000  // 37kHz coil current pwm frequency
#define PWM_PERIOD         1 / PWM_FREQUENCY
#define MAX_INTEGRAL_ERROR 10
#define MIN_INTEGRAL_ERROR -MAX_INTEGRAL_ERROR
#define MAX_DUTY_CYCLE     1023

// fixme: it would be better if this pointer was passed to the file from outside
// during initialisation function call and stored locally. This way the code in
// this file becomes more generic.
extern TIM_HandleTypeDef htim1;

typedef enum {
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_IDLE,
    MOTOR_BRAKE
} EMotor_State;

// fixme: names need to be a bit more descriptive.
typedef struct {
    int duty_cycle;
    int state;
    int channel;
    float req_current;
    float current;
    int p_gain;
    int i_gain;
} SMotor_Parameters;

SMotor_Parameters motor_parameters;
// fixme: should EMotor_State be part of the SMotor_Parameters struct? What is
// the point of struct in the first place?
EMotor_State motor_state;
uint32_t ICVals[2] = {0, 0};    // fixme: give this array a better name.
uint16_t mystery_variable = 0;  // fixme: renamed for a.

// internal function prototypes
void writeMotor();
void runMotorForward(int current_hall_state);
void runMotorBackward(int current_hall_state);
int brakeMotor(int current_hall_state, int last_hall_state);

void motorInit() {
    motor_parameters.req_current = 0;  // Start the motor at 0 current
    motor_parameters.duty_cycle = 0;
    motor_parameters.channel = 0;
    motor_parameters.current = 0;
    // wtf should I set the gain as by default... V/Amp error...Perhaps
    // base it on Rphase and the bus voltage (nominally 48V)? But we
    // don;t know the exact bus voltage yet...
    motor_parameters.p_gain = 1023 * motor.Rphase / 8;
    // Initially, let's just make the i_gain the
    // same as the p_gain, so after 1 second
    // their contributions will be equal.
    motor_parameters.i_gain = motor_parameters.p_gain;
    // fixme: make getHallState() function return actual enumerated value.
    motor_parameters.state = GetHallState();
    motor_state = MOTOR_FORWARD;
}

// fixme: this function requires description of what it does.
void motorCommuteHall() {
    // Borrow the hall state detection from the FOC system
    int current_hall_state = GetHallState();
    static int last_hall_state = 7;

    // If moving forward...
    if (motor_state == MOTOR_FORWARD) {
        runMotorForward(current_hall_state);
        return;
    }

    // if moving backward...
    if (motor_state == MOTOR_BACKWARD) {
        runMotorBackward(current_hall_state);
        return;
    }

    // if braking...
    if (motor_state == MOTOR_BRAKE) {
        // fixme: this is still horrible. Need to understand what hall states do
        // and refactor into a better code.
        last_hall_state = brakeMotor(current_hall_state, last_hall_state);
        return;
    }

    // ...otherwise...
    // Disable the drivers, freewheel
    // fixme: misleading function name. If this is freewheel, then it should
    // be named as such.
    phU_Break();
    phV_Break();
    phW_Break();
}

inline void runMotorForward(int current_hall_state) {
    // fixme: this operation is completely unclear. Needs explanation of
    // what is going on.
    motor_parameters.state = (current_hall_state + 2) % 6;
    // Write the PWM values for the next state to generate forward torque
    writeMotor();
    if (!(motor_parameters.state == (current_hall_state + 1))) {
        // ToDo Fix if the writeMotor command is put in here, the PWM duty
        // gets stuck at 0.
    }
}

inline void runMotorBackward(int current_hall_state) {
    motor_parameters.state = (current_hall_state + 4) % 6;
    writeMotor();  // Write the PWM values for the previous state to generate
                   // reverse torque
                   // FIXME: what is this supposed to accomplish?
    // commented out since this code does nothing and is likely removed by
    // the compiler. 	if(!(CurrentHallState==CurrentHallState)){
    //	}
}

int brakeMotor(int current_hall_state, int last_hall_state) {
    int hallStateChange = current_hall_state - last_hall_state;
    // ToDo Logic to always be on synch or hanging 1 step in front or
    // behind... ToDo this does not cope with the roll-over, making for a
    // very jerky brake
    // TODO: the expression inside if() statement is very hard to read.
    // Create separate variable.
    if (((hallStateChange) % 6) > 1) {
        motor_parameters.state = (current_hall_state + 5) % 6;
    } else if (((current_hall_state - last_hall_state) % 6) < -1) {
        motor_parameters.state = (current_hall_state + 1) % 6;
        last_hall_state = current_hall_state;
    }
    writeMotor();
    return (last_hall_state);
}

// todo: should we put guards around this function to ensure it does not get
// preempted by an RTOS?
void motorCurrentController() {
    // Implement a simple PI controller
    // fixme: do p_error and duty_cycle need to be static?
    static float p_error = 0;
    static float i_error = 0;  // fixme:this arrangement does not allow to reset
                               // integral accumulator.
    static int duty_cycle = 0;

    // fixme: why are we storing current data? is it going to be used later? If
    // so, this needs to be explicitly stated, so someone does not "fix" this
    // during refactoring.
    motor_parameters.current =
        measurement_buffers.ConvertedADC[motor_parameters.channel][0];

    // proportional error calculation
    p_error = (motor_parameters.req_current - motor_parameters.current);
    duty_cycle = p_error * motor_parameters.p_gain;

    // range check proportional input since if it maxes out pwm there is no
    // point calculating integral
    if (duty_cycle > MAX_DUTY_CYCLE) {
        duty_cycle = MAX_DUTY_CYCLE;
    } else {
        // since proportional does not max out pwm duty cycle, add integral
        // update integral error accumulator and range check.
        i_error += p_error * PWM_PERIOD;
        if (i_error > MAX_INTEGRAL_ERROR) i_error = MAX_INTEGRAL_ERROR;
        if (i_error < MIN_INTEGRAL_ERROR) i_error = MIN_INTEGRAL_ERROR;
        // calculate duty cycle of the pwm and range check
        duty_cycle += i_error * motor_parameters.i_gain;
    }

    if (duty_cycle > MAX_DUTY_CYCLE)
        duty_cycle = MAX_DUTY_CYCLE;

    else if (duty_cycle < 0)
        duty_cycle = 0;

    // fixme: it seems to be that storing data in specially dedicated registers
    // is a way to pass data around. This is not very efficient. Instead whoever
    // calls this function should get duty cycle as a return value. This way it
    // is in the stack for very quick access. plus it is a much better coding
    // practice.
    motor_parameters.duty_cycle = duty_cycle;
}

// fixme: this function requires refactoring. magic numbers from case statements
// need to get better meaning. In fact, entire function requires detailed
// explanation of how it works and magical sequence in the switch{} statement.
void writeMotor() {
    switch (motor_parameters.state) {
        case 0:
            // disable phase first
            phW_Break();
            // WritePWM values
            htim1.Instance->CCR1 = motor_parameters.duty_cycle;
            htim1.Instance->CCR2 = 0;
            phU_Enable();
            phV_Enable();
            // Write the field into which the lowside current will
            // flow, to be retrieved from the FOC_measurement_vars
            motor_parameters.channel = 1;
            break;

        case 1:
            phV_Break();
            htim1.Instance->CCR1 = motor_parameters.duty_cycle;
            htim1.Instance->CCR3 = 0;
            phU_Enable();
            phW_Enable();
            motor_parameters.channel = 2;
            break;

        case 2:
            phU_Break();
            htim1.Instance->CCR2 = motor_parameters.duty_cycle;
            htim1.Instance->CCR3 = 0;
            phV_Enable();
            phW_Enable();
            motor_parameters.channel = 2;
            break;

        case 3:
            phW_Break();
            htim1.Instance->CCR1 = 0;
            htim1.Instance->CCR2 = motor_parameters.duty_cycle;
            phU_Enable();
            phV_Enable();
            motor_parameters.channel = 0;
            break;

        case 4:
            phV_Break();
            htim1.Instance->CCR1 = 0;
            htim1.Instance->CCR3 = motor_parameters.duty_cycle;
            phU_Enable();
            phW_Enable();
            motor_parameters.channel = 0;
            break;

        case 5:
            phU_Break();
            htim1.Instance->CCR2 = 0;
            htim1.Instance->CCR3 = motor_parameters.duty_cycle;
            phV_Enable();
            phW_Enable();
            motor_parameters.channel = 1;
            break;
        default:
            break;
    }
}

// fixme: remove all magic numbers and replace with proper definitions. Add
// explanation of what is going on while at it.
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        ICVals[0] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);

        // Target is 20000 guard is +-10000
        if ((ICVals[0] < 10000) || (30000 < ICVals[0])) {
            mystery_variable = 0;
            motor_parameters.req_current = 0;
        } else if (ICVals[0] != 0) {
            motor_state = MOTOR_FORWARD;
            ICVals[1] = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);
            // fixme: is this where 80% duty cycle limit implemented?
            if (ICVals[1] > 2000) ICVals[1] = 2000;
            if (ICVals[1] < 1000) ICVals[1] = 1000;

            // Mid-point is 1500 guard is +-100
            if ((ICVals[1] > 1400) && (1600 > ICVals[1])) {
                ICVals[1] = 1500;
            }
            // Set the current setpoint here
            // fixme: this is always executed, so why if() statement?
            if (1) {  // Current control, ToDo convert to Enum
                if (ICVals[1] > 1600)
                    motor_parameters.req_current =
                        ((float)ICVals[1] - 1600) /
                        5.0;  // Crude hack, which gets current scaled to +/-80A
                              // based on 1000-2000us PWM in
                else if (ICVals[1] < 1400)
                    motor_parameters.req_current =
                        ((float)ICVals[1] - 1400) /
                        5.0;  // Crude hack, which gets current scaled to +/-80A
                              // based on 1000-2000us PWM in
                else
                    motor_parameters.req_current = 0;
            }
            // fixme: this never gets executed. what's the point?
            if (0) {  // Duty cycle control, ToDo convert to Enum
                if (mystery_variable < 10) {
                    motor_parameters.duty_cycle = 0;
                }
                if (mystery_variable > 9) {
                    motor_parameters.duty_cycle = 10 * (mystery_variable - 9);
                }
            }
        }
    }
}
