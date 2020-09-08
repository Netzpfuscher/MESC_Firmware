/*
 **
 ******************************************************************************
 * @file           : MESCfoc.c
 * @brief          : FOC running code and ADC buffers
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

 * MESCfoc.c
 *
 *  Created on: 18 Jul 2020
 *      Author: David Molony
 */

/* Includes ------------------------------------------------------------------*/
#include "MESCfoc.h"

#include "MESCBLDC.h"
#include "MESChw_setup.h"
#include "MESCmotor_state.h"

extern TIM_HandleTypeDef htim1;

void fastLoop() {  // Call this directly from the ADC callback IRQ
    V_I_Check();   // Run the current and voltage checks
    switch (MotorState) {
        break;
        case MOTOR_STATE_SENSORLESS_RUN:
            runMotorSensorless();
            break;

        case MOTOR_STATE_HALL_RUN:
            runMotorHall(0);
            break;

        case MOTOR_STATE_HALL_NEAR_STATIONARY:
            runMotorHall(1);
            break;

        case MOTOR_STATE_OPEN_LOOP_STARTUP:
            // Same as open loop. Convert the ADC values into floats, do Clark
            // transform, ignore result of Clark, just want the float currents
            ADCConversion();
            openLoopPIFF();
            // Write the PWM values
            break;

        case MOTOR_STATE_OPEN_LOOP_TRANSITION:
            // Run open loop
            // Run observer
            // RunFOC
            // Weighted average of the outputs N PWM cycles
            // Write the PWM values
            break;

        case MOTOR_STATE_IDLE:
            __HAL_TIM_MOE_DISABLE(&htim1);
            // ToDo Set PWM to no output state
            break;

        case MOTOR_STATE_DETECTING:
            detectMotorPosition();
            break;

        case MOTOR_STATE_MEASURING:
            if (motor.Rphase == 0) {
                measureResistance();
                break;  // do not do inductance until resistance is measured.
            }
            if (motor.Lphase == 0) {
                measureInductance();
            }
            break;

        case MOTOR_STATE_ERROR:
            generateBreak();
            break;
        case MOTOR_STATE_ALIGN:
            // Turn on at a given voltage at electricalangle0;
            break;
        case MOTOR_STATE_RECOVERING:
            // fixme: this probably should not try to recover without external
            // prompt unless reason for failure is understood.

            // No clue so far. Read the phase voltages and determine position
            // and attempt to restart? Should already be in break state, and
            // should stay there...
            break;
    }
}

inline void runMotorSensorless() {
    ADCConversion();  // Convert the ADC values into floats, do Clark
    // transform
    // Call the observer
    // Call the current and phase controller
    // Write the PWM values
}

inline void runMotorHall(_Bool b_near_stationary) {
    // Convert the ADC values into floats, do Clark transform
    ADCConversion();
    // BLDC is hopefully just a temporary "Get it spinning" kind of thing, to be
    // deprecated in favour of FOC
    if (MotorControlType == MOTOR_CONTROL_TYPE_BLDC) {
        motorCurrentController();
        motorCommuteHall();
    }
    // Get the current position from HallTimer
    // Call the current and phase controller
    // Write the PWM values
}

inline void detectMotorPosition() {
    EHallSensorPhase_t sensor_phase = getHallSensorPhase();

    if (sensor_phase == SENSOR_UNKNOWN) {
        // no hall sensors detected
        MotorSensorMode = MOTOR_SENSOR_MODE_SENSORLESS;
        break;
    } else if (sensor_phase == SENSOR_PANIC) {
        MotorState = MOTOR_STATE_ERROR;
    } else {
        // ToDo add reporting
        // hall sensors detected
        MotorSensorMode = MOTOR_SENSOR_MODE_HALL;
    }
}

// TODO: refactor this function. Is this function called by DMA interrupt?
void V_I_Check() {  // &RawADC1,&RawADC2, &RawADC3 as arguments? Is this the
                    // correct use of &pointers? Just need it to look in the
                    // buffers filled by the DMA
    // Check currents, voltages are within panic limits
    if ((measurement_buffers.RawADC[0][0] > motor.RawCurrLim) |
        (measurement_buffers.RawADC[1][0] > motor.RawCurrLim) |
        (measurement_buffers.RawADC[2][0] > motor.RawCurrLim) |
        (measurement_buffers.RawADC[0][1] > motor.RawVoltLim)) {
        generateBreak();
        MotorState = ERROR;
        // fixme I think this is meant to be MOTOR_STATE_ERROR, not generic
        // system ERROR.
    }
}

void ADCConversion() {
    // Here we take the raw ADC values, offset, cast to (float) and use the
    // hardware gain values to create volt and amp variables
    // fixme: huh? where does "initing" come from? what does "initing" mean?
    // initialising? Can "initing" be passed as an argument to this function?
    // found it... sort of, but still don't understand.
    static int initing = 1000;

    if (initing > 0) {
        measurement_buffers.ADCOffset[0] =
            (255 * measurement_buffers.ADCOffset[0] +
             measurement_buffers.RawADC[0][0]) /
            256;
        measurement_buffers.ADCOffset[1] =
            (255 * measurement_buffers.ADCOffset[1] +
             measurement_buffers.RawADC[1][0]) /
            256;
        measurement_buffers.ADCOffset[2] =
            (255 * measurement_buffers.ADCOffset[2] +
             measurement_buffers.RawADC[2][0]) /
            256;
        initing--;
    } else {
        measurement_buffers.ConvertedADC[0][0] =
            ((float)measurement_buffers.RawADC[0][0] -
             (float)measurement_buffers.ADCOffset[0]) *
            g_hw_setup.Igain;  // Currents
        measurement_buffers.ConvertedADC[1][0] =
            ((float)measurement_buffers.RawADC[1][0] -
             (float)measurement_buffers.ADCOffset[1]) *
            g_hw_setup.Igain;
        measurement_buffers.ConvertedADC[2][0] =
            ((float)measurement_buffers.RawADC[2][0] -
             (float)measurement_buffers.ADCOffset[2]) *
            g_hw_setup.Igain;
        measurement_buffers.ConvertedADC[0][1] =
            (float)measurement_buffers.RawADC[0][1] *
            g_hw_setup.VBGain;  // Vbus
        measurement_buffers.ConvertedADC[0][2] =
            (float)measurement_buffers.RawADC[0][2] * g_hw_setup.VBGain;  // Usw
        measurement_buffers.ConvertedADC[1][1] =
            (float)measurement_buffers.RawADC[1][1] * g_hw_setup.VBGain;  // Vsw
        measurement_buffers.ConvertedADC[1][2] =
            (float)measurement_buffers.RawADC[1][2] * g_hw_setup.VBGain;  // Wsw
    }
}

void generateBreak() {
    // Here we set all the PWMoutputs to LOW, without triggering the timerBRK,
    // which should only be set by the hardware comparators, in the case of a
    // shoot-through orother catastrophic event This function means that the
    // timer can be left running, ADCs sampling etc which enables a recovery, or
    // single PWM period break in which the backEMF can be measured directly
    // This function needs implementing and testing before any high current or
    // voltage is applied, otherwise... DeadFETs
    phU_Break();
    phV_Break();
    phW_Break();
}

EHallSensorPhase_t getHallSensorPhase() {
    switch (((GPIOB->IDR >> 6) & 0x7))
    // switch(hallState)
    {
            // if all pins are high or all low then sensors are malfunctioning
            // or disconnected.
        case 0b000: /* fallthrough case statement */
        case 0b111:
            return (SENSOR_UNKNOWN);
            break;
            // below is the order in which sensor will commutate in forward
            // direction.
        case 0b001:
            return (SENSOR_PHASE0);
            break;
        case 0b011:
            return (SENSOR_PHASE1);
            break;
        case 0b010:
            return (SENSOR_PHASE2);
            break;
        case 0b110:
            return (SENSOR_PHASE3);
            break;
        case 0b100:
            return (SENSOR_PHASE4);
            break;
        case 0b101:
            return (SENSOR_PHASE5);
            break;
        default:
            // This should never happen, so if it does total panic mode. shut
            // down the motor.
            __HAL_TIM_MOE_DISABLE(&htim1);
            return (SENSOR_PANIC);
            break;
    }
}

// Every PWM cycle we enter this function until the resistance measurement has
// converged at a good value. Once the measurement is complete, Rphase is set,
// and this is no longer called
void measureResistance() {
    /*In this function, we are going to use the openloop PIFF controller to
     * create a current, probably 1A, through a pair of motor windings, keeping
     * the third tri-stated. We then generate a pair of V and I values, from the
     * bus voltage and duty cycle, and the current reading We repeat this at
     * higher current, say 5A, and then apply R=dV/dI from the two values to
     * generate a resistance. Don't use a single point, since this is subject to
     * anomolies from switching dead times, ADC sampling position...etc. Use of
     * the derivative eliminates all steady state error sources ToDo Repeat for
     * all phases? Or just assume they are all close enough that it doesn't
     * matter? Could be useful for disconnection detection...
     */
    static float currAcc2 = 0;  // codebase static?
    static float currAcc1 = 0;  // codebase static?

    ADCConversion();  // call the ADC conversion, which gives us float current
                      // values and voltages
    static uint16_t PWMcycles =
        0;  // codebase, this is going to initialise it as 0 once only and then
            // not reset it each time this is called right?
    if (isMotorRunning()) {
        // do nothing
    } else {
        // turn off phW, we are just going to measure RUV
        static uint16_t testPWM1 =
            2;  // ToDo MASSIVE HACK, completely uncontrolled value
                // "2",2/1024/37kHz--> 54ns pulse... Possibly won't even turn on
                // the FETs Assuming the device does respond, and ton==toff,
                // then approx. 48V*2/1024=0.09V. Typically
                // R=2(PCB)+4(2xFETs)+100(Rmotor)=106mOhm-->1A
        static uint16_t testPWM2 =
            10;  // ToDo EVEN BIGGER HACK...uncontrolled value "10" -->270ns
                 // pulse... Probably will turn the FETs on...
        phW_Break();
        phU_Enable();
        phV_Enable();
        if (PWMcycles < 1000) {
            htim1.Instance->CCR2 = 0;
            htim1.Instance->CCR1 = testPWM1;
            // Accumulate the currents with an exponential smoother. This
            // averaging should remove some noise and slightly increase
            // effective resolution
            currAcc1 =
                (99 * currAcc1 + measurement_buffers.ConvertedADC[1][0]) * 0.01;
        }

        else if (PWMcycles < 2000) {
            htim1.Instance->CCR2 = 0;
            htim1.Instance->CCR1 = testPWM2;
            // Accumulate the currents with an exponential smoother
            currAcc2 =
                (99 * currAcc2 + measurement_buffers.ConvertedADC[1][0]) * 0.01;
        } else if (PWMcycles == 2000) {
            // First let's just turn everything off. Nobody likes motors sitting
            // there getting hot while debugging.
            htim1.Instance->CCR2 = 0;
            htim1.Instance->CCR1 = 0;
            phU_Break();
            phV_Break();
            phW_Break();
            // calculate the resistance from two accumulated currents and two
            // voltages
            motor.Rphase = (((float)(testPWM2 - testPWM1)) *
                            measurement_buffers.ConvertedADC[0][1]) /
                           (currAcc2 - currAcc1);
        }
    }
}

void measureInductance() {
    /*
     * In this function, we are going to run at a fixed duty cycle (perhaps as
     * determined by Measure Resistance?), pushing ~5A through the motor coils
     * (~100ADCcounts). We will then wait until steady state achieved... 1000
     * PWM cycles? before modulating CCR4, which triggers the ADC to capture
     * currents at at least 2 time points within the PWM cycle With this change
     * in current, and knowing R from previous measurement, we can calculate L
     * using L=Vdt/dI=IRdt/dI ToDo Actually do this... ToDo Determination of the
     * direct and quadrature inductances for MTPA in future?
     */
}

/*fixme: this variable is not scope limited, so it is not temporary. It needs to
 * get a better name and be placed in a .h file. */
uint32_t tmpccmrx;  // Temporary buffer which is used to turn on/off phase PWMs
// Turn all phase U FETs off, Tristate the HBridge output - For BLDC mode
// mainly, but also used for measuring, software fault detection and recovery
// ToDo TEST THOROUGHLY The register manipulations for the break functions were
// used previously on an STM32F042K6 for my first BLDC drive, on TIM1, which
// should be identical, but definitely needs checking
void phU_Break() {
    tmpccmrx = htim1.Instance->CCMR1;
    tmpccmrx &= ~TIM_CCMR1_OC1M;
    tmpccmrx &= ~TIM_CCMR1_CC1S;
    tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
    htim1.Instance->CCMR1 = tmpccmrx;
    htim1.Instance->CCER &= ~TIM_CCER_CC1E;   // disable
    htim1.Instance->CCER &= ~TIM_CCER_CC1NE;  // disable
}
// Basically un-break phase U, opposite of above...
void phU_Enable() {
    tmpccmrx = htim1.Instance->CCMR1;
    tmpccmrx &= ~TIM_CCMR1_OC1M;
    tmpccmrx &= ~TIM_CCMR1_CC1S;
    tmpccmrx |= TIM_OCMODE_PWM1;
    htim1.Instance->CCMR1 = tmpccmrx;
    htim1.Instance->CCER |= TIM_CCER_CC1E;   // enable
    htim1.Instance->CCER |= TIM_CCER_CC1NE;  // enable
}

void phV_Break() {
    tmpccmrx = htim1.Instance->CCMR1;
    tmpccmrx &= ~TIM_CCMR1_OC2M;
    tmpccmrx &= ~TIM_CCMR1_CC2S;
    tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE << 8;
    htim1.Instance->CCMR1 = tmpccmrx;
    htim1.Instance->CCER &= ~TIM_CCER_CC2E;   // disable
    htim1.Instance->CCER &= ~TIM_CCER_CC2NE;  // disable
}

void phV_Enable() {
    tmpccmrx = htim1.Instance->CCMR1;
    tmpccmrx &= ~TIM_CCMR1_OC2M;
    tmpccmrx &= ~TIM_CCMR1_CC2S;
    tmpccmrx |= TIM_OCMODE_PWM1 << 8;
    htim1.Instance->CCMR1 = tmpccmrx;
    htim1.Instance->CCER |= TIM_CCER_CC2E;   // enable
    htim1.Instance->CCER |= TIM_CCER_CC2NE;  // enable
}

void phW_Break() {
    tmpccmrx = htim1.Instance->CCMR2;
    tmpccmrx &= ~TIM_CCMR2_OC3M;
    tmpccmrx &= ~TIM_CCMR2_CC3S;
    tmpccmrx |= TIM_OCMODE_FORCED_INACTIVE;
    htim1.Instance->CCMR2 = tmpccmrx;
    htim1.Instance->CCER &= ~TIM_CCER_CC3E;   // disable
    htim1.Instance->CCER &= ~TIM_CCER_CC3NE;  // disable
}

void phW_Enable() {
    tmpccmrx = htim1.Instance->CCMR2;
    tmpccmrx &= ~TIM_CCMR2_OC3M;
    tmpccmrx &= ~TIM_CCMR2_CC3S;
    tmpccmrx |= TIM_OCMODE_PWM1;
    htim1.Instance->CCMR2 = tmpccmrx;
    htim1.Instance->CCER |= TIM_CCER_CC3E;   // enable
    htim1.Instance->CCER |= TIM_CCER_CC3NE;  // enable
}
