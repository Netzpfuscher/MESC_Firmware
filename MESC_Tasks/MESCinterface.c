

#include "main.h"
#include "TTerm.h"
#include "MESCmotor_state.h"
#include "MESCcli.h"

#include <stdlib.h>

uint8_t CMD_measure(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
	motor.measure_current = I_MEASURE;
	motor.measure_voltage = V_MEASURE;

	if(argCount==2){
		motor.measure_current = atoff(args[0]);
		motor.measure_voltage = atoff(args[1]);
	}

    MotorState = MOTOR_STATE_MEASURING;
    ttprintf("Waiting for result");

    while(MotorState == MOTOR_STATE_MEASURING){
    	vTaskDelay(200);
    	ttprintf(".");
    }

    ttprintf("\r\nResult:\r\n");

    float R, Lq, Ld;
    char* Runit;
    char* Lunit;
    if(motor.Rphase > 0){
    	R = motor.Rphase;
    	Runit = "Ohm";
    }else{
    	R = motor.Rphase*1000.0;
    	Runit = "mOhm";
    }
    if(motor.Lphase > 0.001){
		Ld = motor.Lphase*1000.0;
		Lq = motor.Lqphase*1000.0;
		Lunit = "mH";
	}else{
		Ld = motor.Lphase*1000.0*1000.0;
		Lq = motor.Lqphase*1000.0*1000.0;
		Lunit = "uH";
	}

    ttprintf("R = %f %s\r\nLd = %f %s\r\nLq = %f %s\r\n", R, Runit, Ld, Lunit, Lq, Lunit);

    return TERM_CMD_EXIT_SUCCESS;
}

uint8_t CMD_getkv(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
	motor.measure_current = I_MEASURE;
	motor.measure_voltage = V_MEASURE;

	if(argCount==1){
		motor.measure_current = atoff(args[0]);
	}

    MotorState = MOTOR_STATE_GET_KV;
    ttprintf("Waiting for result");

    while(MotorState == MOTOR_STATE_GET_KV){
    	vTaskDelay(200);
		ttprintf(".");
	}

	ttprintf("\r\nResult:\r\n");

    ttprintf("Flux = %f mWb\r\n", motor.motor_flux * 1000.0);

    return TERM_CMD_EXIT_SUCCESS;
}


void MESCinterface_init(void){
	cli_register_var_rw("idq_req" , input_vars.Idq_req_UART.q);
    cli_register_var_rw( "id"     , foc_vars.Idq_req.d);
    cli_register_var_rw( "iq"     , foc_vars.Idq_req.q);
    cli_register_var_ro( "vbus"   , measurement_buffers.ConvertedADC[0][1]);
    cli_register_var_rw( "ld"     , motor.Lphase);
    cli_register_var_rw( "lq"     , motor.Lqphase);
    cli_register_var_rw( "r"      , motor.Rphase);


	TERM_addCommand(CMD_measure, "measure", "Measure motor R+L", 0, &TERM_defaultList);
	TERM_addCommand(CMD_getkv, "getkv", "Measure motor kV", 0, &TERM_defaultList);
	TERM_addCommand(cli_read, "read", "Read variable", 0, &TERM_defaultList);
	TERM_addCommand(cli_write, "write", "Write variable", 0, &TERM_defaultList);
	TERM_addCommand(cli_list, "list", "List all variables", 0, &TERM_defaultList);
}
