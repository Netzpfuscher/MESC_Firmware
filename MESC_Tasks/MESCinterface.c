

#include "main.h"
#include "TTerm.h"
#include "MESCmotor_state.h"
#include "MESCcli.h"

uint8_t CMD_measure(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
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
	TERM_addCommand(CMD_measure, "measure", "Measure motor R+L", 0, &TERM_defaultList);
	TERM_addCommand(CMD_getkv, "getkv", "Measure motor kV", 0, &TERM_defaultList);
	TERM_addCommand(cli_read, "read", "Read variable", 0, &TERM_defaultList);
	TERM_addCommand(cli_write, "write", "Write variable", 0, &TERM_defaultList);
	TERM_addCommand(cli_list, "list", "List all variables", 0, &TERM_defaultList);
}
