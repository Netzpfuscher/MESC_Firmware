

#include "main.h"
#include "TTerm.h"
#include "MESCmotor_state.h"

uint8_t CMD_measure(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    MotorState = MOTOR_STATE_MEASURING;
    return TERM_CMD_EXIT_SUCCESS;
}

uint8_t CMD_getkv(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
    MotorState = MOTOR_STATE_GET_KV;
    return TERM_CMD_EXIT_SUCCESS;
}


void MESCinterface_init(void){
	TERM_addCommand(CMD_measure, "measure", "Measure motor R+L", 0, &TERM_defaultList);
	TERM_addCommand(CMD_getkv, "getkv", "Measure motor kV", 0, &TERM_defaultList);

}
