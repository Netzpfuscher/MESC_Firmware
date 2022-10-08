/*
* Copyright 2021-2022 cod3b453
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "MESCuart.h"

#include "MESCcli.h"

#include "MESCfoc.h"
#include "MESCmotor_state.h"

#if MESC_UART_USB
#include "usbd_cdc_if.h"
#else
extern UART_HandleTypeDef HW_UART;
#endif


extern TIM_HandleTypeDef  htim1;

static uint8_t UART_rx_buffer[2];

extern uint8_t b_read_flash;

#if MESC_UART_USB
static void usb_ack( void )
{

}

HAL_StatusTypeDef HAL_USB_Transmit(UART_HandleTypeDef *husb, const uint8_t *pData, uint16_t Size){
	CDC_Transmit_FS((uint8_t*)pData, Size);
	return HAL_OK;
}

#else
static void uart_ack( void )
{
    // Required to allow next receive
    HAL_UART_Receive_IT( &HW_UART, UART_rx_buffer, 1 );
}
#endif



#ifndef RTOS
void USB_CDC_Callback(uint8_t *buffer, uint32_t len){

	for(int i = 0; i<len; i++){
		if (buffer[i] == '\r') // Treat CR...
		{
			//continue;
			buffer[i] = '\n'; // ...as LF
		}
		cli_process( buffer[i] );
	}

}
#endif


void uart_init( void )
{
	cli_register_variable_rw( "idq_req", &input_vars.Idq_req_UART.q, sizeof(input_vars.Idq_req_UART.q), CLI_VARIABLE_FLOAT );
    cli_register_variable_rw( "id"       , &foc_vars.Idq_req.d                   , sizeof(foc_vars.Idq_req.d                   ), CLI_VARIABLE_FLOAT );
    cli_register_variable_rw( "iq"        , &foc_vars.Idq_req.q                   , sizeof(foc_vars.Idq_req.q                   ), CLI_VARIABLE_FLOAT );
    cli_register_variable_ro( "vbus"      , &measurement_buffers.ConvertedADC[0][1], sizeof(measurement_buffers.ConvertedADC[0][1]), CLI_VARIABLE_FLOAT );

}
