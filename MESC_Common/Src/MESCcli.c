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

#define MESC_STM_FIXUP
#include "MESC_STM.h"

#include "MESCcli.h"
#include "MESCfnv.h"
#include "MESCprofile.h"

#include "bit_op.h"
#include "pp_op.h"

#include "TTerm.h"

#include <inttypes.h>
#include <limits.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define MAKE_TYPE_SIZE(type,size)      ((uint32_t)((uint32_t)((type) << BITS_PER_NYBBLE) | ((uint32_t)(size))))
#define MAKE_TYPE_SIZE_CASE(type,size) ((uint32_t)((uint32_t)((JOIN( CLI_VARIABLE_, type)) << BITS_PER_NYBBLE) | ((uint32_t)(size))))

enum CLIState
{
    CLI_STATE_IDLE,

    CLI_STATE_ABORT,

    CLI_STATE_COMMAND,
    CLI_STATE_VARIABLE,
    CLI_STATE_VALUE,

    CLI_STATE_PARAM_1,
    CLI_STATE_PARAM_2,

    CLI_STATE_EXECUTE,

    CLI_STATE_DATA,
};

typedef enum CLIState CLIState;

static CLIState cli_state;

enum CLICommand
{
    CLI_COMMAND_DECREASE = 'D',
    CLI_COMMAND_FLASH    = 'F',
    CLI_COMMAND_INCREASE = 'I',
    CLI_COMMAND_PROBE    = 'P',
    CLI_COMMAND_READ     = 'R',
    CLI_COMMAND_WRITE    = 'W',
    CLI_COMMAND_EXECUTE  = 'X',
};

typedef enum CLICommand CLICommand;

static CLICommand cli_cmd;

static bool     cli_hash_valid = false;
static uint32_t cli_hash = 0;

// Flash functions
static ProfileStatus cli_flash_write_noop( void const * buffer, uint32_t const address, uint32_t const length )
{
    (void)buffer;
    (void)address;
    (void)length;
    return PROFILE_STATUS_UNKNOWN;
}

static ProfileStatus (* cli_flash_write)( void const * buffer, uint32_t const address, uint32_t const length ) = &cli_flash_write_noop;


enum CLIAccess
{
    CLI_ACCESS_NONE = 0x0,

    CLI_ACCESS_R    = 0x1,
    CLI_ACCESS_W    = 0x2,
    CLI_ACCESS_X    = 0x4,

    CLI_ACCESS_RO   = CLI_ACCESS_R,
    CLI_ACCESS_WO   = CLI_ACCESS_W,
    CLI_ACCESS_RW   = (CLI_ACCESS_R | CLI_ACCESS_W),

    CLI_ACCESS_PROBE = 0x8,
};

typedef enum CLIAccess CLIAccess;

enum CLIVarState
{
    CLI_VAR_STATE_IDLE       = 0,
    // int
    CLI_VAR_STATE_INT        = 1,
    // uint
    CLI_VAR_STATE_UINT       = 1,
    // hex
    // ...
    // float
    CLI_VAR_STATE_FLOAT_SIGN = 1,
    CLI_VAR_STATE_FLOAT_INT  = 2,
    CLI_VAR_STATE_FLOAT_FRAC = 3,
    // ...
};

typedef enum CLIVarState CLIVarState;

struct CLIVar
{
    CLIVarState     state;
    union
    {
    int32_t         i;
    uint32_t        u;
    float           f;
    }               var;
};

typedef struct CLIVar CLIVar;

static CLIVar cli_var = {0};

struct CLIEntry
{
    uint32_t        hash;
    char * 			name;
    union
    {
    void       *    w;
    void const *    r;
    void (*         x)( void );
    }               var;
    uint32_t        size;
    CLIAccess       access;
    CLIVariableType type;
};

typedef struct CLIEntry CLIEntry;

#define MAX_CLI_LUT_ENTRIES UINT32_C(32)

struct CLIlut_s
{
	CLIEntry 		entries[MAX_CLI_LUT_ENTRIES];
	uint32_t   		entries_n;
	CLIEntry * 		entry_ptr;
};


typedef uint32_t (* CLIread_t)(TERMINAL_HANDLE * handle, CLIEntry * entry);
typedef uint32_t (* CLIwrite_t)(TERMINAL_HANDLE * handle, CLIEntry * entry, char * val);

typedef struct CLIlut_s CLIlut_s;

static CLIlut_s CLIlut;


static MESC_STM_ALIAS(int,UART_HandleTypeDef) cli_io_write_noop( MESC_STM_ALIAS(void,UART_HandleTypeDef) * handle, MESC_STM_ALIAS(void,uint8_t) * data, uint16_t size )
{
    (void)handle;
    (void)data;
    (void)size;

    return HAL_OK;
}

static void cli_io_read_noop( void )
{

}

static MESC_STM_ALIAS(void,UART_HandleTypeDef) *  cli_io_handle = NULL;
static MESC_STM_ALIAS(int ,HAL_StatusTypeDef ) (* cli_io_write)( MESC_STM_ALIAS(void,UART_HandleTypeDef) *, MESC_STM_ALIAS(void,uint8_t) *, uint16_t ) = cli_io_write_noop;
static void                                    (* cli_io_read )( void ) = cli_io_read_noop;

static void cli_idle( void )
{
    cli_state = CLI_STATE_IDLE;
    cli_hash_valid = false;
    cli_hash = 0;
    cli_var.state = CLI_VAR_STATE_IDLE;
    cli_var.var.u = 0;
}

static void cli_abort( void )
{
    cli_state = CLI_STATE_ABORT;
    cli_hash_valid = false;
    cli_hash = 0;
}

static void cli_noop_write( char const c )
{
    cli_abort();
    (void)c;
}

static void (* cli_process_write_value)( char const ) = cli_noop_write;

static void cli_noop_read( void )
{
    cli_abort();
}

static uint32_t cli_read_noop(TERMINAL_HANDLE * handle, CLIEntry * entry) {
   ttprintf("Error interpreting type\r\n");
   return TERM_CMD_EXIT_SUCCESS;
}

static uint32_t cli_write_noop(TERMINAL_HANDLE * handle, CLIEntry * entry, char * c_val) {
   ttprintf("Error interpreting type\r\n");
   return TERM_CMD_EXIT_SUCCESS;
}

static void (* cli_process_read_value)( void ) = cli_noop_read;

static void cli_execute( void )
{
    switch (cli_cmd)
    {
        case CLI_COMMAND_READ:
            cli_process_read_value();
            break;
        case CLI_COMMAND_WRITE:
            memcpy( CLIlut.entry_ptr->var.w, &cli_var.var, CLIlut.entry_ptr->size );
            break;
        case CLI_COMMAND_EXECUTE:
        	CLIlut.entry_ptr->var.x();
            break;
        case CLI_COMMAND_INCREASE:
        case CLI_COMMAND_DECREASE:
        {
            CLIVariableType const type = CLIlut.entry_ptr->type;
            uint32_t        const size = CLIlut.entry_ptr->size;
            union
            {
                int8_t   i8;
                uint8_t  u8;

                int16_t  i16;
                uint16_t u16;

                int32_t  i32;
                uint32_t u32;

                float    f32;
            } * tmp = CLIlut.entry_ptr->var.w;

            switch (MAKE_TYPE_SIZE( type, size ))
            {
                case MAKE_TYPE_SIZE_CASE(  INT, sizeof(int8_t) ):
                    if (cli_cmd == CLI_COMMAND_INCREASE)
                    {
                        tmp->i8 += (int8_t)cli_var.var.i;
                    }
                    else
                    {
                        tmp->i8 -= (int8_t)cli_var.var.i;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE(  INT, sizeof(int16_t) ):
                    if (cli_cmd == CLI_COMMAND_INCREASE)
                    {
                        tmp->i16 += (int16_t)cli_var.var.i;
                    }
                    else
                    {
                        tmp->i16 -= (int16_t)cli_var.var.i;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE(  INT, sizeof(int32_t) ):
                    if (cli_cmd == CLI_COMMAND_INCREASE)
                    {
                        tmp->i32 += (int32_t)cli_var.var.i;
                    }
                    else
                    {
                        tmp->i32 -= (int32_t)cli_var.var.i;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE( UINT, sizeof(uint8_t) ):
                    if (cli_cmd == CLI_COMMAND_INCREASE)
                    {
                        tmp->u8 += (uint8_t)cli_var.var.u;
                    }
                    else
                    {
                        tmp->u8 -= (uint8_t)cli_var.var.u;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE( UINT, sizeof(uint16_t) ):
                    if (cli_cmd == CLI_COMMAND_INCREASE)
                    {
                        tmp->u16 += (uint16_t)cli_var.var.u;
                    }
                    else
                    {
                        tmp->u16 -= (uint16_t)cli_var.var.u;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE( UINT, sizeof(uint32_t) ):
                    if (cli_cmd == CLI_COMMAND_INCREASE)
                    {
                        tmp->u32 += (uint32_t)cli_var.var.u;
                    }
                    else
                    {
                        tmp->u32 -= (uint32_t)cli_var.var.u;
                    }
                    break;
                case MAKE_TYPE_SIZE_CASE( FLOAT, sizeof(float) ):
                    if (cli_cmd == CLI_COMMAND_INCREASE)
                    {
                        tmp->f32 += (float)cli_var.var.f;
                    }
                    else
                    {
                        tmp->f32 -= (float)cli_var.var.f;
                    }
                    break;
                default:
                    return;
            }

            break;
        }
        case CLI_COMMAND_FLASH:
            break;
        default:
            // error
            break;
    }

    cli_idle();
}

static CLIEntry * cli_lut_alloc( char const * name )
{
    if (CLIlut.entries_n >= MAX_CLI_LUT_ENTRIES)
    {
        return NULL;
    }

    uint32_t const hash = fnv1a_str( name );

    for ( uint32_t i = 0; i < CLIlut.entries_n; ++i)
    {
        if (CLIlut.entries[i].hash == hash)
        {
            // ERROR
            return NULL;
        }
    }

    CLIEntry * entry = &CLIlut.entries[CLIlut.entries_n];
    entry->hash = hash;
    entry->name = name;

    return entry;
}


static uint32_t cli_read_int(TERMINAL_HANDLE * handle, CLIEntry * entry){
	if(entry == NULL) return TERM_CMD_EXIT_ERROR;

	if(entry->access == CLI_ACCESS_RW || entry->access == CLI_ACCESS_R || entry->access == CLI_ACCESS_RO){
		int32_t i32_val;
		switch(entry->size){
			case 1:
				i32_val = *(int8_t*)entry->var.w;
				break;
			case 2:
				i32_val = *(int16_t*)entry->var.w;
				break;
			case 4:
				i32_val = *(int32_t*)entry->var.w;
				break;
			default:
				return TERM_CMD_EXIT_ERROR;
		}
		ttprintf("%d\r\n",i32_val);
		return TERM_CMD_EXIT_SUCCESS;
	}else{
		return TERM_CMD_EXIT_ERROR;
	}
}

static uint32_t cli_read_uint(TERMINAL_HANDLE * handle, CLIEntry * entry){
	if(entry == NULL) return TERM_CMD_EXIT_ERROR;

	if(entry->access == CLI_ACCESS_RW || entry->access == CLI_ACCESS_R || entry->access == CLI_ACCESS_RO){
		uint32_t u32_val;
		switch(entry->size){
			case 1:
				u32_val = *(uint8_t*)entry->var.w;
				break;
			case 2:
				u32_val = *(uint16_t*)entry->var.w;
				break;
			case 4:
				u32_val = *(uint32_t*)entry->var.w;
				break;
			default:
				return TERM_CMD_EXIT_ERROR;
		}
		ttprintf("%u\r\n",u32_val);
		return TERM_CMD_EXIT_SUCCESS;
	}else{
		return TERM_CMD_EXIT_ERROR;
	}
}

static uint32_t cli_read_float(TERMINAL_HANDLE * handle, CLIEntry * entry){
	if(entry == NULL) return TERM_CMD_EXIT_ERROR;

	if(entry->access == CLI_ACCESS_RW || entry->access == CLI_ACCESS_R || entry->access == CLI_ACCESS_RO){
		float val;
		switch(entry->size){
			case 4:
				val = *(float*)entry->var.w;
				break;
			default:
				return TERM_CMD_EXIT_ERROR;
		}
		ttprintf("%f\r\n",val);
		return TERM_CMD_EXIT_SUCCESS;
	}else{
		return TERM_CMD_EXIT_ERROR;
	}
}

static uint32_t cli_write_int(TERMINAL_HANDLE * handle, CLIEntry * entry, char * c_val){
	if(entry == NULL) return TERM_CMD_EXIT_ERROR;

	if(entry->access == CLI_ACCESS_W || entry->access == CLI_ACCESS_WO || entry->access == CLI_ACCESS_RW){
		int32_t i32_val = strtol(c_val, NULL, 10);
		int16_t i16_val;
		int8_t i8_val;
		switch(entry->size){
			case 1:
				i8_val = i32_val;
				*(int8_t*)entry->var.w = i8_val;
				break;
			case 2:
				i16_val = i32_val;
				*(int16_t*)entry->var.w = i16_val;
				break;
			case 4:
				*(int32_t*)entry->var.w = i32_val;
				break;
			default:
				return TERM_CMD_EXIT_ERROR;
		}
		return TERM_CMD_EXIT_SUCCESS;
	}else{
		return TERM_CMD_EXIT_ERROR;
	}
}

static uint32_t cli_write_uint(TERMINAL_HANDLE * handle, CLIEntry * entry, char * c_val){
	if(entry->access == CLI_ACCESS_W || entry->access == CLI_ACCESS_WO || entry->access == CLI_ACCESS_RW){
		uint32_t ui32_val = strtoul(c_val, NULL, 10);
		uint16_t ui16_val;
		uint8_t ui8_val;
		switch(entry->size){
			case 1:
				ui8_val = ui32_val;
				*(uint8_t*)entry->var.w = ui8_val;
				break;
			case 2:
				ui16_val = ui32_val;
				*(uint16_t*)entry->var.w = ui16_val;
				break;
			case 4:
				*(uint32_t*)entry->var.w = ui32_val;
				break;
			default:
				return TERM_CMD_EXIT_ERROR;
		}
		return TERM_CMD_EXIT_SUCCESS;
	}else{
		return TERM_CMD_EXIT_ERROR;
	}
}

static uint32_t cli_write_float(TERMINAL_HANDLE * handle, CLIEntry * entry, char * c_val){
	if(entry->access == CLI_ACCESS_W || entry->access == CLI_ACCESS_WO || entry->access == CLI_ACCESS_RW){
		float f_val = atof(c_val);
		switch(entry->size){
			case 4:
				*(float*)entry->var.w = f_val;
				break;
			default:
				return TERM_CMD_EXIT_ERROR;
		}
		return TERM_CMD_EXIT_SUCCESS;
	}else{
		return TERM_CMD_EXIT_ERROR;
	}
}



void cli_configure_storage_io(
    ProfileStatus (* const write)( void const * buffer, uint32_t const address, uint32_t const length )
    )
{
    if (write != NULL)
    {
        cli_flash_write = write;
    }
}

void cli_register_variable_ro(
    char const * name,
    void const * address, uint32_t const size,
    CLIVariableType const type )
{
    CLIEntry * entry = cli_lut_alloc( name );

    if (entry != NULL)
    {
        entry->var.r  = address;
        entry->size   = size;
        entry->access = CLI_ACCESS_RO;
        entry->type   = type;

        CLIlut.entries_n++;
    }
}

void cli_register_variable_rw(
    char const * name,
    void       * address, uint32_t const size,
    CLIVariableType const type )
{
    CLIEntry * entry = cli_lut_alloc( name );

    if (entry != NULL)
    {
        entry->var.w  = address;
        entry->size   = size;
        entry->access = CLI_ACCESS_RW;
        entry->type   = type;

        CLIlut.entries_n++;
    }
}

void cli_register_variable_wo(
    char const * name,
    void       * address, uint32_t const size,
    CLIVariableType const type )
{
    CLIEntry * entry = cli_lut_alloc( name );

    if (entry != NULL)
    {
        entry->var.w  = address;
        entry->size   = size;
        entry->access = CLI_ACCESS_WO;
        entry->type   = type;

        CLIlut.entries_n++;
    }
}

void cli_register_function(
    char const * name,
    void (* const fn)( void ) )
{
    CLIEntry * entry = cli_lut_alloc( name );

    if (entry != NULL)
    {
        entry->var.x  = fn;
        entry->size   = 0;
        entry->access = CLI_ACCESS_X;
        entry->type   = (CLIVariableType)INT_MAX;

        CLIlut.entries_n++;
    }
}

void cli_register_io(
    MESC_STM_ALIAS(void,UART_HandleTypeDef) * handle,
    MESC_STM_ALIAS(int,HAL_StatusTypeDef) (* const write)( MESC_STM_ALIAS(void,UART_HandleTypeDef) * handle, MESC_STM_ALIAS(void,uint8_t) * data, uint16_t size ),
    void (* const read)( void ) )
{
    cli_io_handle = handle;
    cli_io_write  = write;
    cli_io_read   = read;
}

static uint32_t cli_lookup_index;

static CLIEntry * cli_lookup( uint32_t const hash )
{
    for ( cli_lookup_index = 0; cli_lookup_index < CLIlut.entries_n; ++cli_lookup_index )
    {
        if (CLIlut.entries[cli_lookup_index].hash == hash)
        {
            return &CLIlut.entries[cli_lookup_index];
        }
    }

    return NULL;
}


static CLIread_t cli_get_read_func(CLIVariableType const type){
    switch (type)
    {
        case CLI_VARIABLE_INT:
        	return cli_read_int;
        case CLI_VARIABLE_UINT:
            return cli_read_uint;
        case CLI_VARIABLE_FLOAT:
            return cli_read_float;
        default:
            return cli_read_noop;
    }
}

static CLIwrite_t cli_get_write_func(CLIVariableType const type){
    switch (type)
    {
        case CLI_VARIABLE_INT:
            return cli_write_int;
        case CLI_VARIABLE_UINT:
            return cli_write_uint;
        case CLI_VARIABLE_FLOAT:
            return cli_write_float;
        default:
            return cli_write_noop;
    }
}

static uint32_t byte_swap(uint32_t const value)
{
    return  (
                ((value & UINT32_C(0x000000FF)) << 24)
            |   ((value & UINT32_C(0x0000FF00)) <<  8)
            |   ((value & UINT32_C(0x00FF0000)) >>  8)
            |   ((value & UINT32_C(0xFF000000)) >> 24)
        )   ;
}


uint8_t cli_read(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
	if(argCount == 1){
		uint32_t hash = fnv1a_process_data(fnv1a_init(), args[0], strlen(args[0]));
		CLIEntry * entry = cli_lookup(hash);
		CLIread_t read_func = cli_get_read_func(entry->type);
		return read_func(handle, entry);
	}else{
		return TERM_CMD_EXIT_ERROR;
	}
}

uint8_t cli_write(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
	if(argCount == 2){
		uint32_t hash = fnv1a_process_data(fnv1a_init(), args[0], strlen(args[0]));
		CLIEntry * entry = cli_lookup(hash);
		CLIwrite_t write_func = cli_get_write_func(entry->type);
		return write_func(handle, entry, args[1]);
	}else{
		return TERM_CMD_EXIT_ERROR;
	}
}

uint8_t cli_list(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
	for ( uint32_t i = 0; i < CLIlut.entries_n; ++i){
		if(CLIlut.entries[i].name != NULL){
			ttprintf("%s\r\n", CLIlut.entries[i].name);
		}
	}
	return TERM_CMD_EXIT_SUCCESS;
}
