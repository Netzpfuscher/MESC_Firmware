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
	CLI_ACCESS_FLASH = 0x16,
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

struct CLIEntry
{
    uint32_t        hash;
    const char * 			name;
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


typedef uint32_t (* CLIread_t)(TERMINAL_HANDLE * handle, CLIEntry * entry, bool newline);
typedef uint32_t (* CLIwrite_t)(TERMINAL_HANDLE * handle, CLIEntry * entry, char * val);

typedef struct CLIlut_s CLIlut_s;

static CLIlut_s CLIlut;


static uint32_t cli_read_noop(TERMINAL_HANDLE * handle, CLIEntry * entry, bool newline) {
   ttprintf("Error interpreting type\r\n");
   return TERM_CMD_EXIT_SUCCESS;
}

static uint32_t cli_write_noop(TERMINAL_HANDLE * handle, CLIEntry * entry, char * c_val) {
   ttprintf("Error interpreting type\r\n");
   return TERM_CMD_EXIT_SUCCESS;
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


static uint32_t cli_read_int(TERMINAL_HANDLE * handle, CLIEntry * entry, bool newline){
	if(entry == NULL) return TERM_CMD_EXIT_ERROR;

	if(entry->access & CLI_ACCESS_R){
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

static uint32_t cli_read_uint(TERMINAL_HANDLE * handle, CLIEntry * entry, bool newline){
	if(entry == NULL) return TERM_CMD_EXIT_ERROR;

	if(entry->access & CLI_ACCESS_R){
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

static uint32_t cli_read_float(TERMINAL_HANDLE * handle, CLIEntry * entry, bool newline){
	if(entry == NULL) return TERM_CMD_EXIT_ERROR;

	if(entry->access & CLI_ACCESS_R){
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

	if(entry->access & CLI_ACCESS_W){
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
	if(entry->access & CLI_ACCESS_W){
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
	if(entry->access & CLI_ACCESS_W){
		float f_val = strtof(c_val, NULL);
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

uint8_t cli_read(TERMINAL_HANDLE * handle, uint8_t argCount, char ** args){
	if(argCount == 1){
		uint32_t hash = fnv1a_process_data(fnv1a_init(), args[0], strlen(args[0]));
		CLIEntry * entry = cli_lookup(hash);
		CLIread_t read_func = cli_get_read_func(entry->type);
		return read_func(handle, entry, true);
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
			if(argCount==1){
				if(args[0][0] != CLIlut.entries[i].name[0]) continue;
			}
			char R = CLIlut.entries[i].access & CLI_ACCESS_R ? 'r' : '-';
			char W = CLIlut.entries[i].access & CLI_ACCESS_W ? 'w' : '-';
			char X = CLIlut.entries[i].access & CLI_ACCESS_X ? 'x' : '-';
			ttprintf("%c%c%c    %s = ", R, W, X, CLIlut.entries[i].name);
			CLIread_t read_func = cli_get_read_func(CLIlut.entries[i].type);
			read_func(handle, &CLIlut.entries[i], true);
		}
	}
	return TERM_CMD_EXIT_SUCCESS;
}
