/*
 * This file is part of the stm32-template project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
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
 */

/* This file contains all parameters used in your project
 * See main.cpp on how to access them.
 * If a parameters unit is of format "0=Choice, 1=AnotherChoice" etc.
 * It will be displayed as a dropdown in the web interface
 * If it is a spot value, the decimal is translated to the name, i.e. 0 becomes "Choice"
 * If the enum values are powers of two, they will be displayed as flags, example
 * "0=None, 1=Flag1, 2=Flag2, 4=Flag3, 8=Flag4" and the value is 5.
 * It means that Flag1 and Flag3 are active -> Display "Flag1 | Flag3"
 *
 * Every parameter/value has a unique ID that must never change. This is used when loading parameters
 * from flash, so even across firmware versions saved parameters in flash can always be mapped
 * back to our list here. If a new value is added, it will receive its default value
 * because it will not be found in flash.
 * The unique ID is also used in the CAN module, to be able to recover the CAN map
 * no matter which firmware version saved it to flash.
 * Make sure to keep track of your ids and avoid duplicates. Also don't re-assign
 * IDs from deleted parameters because you will end up loading some random value
 * into your new parameter!
 * IDs are 16 bit, so 65535 is the maximum
 */

 //Define a version string of your firmware here
#define VER 1.00.R

/* Entries must be ordered as follows:
   1. Saveable parameters (id != 0)
   2. Temporary parameters (id = 0)
   3. Display values
 */
//Next param id (increase when adding new parameter!): 39
//Next value Id: 2005
/*              category     name         unit       min     max     default id */
#define PARAM_LIST \
    PARAM_ENTRY(CAT_LIN,     linbaud,  LINSPEED,  0,    1,     0,      9   ) \
    PARAM_ENTRY(CAT_LIN,     linrxid,    "",      0,   62,    24,     10   ) \
    PARAM_ENTRY(CAT_LIN,     lintxid,    "",      0,   62,    21,     11   ) \
	PARAM_ENTRY(CAT_LIN,     linsend0,   "",      0,   255,   0,      12   ) \
	PARAM_ENTRY(CAT_LIN,     linsend1,   "",      0,   255,   0,      13   ) \
	PARAM_ENTRY(CAT_LIN,     linsend2,   "",      0,   255,   0,      14   ) \
	PARAM_ENTRY(CAT_LIN,     linsend3,   "",      0,   255,   0,      15   ) \
	PARAM_ENTRY(CAT_LIN,     linsend4,   "",      0,   255,   0,      16   ) \
	PARAM_ENTRY(CAT_LIN,     linsend5,   "",      0,   255,   0,      17   ) \
	PARAM_ENTRY(CAT_LIN,     linsend6,   "",      0,   255,   0,      18   ) \
	PARAM_ENTRY(CAT_LIN,     linsend7,   "",      0,   255,   0,      19   ) \
	PARAM_ENTRY(CAT_LIN,	 linavail,   "",      0,     1,   0,      20   ) \
	VALUE_ENTRY(version,     VERSTR,  2001 ) \
	VALUE_ENTRY(lindata0,    "",      30   ) \
    VALUE_ENTRY(lindata1,    "",      31   ) \
	VALUE_ENTRY(lindata2,    "",      32   ) \
	VALUE_ENTRY(lindata3,    "",      33   ) \
	VALUE_ENTRY(lindata4,    "",      34   ) \
	VALUE_ENTRY(lindata5,    "",      35   ) \
	VALUE_ENTRY(lindata6,    "",      36   ) \
	VALUE_ENTRY(lindata7,    "",      37   ) \
    VALUE_ENTRY(lasterr,     errorListString,  2002 ) \
    VALUE_ENTRY(testain,     "dig",   2003 ) \
    VALUE_ENTRY(cpuload,     "%",     2004 )


/***** Enum String definitions *****/
#define OPMODES      "0=Off, 1=Run"
#define CAT_TEST     "Testing"
#define CAT_COMM     "Communication"
#define CAT_LIN      "Linbus"
#define LINSPEED     "0=9600, 1=19200"
#define VERSTR STRINGIFY(4=VER-Lin)

/***** enums ******/


enum _canperiods
{
   CAN_PERIOD_100MS = 0,
   CAN_PERIOD_10MS,
   CAN_PERIOD_LAST
};

enum _modes
{
   MOD_OFF = 0,
   MOD_RUN,
   MOD_LAST
};

//Generated enum-string for possible errors
extern const char* errorListString;

