/* 
 * Windows User Space AppleSMC Driver
 * applesmc-win32.c export header file
 * Copyright (C) 2015 Yudi Xie <xieyudi1990@gmail.com>
 * 
 * applesmc.c
 * Copyright (C) 2007 Nicolas Boichat <nicolas@boichat.ch>
 * Copyright (C) 2010 Henrik Rydberg <rydberg@euromail.se>
 *
 * Based on hdaps.c driver:
 * Copyright (C) 2005 Robert Love <rml@novell.com>
 * Copyright (C) 2005 Jesper Juhl <jj@chaosbits.net>
 *
 * Fan control based on smcFanControl:
 * Copyright (C) 2006 Hendrik Holtmann <holtmann@mac.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License v2 as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 */

#ifndef __APPLESMC_WIN32
#define __APPLESMC_WIN32

#include <stdbool.h>

typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;

typedef unsigned int __be32;

/* AppleSMC entry - cached register information */
struct applesmc_entry {
	char key[5];		/* four-letter key code */
	u8 valid;		/* set when entry is successfully read once */
	u8 len;			/* bounded by APPLESMC_MAX_DATA_LENGTH */
	char type[5];		/* four-letter type code */
	u8 flags;		/* 0x10: func; 0x40: write; 0x80: read */
};

struct applesmc_registers {
	unsigned int key_count;		/* number of SMC registers */
	unsigned int fan_count;		/* number of fans */
	unsigned int temp_count;	/* number of temperature registers */
	unsigned int temp_begin;	/* temperature lower index bound */
	unsigned int temp_end;		/* temperature upper index bound */
	unsigned int index_count;	/* size of temperature index array */
	bool init_complete;		/* true when fully initialized */
	struct applesmc_entry *cache;	/* cached key entries */
	const char **index;		/* temperature key index */
};

extern struct applesmc_registers smcreg;

extern int applesmc_init_smcreg(void);
extern void applesmc_destroy_smcreg(void);
extern int applesmc_show_sensor_label(int index, char *buf);
extern int applesmc_show_temperature(int index, char *buf);
extern int applesmc_show_fan_speed(int index, int option, char *buf);
extern int applesmc_store_fan_manual(int index, int manual);
extern int applesmc_store_fan_speed(int index, unsigned long speed);

#endif