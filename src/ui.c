/* 
 * Windows User Space AppleSMC Driver
 * A sloppily made user interface
 * Copyright (C) 2015 Yudi Xie <xieyudi1990@gmail.com>
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

#include <stdio.h>
#include <string.h>
#include <signal.h>
#include <Windows.h>
#include "applesmc-win32.h"

/* 
 * sensor monitor interface
 */
static int ui_monitor()
{
	COORD pos = {0, 0};
	HANDLE hdl = GetStdHandle(STD_OUTPUT_HANDLE);

	int i;
	int ret;

	char buf[16];
	int speed_act, speed_min, speed_max;
	int temp;
	int toggle = 0;
	
	ret = applesmc_init_smcreg();
	if (ret)
		return -1;
	
	system("cls");
	
	while (1)
	{
		pos.Y = 0;
		SetConsoleCursorPosition(hdl,pos);
		
		/* print fan speed */
		for (i = 0; i < smcreg.fan_count; i++)
		{
			speed_act = applesmc_show_fan_speed(i, 0, NULL);
			speed_min = applesmc_show_fan_speed(i, 1, NULL);
			speed_max = applesmc_show_fan_speed(i, 2, NULL);
			printf("fan%d:%12d RPM  (min = %8d RPM, max = %8d RPM)", i, speed_act, speed_min, speed_max);
			if (toggle)
				printf("*\n");
			else
				printf(" \n");
		}
		
		/* print temperature */
		for (i = 0; i < smcreg.temp_count; i++)
		{
			applesmc_show_sensor_label(i, buf);
			if (!strcmp(buf, "(null)")) {
				printf("                               \n");
			} else {
				temp = applesmc_show_temperature(i, NULL);
				printf("%s:%12.1fC", buf, (float)(temp/1000));
				if (toggle)
					printf("*                              \n");
				else
					printf("                               \n");
			}
		}
		
		toggle ^= 1;
		Sleep(1000);
	}
	
	CloseHandle(hdl);
	applesmc_destroy_smcreg();
	
	return 0;
}

/* 
 * set fan speed manually interface
 * the maximum speed is 6500, because in my case though it's above F%dMx, but actually this value is OK for me
 * one can try to increase this limit to whatever value, but don't push it too hard
 */
static int ui_fanspeed(int num, int manual, unsigned long const speed[])
{
	int i;
	int ret;
	
	ret = applesmc_init_smcreg();
	if (ret)
		return -1;
	
	if (manual && num != smcreg.fan_count) {
		printf("wrong number of fan count, expecting %d\n", smcreg.fan_count);
		return -1;
	}
	
	for (i = 0; i < smcreg.fan_count; i++) {
		if (manual) {
			/* posing a limit to speed of 6500 */
			if (speed[i] > 6500) continue;

			printf("trying to set the speed of fan%d to %d\n", i, speed[i]);
			applesmc_store_fan_manual(i, 1);
			applesmc_store_fan_speed(i, speed[i]);
		} else {
			printf("trying to put fan%d to auto\n", i);
			applesmc_store_fan_manual(i, 0);
		}
	}
	
	applesmc_destroy_smcreg();
	return 0;
}

int main(int argc, char *argv[])
{
	int i;
	int	ret;
	int num;
	unsigned long *speed;
	
	puts("!!! use at your own risk !!!");
	puts("Try several times in case of not working");
	
	if (argc < 2)
		goto help;
	
	switch (argv[1][0]) {
	case 'm':	/* monitor */
		ui_monitor();
		break;
	case 'f':	/* set fan speed */
		if (argc < 3)
			goto help;
	
		if (argv[2][0] == 'y') {
			if (argc < 4)
				goto help;
			
			ret = sscanf(argv[3], "%d", &num);
			if (EOF == ret)
				goto help;
			if (num + 4 != argc)
				goto help;
			
			speed = malloc(num * sizeof(unsigned long));
			if (!speed)
				goto error;
			
			for (i = 0; i < num; i++) {
			ret = sscanf(argv[i+4], "%d", &speed[i]);
			if (EOF == ret)
				goto help;
			}
			
			ret = ui_fanspeed(num, 1, speed);
			if (ret)
				goto error;
			
			free(speed);
		}
		else if (argv[2][0] == 'n') {
			ret = ui_fanspeed(num, 0, speed);
			if (ret)
				goto error;
		}
		else
			goto help;
		
		break;
	default:
		goto help;
		break;
	}

	return 0;

help:
	printf(	"usage: applesmc-win32 [m|f ...]\n"
			"  m\t\t\t\t\t\tmonitor sensors\n" 
			"  f <y|n> <num> <s0> ... <sn> ... <snum-1>\tset fan speed manually\n"
			"    <y|n>\t\t\t\t\t" "y=manual, n=auto\n"
			"    <num>\t\t\t\t\t" "number of fans installed\n"
			"    <sn>\t\t\t\t\t" "speed of fan n, n in the range of 0 to num-1\n"
			"\n"
			"  example:\n"
			"    I have 2 fans, and I want to set them to manual, with speed of 5500 and 6000 respectively:\n"
			"      \"applesmc-win32.exe f y 2 5500 6000\"\n"
			"    to revert back to auto (default):\n"
			"      \"applesmc-win32.exe f n\"\n");
	return -1;
	
error:
	puts("error");
	return -1;
}

