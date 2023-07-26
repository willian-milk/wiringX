/*
  Copyright (c) 2023 MilkV Ltd.
  Author: Willian <willian@milkv.io>

  This Source Code Form is subject to the terms of the Mozilla Public
  License, v. 2.0. If a copy of the MPL was not distributed with this
  file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#include <sys/mman.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <signal.h>

#include "../../soc/soc.h"
#include "../../wiringx.h"
#include "../platform.h"
#include "duo.h"

struct platform_t *duo = NULL;

static int map[] = {
	/*	NULL	XGPIOA[28] XGPIOA[29] GND	*/
	-1,	0,	1,	-1,
	/*	PWR_GPIO[26] PWR_GPIO[25] PWR_GPIO[19] PWR_GPIO[20]	*/
	2,	3,	5,	4,
	/*	GND	PWR_GPIO[23]	PWR_GPIO[22]	PWR_GPIO[21]	*/
	-1,	6,	7,	8,
	/*	PWR_GPIO[18]	GND	XGPIOC[9] XGPIOC[10]	*/
	9,	-1,	10,	11,
	/*	XGPIOA[16] XGPIOA[17]	GND	XGPIOA[14]	*/
	12,	13,	-1,	14,
	/*	XGPIOA[15]	XGPIOA[23]	XGPIOA[24]	GND	*/
	15,	16,	17,	-1,
	/*	XGPIOA[22]	XGPIOA[25]	XGPIOA[27]	XGPIOA[26]	*/
	18,	19,	20,	21,
	/*	GND	PWR_GPIO[4]	VCC	XGPIOB[3]	*/
	-1,	22,	-1,	23,
};

#define _sizeof(arr) (sizeof(arr) / sizeof(arr[0]))

static int duoValidGPIO(int pin) {
	if(pin >= 0 && pin < _sizeof(map)) {
		if(map[pin] == -1) {
			return -1;
		}
		return 0;
	} else {
		return -1;
	}
}

static int duoSetup(void) {
	duo->soc->setup();
	duo->soc->setMap(map, _sizeof(map));
	duo->soc->setIRQ(map, _sizeof(map));
	return 0;
}

void milkv_duoInit(void) {
	platform_register(&duo, "duo");

	duo->soc = soc_get("Sophgo", "CV180");
	duo->soc->setMap(map, _sizeof(map));

	duo->digitalRead = duo->soc->digitalRead;
	duo->digitalWrite = duo->soc->digitalWrite;
	duo->pinMode = duo->soc->pinMode;
	duo->setup = &duoSetup;

	duo->isr = duo->soc->isr;
	duo->waitForInterrupt = duo->soc->waitForInterrupt;

	duo->selectableFd = duo->soc->selectableFd;
	duo->gc = duo->soc->gc;

	duo->validGPIO = &duoValidGPIO;
}
