/*
  Copyright (c) 2024 Shenzhen Milk-V Technology Co., Ltd
  Author: William James <willian@milkv.io>

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
#include "meles.h"

struct platform_t *meles = NULL;

static int map[] = {
	/*	GPIO0_17	I2S2_SCLK	GPIO2_25	GPIO2_24	*/
			17,			-1,			88,			87,
	/*	GPIO2_22	GPIO2_31	GPIO3_2		GPIO0_16	*/
			85,			94,			97,			16,
	/*	GPIO0_9		GPIO0_8		GPIO2_15	ADC			*/
			9,			8,			78,			-1,
	/*	GPIO2_16	GPIO2_17	GPIO2_14	UART0_TXD	*/
			79,			80,			77,			63,
	/*	UART0_TXD		17		18		19	*/
			64,			-1,			-1,			-1,
	/*	20,				GPIO2_23	GPIO3_1		GPIO0_11	*/
			-1,			86,			96,			11,
	/*	I2S2-LRCK	GPIO3_0		GPIO0_10	GPIO3_3	*/
			-1,			95,			10,			98,
	/*	I2S_DIN		I2S2_DOUT	GPIO2_12	GPIO2_11	*/
			-1,			-1,			75,			74
};

#define _sizeof(arr) (sizeof(arr) / sizeof(arr[0]))

static int melesValidGPIO(int pin) {
	if(pin >= 0 && pin < _sizeof(map)) {
		if(map[pin] == -1) {
			return -1;
		}
		return 0;
	} else {
		return -1;
	}
}

static int melesSetup(void) {
	meles->soc->setup();
	meles->soc->setMap(map, _sizeof(map));
	meles->soc->setIRQ(map, _sizeof(map));
	return 0;
}

void melesInit(void) {
	platform_register(&meles, "meles");

	meles->soc = soc_get("Thead", "TH1520");
	meles->soc->setMap(map, _sizeof(map));

	meles->digitalRead = meles->soc->digitalRead;
	meles->digitalWrite = meles->soc->digitalWrite;
	meles->pinMode = meles->soc->pinMode;
	meles->setup = &melesSetup;

	meles->isr = meles->soc->isr;
	meles->waitForInterrupt = meles->soc->waitForInterrupt;

	meles->selectableFd = meles->soc->selectableFd;
	meles->gc = meles->soc->gc;

	meles->validGPIO = &melesValidGPIO;
}
