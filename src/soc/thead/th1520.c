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
#include <ctype.h>

#include "th1520.h"
#include "../../wiringx.h"
#include "../soc.h"
#include "../../i2c-dev.h"

#define GPIO_GROUP_COUNT 4
#define PAD_GROUP_COUNT 4
static uintptr_t pad_groups_reg[MAX_REG_AREA] = {0x0};
const static uintptr_t gpio_group_register_physical_address[MAX_REG_AREA] = {0xffec005000, 0xffec006000, 0xffe7f34000, 0xffe7f38000};	// gpio base address
const static uintptr_t pad_groups_register_physical_address[MAX_REG_AREA] = {0xfffff4a000, 0xffe7f3c000, 0xffec007000, 0xffcb01d000};	// pinmux base address
#define GPIO_SWPORTA_DR			0x0000	// gpio data register offset
#define GPIO_SWPORTA_DDR		0x0004	// gpio direction control register offset
#define GPIO_EXT_PORTA		0x0050	// gpio data read register offset
#define GPIO_UNAVAILABLE(x) {(x), 0, 0, {0, 0, 0}, {0, 0}, {0, 0}, {0, 0}, FUNCTION_UNKNOWN, 0, 0}

// pad group 2
#define G2_MUXCFG_001			0x400
#define G2_MUXCFG_002			0x404
#define G2_MUXCFG_003			0x408
#define G2_MUXCFG_004			0x40c
#define G2_MUXCFG_005			0x410
#define G2_MUXCFG_006			0x414
#define G2_MUXCFG_007			0x418
#define G2_MUXCFG_008			0x41c

// pad group 3
#define G3_MUXCFG_001			0x400
#define G3_MUXCFG_002			0x404
#define G3_MUXCFG_003			0x408
#define G3_MUXCFG_004			0x40c
#define G3_MUXCFG_005			0x410
#define G3_MUXCFG_006			0x414
#define G3_MUXCFG_007			0x418

struct soc_t *th1520 = NULL;

static struct layout_t {
	char *name;
	int pad_num;
	int bank;

	struct {
		unsigned long offset;
		unsigned long bit;
		unsigned long value;
	} pad;
	struct {
		unsigned long offset;
		unsigned long bit;
	} direction;
	struct {
		unsigned long offset;
		unsigned long bit;
	} out;
	struct {
		unsigned long offset;
		unsigned long bit;
	} in;

	int support;
	enum pinmode_t mode;
	int fd;
} layout[] = {
	{"GPIO0_0", 1, 0, {G2_MUXCFG_001, 0, 0x03}, {GPIO_SWPORTA_DDR, 0}, {GPIO_SWPORTA_DR, 0}, {GPIO_EXT_PORTA, 0}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_1", 1, 0, {G2_MUXCFG_001, 4, 0x03}, {GPIO_SWPORTA_DDR, 1}, {GPIO_SWPORTA_DR, 1}, {GPIO_EXT_PORTA, 1}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_2", 1, 0, {G2_MUXCFG_001, 8, 0x03}, {GPIO_SWPORTA_DDR, 2}, {GPIO_SWPORTA_DR, 2}, {GPIO_EXT_PORTA, 2}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_3", 1, 0, {G2_MUXCFG_001, 12, 0x03}, {GPIO_SWPORTA_DDR, 3}, {GPIO_SWPORTA_DR, 3}, {GPIO_EXT_PORTA, 3}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_4", 1, 0, {G2_MUXCFG_001, 16, 0x03}, {GPIO_SWPORTA_DDR, 4}, {GPIO_SWPORTA_DR, 4}, {GPIO_EXT_PORTA, 4}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_5", 1, 0, {G2_MUXCFG_001, 20, 0x03}, {GPIO_SWPORTA_DDR, 5}, {GPIO_SWPORTA_DR, 5}, {GPIO_EXT_PORTA, 5}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_6", 1, 0, {G2_MUXCFG_001, 24, 0x03}, {GPIO_SWPORTA_DDR, 6}, {GPIO_SWPORTA_DR, 6}, {GPIO_EXT_PORTA, 6}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_7", 1, 0, {G2_MUXCFG_001, 28, 0x03}, {GPIO_SWPORTA_DDR, 7}, {GPIO_SWPORTA_DR, 7}, {GPIO_EXT_PORTA, 7}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_8", 1, 0, {G2_MUXCFG_002, 0, 0x03}, {GPIO_SWPORTA_DDR, 8}, {GPIO_SWPORTA_DR, 8}, {GPIO_EXT_PORTA, 8}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_9", 1, 0, {G2_MUXCFG_002, 4, 0x03}, {GPIO_SWPORTA_DDR, 9}, {GPIO_SWPORTA_DR, 9}, {GPIO_EXT_PORTA, 9}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_10", 1, 0, {G2_MUXCFG_002, 8, 0x03}, {GPIO_SWPORTA_DDR, 10}, {GPIO_SWPORTA_DR, 10}, {GPIO_EXT_PORTA, 10}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_11", 1, 0, {G2_MUXCFG_002, 12, 0x03}, {GPIO_SWPORTA_DDR, 11}, {GPIO_SWPORTA_DR, 11}, {GPIO_EXT_PORTA, 11}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_12", 1, 0, {G2_MUXCFG_002, 16, 0x03}, {GPIO_SWPORTA_DDR, 12}, {GPIO_SWPORTA_DR, 12}, {GPIO_EXT_PORTA, 12}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_13", 1, 0, {G2_MUXCFG_002, 20, 0x03}, {GPIO_SWPORTA_DDR, 13}, {GPIO_SWPORTA_DR, 13}, {GPIO_EXT_PORTA, 13}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_14", 1, 0, {G2_MUXCFG_002, 24, 0x03}, {GPIO_SWPORTA_DDR, 14}, {GPIO_SWPORTA_DR, 14}, {GPIO_EXT_PORTA, 14}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_15", 1, 0, {G2_MUXCFG_006, 28, 0x03}, {GPIO_SWPORTA_DDR, 15}, {GPIO_SWPORTA_DR, 15}, {GPIO_EXT_PORTA, 15}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_16", 1, 0, {G2_MUXCFG_003, 0, 0x03}, {GPIO_SWPORTA_DDR, 16}, {GPIO_SWPORTA_DR, 16}, {GPIO_EXT_PORTA, 16}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_17", 1, 0, {G2_MUXCFG_003, 4, 0x03}, {GPIO_SWPORTA_DDR, 17}, {GPIO_SWPORTA_DR, 17}, {GPIO_EXT_PORTA, 17}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_18", 1, 0, {G2_MUXCFG_003, 8, 0x00}, {GPIO_SWPORTA_DDR, 18}, {GPIO_SWPORTA_DR, 18}, {GPIO_EXT_PORTA, 18}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_19", 1, 0, {G2_MUXCFG_003, 12, 0x00}, {GPIO_SWPORTA_DDR, 19}, {GPIO_SWPORTA_DR, 19}, {GPIO_EXT_PORTA, 19}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_20", 1, 0, {G2_MUXCFG_003, 16, 0x00}, {GPIO_SWPORTA_DDR, 20}, {GPIO_SWPORTA_DR, 20}, {GPIO_EXT_PORTA, 20}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_21", 1, 0, {G2_MUXCFG_003, 20, 0x00}, {GPIO_SWPORTA_DDR, 21}, {GPIO_SWPORTA_DR, 21}, {GPIO_EXT_PORTA, 21}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_22", 1, 0, {G2_MUXCFG_003, 24, 0x00}, {GPIO_SWPORTA_DDR, 22}, {GPIO_SWPORTA_DR, 22}, {GPIO_EXT_PORTA, 22}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_23", 1, 0, {G2_MUXCFG_003, 28, 0x00}, {GPIO_SWPORTA_DDR, 23}, {GPIO_SWPORTA_DR, 23}, {GPIO_EXT_PORTA, 23}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_24", 1, 0, {G2_MUXCFG_004, 0, 0x00}, {GPIO_SWPORTA_DDR,  24}, {GPIO_SWPORTA_DR, 24}, {GPIO_EXT_PORTA, 24}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_25", 1, 0, {G2_MUXCFG_004, 4, 0x00}, {GPIO_SWPORTA_DDR,  25}, {GPIO_SWPORTA_DR, 25}, {GPIO_EXT_PORTA, 25}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_26", 1, 0, {G2_MUXCFG_004, 8, 0x00}, {GPIO_SWPORTA_DDR,  26}, {GPIO_SWPORTA_DR, 26}, {GPIO_EXT_PORTA, 26}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_27", 1, 0, {G2_MUXCFG_004, 12, 0x00}, {GPIO_SWPORTA_DDR, 27}, {GPIO_SWPORTA_DR, 27}, {GPIO_EXT_PORTA, 27}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_28", 1, 0, {G2_MUXCFG_004, 16, 0x00}, {GPIO_SWPORTA_DDR, 28}, {GPIO_SWPORTA_DR, 28}, {GPIO_EXT_PORTA, 28}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_29", 1, 0, {G2_MUXCFG_004, 20, 0x00}, {GPIO_SWPORTA_DDR, 29}, {GPIO_SWPORTA_DR, 29}, {GPIO_EXT_PORTA, 29}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_30", 1, 0, {G2_MUXCFG_004, 24, 0x00}, {GPIO_SWPORTA_DDR, 30}, {GPIO_SWPORTA_DR, 30}, {GPIO_EXT_PORTA, 30}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO0_31", 1, 0, {G2_MUXCFG_004, 28, 0x00}, {GPIO_SWPORTA_DDR, 31}, {GPIO_SWPORTA_DR, 31}, {GPIO_EXT_PORTA, 31}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_0", 1, 1, {G2_MUXCFG_005, 0, 0x00}, {GPIO_SWPORTA_DDR, 0}, {GPIO_SWPORTA_DR, 0}, {GPIO_EXT_PORTA, 0}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_1", 1, 1, {G2_MUXCFG_005, 4, 0x00}, {GPIO_SWPORTA_DDR, 1}, {GPIO_SWPORTA_DR, 1}, {GPIO_EXT_PORTA, 1}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_2", 1, 1, {G2_MUXCFG_005, 8, 0x00}, {GPIO_SWPORTA_DDR, 2}, {GPIO_SWPORTA_DR, 2}, {GPIO_EXT_PORTA, 2}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_3", 1, 1, {G2_MUXCFG_005, 12, 0x00}, {GPIO_SWPORTA_DDR, 3}, {GPIO_SWPORTA_DR, 3}, {GPIO_EXT_PORTA, 3}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_4", 1, 1, {G2_MUXCFG_005, 16, 0x00}, {GPIO_SWPORTA_DDR, 4}, {GPIO_SWPORTA_DR, 4}, {GPIO_EXT_PORTA, 4}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_5", 1, 1, {G2_MUXCFG_005, 20, 0x00}, {GPIO_SWPORTA_DDR, 5}, {GPIO_SWPORTA_DR, 5}, {GPIO_EXT_PORTA, 5}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_6", 1, 1, {G2_MUXCFG_005, 24, 0x00}, {GPIO_SWPORTA_DDR, 6}, {GPIO_SWPORTA_DR, 6}, {GPIO_EXT_PORTA, 6}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_7", 1, 1, {G2_MUXCFG_005, 28, 0x00}, {GPIO_SWPORTA_DDR, 7}, {GPIO_SWPORTA_DR, 7}, {GPIO_EXT_PORTA, 7}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_8", 1, 1, {G2_MUXCFG_006, 0, 0x00}, {GPIO_SWPORTA_DDR, 8}, {GPIO_SWPORTA_DR, 8}, {GPIO_EXT_PORTA, 8}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_9", 1, 1, {G2_MUXCFG_006, 4, 0x00}, {GPIO_SWPORTA_DDR, 9}, {GPIO_SWPORTA_DR, 9}, {GPIO_EXT_PORTA, 9}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_10", 1, 1, {G2_MUXCFG_006, 8, 0x00}, {GPIO_SWPORTA_DDR, 10}, {GPIO_SWPORTA_DR, 10}, {GPIO_EXT_PORTA, 10}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_11", 1, 1, {G2_MUXCFG_006, 12, 0x00}, {GPIO_SWPORTA_DDR, 11}, {GPIO_SWPORTA_DR, 11}, {GPIO_EXT_PORTA, 11}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_12", 1, 1, {G2_MUXCFG_006, 16, 0x00}, {GPIO_SWPORTA_DDR, 12}, {GPIO_SWPORTA_DR, 12}, {GPIO_EXT_PORTA, 12}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_13", 1, 1, {G2_MUXCFG_006, 20, 0x00}, {GPIO_SWPORTA_DDR, 13}, {GPIO_SWPORTA_DR, 13}, {GPIO_EXT_PORTA, 13}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_14", 1, 1, {G2_MUXCFG_006, 24, 0x00}, {GPIO_SWPORTA_DDR, 14}, {GPIO_SWPORTA_DR, 14}, {GPIO_EXT_PORTA, 14}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_15", 1, 1, {G2_MUXCFG_006, 28, 0x00}, {GPIO_SWPORTA_DDR, 15}, {GPIO_SWPORTA_DR, 15}, {GPIO_EXT_PORTA, 15}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_16", 1, 1, {G2_MUXCFG_007, 0, 0x00}, {GPIO_SWPORTA_DDR, 16}, {GPIO_SWPORTA_DR, 16}, {GPIO_EXT_PORTA, 16}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_17", 1, 1, {G2_MUXCFG_007, 4, 0x03}, {GPIO_SWPORTA_DDR, 17}, {GPIO_SWPORTA_DR, 17}, {GPIO_EXT_PORTA, 17}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_18", 1, 1, {G2_MUXCFG_007, 8, 0x03}, {GPIO_SWPORTA_DDR, 18}, {GPIO_SWPORTA_DR, 18}, {GPIO_EXT_PORTA, 18}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_19", 1, 1, {G2_MUXCFG_007, 12, 0x03}, {GPIO_SWPORTA_DDR, 19}, {GPIO_SWPORTA_DR, 19}, {GPIO_EXT_PORTA, 19}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_20", 1, 1, {G2_MUXCFG_007, 16, 0x03}, {GPIO_SWPORTA_DDR, 20}, {GPIO_SWPORTA_DR, 20}, {GPIO_EXT_PORTA, 20}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_21", 1, 1, {G2_MUXCFG_007, 20, 0x03}, {GPIO_SWPORTA_DDR, 21}, {GPIO_SWPORTA_DR, 21}, {GPIO_EXT_PORTA, 21}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_22", 1, 1, {G2_MUXCFG_007, 24, 0x03}, {GPIO_SWPORTA_DDR, 22}, {GPIO_SWPORTA_DR, 22}, {GPIO_EXT_PORTA, 22}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_23", 1, 1, {G2_MUXCFG_007, 28, 0x03}, {GPIO_SWPORTA_DDR, 22}, {GPIO_SWPORTA_DR, 22}, {GPIO_EXT_PORTA, 22}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_24", 1, 1, {G2_MUXCFG_008, 0, 0x00}, {GPIO_SWPORTA_DDR, 16}, {GPIO_SWPORTA_DR, 16}, {GPIO_EXT_PORTA, 16}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_25", 1, 1, {G2_MUXCFG_008, 4, 0x03}, {GPIO_SWPORTA_DDR, 17}, {GPIO_SWPORTA_DR, 17}, {GPIO_EXT_PORTA, 17}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_26", 1, 1, {G2_MUXCFG_008, 8, 0x00}, {GPIO_SWPORTA_DDR, 18}, {GPIO_SWPORTA_DR, 18}, {GPIO_EXT_PORTA, 18}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_27", 1, 1, {G2_MUXCFG_008, 12, 0x00}, {GPIO_SWPORTA_DDR, 19}, {GPIO_SWPORTA_DR, 19}, {GPIO_EXT_PORTA, 19}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_28", 1, 1, {G2_MUXCFG_008, 16, 0x00}, {GPIO_SWPORTA_DDR, 20}, {GPIO_SWPORTA_DR, 20}, {GPIO_EXT_PORTA, 20}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_29", 1, 1, {G2_MUXCFG_008, 20, 0x00}, {GPIO_SWPORTA_DDR, 21}, {GPIO_SWPORTA_DR, 21}, {GPIO_EXT_PORTA, 21}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO1_30", 1, 1, {G2_MUXCFG_008, 24, 0x00}, {GPIO_SWPORTA_DDR, 22}, {GPIO_SWPORTA_DR, 22}, {GPIO_EXT_PORTA, 22}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	GPIO_UNAVAILABLE("GPIO1_31"),
	{"GPIO2_0", 2, 2, {G3_MUXCFG_001, 0, 0x03}, {GPIO_SWPORTA_DDR, 0}, {GPIO_SWPORTA_DR, 0}, {GPIO_EXT_PORTA, 0}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_1", 2, 2, {G3_MUXCFG_001, 4, 0x03}, {GPIO_SWPORTA_DDR, 1}, {GPIO_SWPORTA_DR, 1}, {GPIO_EXT_PORTA, 1}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_2", 2, 2, {G3_MUXCFG_001, 8, 0x03}, {GPIO_SWPORTA_DDR, 2}, {GPIO_SWPORTA_DR, 2}, {GPIO_EXT_PORTA, 2}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_3", 2, 2, {G3_MUXCFG_001, 12, 0x03}, {GPIO_SWPORTA_DDR, 3}, {GPIO_SWPORTA_DR, 3}, {GPIO_EXT_PORTA, 3}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_4", 2, 2, {G3_MUXCFG_001, 16, 0x03}, {GPIO_SWPORTA_DDR, 4}, {GPIO_SWPORTA_DR, 4}, {GPIO_EXT_PORTA, 4}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_5", 2, 2, {G3_MUXCFG_001, 20, 0x03}, {GPIO_SWPORTA_DDR, 5}, {GPIO_SWPORTA_DR, 5}, {GPIO_EXT_PORTA, 5}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_6", 2, 2, {G3_MUXCFG_001, 24, 0x03}, {GPIO_SWPORTA_DDR, 6}, {GPIO_SWPORTA_DR, 6}, {GPIO_EXT_PORTA, 6}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_7", 2, 2, {G3_MUXCFG_001, 28, 0x03}, {GPIO_SWPORTA_DDR, 7}, {GPIO_SWPORTA_DR, 7}, {GPIO_EXT_PORTA, 7}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_8", 2, 2, {G3_MUXCFG_002, 0, 0x03}, {GPIO_SWPORTA_DDR, 8}, {GPIO_SWPORTA_DR, 8}, {GPIO_EXT_PORTA, 18}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_9", 2, 2, {G3_MUXCFG_002, 4, 0x03}, {GPIO_SWPORTA_DDR, 9}, {GPIO_SWPORTA_DR, 9}, {GPIO_EXT_PORTA, 19}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_10", 2, 2, {G3_MUXCFG_002, 8, 0x03}, {GPIO_SWPORTA_DDR, 10}, {GPIO_SWPORTA_DR, 10}, {GPIO_EXT_PORTA, 10}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_11", 2, 2, {G3_MUXCFG_002, 12, 0x03}, {GPIO_SWPORTA_DDR, 11}, {GPIO_SWPORTA_DR, 11}, {GPIO_EXT_PORTA, 11}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_12", 2, 2, {G3_MUXCFG_002, 16, 0x03}, {GPIO_SWPORTA_DDR, 12}, {GPIO_SWPORTA_DR, 12}, {GPIO_EXT_PORTA, 12}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_13", 2, 2, {G3_MUXCFG_002, 20, 0x00}, {GPIO_SWPORTA_DDR, 13}, {GPIO_SWPORTA_DR, 13}, {GPIO_EXT_PORTA, 13}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_14", 2, 2, {G3_MUXCFG_002, 24, 0x03}, {GPIO_SWPORTA_DDR, 14}, {GPIO_SWPORTA_DR, 14}, {GPIO_EXT_PORTA, 14}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_15", 2, 2, {G3_MUXCFG_002, 28, 0x03}, {GPIO_SWPORTA_DDR, 15}, {GPIO_SWPORTA_DR, 15}, {GPIO_EXT_PORTA, 15}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_16", 2, 2, {G3_MUXCFG_003, 0, 0x03}, {GPIO_SWPORTA_DDR, 16}, {GPIO_SWPORTA_DR, 16}, {GPIO_EXT_PORTA, 16}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_17", 2, 2, {G3_MUXCFG_003, 4, 0x03}, {GPIO_SWPORTA_DDR, 17}, {GPIO_SWPORTA_DR, 17}, {GPIO_EXT_PORTA, 17}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_18", 2, 2, {G3_MUXCFG_003, 8, 0x00}, {GPIO_SWPORTA_DDR, 18}, {GPIO_SWPORTA_DR, 18}, {GPIO_EXT_PORTA, 18}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_19", 2, 2, {G3_MUXCFG_003, 12, 0x00}, {GPIO_SWPORTA_DDR, 19}, {GPIO_SWPORTA_DR, 19}, {GPIO_EXT_PORTA, 19}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_20", 2, 2, {G3_MUXCFG_003, 16, 0x00}, {GPIO_SWPORTA_DDR, 20}, {GPIO_SWPORTA_DR, 20}, {GPIO_EXT_PORTA, 20}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_21", 2, 2, {G3_MUXCFG_003, 20, 0x00}, {GPIO_SWPORTA_DDR, 21}, {GPIO_SWPORTA_DR, 21}, {GPIO_EXT_PORTA, 21}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_22", 2, 2, {G3_MUXCFG_003, 24, 0x00}, {GPIO_SWPORTA_DDR, 22}, {GPIO_SWPORTA_DR, 22}, {GPIO_EXT_PORTA, 22}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_23", 2, 2, {G3_MUXCFG_003, 28, 0x00}, {GPIO_SWPORTA_DDR, 23}, {GPIO_SWPORTA_DR, 22}, {GPIO_EXT_PORTA, 22}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_24", 2, 2, {G3_MUXCFG_004, 0, 0x00}, {GPIO_SWPORTA_DDR,  24}, {GPIO_SWPORTA_DR, 24}, {GPIO_EXT_PORTA, 24}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_25", 2, 2, {G3_MUXCFG_004, 4, 0x00}, {GPIO_SWPORTA_DDR,  25}, {GPIO_SWPORTA_DR, 25}, {GPIO_EXT_PORTA, 25}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_26", 2, 2, {G3_MUXCFG_004, 8, 0x03}, {GPIO_SWPORTA_DDR,  26}, {GPIO_SWPORTA_DR, 26}, {GPIO_EXT_PORTA, 26}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_27", 2, 2, {G3_MUXCFG_004, 10, 0x03}, {GPIO_SWPORTA_DDR, 27}, {GPIO_SWPORTA_DR, 27}, {GPIO_EXT_PORTA, 27}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_28", 2, 2, {G3_MUXCFG_004, 16, 0x03}, {GPIO_SWPORTA_DDR, 28}, {GPIO_SWPORTA_DR, 28}, {GPIO_EXT_PORTA, 28}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_29", 2, 2, {G3_MUXCFG_004, 20, 0x03}, {GPIO_SWPORTA_DDR, 29}, {GPIO_SWPORTA_DR, 29}, {GPIO_EXT_PORTA, 29}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_30", 2, 2, {G3_MUXCFG_004, 24, 0x00}, {GPIO_SWPORTA_DDR, 30}, {GPIO_SWPORTA_DR, 30}, {GPIO_EXT_PORTA, 30}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO2_31", 2, 2, {G3_MUXCFG_004, 28, 0x00}, {GPIO_SWPORTA_DDR, 31}, {GPIO_SWPORTA_DR, 31}, {GPIO_EXT_PORTA, 31}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_0", 2, 3, {G3_MUXCFG_005, 0, 0x00}, {GPIO_SWPORTA_DDR, 0}, {GPIO_SWPORTA_DR, 0}, {GPIO_EXT_PORTA, 0}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_1", 2, 3, {G3_MUXCFG_005, 4, 0x00}, {GPIO_SWPORTA_DDR, 1}, {GPIO_SWPORTA_DR, 1}, {GPIO_EXT_PORTA, 1}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_2", 2, 3, {G3_MUXCFG_005, 8, 0x00}, {GPIO_SWPORTA_DDR, 2}, {GPIO_SWPORTA_DR, 2}, {GPIO_EXT_PORTA, 2}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_3", 2, 3, {G3_MUXCFG_005, 12, 0x00}, {GPIO_SWPORTA_DDR, 3}, {GPIO_SWPORTA_DR, 3}, {GPIO_EXT_PORTA, 3}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_4", 2, 3, {G3_MUXCFG_005, 16, 0x03}, {GPIO_SWPORTA_DDR, 4}, {GPIO_SWPORTA_DR, 4}, {GPIO_EXT_PORTA, 4}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_5", 2, 3, {G3_MUXCFG_005, 20, 0x03}, {GPIO_SWPORTA_DDR, 5}, {GPIO_SWPORTA_DR, 5}, {GPIO_EXT_PORTA, 5}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_6", 2, 3, {G3_MUXCFG_005, 24, 0x03}, {GPIO_SWPORTA_DDR, 6}, {GPIO_SWPORTA_DR, 6}, {GPIO_EXT_PORTA, 6}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_7", 2, 3, {G3_MUXCFG_005, 28, 0x03}, {GPIO_SWPORTA_DDR, 7}, {GPIO_SWPORTA_DR, 7}, {GPIO_EXT_PORTA, 7}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_8", 2, 3, {G3_MUXCFG_006, 0, 0x03}, {GPIO_SWPORTA_DDR, 8}, {GPIO_SWPORTA_DR, 8}, {GPIO_EXT_PORTA, 18}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_9", 2, 3, {G3_MUXCFG_006, 4, 0x03}, {GPIO_SWPORTA_DDR, 9}, {GPIO_SWPORTA_DR, 9}, {GPIO_EXT_PORTA, 19}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_10", 2, 3, {G3_MUXCFG_006, 8, 0x03}, {GPIO_SWPORTA_DDR, 10}, {GPIO_SWPORTA_DR, 10}, {GPIO_EXT_PORTA, 10}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_11", 2, 3, {G3_MUXCFG_006, 12, 0x03}, {GPIO_SWPORTA_DDR, 11}, {GPIO_SWPORTA_DR, 11}, {GPIO_EXT_PORTA, 11}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_12", 2, 3, {G3_MUXCFG_006, 16, 0x03}, {GPIO_SWPORTA_DDR, 12}, {GPIO_SWPORTA_DR, 12}, {GPIO_EXT_PORTA, 12}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_13", 2, 3, {G3_MUXCFG_006, 20, 0x03}, {GPIO_SWPORTA_DDR, 13}, {GPIO_SWPORTA_DR, 13}, {GPIO_EXT_PORTA, 13}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_14", 2, 3, {G3_MUXCFG_006, 24, 0x03}, {GPIO_SWPORTA_DDR, 14}, {GPIO_SWPORTA_DR, 14}, {GPIO_EXT_PORTA, 14}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_15", 2, 3, {G3_MUXCFG_006, 28, 0x03}, {GPIO_SWPORTA_DDR, 15}, {GPIO_SWPORTA_DR, 15}, {GPIO_EXT_PORTA, 15}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_16", 2, 3, {G3_MUXCFG_007, 0, 0x03}, {GPIO_SWPORTA_DDR, 16}, {GPIO_SWPORTA_DR, 16}, {GPIO_EXT_PORTA, 16}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_17", 2, 3, {G3_MUXCFG_007, 4, 0x03}, {GPIO_SWPORTA_DDR, 17}, {GPIO_SWPORTA_DR, 17}, {GPIO_EXT_PORTA, 17}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_18", 2, 3, {G3_MUXCFG_007, 8, 0x03}, {GPIO_SWPORTA_DDR, 18}, {GPIO_SWPORTA_DR, 18}, {GPIO_EXT_PORTA, 18}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_19", 2, 3, {G3_MUXCFG_007, 12, 0x03}, {GPIO_SWPORTA_DDR, 19}, {GPIO_SWPORTA_DR, 19}, {GPIO_EXT_PORTA, 19}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_20", 2, 3, {G3_MUXCFG_007, 16, 0x03}, {GPIO_SWPORTA_DDR, 20}, {GPIO_SWPORTA_DR, 20}, {GPIO_EXT_PORTA, 20}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_21", 2, 3, {G3_MUXCFG_007, 20, 0x03}, {GPIO_SWPORTA_DDR, 21}, {GPIO_SWPORTA_DR, 21}, {GPIO_EXT_PORTA, 21}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"GPIO3_22", 2, 3, {G3_MUXCFG_007, 24, 0x03}, {GPIO_SWPORTA_DDR, 22}, {GPIO_SWPORTA_DR, 22}, {GPIO_EXT_PORTA, 22}, FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	GPIO_UNAVAILABLE("GPIO3_22"),
	GPIO_UNAVAILABLE("GPIO3_23"),
	GPIO_UNAVAILABLE("GPIO3_24"),
	GPIO_UNAVAILABLE("GPIO3_25"),
	GPIO_UNAVAILABLE("GPIO3_25"),
	GPIO_UNAVAILABLE("GPIO3_26"),
	GPIO_UNAVAILABLE("GPIO3_27"),
	GPIO_UNAVAILABLE("GPIO3_28"),
	GPIO_UNAVAILABLE("GPIO3_29"),
	GPIO_UNAVAILABLE("GPIO3_30"),			
	GPIO_UNAVAILABLE("GPIO3_31"),
};

static int th1520Setup(void) {
	int i = 0;

	if((th1520->fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
		wiringXLog(LOG_ERR, "wiringX failed to open /dev/mem for raw memory access");
		return -1;
	}
	for(i = 0; i < GPIO_GROUP_COUNT; i++) {
		if((th1520->gpio[i] = (unsigned char *)mmap(0, th1520->page_size, PROT_READ | PROT_WRITE, MAP_SHARED, th1520->fd, th1520->base_addr[i])) == NULL) {
			wiringXLog(LOG_ERR, "wiringX failed to map The %s %s gpio memory address", th1520->brand, th1520->chip);
			return -1;
		}
	}
	for(i = 0; i < PAD_GROUP_COUNT; i++) {
		if((pad_groups_reg[i] = (unsigned char *)mmap(0, th1520->page_size, PROT_READ | PROT_WRITE, MAP_SHARED, th1520->fd, pad_groups_register_physical_address[i])) == NULL) {
			wiringXLog(LOG_ERR, "wiringX failed to map The %s %s pad memory address", th1520->brand, th1520->chip);
			return -1;
		}
	}

	return 0;
}

static char *th1520GetPinName(int pin) {
	return th1520->layout[pin].name;
}

static void th1520SetMap(int *map, size_t size) {
	th1520->map = map;
	th1520->map_size = size;
}

static void th1520SetIRQ(int *irq, size_t size) {
	th1520->irq = irq;
	th1520->irq_size = size;
}

struct layout_t *th1520GetLayout(int i, int *mapping) {
	struct layout_t *pin = NULL;
	unsigned int *grf_reg = NULL;
	unsigned int iomux_value = 0;

	if(mapping == NULL) {
		wiringXLog(LOG_ERR, "The %s %s has not yet been mapped", th1520->brand, th1520->chip);
		return NULL;
	}
	if(wiringXValidGPIO(i) != 0) {
		wiringXLog(LOG_ERR, "The %i is not the right gpio number");
		return NULL;
	}
	if(th1520->fd <= 0 || th1520->gpio == NULL) {
		wiringXLog(LOG_ERR, "The %s %s has not yet been setup by wiringX", th1520->brand, th1520->chip);
		return NULL;
	}

	pin = &th1520->layout[mapping[i]];
	if(pin->bank < 0 || pin->bank >= GPIO_GROUP_COUNT) {
		wiringXLog(LOG_ERR, "pin->bank out of range: %i, expect 0~4", pin->bank);
		return NULL;
	}

	return pin;
}

#define th1520GetPinLayout(i) (th1520GetLayout(i, th1520->map))
#define th1520GetIrqLayout(i) (th1520GetLayout(i, th1520->irq))

static int th1520DigitalWrite(int i, enum digital_value_t value) {
	struct layout_t *pin = NULL;
	unsigned int *out_reg = 0;

	if((pin = th1520GetPinLayout(i)) == NULL) {
		return -1;
	}

	if(pin->mode != PINMODE_OUTPUT) {
		wiringXLog(LOG_ERR, "The %s %s gpio%d is not set to output mode", th1520->brand, th1520->chip, i);
		return -1;
	}

	out_reg = (volatile unsigned int *)(th1520->gpio[pin->bank] + pin->out.offset);
	if(value == HIGH) {
		*out_reg |= (1 << (pin->out.bit));
	} else if (value == LOW) {
		*out_reg &= ~(1 << pin->out.bit);
	} else {
		wiringXLog(LOG_ERR, "invaild value %i for GPIO %i", value, i);
	}

	return 0;
}

static int th1520DigitalRead(int i) {
	struct layout_t *pin = NULL;
	unsigned int *in_reg = NULL;
	uint32_t val = 0;

	if((pin = th1520GetPinLayout(i)) == NULL) {
		return -1;
	}

	if(pin->mode != PINMODE_INPUT) {
		wiringXLog(LOG_ERR, "The %s %s GPIO%d is not set to input mode", th1520->brand, th1520->chip, i);
		return -1;
	}

	in_reg = (volatile unsigned int *)(th1520->gpio[pin->bank] + pin->in.offset);
	val = *in_reg;

	return (int)((val & (1 << pin->in.bit)) >> pin->in.bit);
}

static int th1520PinMode(int i, enum pinmode_t mode) {
	struct layout_t *pin = NULL;
	unsigned int *pad_reg = NULL;
	unsigned int *dir_reg = NULL;

	if((pin = th1520GetPinLayout(i)) == NULL) {
		return -1;
	}

	pad_reg = (volatile unsigned int *)(pad_groups_reg[pin->pad_num] + pin->pad.offset);
	*pad_reg = (*pad_reg & ~(0xf << pin->pad.bit)) | (pin->pad.value << pin->pad.bit);

	dir_reg = (volatile unsigned int *)(th1520->gpio[pin->bank] + pin->direction.offset);
	if(mode == PINMODE_INPUT) {
		*dir_reg &= ~(1 << pin->direction.bit);
	} else if(mode == PINMODE_OUTPUT) {
		*dir_reg |= (1 << pin->direction.bit);
	} else {
		wiringXLog(LOG_ERR, "invalid pin mode %i for GPIO %i", mode, i);
		return -1;
	}

	pin->mode = mode;

	return 0;
}

static int th1520ISR(int i, enum isr_mode_t mode) {
	struct layout_t *pin = NULL;
	char path[PATH_MAX];
	memset(path, 0, sizeof(path));

	if((pin = th1520GetIrqLayout(i)) == NULL) {
		return -1;
	}

	sprintf(path, "/sys/class/gpio/gpio%d", th1520->irq[i]);
	if((soc_sysfs_check_gpio(th1520, path)) == -1) {
		sprintf(path, "/sys/class/gpio/export");
		if(soc_sysfs_gpio_export(th1520, path, th1520->irq[i]) == -1) {
			return -1;
		}
	}

	sprintf(path, "/sys/class/gpio/gpio%d/direction", th1520->irq[i]);
	if(soc_sysfs_set_gpio_direction(th1520, path, "in") == -1) {
		return -1;
	}

	sprintf(path, "/sys/class/gpio/gpio%d/edge", th1520->irq[i]);
	if(soc_sysfs_set_gpio_interrupt_mode(th1520, path, mode) == -1) {
		return -1;
	}

	sprintf(path, "/sys/class/gpio/gpio%d/value", th1520->irq[i]);
	if((pin->fd = soc_sysfs_gpio_reset_value(th1520, path)) == -1) {
		return -1;
	}

	pin->mode = PINMODE_INTERRUPT;

	return 0;
}

static int th1520WaitforInterrupt(int i, int ms) {
	struct layout_t *pin = NULL;

	if((pin = th1520GetIrqLayout(i)) == NULL) {
		return -1;
	}

	if(pin->mode != PINMODE_INTERRUPT) {
		wiringXLog(LOG_ERR, "The %s %s gpio %d is not set to interrupt mode", th1520->brand, th1520->chip, i);
		return -1;
	}

	return soc_wait_for_interrupt(th1520, pin->fd, ms);
}

static int th1520GC(void) {
	struct layout_t *pin = NULL;
	char path[PATH_MAX];
	int i = 0;
	memset(path, 0, sizeof(path));

	if(th1520->map != NULL) {
		for(i = 0; i < th1520->map_size; i++) {
			pin = &th1520->layout[th1520->map[i]];
			if(pin->mode == PINMODE_OUTPUT) {
				pinMode(i, PINMODE_INPUT);
			} else if(pin->mode == PINMODE_INTERRUPT) {
				sprintf(path, "/sys/class/gpio/gpio%d", th1520->irq[i]);
				if((soc_sysfs_check_gpio(th1520, path)) == 0) {
					sprintf(path, "/sys/class/gpio/unexport");
					soc_sysfs_gpio_unexport(th1520, path, th1520->irq[i]);
				}
			}

			if(pin->fd > 0) {
				close(pin->fd);
				pin->fd = 0;
			}
		}
	}

	for(i = 0; i < GPIO_GROUP_COUNT; i++) {
		if(th1520->gpio[i] != NULL) {
			munmap(th1520->gpio[i], th1520->page_size);
			th1520->gpio[i] = NULL;
		}
	}

	for(i = 0; i < GPIO_GROUP_COUNT; i++) {
		if(th1520->gpio[i] != NULL) {
			munmap(th1520->gpio[i], th1520->page_size);
			th1520->gpio[i] = NULL;
		}
	}

	return 0;
}

static int th1520Selectablefd(int i) {
	struct layout_t *pin = NULL;

	if((pin = th1520GetIrqLayout(i)) == NULL) {
		return -1;
	}

	return pin->fd;
}

void th1520Init(void) {
	soc_register(&th1520, "T-Head", "TH1520");

	th1520->layout = layout;

	th1520->support.isr_modes = ISR_MODE_RISING | ISR_MODE_FALLING | ISR_MODE_BOTH | ISR_MODE_NONE;
	th1520->page_size = (1024*64);
	memcpy(th1520->base_addr, gpio_group_register_physical_address, sizeof(gpio_group_register_physical_address));

	th1520->gc = &th1520GC;
	th1520->selectableFd = &th1520Selectablefd;
	th1520->pinMode = &th1520PinMode;
	th1520->setup = &th1520Setup;
	th1520->digitalRead = &th1520DigitalRead;
	th1520->digitalWrite = &th1520DigitalWrite;
	th1520->getPinName = &th1520GetPinName;
	th1520->setMap = &th1520SetMap;
	th1520->setIRQ = &th1520SetIRQ;
	th1520->isr = &th1520ISR;
	th1520->waitForInterrupt = &th1520WaitforInterrupt;
}
