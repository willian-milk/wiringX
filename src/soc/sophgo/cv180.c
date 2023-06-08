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
#include <ctype.h>

#include "cv180.h"
#include "../../wiringx.h"
#include "../soc.h"
#include "../../i2c-dev.h"

#define CV180_GPIO_GROUP_COUNT 4

const static uintptr_t gpio_register_physical_address[MAX_REG_AREA] = {0x03020000, 0x03021000, 0x03022000, 0x05021000};
#define GPIO_SWPORTA_DR		0x000	
#define GPIO_SWPORTA_DDR		0x004
#define GPIO_EXT_PORTA		0x050

static uintptr_t pinmux_register_virtual_address = NULL;

#define PINMUX_BASE		0x03000000	// pinmux group 1

#define CLEAR_BITS(addr, bit, size) \
	(*addr = *addr & ~(~(-1 << size) << bit) | (~(-1 << size) << bit << REGISTER_WRITE_MASK))
#define GET_BITS(addr, bit, size) \
	((*addr & ~(-1 << size) << bit) >> (bit - size))

struct soc_t *cv180 = NULL;

static struct layout_t {
	char *name;
	int gpio_group;
	int num;

	struct {
		unsigned long offset;
		unsigned long value;
	} pinmux;

	struct {
		unsigned long offset;
		unsigned long bit;
	} direction;

	struct {
		unsigned long offset;
		unsigned long bit;
	} data;

	int support;
	enum pinmode_t mode;
	int fd;
} layout[] = {
	{"XGPIOA_28", 0, 508, {0x104c, 0x3}, {GPIO_SWPORTA_DDR, 28}, {GPIO_SWPORTA_DR, 28},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"XGPIOA_29", 0, 509, {0x1050, 0x3}, {GPIO_SWPORTA_DDR, 29}, {GPIO_SWPORTA_DR, 29},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"PWR_GPIO_26", 3, 406, {0x1084, 0x3}, {GPIO_SWPORTA_DDR, 26}, {GPIO_SWPORTA_DR, 26},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"PWR_GPIO_25", 3, 405, {0x1088, 0x3}, {GPIO_SWPORTA_DDR, 25}, {GPIO_SWPORTA_DR, 25},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"PWR_GPIO_20", 3, 500, {0x1094, 0x3}, {GPIO_SWPORTA_DDR, 20}, {GPIO_SWPORTA_DR, 20},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"PWR_GPIO_19", 3, 499, {0x1090, 0x3}, {GPIO_SWPORTA_DDR, 19}, {GPIO_SWPORTA_DR, 19},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"PWR_GPIO_23", 3, 403, {0x10a0, 0x3}, {GPIO_SWPORTA_DDR, 23}, {GPIO_SWPORTA_DR, 23},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"PWR_GPIO_22", 3, 402, {0x109c, 0x3}, {GPIO_SWPORTA_DDR, 22}, {GPIO_SWPORTA_DR, 22},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"PWR_GPIO_21", 3, 401, {0x1098, 0x3}, {GPIO_SWPORTA_DDR, 21}, {GPIO_SWPORTA_DR, 21},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"PWR_GPIO_18", 3, 398, {0x108c, 0x3}, {GPIO_SWPORTA_DDR, 18}, {GPIO_SWPORTA_DR, 18},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"XGPIOC_9", 2, 425, {0x10f0, 0x3}, {GPIO_SWPORTA_DDR, 9}, {GPIO_SWPORTA_DR, 9},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"XGPIOC_10", 2, 426, {0x10f4, 0x3}, {GPIO_SWPORTA_DDR, 22}, {GPIO_SWPORTA_DR, 22},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"XGPIOA_16", 0, 496, {0x1024, 0x3}, {GPIO_SWPORTA_DDR, 16}, {GPIO_SWPORTA_DR, 16},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"XGPIOA_17", 0, 497, {0x1028, 0x3}, {GPIO_SWPORTA_DDR, 17}, {GPIO_SWPORTA_DR, 17},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"XGPIOA_14", 0, 494, {0x101c, 0x1}, {GPIO_SWPORTA_DDR, 14}, {GPIO_SWPORTA_DR, 14},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"XGPIOA_15", 0, 495, {0x1020, 0x3}, {GPIO_SWPORTA_DDR, 15}, {GPIO_SWPORTA_DR, 15},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"XGPIOA_23", 0, 503, {0x103c, 0x3}, {GPIO_SWPORTA_DDR, 23}, {GPIO_SWPORTA_DR, 23},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"XGPIOA_24", 0, 504, {0x1040, 0x3}, {GPIO_SWPORTA_DDR, 24}, {GPIO_SWPORTA_DR, 24},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"XGPIOA_22", 0, 502, {0x1030, 0x3}, {GPIO_SWPORTA_DDR, 22}, {GPIO_SWPORTA_DR, 22},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"XGPIOA_25", 0, 505, {0x1034, 0x3}, {GPIO_SWPORTA_DDR, 25}, {GPIO_SWPORTA_DR, 25},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"XGPIOA_27", 0, 507, {0x1038, 0x3}, {GPIO_SWPORTA_DDR, 27}, {GPIO_SWPORTA_DR, 27},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"XGPIOA_26", 0, 506, {0x102c, 0x3}, {GPIO_SWPORTA_DDR, 26}, {GPIO_SWPORTA_DR, 26},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"PWR_GPIO_4", 3, 384, {0x1068, 0x3}, {GPIO_SWPORTA_DDR, 4}, {GPIO_SWPORTA_DR, 4},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"XGPIOB_3", 2, 454, {0x10a8, 0x3}, {GPIO_SWPORTA_DDR, 3}, {GPIO_SWPORTA_DR, 3},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
	{"XGPIOB_6", 2, 451, {0x10ac, 0x3}, {GPIO_SWPORTA_DDR, 6}, {GPIO_SWPORTA_DR, 6},  FUNCTION_DIGITAL, PINMODE_NOT_SET, 0},
};

static int cv180Setup(void) {
	int i = 0;

	if((cv180->fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
		wiringXLog(LOG_ERR, "wiringX failed to open /dev/mem for raw memory access");
		return -1;
	}

	for(i = 0; i < CV180_GPIO_GROUP_COUNT; i++) {
		if((cv180->gpio[i] = (unsigned char *)mmap(0, cv180->page_size, PROT_READ | PROT_WRITE, MAP_SHARED, cv180->fd, cv180->base_addr[i])) == NULL) {
			wiringXLog(LOG_ERR, "wiringX failed to map The %s %s GPIO memory address", cv180->brand, cv180->chip);
			return -1;
		}
	}
	if((pinmux_register_virtual_address = (unsigned char *)mmap(0, cv180->page_size, PROT_READ | PROT_WRITE, MAP_SHARED, cv180->fd, PINMUX_BASE)) == NULL) {
		wiringXLog(LOG_ERR, "wiringX failed to map The %s %s CRU memory address", cv180->brand, cv180->chip);
		return -1;
	}

	return 0;
}

static char *cv180GetPinName(int pin) {
	return cv180->layout[pin].name;
}

static void cv180SetMap(int *map, size_t size) {
	cv180->map = map;
	cv180->map_size = size;
}

static void cv180SetIRQ(int *irq, size_t size) {
	cv180->irq = irq;
	cv180->irq_size = size;
}

struct layout_t *cv180GetLayout(int i, int *mapping) {
	struct layout_t *pin = NULL;
	unsigned int *grf_reg = NULL;
	unsigned int iomux_value = 0;

	if(mapping == NULL) {
		wiringXLog(LOG_ERR, "The %s %s has not yet been mapped", cv180->brand, cv180->chip);
		return NULL;
	}
	if(wiringXValidGPIO(i) != 0) {
		wiringXLog(LOG_ERR, "The %i is not the right GPIO number");
		return NULL;
	}
	if(cv180->fd <= 0 || cv180->gpio == NULL) {
		wiringXLog(LOG_ERR, "The %s %s has not yet been setup by wiringX", cv180->brand, cv180->chip);
		return NULL;
	}

	pin = &cv180->layout[mapping[i]];
	if(pin->gpio_group < 0 || pin->gpio_group >= CV180_GPIO_GROUP_COUNT) {
		wiringXLog(LOG_ERR, "pin->group out of range: %i, expect 0~3", pin->gpio_group);
		return NULL;
	}

	return pin;
}

#define cv180GetPinLayout(i) (cv180GetLayout(i, cv180->map))
#define cv180GetIrqLayout(i) (cv180GetLayout(i, cv180->irq))

static int cv180DigitalWrite(int i, enum digital_value_t value) {
	struct layout_t *pin = NULL;
	unsigned int *data_reg = 0;
	uint32_t val = 0;

	if((pin = cv180GetPinLayout(i)) == NULL) {
		return -1;
	}

	if(pin->mode != PINMODE_OUTPUT) {
		wiringXLog(LOG_ERR, "The %s %s GPIO%d is not set to output mode", cv180->brand, cv180->chip, i);
		return -1;
	}

	data_reg = (volatile unsigned int *)(cv180->gpio[pin->gpio_group] + pin->data.offset + GPIO_SWPORTA_DR);
	if(value == HIGH) {
		*data_reg |= (1 << (pin->data.bit));
	} else if(value == LOW) {
		*data_reg &= ~(1 << (pin->data.bit));
	} else {
		wiringXLog(LOG_ERR, "invalid value %i for GPIO %i", value, i);
		return -1;
	}

	return 0;
}

static int cv180DigitalRead(int i) {
	struct layout_t *pin = NULL;
	unsigned int *data_reg = NULL;
	uint32_t val = 0;

	if((pin = cv180GetPinLayout(i)) == NULL) {
		return -1;
	}

	if(pin->mode != PINMODE_INPUT) {
		wiringXLog(LOG_ERR, "The %s %s GPIO%d is not set to input mode", cv180->brand, cv180->chip, i);
		return -1;
	}

	data_reg = (volatile unsigned int *)(cv180->gpio[pin->gpio_group] + pin->data.offset + GPIO_EXT_PORTA);
	val = *data_reg;

	return (int)((val & (1 << pin->data.bit)) >> pin->data.bit);
}

static int cv180PinMode(int i, enum pinmode_t mode) {
	struct layout_t *pin = NULL;
	unsigned int *pinmux_reg = NULL;
	unsigned int *dir_reg = NULL;
	unsigned int mask = 0;

	if((pin = cv180GetPinLayout(i)) == NULL) {
		return -1;
	}

	pinmux_reg = (volatile unsigned int *) (pinmux_register_virtual_address + pin->pinmux.offset);
	*pinmux_reg = pin->pinmux.value;

	dir_reg = (volatile unsigned int *)(cv180->gpio[pin->gpio_group] + pin->direction.offset);
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

static int cv180ISR(int i, enum isr_mode_t mode) {
	struct layout_t *pin = NULL;
	char path[PATH_MAX];
	memset(path, 0, sizeof(path));

	if((pin = cv180GetIrqLayout(i)) == NULL) {
		return -1;
	}

	sprintf(path, "/sys/class/gpio/gpio%d", pin->num);
	if((soc_sysfs_check_gpio(cv180, path)) == -1) {
		sprintf(path, "/sys/class/gpio/export");
		if(soc_sysfs_gpio_export(cv180, path, pin->num) == -1) {
			return -1;
		}
	}

	sprintf(path, "/sys/devices/platform/%x.gpio/gpiochip%d/gpio/gpio%d/direction", gpio_register_physical_address[pin->gpio_group], pin->gpio_group, pin->num);
	if(soc_sysfs_set_gpio_direction(cv180, path, "in") == -1) {
		return -1;
	}

	sprintf(path, "/sys/devices/platform/%x.gpio/gpiochip%d/gpio/gpio%d/edge", gpio_register_physical_address[pin->gpio_group], pin->gpio_group, pin->num);
	if(soc_sysfs_set_gpio_interrupt_mode(cv180, path, mode) == -1) {
		return -1;
	}

	sprintf(path, "/sys/devices/platform/%x.gpio/gpiochip%d/gpio/gpio%d/value", gpio_register_physical_address[pin->gpio_group], pin->gpio_group, pin->num);
	if((pin->fd = soc_sysfs_gpio_reset_value(cv180, path)) == -1) {
		return -1;
	}

	pin->mode = PINMODE_INTERRUPT;

	return 0;
}

static int cv180WaitForInterrupt(int i, int ms) {
	struct layout_t *pin = NULL;

	if((pin = cv180GetIrqLayout(i)) == NULL) {
		return -1;
	}

	if(pin->mode != PINMODE_INTERRUPT) {
		wiringXLog(LOG_ERR, "The %s %s GPIO %d is not set to interrupt mode", cv180->brand, cv180->chip, i);
		return -1;
	}

	return soc_wait_for_interrupt(cv180, pin->fd, ms);
}

static int cv180GC(void) {
	struct layout_t *pin = NULL;
	char path[PATH_MAX];
	int i = 0;
	memset(path, 0, sizeof(path));

	if(cv180->map != NULL) {
		for(i = 0; i < cv180->map_size; i++) {
			pin = &cv180->layout[cv180->map[i]];
			if(pin->mode == PINMODE_OUTPUT) {
				pinMode(i, PINMODE_INPUT);
			} else if(pin->mode == PINMODE_INTERRUPT) {
				sprintf(path, "/sys/class/gpio/gpio%d", pin->num);
				if((soc_sysfs_check_gpio(cv180, path)) == 0) {
					sprintf(path, "/sys/class/gpio/unexport");
					soc_sysfs_gpio_unexport(cv180, path, pin->num);
				}
			}

			if(pin->fd > 0) {
				close(pin->fd);
				pin->fd = 0;
			}
		}
	}

	if(pinmux_register_virtual_address != NULL) {
		munmap(pinmux_register_virtual_address, cv180->page_size);
		pinmux_register_virtual_address = NULL;
	}
	for(i = 0; i < CV180_GPIO_GROUP_COUNT; i++) {
		if(cv180->gpio[i] != NULL) {
			munmap(cv180->gpio[i], cv180->page_size);
			cv180->gpio[i] = NULL;
		}
	}

	return 0;
}

static int cv180SelectableFd(int i) {
	struct layout_t *pin = NULL;

	if((pin = cv180GetIrqLayout(i)) == NULL) {
		return -1;
	}

	return pin->fd;
}

void cv180Init(void) {
	soc_register(&cv180, "Sophgo", "CV180");

	cv180->layout = layout;

	cv180->support.isr_modes = ISR_MODE_RISING | ISR_MODE_FALLING | ISR_MODE_BOTH | ISR_MODE_NONE;
	cv180->page_size = (1024*4);
	memcpy(cv180->base_addr, gpio_register_physical_address, sizeof(gpio_register_physical_address));

	cv180->gc = &cv180GC;
	cv180->selectableFd = &cv180SelectableFd;
	cv180->pinMode = &cv180PinMode;
	cv180->setup = &cv180Setup;
	cv180->digitalRead = &cv180DigitalRead;
	cv180->digitalWrite = &cv180DigitalWrite;
	cv180->getPinName = &cv180GetPinName;
	cv180->setMap = &cv180SetMap;
	cv180->setIRQ = &cv180SetIRQ;
	cv180->isr = &cv180ISR;
	cv180->waitForInterrupt = &cv180WaitForInterrupt;
}
