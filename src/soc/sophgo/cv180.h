/*
  Copyright (c) 2023 MilkV Ltd.
  Author: Willian <willian@milkv.io>

  This Source Code Form is subject to the terms of the Mozilla Public
  License, v. 2.0. If a copy of the MPL was not distributed with this
  file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#ifndef __WIRINGX_CV180_H
#define __WIRINGX_CV180_H

#include "../soc.h"
#include "../../wiringx.h"

extern struct soc_t *cv180;

void cv180Init(void);

#endif
