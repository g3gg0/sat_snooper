/*
 * CX24116/CX24118 DVB-S/S2 Frontend Driver Interface
 *
 * This file provides the public API for controlling the Conexant
 * CX24116/CX24118 demodulator.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 * @brief Register console commands for CX24116 control.
 */
void tuner_init(void);
