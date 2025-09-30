/*!
 * Basic definition
 * Copyright 2025 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once
#include <Arduino.h>

// 基本使用設定
#define CPU_CLOCK 133000000.0
#define INTR_PWM_RESO 512
#define PWM_BIT 11
#define PWM_RESO 2048
#define ADC_BIT 12
#define ADC_RESO 4096
// #define DAC_MAX_MILLVOLT 5000 // mV
#define DAC_BIT 12
#define DAC_RESO 4096
// #define SAMPLE_FREQ ((CPU_CLOCK / INTR_PWM_RESO) / 8) // 32470.703125khz
#define SAMPLE_FREQ ((CPU_CLOCK / INTR_PWM_RESO) / 4.0) // 65khz
